"""Byte transports for the link: pyserial, a raw fd (ptys), and an in-memory loopback.

Interface (structural)::

    transport.read(n: int) -> bytes   up to n bytes; blocks at most ~20-50 ms and
                                      returns b"" on timeout; raises OSError when
                                      the device is gone
    transport.write(data: bytes) -> None   raises OSError on failure
    transport.close() -> None

A TransportFactory is a zero-argument callable returning an open transport;
SerialLink calls it again to reconnect.
"""

from __future__ import annotations

import os
import select
import threading
import time
import tty
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Protocol

DEFAULT_BAUD = 250000
READ_TIMEOUT_S = 0.02


class Transport(Protocol):
    def read(self, n: int) -> bytes: ...

    def write(self, data: bytes) -> None: ...

    def close(self) -> None: ...


TransportFactory = Callable[[], Transport]


def clear_hupcl(fd: int, termios_mod: Any = None) -> bool:
    """Clear HUPCL on a tty fd, so closing it leaves DTR asserted.

    On the NyBoard DTR is AC-coupled to RESET: every DTR rising edge resets
    the 328P. The kernel raises DTR on open and, with HUPCL set, drops it on
    the last close, so each ``kilo`` command would reset the body. termios
    state persists per device across opens, and avrdude (or a replug) sets
    HUPCL again, so this is done on every open. Returns True on success;
    False (silently) for non-ttys, Windows, or any termios error.
    """
    try:
        if termios_mod is None:
            import termios as termios_mod  # POSIX only
        attrs = termios_mod.tcgetattr(fd)
        if attrs[2] & termios_mod.HUPCL:
            attrs[2] &= ~termios_mod.HUPCL
            termios_mod.tcsetattr(fd, termios_mod.TCSANOW, attrs)
        return True
    except Exception:
        return False


def clear_hupcl_path(path: str, termios_mod: Any = None) -> bool:
    """Open ``path`` (non-blocking, no controlling tty), clear HUPCL, close.

    Opening raises DTR (one reset if it was low); the close then keeps it up."""
    try:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except (OSError, AttributeError):
        return False
    try:
        return clear_hupcl(fd, termios_mod)
    finally:
        os.close(fd)


# ---- post-flash settle ---------------------------------------------------------------
@dataclass
class SettleCycle:
    """What one settle open saw (for ``kilo -v flash`` and the hardware notes)."""
    cycle: int
    hupcl_was_set: bool | None = None  # HUPCL in the saved termios when this open found it
    hupcl_clear: bool | None = None    # HUPCL clear when this open closed
    dtr: bool | None = None            # DTR / RTS asserted (TIOCMGET) before close
    rts: bool | None = None
    error: str | None = None


def settle_port(port: str, baud: int = DEFAULT_BAUD, cycles: int = 3, gap_s: float = 0.3, *,
                serial_mod: Any = None, termios_mod: Any = None, ioctl: Callable[..., Any] | None = None,
                sleep: Callable[[float], None] = time.sleep) -> list[SettleCycle]:
    """Leave ``port`` "settled" after avrdude: link baud, HUPCL clear, DTR and
    RTS asserted, so that a later open (SerialTransport) changes nothing on
    the line and does not reset the NyBoard (DTR is AC-coupled to RESET).

    Mechanism (docs: mind/README.md "DTR and resets"). avrdude's arduino
    programmer ends with DTR and RTS explicitly cleared, restores its saved
    termios (HUPCL on) and leaves the line coding at 115200. Each of these is
    a separate way for the *next* opens to make a rising DTR edge: the kernel
    raises DTR on open (unless the saved baud is B0), cdc-acm raises it on a
    termios change from B0 to a real baud, HUPCL drops it again on close, and
    some USB bridges glitch the modem lines on a line-coding change. The old
    sequence fixed one of them per open (a raw open that only cleared HUPCL,
    then the verify open that set 250000), which matches the measured two
    resets before the line stayed up.

    Here every open is the production open (pyserial at the link baud, DTR
    and RTS requested before open() so it asserts them, HUPCL cleared,
    DTR/RTS asserted again explicitly), repeated ``cycles`` times: the first
    makes all the transitions in one go (the one unavoidable reset), the
    rest must then be no-ops whatever the mechanism, and absorb a second
    reset if there is one, before the verify open. ``gap_s`` lets a cycle's
    USB control requests land before the next open.

    Errors are recorded per cycle, never raised (a pty or a missing port just
    reports them). Ptys (the virtual body) have no modem lines: skipped.
    """
    if port.startswith("/dev/pts/"):
        return []
    if serial_mod is None:
        import serial as serial_mod  # noqa: PLC0415
    if termios_mod is None:
        try:
            import termios as termios_mod  # noqa: PLC0415
        except ImportError:  # Windows
            termios_mod = None
    if ioctl is None:
        try:
            import fcntl  # noqa: PLC0415

            ioctl = fcntl.ioctl
        except ImportError:
            ioctl = None
    out: list[SettleCycle] = []
    for i in range(1, cycles + 1):
        rec = SettleCycle(i)
        out.append(rec)
        ser = None
        try:
            ser = serial_mod.Serial()   # port None: configure first, open below
            ser.port = port
            ser.baudrate = baud
            ser.dtr = True              # open() asserts DTR/RTS from these
            ser.rts = True
            ser.open()
            fd = ser.fileno()
            if termios_mod is not None:
                attrs = termios_mod.tcgetattr(fd)
                rec.hupcl_was_set = bool(attrs[2] & termios_mod.HUPCL)
                clear_hupcl(fd, termios_mod)
                rec.hupcl_clear = not termios_mod.tcgetattr(fd)[2] & termios_mod.HUPCL
            ser.dtr = True              # explicit TIOCMBIS, in case the driver dropped it
            ser.rts = True
            rec.dtr, rec.rts = _modem_bits(fd, ioctl)
        except Exception as e:  # SerialException, OSError, termios.error
            rec.error = f"{type(e).__name__}: {e}"
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
        if i < cycles:
            sleep(gap_s)
    return out


def _modem_bits(fd: int, ioctl: Callable[..., Any] | None) -> tuple[bool | None, bool | None]:
    if ioctl is None:
        return None, None
    try:
        import struct
        import termios

        bits = struct.unpack("I", ioctl(fd, termios.TIOCMGET, struct.pack("I", 0)))[0]
        return bool(bits & termios.TIOCM_DTR), bool(bits & termios.TIOCM_RTS)
    except Exception:
        return None, None


def describe_settle(cycles: list[SettleCycle]) -> str:
    """One line per cycle, e.g. ``1: HUPCL was set, now clear; DTR 1 RTS 1``."""
    def b(v: bool | None) -> str:
        return "?" if v is None else str(int(v))
    lines = []
    for c in cycles:
        if c.error:
            lines.append(f"{c.cycle}: {c.error}")
        else:
            lines.append(f"{c.cycle}: HUPCL {'was set' if c.hupcl_was_set else 'was clear'}"
                         f"{'' if c.hupcl_clear else ', NOT cleared'}; DTR {b(c.dtr)} RTS {b(c.rts)}")
    return "\n".join(lines)


class SerialTransport:
    """pyserial port, 8N1, no flow control, low-latency mode where supported."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD):
        import serial  # pyserial; imported lazily so tests need not have it

        self.port = port
        self._ser = serial.Serial(
            port, baud, timeout=READ_TIMEOUT_S, write_timeout=0.5,
            exclusive=True,  # flock: a second kilo process cannot grab the port
        )
        try:  # keep DTR asserted after close: no reset on the next open
            clear_hupcl(self._ser.fileno())
        except (AttributeError, OSError, ValueError):
            pass  # no fileno() (Windows)
        try:  # ASYNC_LOW_LATENCY (§5); not supported on every driver
            self._ser.set_low_latency_mode(True)
        except (AttributeError, OSError, ValueError, NotImplementedError):
            pass
        self._ser.reset_input_buffer()

    def read(self, n: int) -> bytes:
        import serial

        try:
            first = self._ser.read(1)
            if not first:
                return b""
            waiting = self._ser.in_waiting
            return first + (self._ser.read(min(waiting, n - 1)) if waiting else b"")
        except serial.SerialException as e:
            raise OSError(str(e)) from e

    def write(self, data: bytes) -> None:
        import serial

        try:
            self._ser.write(data)
        except serial.SerialException as e:
            raise OSError(str(e)) from e

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass


class FdTransport:
    """A raw file descriptor, e.g. a pty end. Used for the virtual body.

    Writes are non-blocking: when nobody drains the other end, bytes are
    dropped (like a UART with nobody listening) instead of wedging the writer.
    """

    def __init__(self, fd: int, owns_fd: bool = True):
        self.fd = fd
        self._owns = owns_fd
        os.set_blocking(fd, False)

    @classmethod
    def open(cls, path: str) -> "FdTransport":
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        try:
            tty.setraw(fd)
        except Exception:
            pass
        return cls(fd)

    def read(self, n: int) -> bytes:
        r, _, _ = select.select([self.fd], [], [], READ_TIMEOUT_S)
        if not r:
            return b""
        try:
            data = os.read(self.fd, n)
        except BlockingIOError:
            return b""
        if not data:
            raise OSError("EOF on fd")
        return data

    def write(self, data: bytes) -> None:
        try:
            os.write(self.fd, data)
        except BlockingIOError:
            pass  # buffer full: drop

    def close(self) -> None:
        if self._owns:
            try:
                os.close(self.fd)
            except OSError:
                pass


class _Pipe:
    def __init__(self) -> None:
        self.buf: deque[bytes] = deque()
        self.cond = threading.Condition()


class LoopbackEnd:
    """One end of an in-memory duplex byte pipe. Test transport.

    ``fail()`` makes subsequent reads and writes raise OSError, to simulate an
    unplugged adapter. ``drop_tx`` is a hook ``f(data) -> bool``; returning True
    drops the write (lossy link).
    """

    def __init__(self, rx: _Pipe, tx: _Pipe):
        self._rx, self._tx = rx, tx
        self.failed = False
        self.closed = False
        self.drop_tx: Callable[[bytes], bool] | None = None

    def read(self, n: int) -> bytes:
        if self.failed or self.closed:
            time.sleep(READ_TIMEOUT_S)
            raise OSError("loopback failed")
        with self._rx.cond:
            if not self._rx.buf:
                self._rx.cond.wait(READ_TIMEOUT_S)
            if not self._rx.buf:
                return b""
            out = bytearray()
            while self._rx.buf and len(out) < n:
                chunk = self._rx.buf.popleft()
                take = n - len(out)
                out += chunk[:take]
                if len(chunk) > take:
                    self._rx.buf.appendleft(chunk[take:])
            return bytes(out)

    def write(self, data: bytes) -> None:
        if self.failed or self.closed:
            raise OSError("loopback failed")
        if self.drop_tx and self.drop_tx(data):
            return
        with self._tx.cond:
            self._tx.buf.append(bytes(data))
            self._tx.cond.notify_all()

    def close(self) -> None:
        self.closed = True

    def fail(self) -> None:
        self.failed = True


def loopback_pair() -> tuple[LoopbackEnd, LoopbackEnd]:
    """Two connected ends: what one writes, the other reads."""
    a, b = _Pipe(), _Pipe()
    return LoopbackEnd(a, b), LoopbackEnd(b, a)


def open_port(port: str, baud: int = DEFAULT_BAUD) -> Transport:
    """Open ``port``: ptys (virtual body) as raw fds, everything else via pyserial."""
    if port.startswith("/dev/pts/"):
        return FdTransport.open(port)
    return SerialTransport(port, baud)
