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
from typing import Callable, Protocol

DEFAULT_BAUD = 250000
READ_TIMEOUT_S = 0.02


class Transport(Protocol):
    def read(self, n: int) -> bytes: ...

    def write(self, data: bytes) -> None: ...

    def close(self) -> None: ...


TransportFactory = Callable[[], Transport]


class SerialTransport:
    """pyserial port, 8N1, no flow control, low-latency mode where supported."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD):
        import serial  # pyserial; imported lazily so tests need not have it

        self.port = port
        self._ser = serial.Serial(
            port, baud, timeout=READ_TIMEOUT_S, write_timeout=0.5,
            exclusive=True,  # flock: a second kilo process cannot grab the port
        )
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
