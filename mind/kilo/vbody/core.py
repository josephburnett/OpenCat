"""The real body core as a virtual body (docs/architecture.md §6 "Virtual body").

``body/test/core_harness.cpp`` links the firmware's ``core/`` (the exact
dispatch, reflexes, gait, keyframe player and arbiter that run on Kilo) with
the host fakes of its hardware seams and drives it from stdin, one command
per line (see that file). This module runs it as a subprocess and presents it
to the mind as a transport, so ``SerialLink``/``BodyClient`` talk to the real
body logic instead of the Python re-implementation in ``kilo.vbody.sim``.

Bytes the mind writes are fed to the core as they arrive (``rx``: link_poll,
as the AVR's main loop polls the UART between ticks); every byte the core
writes goes back to the mind. Ticks (20 ms of body time each) run main.cpp's
``run_tick`` with scripted inputs: attitude, IMU status, battery counts.

Two clocks:

- **stepped** (tests; the default): body time only moves when the caller runs
  ``step(n)``. The mind's ``SerialLink`` gets the same ``SimClock`` (its
  ``clock`` argument) and talks over an in-process transport
  (``CoreBody.transport``), and every tick is in lockstep with it: advance the
  clock 20 ms, stamp the mind's liveness (``alive=True``), wait for one full
  pass of the link's writer (the heartbeat and the 5 Hz goal refresh due at
  the new time are written), feed those bytes, run the tick, and wait until
  the link's reader has handled everything the tick sent. So a 12.5 s
  watchdog timeline takes about a second, the order of mind and body actions
  per tick is fixed, and nothing depends on how loaded the machine is.
  Commands (``request()``) are answered at once without advancing body time,
  like the real body answering between ticks. Caveat: anything in the mind
  that waits on the link's clock with a timeout (``wait_until``,
  ``wait_ready``, ``stop_walk(wait_s)``) never times out while nobody steps;
  use ``run_until`` instead.
- **real time** (``start_realtime()``; ``open_pty()`` for ``kilo --port``):
  a thread runs one tick every 20 ms of wall time, for a mind in another
  process (the CLI, the daemon) whose clock is ``time.monotonic`` and cannot
  be shared.

Boots (power-on, ``reset()``, an acked REBOOT) take ``boot_s`` (0.3 s, the
AVR's DMP init) of body time: no ticks, the boot's HELLO is held, and what
the mind sends meanwhile waits (the UART's RX ring) and is handled first.
A REBOOT boots warm (the PCA9685 kept driving), as on Kilo.

Harness limits (the fakes, not the core): the TX ring never fills (no stream
drops), the ultrasonic reads nothing, unsaved calibration survives a reboot
(the config image is not re-read from the fake EEPROM), and the IMU is a
scripted attitude rather than DMP packets.

``python -m kilo.vbody.core`` builds the harness and serves it on a pty.
"""

from __future__ import annotations

import os
import subprocess
import tempfile
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable

from kilo.link.codec import DecodeError, default_codec
from kilo.link.transport import READ_TIMEOUT_S, FdTransport
from kilo.protocol import kilo_protocol as P

TICK_S = P.TICK_MS / 1000.0
REPO = Path(__file__).resolve().parents[3]
BODY = REPO / "body"
BUILD_SCRIPT = BODY / "tools" / "build_link_harness.sh"
CORE_BUILD = 0x45524F43  # HELLO.fw_build of the host core ("CORE")
BATT_MV_PER_64 = 646     # config_defaults' ADC scale (core/config.cpp)
LOCKSTEP_TIMEOUT_S = 5.0
BOOT_S = 0.3  # reset to HELLO on the AVR: mostly the DMP firmware upload (doc §3a)


def build_harness(out: Path | str | None = None) -> Path:
    """Build body/test/core_harness.cpp (cached: rebuilt when a body source
    is newer than the binary). Needs g++/gcc. Returns the binary's path."""
    if out is None:
        out = Path(tempfile.gettempdir()) / f"kilo-core-harness-{os.getuid()}" / "core_harness"
    out = Path(out)
    srcs = [*BODY.glob("src/core/*"), *BODY.glob("src/proto/*"), *BODY.glob("test/*.cpp"),
            *BODY.glob("test/*.h"), BUILD_SCRIPT]
    if out.exists() and out.stat().st_mtime >= max(s.stat().st_mtime for s in srcs):
        return out
    subprocess.run(["bash", str(BUILD_SCRIPT), str(out), "core_harness"], check=True, capture_output=True,
                   timeout=600)
    return out


class SimClock:
    """Body time for the stepped mode: a callable like ``time.monotonic``."""

    def __init__(self, start: float = 1000.0):
        self._t = start

    def __call__(self) -> float:
        return self._t

    def advance(self, dt: float) -> None:
        self._t += dt


class _Pipe:
    """One direction of the in-process transport. ``writes`` counts writes;
    ``idle_mark`` is ``writes`` as of the last time the reader came back for
    more with nothing left (so it had handled everything before)."""

    def __init__(self) -> None:
        self.buf: deque[bytes] = deque()
        self.cond = threading.Condition()
        self.writes = 0
        self.idle_mark = 0

    def put(self, data: bytes) -> None:
        with self.cond:
            self.buf.append(bytes(data))
            self.writes += 1
            self.cond.notify_all()

    def take_all(self) -> bytes:
        with self.cond:
            out = b"".join(self.buf)
            self.buf.clear()
            return out

    def wait_data(self, timeout: float) -> bool:
        with self.cond:
            if not self.buf:
                self.cond.wait(timeout)
            return bool(self.buf)


class CoreEnd:
    """The mind's end of the in-process transport (the Transport interface)."""

    def __init__(self, rx: _Pipe, tx: _Pipe):
        self._rx, self._tx = rx, tx
        self.closed = False

    def read(self, n: int) -> bytes:
        if self.closed:
            raise OSError("closed")
        with self._rx.cond:
            if not self._rx.buf:
                self._rx.idle_mark = self._rx.writes  # everything before was handled
                self._rx.cond.wait(READ_TIMEOUT_S)
            out = bytearray()
            while self._rx.buf and len(out) < n:
                chunk = self._rx.buf.popleft()
                take = n - len(out)
                out += chunk[:take]
                if len(chunk) > take:
                    self._rx.buf.appendleft(chunk[take:])
            return bytes(out)

    def write(self, data: bytes) -> None:
        if self.closed:
            raise OSError("closed")
        self._tx.put(data)

    def close(self) -> None:
        self.closed = True


class CoreBody:
    """The C core as a body. See the module docstring.

    ``sent`` records every frame the core sent, decoded: (clock, MsgId name,
    message). ``events(name)`` filters EVENTs.
    """

    def __init__(self, binary: Path | str | None = None, *, clock: Callable[[], float] | None = None,
                 config_state: int = P.ConfigState.OK, imu_ok: bool = True, boot_s: float = BOOT_S):
        self.binary = Path(binary) if binary is not None else build_harness()
        self.clock: Any = clock if clock is not None else SimClock()
        self.proc = subprocess.Popen([str(self.binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                                     bufsize=1)
        self._hlock = threading.RLock()
        self._decoder = default_codec().new_decoder()
        self.sent: list[tuple[float, str, Any]] = []
        self.link: Any = None  # the SerialLink kept in lockstep (attach())
        self._to_body, self._to_mind = _Pipe(), _Pipe()
        self._port: FdTransport | None = None  # pty mode
        self._running = True
        self._rt: threading.Thread | None = None
        # Boot time: from a reset to the HELLO the body is busy (the DMP init);
        # what it sends is held and what the mind sends waits in the UART ring.
        self.boot_s = boot_s
        self._booting_until: float | None = None
        self._held = bytearray()
        with self._hlock:
            self._reply("(power-on)")
        if config_state != P.ConfigState.OK or not imu_ok:
            self.reset(config_state=config_state, imu_ok=imu_ok)
        self._pump_t = threading.Thread(target=self._pump, name="corebody-pump", daemon=True)
        self._pump_t.start()

    # ---- transports ----------------------------------------------------------------
    def transport(self) -> CoreEnd:
        """A transport for SerialLink (pass ``lambda: core.transport()`` as the factory)."""
        return CoreEnd(self._to_mind, self._to_body)

    def attach(self, link: Any) -> Any:
        """Keep ``link`` (a SerialLink on ``transport()`` with ``clock=core.clock``)
        in lockstep with ``step()``. Returns it."""
        self.link = link
        return link

    def open_pty(self) -> str:
        """Serve the core on a pty (real time: call ``start_realtime()``).
        Returns the slave path for ``kilo --port``."""
        master, slave = os.openpty()
        import tty

        tty.setraw(master)
        tty.setraw(slave)
        self._port = FdTransport(master)
        self._slave_fd = slave  # kept open: the master never sees EIO while no client is attached
        early = self._to_mind.take_all()  # already recorded in `sent`
        if early:
            self._port.write(early)
        return os.ttyname(slave)

    # ---- the harness -------------------------------------------------------------
    def _cmd(self, line: str) -> list[str]:
        """Run one harness command; deliver what the core wrote; return the
        other lines it printed (dump)."""
        with self._hlock:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
            return self._reply(line)

    def _reply(self, line: str) -> list[str]:
        with self._hlock:
            other = []
            while True:
                ln = self.proc.stdout.readline()
                if not ln:
                    raise RuntimeError(f"core harness died after {line!r}")
                ln = ln.rstrip("\n")
                if ln == "end":
                    return other
                if ln.startswith("tx"):
                    self._deliver(bytes.fromhex(ln[3:]))
                elif ln == "boot":
                    if self.boot_s > 0:
                        self._booting_until = self.clock() + self.boot_s
                elif ln.startswith("error"):
                    raise RuntimeError(ln)
                else:
                    other.append(ln)

    def _deliver(self, data: bytes) -> None:
        if not data:
            return
        if self._booting_until is not None:
            self._held += data
            return
        now = self.clock()
        for f in self._decoder.feed(data):
            if isinstance(f, DecodeError):
                self.sent.append((now, "DECODE_ERROR", f))
                continue
            try:
                name, msg = P.MsgId(f.type).name, P.decode_payload(f.type, f.payload)
            except ValueError:
                name, msg = str(f.type), f.payload
            self.sent.append((now, name, msg))
        if self._port is not None:
            self._port.write(data)
        else:
            self._to_mind.put(data)

    def _booting(self) -> bool:
        """Still in the boot time; at its end, send what the boot sent (HELLO)."""
        if self._booting_until is None:
            return False
        if self.clock() < self._booting_until - 1e-9:
            return True
        self._booting_until = None
        data, self._held = bytes(self._held), bytearray()
        self._deliver(data)
        return False

    def _forward_rx(self) -> None:
        if self._booting():
            return  # the bytes wait (the UART's RX ring) until the core runs
        data = self._to_body.take_all()
        if data:
            self._cmd("rx " + data.hex())

    def _pump(self) -> None:
        """Bytes from the mind go to the core as they arrive (link_poll)."""
        while self._running:
            try:
                if self._port is not None:
                    data = self._port.read(256)
                    if not data:
                        continue
                    self._to_body.put(data)
                if self._to_body.wait_data(READ_TIMEOUT_S):
                    with self._hlock:
                        self._forward_rx()
            except (OSError, RuntimeError, ValueError):
                if not self._running:
                    return
                time.sleep(0.01)

    # ---- inputs ------------------------------------------------------------------------
    def reset(self, warm: bool = False, *, cause: int = P.ResetCause.WARM_UNEXPECTED,
              config_state: int = P.ConfigState.OK, imu_ok: bool = True) -> None:
        """Cold (power-on: epoch 0, limp, inputs back to level/fresh/8.1 V) or
        warm (the PCA kept driving: the pose is held; epoch = last + 1)."""
        if warm:
            self._cmd(f"reset warm {int(cause)}")
        else:
            self._cmd(f"reset cold {int(config_state)} {int(bool(imu_ok))}")

    def set_attitude(self, pitch_deg: float = 0, roll_deg: float = 0, rate_pitch_dps: float = 0,
                     rate_roll_dps: float = 0, rate_yaw_dps: float = 0) -> None:
        """What the IMU reports from the next tick on (pitch nose down +, roll right side up +)."""
        v = [round(x * 16) for x in (pitch_deg, roll_deg, rate_pitch_dps, rate_roll_dps, rate_yaw_dps)]
        self._cmd("att " + " ".join(map(str, v)))

    def set_imu(self, status: str) -> None:
        """The IMU read result from the next tick on: "fresh", "none" (stale) or "err"."""
        assert status in ("fresh", "none", "err")
        self._cmd(f"imu {status}")

    def set_battery(self, mv: float | None = None, *, counts: int | None = None) -> None:
        """The pack voltage the ADC samples at 10 Hz (or raw counts); mv=None
        and counts=None: no samples at all."""
        if counts is None and mv is not None:
            counts = round(mv * 64 / BATT_MV_PER_64)
        self._cmd("batt none" if counts is None else f"batt {int(counts)}")

    # ---- time --------------------------------------------------------------------------
    def step(self, n: int = 1, *, alive: bool = True) -> None:
        """Run ``n`` ticks in lockstep with the attached link. ``alive``:
        stamp the mind's liveness each tick (False = a stalled mind)."""
        assert isinstance(self.clock, SimClock), "step() needs the stepped clock"
        for _ in range(n):
            self.clock.advance(TICK_S)
            link = self.link
            if link is not None and link.running:
                if alive:
                    link.mark_alive()
                self._wait_writer(link)
            with self._hlock:
                if self._booting():
                    continue  # no tick until the init is done
                self._forward_rx()
                self._cmd("tick 1")
            if link is not None and link.running:
                self._wait_reader()

    def run_until(self, pred: Callable[[], bool], timeout_s: float, *, alive: bool = True) -> int | None:
        """Step until ``pred()``; returns the ticks run, or None after ``timeout_s`` of body time."""
        for i in range(int(round(timeout_s / TICK_S)) + 1):
            if pred():
                return i
            self.step(alive=alive)
        return None

    def _wait_writer(self, link: Any) -> None:
        p0 = link.writer_passes
        end = time.monotonic() + LOCKSTEP_TIMEOUT_S
        while link.writer_passes < p0 + 2:  # one whole pass that started after the advance
            if time.monotonic() > end:
                raise RuntimeError("link writer did not run")
            time.sleep(0.0002)

    def _wait_reader(self) -> None:
        p = self._to_mind
        end = time.monotonic() + LOCKSTEP_TIMEOUT_S
        while True:
            with p.cond:
                if not p.buf and p.idle_mark == p.writes:
                    return
            if time.monotonic() > end:
                raise RuntimeError("link reader did not drain")
            time.sleep(0.0002)

    def start_realtime(self) -> "CoreBody":
        """Tick every 20 ms of wall time in a thread (the clock should be time.monotonic)."""
        def run() -> None:
            nxt = time.monotonic()
            while self._running:
                nxt += TICK_S
                d = nxt - time.monotonic()
                if d > 0:
                    time.sleep(d)
                elif d < -0.2:
                    nxt = time.monotonic()  # suspended: do not burst
                try:
                    with self._hlock:
                        if self._booting():
                            continue
                        self._forward_rx()
                        self._cmd("tick 1")
                except (RuntimeError, ValueError, OSError):
                    return

        self._rt = threading.Thread(target=run, name="corebody-tick", daemon=True)
        self._rt.start()
        return self

    # ---- observation ---------------------------------------------------------------------
    def dump(self) -> dict[str, Any]:
        """The core's state (body/test/core_harness.cpp dump()): ints, and lists for
        owner, out (canonical joint order, a16, -32768 = limp), pose, balance_corr."""
        for ln in self._cmd("dump"):
            if ln.startswith("state "):
                out: dict[str, Any] = {}
                for kv in ln.split()[1:]:
                    k, v = kv.split("=", 1)
                    out[k] = [int(x) for x in v.split(",")] if "," in v else int(v)
                return out
        raise RuntimeError("no state line")

    def events(self, name: str | None = None, since: float = -1e18) -> list[tuple[float, Any]]:
        """EVENT messages sent (clock, message), optionally of one EventCode name."""
        return [(t, m) for t, n, m in list(self.sent) if n == "EVENT" and t >= since
                and (name is None or m.code == P.EventCode[name])]

    def frames(self, name: str, since: float = -1e18) -> list[tuple[float, Any]]:
        return [(t, m) for t, n, m in list(self.sent) if n == name and t >= since]

    def close(self) -> None:
        self._running = False
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        if self._port is not None:
            self._port.close()
            os.close(self._slave_fd)

    def __enter__(self) -> "CoreBody":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()


def main(argv: list[str] | None = None) -> int:
    """Serve the real core on a pty in real time: ``kilo --port <path> ...``."""
    import argparse

    ap = argparse.ArgumentParser(prog="python -m kilo.vbody.core", description=main.__doc__)
    ap.add_argument("--config-state", type=int, default=0, help="HELLO config_state (2 = DEFAULTED)")
    args = ap.parse_args(argv)
    body = CoreBody(clock=time.monotonic, config_state=args.config_state)
    path = body.open_pty()
    body.start_realtime()
    print(f"real body core on {path}  (kilo --port {path} ...); Ctrl-C to stop", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        body.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
