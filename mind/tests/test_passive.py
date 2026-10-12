"""Passive observation: `kilo monitor --passive` and `kilo posture --then-watch`.

The link must write nothing while silent, so the body's link watchdog (armed
by the first frame, fed by every frame) runs its own timeline: stop and
LINK_LOST at WD_STOP_MS, REST at WD_REST_MS, LIMP WD_LIMP_S after that (on
the vbody; the real body adds the 1 s REST move first, see body.cpp).
"""

import io
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from kilo.body.client import BodyClient
from kilo.body.timeline import Timeline
from kilo.link.serial_link import LinkConfig, LinkError, SerialLink
from kilo.link.transport import loopback_pair
from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody, open_pty

from conftest import FAST, keep_alive

MIND = Path(__file__).resolve().parents[1]


class CountingTransport:
    def __init__(self, inner):
        self.inner = inner
        self.written = 0
        self.writes = 0

    def read(self, n):
        return self.inner.read(n)

    def write(self, data):
        self.writes += 1
        self.written += len(data)
        self.inner.write(data)

    def close(self):
        self.inner.close()


def test_passive_link_writes_zero_bytes():
    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    t = CountingTransport(a)
    link = SerialLink(lambda: t, config=LinkConfig(**FAST), port_name="loopback", passive=True)
    link.mark_alive()
    link.start()
    try:
        out = io.StringIO()
        tl = Timeline(link, link.clock(), out=out)
        end = time.monotonic() + 1.2
        while time.monotonic() < end:
            link.mark_alive()  # even a live mind must not make a passive link talk
            tl.poll()
            time.sleep(0.02)
        with pytest.raises(LinkError):
            BodyClient(link).hello()
        assert t.writes == 0 and t.written == 0
        assert link.state.stats.frames_tx == 0 and link.state.stats.bytes_tx == 0
        assert vb.received == [] and not vb.wd_armed
        assert link.state.alive and len(link.state.history) >= 20  # telemetry flows unprompted
        text = out.getvalue()
        assert "state: mode LIMP" in text and "tel tick" in text, text
    finally:
        link.stop()
        vb.stop()


def test_go_silent_stops_everything_at_once():
    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    t = CountingTransport(a)
    link = SerialLink(lambda: t, config=LinkConfig(**FAST), port_name="loopback")
    link.mark_alive()
    link.start()
    try:
        client = BodyClient(link)
        client.wait_ready(2)
        client.stand()
        keep_alive(link, 0.5)
        assert vb.posture == "STAND"
        last = link.go_silent()
        n, rx = t.writes, len(vb.received)
        keep_alive(link, 0.5)  # the mind is alive, the goal was latched: still nothing goes out
        assert t.writes == n
        assert last is not None and last[2] in ("HEARTBEAT", "SET_POSTURE")
        assert link.goals() == {}
        # The body's watchdog trips WD_STOP_MS after the last frame *it* took
        # in, on one of its own ticks: that is up to WD_STOP_MS after
        # go_silent() returned plus a tick (a silent vbody's loop sits in a
        # 20 ms read), so wait for it, still alive and still sending nothing.
        stop_s = vb.params[P.ParamId.WD_STOP_MS] / 1000
        end = time.monotonic() + stop_s + 2.0
        while vb.wd_stage < 1 and time.monotonic() < end:
            keep_alive(link, 0.02)
        assert vb.wd_stage >= 1
        assert t.writes == n and len(vb.received) == rx
    finally:
        link.stop()
        vb.stop()


class GateTransport(CountingTransport):
    """Blocks the next write of frame type ``gate_type`` (inside the writer's
    _tx_lock) until ``release`` is set."""

    def __init__(self, inner, codec):
        super().__init__(inner)
        self.codec = codec
        self.gate_type = None
        self.blocked = threading.Event()
        self.release = threading.Event()
        self.types = []

    def write(self, data):
        typ = [f.type for f in self.codec.new_decoder().feed(data)][0]
        self.types.append(typ)
        if typ == self.gate_type:
            self.gate_type = None
            self.blocked.set()
            self.release.wait(5)
        super().write(data)


def test_go_silent_during_a_goal_refresh_write():
    """go_silent() while the writer is inside a goal refresh: it waits for the
    frame on the wire, reports it as the last one, and the rest of that
    refresh (and everything after) never goes out."""
    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    link = SerialLink(lambda: t, config=LinkConfig(**FAST), port_name="loopback")
    t = GateTransport(a, link.codec)
    link.mark_alive()
    link.start()
    try:
        client = BodyClient(link)
        client.wait_ready(2)
        client.stand()
        client.set_balance(False)  # a second goal, refreshed in the same batch after SET_POSTURE
        keep_alive(link, 0.3)
        t.gate_type = link.messages.type_of("SET_POSTURE")
        alive = threading.Event()  # set: stop stamping liveness

        def stamp():
            while not alive.is_set():
                link.mark_alive()
                time.sleep(0.01)

        pump = threading.Thread(target=stamp)
        pump.start()
        try:
            assert t.blocked.wait(2)  # the writer is mid-refresh, holding _tx_lock
            done = []
            th = threading.Thread(target=lambda: done.append(link.go_silent()))
            th.start()
            th.join(0.1)
            assert not done  # go_silent() waits for the write in progress
            n = t.writes
            t.release.set()
            th.join(2)
            assert done and done[0] is not None and done[0][2] == "SET_POSTURE"
            assert t.writes == n + 1  # the frame that was mid-write, and nothing more
            time.sleep(0.5)
            assert t.writes == n + 1 and t.types[-1] == link.messages.type_of("SET_POSTURE")
            assert link.state.stats.tx_silenced >= 1  # SET_BALANCE of the same refresh was dropped
        finally:
            alive.set()
            pump.join()
    finally:
        t.release.set()
        link.stop()
        vb.stop()


def test_go_silent_wakes_a_request_in_flight():
    """A request() waiting for its reply when the link goes silent fails at
    once with LinkError (its frame is dropped), not after every retry."""
    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    t = CountingTransport(a)
    link = SerialLink(lambda: t, config=LinkConfig(**FAST, ack_timeout_s=1.0, retries=3), port_name="loopback")
    link.mark_alive()
    link.start()
    try:
        BodyClient(link).wait_ready(2)
        a.drop_tx = lambda data: True  # the body never answers
        err = []

        def ask():
            try:
                link.request("HELLO_REQ", reply="HELLO")
            except Exception as e:
                err.append(e)

        th = threading.Thread(target=ask)
        th.start()
        time.sleep(0.1)
        t0 = time.monotonic()
        link.go_silent()
        th.join(2)
        assert not th.is_alive() and time.monotonic() - t0 < 0.5
        assert len(err) == 1 and type(err[0]) is LinkError and "silent" in str(err[0])
        with pytest.raises(LinkError):
            link.request("HELLO_REQ", reply="HELLO")
    finally:
        link.stop()
        vb.stop()


@pytest.fixture
def vbody_pty_fast_limp():
    transport, path, slave = open_pty()
    vb = VirtualBody(transport)
    vb.params[P.ParamId.WD_LIMP_S] = 1  # the minimum; the default 10 s is too slow for a test
    vb.start()
    yield vb, path
    vb.stop()
    transport.close()
    os.close(slave)


def _run(path, *args, timeout=20):
    return subprocess.run([sys.executable, "-m", "kilo.cli", "--port", path, *args], cwd=MIND,
                          capture_output=True, text=True, timeout=timeout)


def _when(lines, pattern):
    for t, text in lines:
        if re.search(pattern, text):
            return t
    raise AssertionError(f"no line matching {pattern!r}")


def _timeline(out):
    return [(float(m.group(1)), m.group(2)) for m in
            (re.match(r"\s*([+-]\d+\.\d)s  (.*)", ln) for ln in out.splitlines()) if m]


@pytest.mark.slow
def test_monitor_passive_sends_nothing(vbody_pty_fast_limp):
    vb, path = vbody_pty_fast_limp
    p = _run(path, "monitor", "--passive", "--duration", "1.5")
    assert p.returncode == 0, p.stderr
    assert vb.received == [] and not vb.wd_armed and vb.rx_crc_err == 0
    lines = _timeline(p.stdout)
    assert "sending nothing" in p.stdout and "done: 0 frames sent while watching (0 in total)" in p.stdout, p.stdout
    _when(lines, r"^state: mode LIMP")
    assert sum(1 for _, t in lines if t.startswith("tel tick")) >= 1, p.stdout


@pytest.mark.slow
def test_posture_then_watch_shows_the_watchdog_timeline(vbody_pty_fast_limp):
    vb, path = vbody_pty_fast_limp
    p = _run(path, "posture", "stand", "--for", "0.6", "--then-watch", "4.5")
    assert p.returncode == 0, p.stderr
    out = p.stdout
    assert re.search(r"silent: last frame \w+ sent at \d\d:\d\d:\d\d\.\d{3} \(t=0\)", out), out
    assert "last HEARTBEAT at" in out, out
    lines = _timeline(out)
    # stage 0 at the start: still standing, IDLE
    assert _when(lines, r"^state: mode IDLE") < 0.3, out
    # stage 1 at WD_STOP_MS (0.5 s): epoch bump, EVENT WATCHDOG, LINK_LOST
    t_wd = _when(lines, r"^EVENT WATCHDOG")
    t_ll = _when(lines, r"^mode IDLE -> LINK_LOST")
    assert 0.4 <= t_wd <= 0.75 and 0.4 <= t_ll <= 0.8, out
    _when(lines, r"^epoch 0 -> 1")
    # stage 2 at WD_REST_MS (1.5 s): the watchdog reflex takes over and lies down
    t_rest = _when(lines, r"^owners IDLE/IDLE -> REFLEX/REFLEX.*watchdog REST")
    assert 1.4 <= t_rest <= 1.8, out
    assert vb.posture in ("REST", "LIMP")
    # LIMP the 1 s REST move + WD_LIMP_S (1 s here) after REST: servos off
    t_limp = _when(lines, r"^flags .*-servos_powered")
    assert 1.9 <= t_limp - t_rest <= 2.3, out
    assert vb.posture == "LIMP"
    # nothing was sent after the hold: no LINK_RESTORED, no heartbeat
    assert not any("LINK_RESTORED" in t for _, t in lines), out
    assert "done: 0 frames sent while watching" in out, out


def test_then_watch_needs_for():
    from kilo.cli import parse_args

    with pytest.raises(SystemExit):
        parse_args(["posture", "stand", "--then-watch", "5"])
    a = parse_args(["posture", "stand", "--for", "2", "--then-watch", "15"])
    assert a.hold == 2 and a.then_watch == 15
    assert parse_args(["monitor", "--passive", "--duration", "20"]).passive
