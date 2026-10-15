"""The walk's plumbing: the body clock, the stream's ends, SerialLink's
FEET path, BodyClient.walk against the threaded virtual body, the daemon's
walk ops, `kilo walk`, and the walk in the monitor, the log summary and the
behaviour adapter."""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from kilo.behaviour.adapter import BodyAdapter
from kilo.body.client import BodyClient
from kilo.body.state import BodyState
from kilo.cli import _event_text, parse_args, render_monitor
from kilo.link import consts
from kilo.link.serial_link import LinkConfig, SerialLink
from kilo.link.transport import loopback_pair
from kilo.logs.analysis import format_summary, gait_summary
from kilo.logs.reader import LogEvent, TelemetrySample
from kilo.planner.model import BodyModel
from kilo.planner.plan import Planner
from kilo.planner import walker as walker_mod
from kilo.planner.walker import CLOCK_RESET_FRAMES, BodyClock, Stream, WalkError
from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody, open_pty

from conftest import FAST
from test_daemon import remote, sockdir, start_daemon, vbody_pty, wait_for  # noqa: F401  (fixtures)

MIND = Path(__file__).resolve().parents[1]
EV = P.EventCode
R = P.GoalRefusedReason
M = P.MsgId


# ---- the body clock and the stream's ends, without a body -----------------------------------------

def test_body_clock_takes_the_least_delayed_frame():
    bc = BodyClock()
    assert bc.now(5.0) is None
    for i, delay in enumerate((0.004, 0.001, 0.009, 0.002)):
        bc.add(100 + i, 10.0 + i * 0.02 + delay)
    # the frame of tick 101 arrived 1 ms after its tick: body time = local + (101 x 20 ms - 10.021)
    assert bc.now(10.021) == pytest.approx(101.0)
    assert bc.now(10.031) == pytest.approx(101.5)
    bc.add(0xFFFF, 0.0)
    assert bc.tick == 0xFFFF and bc.add(2, 0.06) == 0x10002  # unwrapped across the u16 wrap


def test_body_clock_follows_a_body_stall():
    """The body's tick falls back 80 ms (a stall over 40 ms makes its
    scheduler resync): within CLOCK_RESET_FRAMES frames the estimate follows,
    instead of staying ahead for the window's 2 s (FEET more than the
    queue's depth ahead are dropped)."""
    bc = BodyClock()
    t = 10.0
    for i in range(100):
        bc.add(i, t + i * 0.02 + 0.001 + (i % 3) * 0.002)  # a little jitter
    assert bc.now(10.0 + 99 * 0.02 + 0.001) == pytest.approx(99.0)
    stall = 0.08
    for i in range(100, 100 + CLOCK_RESET_FRAMES):
        bc.add(i, t + i * 0.02 + stall + 0.001)
    rx = t + (100 + CLOCK_RESET_FRAMES - 1) * 0.02 + stall + 0.001
    assert bc.now(rx) == pytest.approx(100 + CLOCK_RESET_FRAMES - 1, abs=0.15) and bc.resets == 1
    # a late frame or two (the reader paused) does not restart it
    bc.add(105, rx + 0.02 + 0.05)
    bc.add(106, rx + 0.04 + 0.001)
    assert bc.resets == 1 and bc.now(rx + 0.04 + 0.001) == pytest.approx(106, abs=0.15)


class _Link:
    def __init__(self):
        self.state = BodyState(clock=lambda: 0.0)
        self.state.epoch = 3
        self.logger = None
        self.alive = True
        self.sent = []

    def send_stream(self, name, payload):
        self.sent.append(P.Feet.unpack(payload))
        return True

    def mind_alive(self):
        return self.alive

    def tel(self, tick, rx, mode=P.Mode.GAIT, epoch=3):
        st = self.state
        st.on_telemetry(P.Telemetry(tick=tick, mode=mode, epoch=epoch), rx)

    def event(self, code, arg=0, epoch=3):
        st = self.state
        st.events.append((0.0, P.Event(code=code, arg=arg, epoch=epoch)))
        st.event_count += 1
        st.epoch = epoch


def _stream(link, done=None):
    link.tel(50, 1.001)
    bc = BodyClock()
    bc.add(50, 1.001)
    s = Stream(link, Planner(BodyModel(), "trot", tick=55), 3, bc, done or (lambda: None))
    return s


def test_stream_sends_five_ticks_ahead_and_catches_up():
    link = _Link()
    s = _stream(link)
    assert s.wake(1.005) and [f.tick for f in link.sent] == [55]
    assert link.sent[0].epoch == 3 and link.sent[0].z_mm == [76] * 4
    link.tel(51, 1.021)
    assert s.wake(1.025) and link.sent[-1].tick == 56
    # three wakes missed: the next one sends what the body has not played yet
    link.tel(55, 1.101)
    assert s.wake(1.105) and [f.tick for f in link.sent[2:]] == [57, 58, 59, 60] and s.max_behind == 4
    # far behind: ticks already past are planned (the plan stays continuous) but not sent
    link.tel(70, 1.401)
    s.wake(1.405)
    assert s.skipped == 10 and link.sent[-1].tick == 75


@pytest.mark.parametrize("what,why", [
    (lambda l: l.event(EV.GAIT_STOPPED, 1), "settled after a FEET underrun"),
    (lambda l: l.event(EV.GAIT_STOPPED, 0), "a posture or an epoch bump"),
    (lambda l: l.event(EV.GOAL_REFUSED, M.FEET << 8 | R.FALLEN), "refused FEET: FALLEN"),
    (lambda l: l.event(EV.FELL, 1, epoch=4), "Kilo fell"),
    (lambda l: setattr(l.state, "reboots", 1), "rebooted"),
    (lambda l: setattr(l.state, "epoch", 9), "epoch changed"),
])
def test_stream_ends(what, why):
    link = _Link()
    s = _stream(link)
    assert s.wake(1.005)
    what(link)
    assert s.wake(1.025) is False and why in s.ended and not s.ok


def test_stream_ends_when_gait_never_comes_or_goes_away():
    link = _Link()
    s = _stream(link)
    link.state.history.clear()
    for i in range(40):
        link.tel(51 + i, 1.021 + i * 0.02, mode=P.Mode.IDLE)
        if not s.wake(1.025 + i * 0.02):
            break
    assert "did not enter GAIT" in s.ended
    link = _Link()
    s = _stream(link)
    link.tel(51, 1.021)
    assert s.wake(1.025) and s.gait_seen
    link.tel(52, 1.041, mode=P.Mode.IDLE)
    assert s.wake(1.045) is False and "left GAIT (mode IDLE)" in s.ended


def test_stream_stops_sending_while_the_mind_stalls():
    link = _Link()
    s = _stream(link)
    s.wake(1.005)
    link.alive = False
    n = len(link.sent)
    assert s.wake(1.025) and s.wake(1.3) and len(link.sent) == n
    assert s.wake(1.6) is False and "stalled" in s.ended


def test_stream_handover_on_the_planned_stop():
    link = _Link()
    calls = []
    s = _stream(link, lambda: calls.append("stance"))
    s.planner.stop()
    t, tick = 1.005, 50
    while s.handover_at is None and tick < 400:
        assert s.wake(t)
        tick += 1
        t += 0.02
        link.tel(tick, t - 0.004)
    assert calls == ["stance"]
    n = len(link.sent)
    link.event(EV.GAIT_STOPPED, 0)
    assert s.wake(t + 0.02) is False and s.ended == "stopped" and s.ok and len(link.sent) == n


# ---- SerialLink: FEET is a stream, not a goal ------------------------------------------------------

def test_send_stream_bypasses_the_goal_machinery(make_link):
    link, peer = make_link()
    payload = P.Feet(epoch=0, tick=5, z_mm=[76] * 4).pack()
    assert link.send_stream("FEET", payload) is False  # the body not heard yet: nothing goes out
    peer.hello(epoch=0)
    peer.telemetry(tick=1, epoch=0)
    assert link.wait_until(lambda: link._contact, 1)
    assert link.send_stream("FEET", payload) is True
    assert peer.wait_for(lambda: peer.of("FEET"), 1)
    assert "FEET" not in link.goals()
    time.sleep(0.5)
    assert len(peer.of("FEET")) == 1  # never refreshed, never retried
    assert link.send_stream("SET_POSTURE", payload) is False  # only Pi->body streams
    link.go_silent()
    assert link.send_stream("FEET", payload) is False


# ---- BodyClient.walk against the threaded virtual body -----------------------------------------------

@pytest.fixture
def rig():
    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="loopback")
    link.mark_alive()
    link.start()
    client = BodyClient(link)
    client.walker.stance_wait_s = 0.1
    yield client, vb
    link.stop()
    vb.stop()


def _keep(client, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        client.link.mark_alive()
        time.sleep(0.02)


def test_client_walks_and_stops(rig):
    client, vb = rig
    client.wait_ready(2)
    client.set_balance(True)
    st = client.walk("forward", "trot", balance=False)
    assert st["pattern"] == "trot" and st["active"] and st["speed_mm_s"] == pytest.approx(55.6, abs=0.1)
    assert vb.telem_div == 1  # 50 Hz telemetry while streaming
    assert vb.balance["enable"] == 0 or wait_for(lambda: vb.balance["enable"] == 0, 1)
    assert "SET_POSTURE" not in client.link.goals()  # a posture refresh would end GAIT
    assert wait_for(lambda: vb.feet is not None and client.state.mode == "GAIT", 2)
    _keep(client, 0.6)
    client.walk("back", "trot", 30)  # changes the walk in place
    assert client.walk_status()["direction"] == "back"
    _keep(client, 0.4)
    assert client.stop_walk(wait_s=3.0) is True
    st = client.walk_status()
    assert st["ended"] == "stopped" and st["ok"] and st["underruns"] == 0 and st["sent"] > 40
    assert vb.feet is None and vb.gait_stops == [0] and vb.posture == "STANCE"
    assert vb.telem_div == P.TELEMETRY_DEFAULT_DIV
    assert client.link.goals()["SET_BALANCE"].fields["enable"] == 1  # restored
    assert "SET_POSTURE" not in client.link.goals()
    assert client.stop_walk(wait_s=1.0) is True  # nothing walks: at once


def test_client_walk_refusals(rig):
    client, vb = rig
    client.wait_ready(2)
    with pytest.raises(ValueError):
        client.walk("sideways")
    with pytest.raises(ValueError):
        client.walk("forward", "gallop")
    vb.trigger_fall(1)
    assert client.link.wait_until(lambda: client.state.mode == "FALLEN", 1)
    with pytest.raises(WalkError, match="down"):
        client.walk()


def test_a_start_during_a_long_stop_aborts_the_old_stream(rig, monkeypatch):
    """A walk's planned stop can take ~10 s; a start that waited
    START_WAIT_S for it aborts the old stream first, so two streams never
    send FEET for the same ticks and the old one's teardown leaves the new
    walk alone."""
    client, vb = rig
    client.wait_ready(2)
    monkeypatch.setattr(walker_mod, "START_WAIT_S", 0.2)
    client.walk("forward", "walk")
    assert wait_for(lambda: vb.feet is not None, 2)
    _keep(client, 0.4)
    w = client.walker
    old_thread, old_stream = w._thread, w.stream
    client.stop_walk()
    st = client.walk("forward", "trot")
    assert not old_thread.is_alive() and old_stream.ended == "aborted"
    assert st["active"] and st["pattern"] == "trot" and w.stream is not old_stream
    _keep(client, 0.6)
    assert w.active and w.stream.ended is None and vb.telem_div == 1
    assert client.stop_walk(wait_s=3.0) is True and client.walk_status()["ok"]


def test_the_posture_balance_flag_is_carried_through_a_walk(rig):
    client, vb = rig
    client.wait_ready(2)
    client.posture("balance")  # STAND with the balance flag: the only balance source
    assert wait_for(lambda: vb.balance_req, 1) and vb.balance["enable"] == 0
    client.walk("march", "trot")
    assert wait_for(lambda: vb.feet is not None, 2)
    assert vb.balance_req  # the STANCE at the start carried it
    _keep(client, 0.3)
    assert client.stop_walk(wait_s=3.0) is True
    assert vb.balance_req and vb.posture == "STANCE"  # ...and the STANCE at the end
    # without the balance for this walk: off while walking, back after
    client.walk("march", "trot", balance=False)
    assert wait_for(lambda: vb.feet is not None, 2) and not vb.balance_req
    assert client.stop_walk(wait_s=3.0) is True and vb.balance_req


def test_a_walk_keeps_the_monitors_streams(rig):
    client, vb = rig
    client.wait_ready(2)
    client.set_stream(telem_div=2, joints_div=5, imu_raw_div=10)
    client.walk("march", "trot")
    assert wait_for(lambda: vb.feet is not None, 2)
    assert (vb.telem_div, vb.joints_div, vb.imu_raw_div) == (1, 5, 10)
    assert client.stop_walk(wait_s=3.0) is True
    assert (vb.telem_div, vb.joints_div, vb.imu_raw_div) == (2, 5, 10)


def test_a_failing_teardown_still_ends_the_wait(rig):
    client, vb = rig
    client.wait_ready(2)
    client.walk("march", "trot")
    w = client.walker

    def boom(s):
        raise RuntimeError("teardown")
    w._teardown = boom
    t0 = time.monotonic()
    w.stream._end("test")
    assert w.wait(3.0) is True and time.monotonic() - t0 < 1.0


@pytest.mark.slow
def test_a_fall_ends_the_walk(rig):
    client, vb = rig
    client.wait_ready(2)
    client.walk("march", "trot")
    assert wait_for(lambda: vb.feet is not None, 2)
    vb.trigger_fall(-1)
    assert client.walker.wait(2)
    assert "fell" in client.walk_status()["ended"] or "epoch" in client.walk_status()["ended"]
    time.sleep(0.3)
    assert vb.feet is None  # nothing streamed after it


@pytest.mark.slow
def test_a_body_reboot_ends_the_walk(rig):
    client, vb = rig
    client.wait_ready(2)
    client.walk("forward", "walk")
    assert wait_for(lambda: vb.feet is not None, 2)
    vb.reboot()
    assert client.walker.wait(2)
    assert client.walk_status()["ended"] is not None and not client.walk_status()["ok"]
    assert vb.feet is None


# ---- the daemon ---------------------------------------------------------------------------------------

@pytest.mark.slow
def test_daemon_walk_ops(start_daemon):  # noqa: F811
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        st = body.walk("back", "crawl", 8)
        assert st["pattern"] == "crawl" and st["direction"] == "back"
        assert wait_for(lambda: vb.feet is not None, 2)
        info = body.conn.call("daemon")
        assert "WALK" in info["held_goals"] and info["walk"]["active"]
        assert body.walk_status()["speed_mm_s"] == pytest.approx(8, abs=0.1)
        with pytest.raises(ValueError):
            body.walk("sideways")
        assert body.stop_walk(wait_s=8.0) is True
        assert vb.feet is None and vb.posture == "STANCE"
        # a walk held by a session that goes away is stopped the planned way, then the idle timeline runs
        body.walk("forward", "trot")
        assert wait_for(lambda: vb.feet is not None, 2)
    finally:
        body.close()
    assert wait_for(lambda: vb.feet is None and vb.gait_stops == [0, 0], 6)
    assert wait_for(lambda: vb.posture == "REST", 3)


def test_daemon_walk_under_the_behaviour_runner(start_daemon):  # noqa: F811
    from kilo.link.serial_link import LinkError

    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        d.behaviour_state = "running"  # pretend: the runner owns the body
        with pytest.raises(LinkError, match="behaviour runner active"):
            body.walk()
        assert body.stop_walk() is None  # always allowed
    finally:
        d.behaviour_state = "off"
        body.close()


# ---- kilo walk -------------------------------------------------------------------------------------------

def test_cli_parsing():
    a = parse_args(["walk"])
    assert (a.direction, a.pattern, a.speed, a.no_balance, a.hold) == ("forward", "trot", None, False, None)
    a = parse_args(["walk", "left", "--pattern", "crawl", "--speed", "9", "--no-balance", "--for", "3"])
    assert (a.direction, a.pattern, a.speed, a.no_balance, a.hold) == ("left", "crawl", 9, True, 3)
    assert parse_args(["walk", "stop"]).direction == "stop"
    for gone in ("--stride", "--turn", "--period", "--duty", "--no-sway", "--no-catch", "--place"):
        with pytest.raises(SystemExit):
            parse_args(["walk", gone, "1"] if gone in ("--stride", "--turn", "--period", "--duty") else ["walk", gone])
    with pytest.raises(SystemExit):
        parse_args(["walk", "--pattern", "gallop"])


def _kilo(path, *args, timeout=30):
    return subprocess.run([sys.executable, "-m", "kilo.cli", "--direct", "--port", path, *args], cwd=MIND,
                          capture_output=True, text=True, timeout=timeout)


@pytest.mark.slow
def test_kilo_walk_direct(vbody_pty):  # noqa: F811
    vb, path = vbody_pty
    r = _kilo(path, "walk", "back", "--pattern", "trot", "--speed", "40", "--for", "1.5")
    assert r.returncode == 0, r.stdout + r.stderr
    assert r.stdout.startswith("SAFETY: Kilo walks on the floor only")
    assert "walking back: trot 40.0 mm/s" in r.stdout and "stopped (" in r.stdout and "model error" in r.stdout
    assert vb.feet is None and vb.gait_stops == [0] and vb.posture == "STANCE"
    feet = [P.Feet.unpack(p) for n, _, p in list(vb.received) if n == "FEET"]
    assert len(feet) > 50 and [f.tick for f in feet] == sorted(f.tick for f in feet)
    r = _kilo(path, "walk", "stop")
    assert r.returncode == 0 and "SAFETY" not in r.stdout and ("not walking" in r.stdout or "stopped" in r.stdout)


@pytest.mark.slow
def test_kilo_walk_sigterm_stops(vbody_pty):  # noqa: F811
    vb, path = vbody_pty
    p = subprocess.Popen([sys.executable, "-m", "kilo.cli", "--direct", "--port", path, "walk", "march"], cwd=MIND,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert wait_for(lambda: vb.feet is not None, 10)
        p.send_signal(signal.SIGTERM)
        out, err = p.communicate(timeout=15)
    finally:
        if p.poll() is None:
            p.kill()
    assert p.returncode == 0, out + err
    assert "stopped" in out and vb.gait_stops == [0]


def test_kilo_walk_refused_by_a_body_without_feet():
    transport, path, slave = open_pty()
    vb = VirtualBody(transport, p2=False).start()
    try:
        r = _kilo(path, "walk", "--for", "3")
    finally:
        vb.stop()
        transport.close()
        os.close(slave)
    assert r.returncode == 1
    assert "body refused FEET: UNKNOWN_TYPE (this firmware does not play FEET" in r.stdout, r.stdout


def test_kilo_walk_refuses_an_older_body():
    transport, path, slave = open_pty()
    vb = VirtualBody(transport).start()
    import kilo.vbody.sim as sim
    try:
        orig = sim.P.PROTO_MAJOR
        sim.P.PROTO_MAJOR = 1  # the body of yesterday: protocol 1 (SET_GAIT)
        try:
            vb._send_hello()
            r = _kilo(path, "walk", "--for", "1")
        finally:
            sim.P.PROTO_MAJOR = orig
    finally:
        vb.stop()
        transport.close()
        os.close(slave)
    assert r.returncode != 0
    assert "needs 2" in r.stdout + r.stderr and "kilo flash" in r.stdout + r.stderr


# ---- renderings and the behaviour adapter -------------------------------------------------------------

def test_monitor_and_log_render_the_walk():
    snap = {"alive": True, "port": "p", "telemetry_age_s": 0.01, "reboots": 0, "tick": 5, "tick_load_ms": 3.1,
            "rx_err": 0, "last_rx_seq": 1, "mode": "GAIT", "owners": ("GAIT", "IDLE"), "epoch": 0,
            "flags": ["gait"], "balance": True, "pitch_deg": 1.0, "roll_deg": -6.0, "upright": True,
            "fallen": False, "moving": True, "rates_dps": (0, 0, 0), "dist_mm": None, "approach_mm_s": None,
            "batt_mv": 8000, "battery_pct": 80, "ref_deg": (0.5, -5.0), "err_deg": (0.5, -1.0),
            "corr_deg": (0.0, 0.25), "support": 0b0111, "feet_queued": 5,
            "events": [(0.5, {"code": EV.GOAL_REFUSED, "arg": M.FEET << 8 | R.STALE}),
                       (0.2, {"code": EV.FEET_UNDERRUN, "arg": 1})],
            "stats": {"frames_rx": 1, "frames_tx": 1, "heartbeats": 0, "decode_errors": {}, "retries": 0,
                      "timeouts": 0, "reconnects": 0}}
    text = render_monitor(snap)
    assert "walk  ref +0.5° -5.0°  error +0.5° -1.0°  corr +0.0° +0.2°  down LFRFRB  queued 5" in text
    assert "GOAL_REFUSED(FEET: STALE)" in text and "FEET_UNDERRUN(1)" in text
    assert _event_text(EV.GOAL_REFUSED, M.SET_BODY_POSE << 8 | R.STALE) == "GOAL_REFUSED(SET_BODY_POSE: STALE)"
    ss = [TelemetrySample(t=i * 0.02, tick=i, mode="GAIT", flags={"gait"}, pitch_deg=0.0, roll_deg=0.5,
                          ref_pitch_deg=0.0, ref_roll_deg=0.0, corr_pitch_deg=0.0, corr_roll_deg=0.0)
          for i in range(100)]
    ss += [TelemetrySample(t=2.0 + i * 0.02, tick=100 + i, mode="IDLE", flags=set()) for i in range(10)]
    events = [LogEvent(2.0, "GAIT_STOPPED", 1, 100, 0), LogEvent(1.9, "FEET_UNDERRUN", 1, 95, 0),
              LogEvent(0.0, "GOAL_REFUSED", M.FEET << 8 | R.STALE, 0, 0)]
    g = gait_summary(ss, events)
    assert (g["walking_s"], g["stops"], g["stopped_underrun"], g["underruns"], g["refused"]) == (
        2.0, 1, 1, 1, ["FEET: STALE"])
    assert g["error"]["offset_roll_deg"] == 0.5
    assert gait_summary(ss[100:], []) is None
    text = format_summary({"path": "x", "format": "kilolog", "duration_s": 2.2, "samples": 110, "gait": g,
                           "events": [(0.0, "GOAL_REFUSED", M.FEET << 8 | R.STALE)]})
    assert "gait: walking 2.0s, 1 GAIT_STOPPED (1 after an underrun), 1 FEET_UNDERRUN, refused FEET: STALE" in text
    assert "model error: offset pitch +0.0° roll +0.5°" in text


def test_adapter_maps_walk():
    calls = []
    client = SimpleNamespace(walk=lambda *a: calls.append(("walk", a)), stop_walk=lambda: calls.append(("stop",)))
    a = BodyAdapter(client)
    a.walk("left", "crawl", 9)
    a.stop_walk()
    assert calls == [("walk", ("left", "crawl", 9)), ("stop",)]
    assert not a.unsupported
    bare = BodyAdapter(SimpleNamespace())
    bare.walk()
    assert bare.unsupported["walk"] == 1
    assert consts.STREAMS_OUT == {"FEET"}
