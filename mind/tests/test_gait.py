"""Walking (P2, docs §3d): the gait and body-pose goals in BodyClient, the
virtual body's gait state machine, the daemon's walk/pose ops, `kilo walk`
and `kilo pose`, and the gait in the monitor, timeline and log summary.

The real firmware's gait is not reachable here (a separate agent writes it);
the virtual body is the test target."""

import os
import signal
import subprocess
import sys
import time
import tomllib
from pathlib import Path
from types import SimpleNamespace

import pytest

from kilo.behaviour.adapter import BodyAdapter
from kilo.body.client import (GAIT_DEFAULTS, GAIT_DUTY_MIN, GAIT_STRIDE_MIN_MM, BodyClient, gait_default_speed,
                              gait_speed, gait_speed_range, gait_swing_ms, gait_timing, walk_args)
from kilo.cli import _event_text, parse_args, render_monitor
from kilo.link import consts
from kilo.link.serial_link import Goal, LinkConfig, SerialLink, _is_motion
from kilo.link.transport import loopback_pair
from kilo.logs.analysis import format_summary, gait_summary
from kilo.logs.reader import LogEvent, TelemetrySample
from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody, open_pty

from conftest import FAST
from test_daemon import remote, sockdir, start_daemon, vbody_pty, wait_for  # noqa: F401  (fixtures)

MIND = Path(__file__).resolve().parents[1]
SPEC = MIND.parent / "protocol" / "spec.toml"
EV = P.EventCode
R = P.GoalRefusedReason
M = P.MsgId


# ---- ranges and wire fields --------------------------------------------------------------

def test_field_ranges_come_from_the_spec():
    spec = tomllib.loads(SPEC.read_text())
    for m in spec["messages"]:
        if m["dir"] != "pi_to_body" or not all(f["type"] in ("u8", "i8", "u16", "i16", "u32") for f in m["fields"]):
            continue
        rng = consts.field_ranges(m["name"])
        for f in m["fields"]:
            if "range" in f:
                assert rng[f["name"]] == tuple(f["range"]), (m["name"], f["name"])
    r = consts.field_ranges("SET_GAIT")
    assert r["period_ms"] == (300, 2000) and r["stride_mm"] == (-60, 60) and r["turn_mm"] == (-40, 40)
    assert r["body_h_mm"] == (0, 255)  # no spec range: the u8's
    assert consts.field_ranges("SET_BODY_POSE")["pitch"] == (-32768, 32767)


def _client(leg=(50, 50)):
    c = BodyClient.__new__(BodyClient)
    c.leg_mm = leg
    return c


def test_gait_flags_for_older_body():
    """A body on protocol 1.4 does not know SWAY (8) or CATCH (16) and would refuse
    them (BAD_ARG): walk() drops them for it, keeps them for 1.5, keeps all for
    an unknown body."""
    from kilo.body.client import gait_flags_for
    assert gait_flags_for(31, 4) == 7
    assert gait_flags_for(31, 5) == 31
    assert gait_flags_for(31, None) == 31
    assert gait_flags_for(7, 0) == 7


def test_gait_fields_defaults_per_pattern():
    c = _client()
    assert c.gait_fields() == {"run": 1, "pattern": 1, "period_ms": 600, "duty": 192, "stride_mm": 25,
                               "turn_mm": 0, "step_h_mm": 8, "body_h_mm": 80, "flags": 23}
    w = c.gait_fields(pattern="walk")
    assert (w["pattern"], w["period_ms"], w["duty"], w["stride_mm"], w["step_h_mm"]) == (0, 2000, 192, 30, 15)
    assert w["flags"] == 31  # bob, wag, balance, sway (on by default for walk and crawl), catch
    assert c.gait_fields(pattern="walk", sway=False)["flags"] == 23
    assert c.gait_fields(pattern="trot", sway=True)["flags"] == 31
    assert c.gait_fields(pattern="trot", catch=False)["flags"] == 7  # catch: on for every pattern by default
    cr = c.gait_fields(pattern="crawl", bob=False, wag=False)
    assert (cr["pattern"], cr["period_ms"], cr["duty"], cr["body_h_mm"], cr["flags"]) == (2, 2000, 218, 70, 28)
    assert _client((60, 70)).gait_fields(pattern="crawl")["body_h_mm"] == 91  # 0.7 x 130
    assert c.gait_fields(stride_mm=-20, turn_mm=10, period_ms=600, run=False, balance=False)["flags"] == 19
    for bad in ({"stride_mm": 61}, {"turn_mm": -41}, {"period_ms": 299}, {"duty": 231}, {"step_h_mm": 41},
                {"body_h_mm": 256}, {"body_h_mm": 0}, {"pattern": "gallop"}, {"pattern": 3}):
        with pytest.raises(ValueError):
            c.gait_fields(**bad)
    assert BodyClient.pattern_name(2) == "crawl"


def test_walk_args():
    assert walk_args("forward") == {"pattern": "trot", "stride_mm": 25, "turn_mm": 0}
    assert walk_args("back", "walk") == {"pattern": "walk", "stride_mm": -30, "turn_mm": 0}
    assert walk_args("back", stride_mm=20, turn_mm=-5) == {"pattern": "trot", "stride_mm": -20, "turn_mm": -5}
    assert walk_args("left") == {"pattern": "trot", "stride_mm": 0, "turn_mm": 20}
    assert walk_args("right", turn_mm=30, stride_mm=10) == {"pattern": "trot", "stride_mm": 10, "turn_mm": -30}
    assert walk_args("march", "crawl", 40, 10) == {"pattern": "crawl", "stride_mm": 0, "turn_mm": 0}
    with pytest.raises(ValueError):
        walk_args("up")
    # with a speed: period and duty from gait_timing, the stride too unless given
    assert walk_args("forward", speed_mm_s=gait_default_speed("trot")) == {
        "pattern": "trot", "stride_mm": 25, "turn_mm": 0, "period_ms": 600, "duty": 192}
    assert walk_args("back", speed_mm_s=10) == {"pattern": "trot", "stride_mm": -13, "turn_mm": 0,
                                                "period_ms": 1477, "duty": 230}
    assert walk_args("back", stride_mm=20, speed_mm_s=10)["stride_mm"] == -20
    assert walk_args("march", speed_mm_s=80) == {"pattern": "trot", "stride_mm": 0, "turn_mm": 0,
                                                 "period_ms": 462, "duty": 173}
    assert walk_args("left", "walk", speed_mm_s=10) == {"pattern": "walk", "stride_mm": 0, "turn_mm": 20,
                                                        "period_ms": 2000, "duty": 192}


# ---- speed: a fixed swing, stride and stance from the speed (docs/walking-and-balance.md §3) ----

SPEEDS = [0, 0.5, 1, 2, 3.7] + [x / 4 for x in range(20, 800)] + [250, 1000, 1e6]


def test_gait_timing_defaults_reproduce_the_spec():
    assert gait_swing_ms("trot") == 150 and gait_swing_ms("walk") == 500
    assert gait_default_speed("trot") == pytest.approx(25 / 0.45)  # 56 mm/s
    for name, d in GAIT_DEFAULTS.items():
        want = {"period_ms": d["period_ms"], "duty": d["duty"], "stride_mm": d["stride_mm"]}
        assert gait_timing(name) == want
        assert gait_timing(name, gait_default_speed(name)) == want
        assert gait_timing(name.upper(), gait_default_speed(name)) == want
    assert gait_timing("trot", 56) == {"period_ms": 596, "duty": 192, "stride_mm": 25}


@pytest.mark.parametrize("name", list(GAIT_DEFAULTS))
def test_gait_timing_clamps(name):
    rng = consts.field_ranges("SET_GAIT")
    d = GAIT_DEFAULTS[name]
    swing = gait_swing_ms(name)
    lo, hi = gait_speed_range(name)
    prev = None
    for v in SPEEDS:
        t = gait_timing(name, v)
        p, du, s = t["period_ms"], t["duty"], t["stride_mm"]
        for k in t:  # inside the SET_GAIT ranges
            assert rng[k][0] <= t[k] <= rng[k][1], (v, t)
        assert GAIT_DUTY_MIN[name] <= du  # trot: four-foot phases; walk, crawl: three feet down
        assert GAIT_STRIDE_MIN_MM <= s <= d["stride_mm"]  # never a longer stride than the default
        assert p * (256 - du) / 256 == pytest.approx(swing, abs=4)  # the swing time is constant
        assert gait_speed(p, du, s) == pytest.approx(min(max(v, lo), hi), rel=0.05, abs=0.6)
        if prev is not None:  # monotonic: faster = a longer (or equal) stride, a shorter (or equal) stance
            assert s >= prev["stride_mm"] and p <= prev["period_ms"] and du <= prev["duty"], (v, prev, t)
            if s > prev["stride_mm"]:  # the stride grows only while the stance is at its longest
                assert p == prev["period_ms"]
        prev = t
    # slow: the stride shrinks at the longest stance, not a longer stance; fast: capped
    slow, slower = gait_timing(name, lo * 2), gait_timing(name, lo * 1.5)
    assert slower["stride_mm"] < slow["stride_mm"] and slower["period_ms"] == slow["period_ms"]
    assert gait_timing(name, 0) == gait_timing(name, lo / 2) and gait_timing(name, 0)["stride_mm"] == 8
    assert gait_timing(name, hi * 10) == gait_timing(name, hi)
    assert gait_timing(name, -20) == gait_timing(name, 20)  # a magnitude


def test_gait_timing_trot_numbers():
    lo, hi = gait_speed_range("trot")
    assert lo == pytest.approx(6.0, abs=0.1) and hi == pytest.approx(110, abs=1)  # 0.1x .. 2x the default
    assert gait_timing("trot", 1) == {"period_ms": 1477, "duty": 230, "stride_mm": 8}  # duty 230: 150/26*256
    assert gait_timing("trot", 500) == {"period_ms": 376, "duty": 154, "stride_mm": 25}  # duty 0.6
    assert gait_speed_range("walk")[1] == pytest.approx(20)  # walk: at its period and duty limits already
    with pytest.raises(ValueError):
        gait_timing("trot", 50, swing_ms=0)
    with pytest.raises(ValueError):
        gait_timing("walk", 50, swing_ms=1900)  # no stance fits
    assert gait_timing("trot", 50, swing_ms=200)["period_ms"] * (256 - gait_timing("trot", 50, swing_ms=200)[
        "duty"]) / 256 == pytest.approx(200, abs=4)


def test_a_gait_stop_is_not_held_back_before_contact():
    go = Goal("SET_GAIT", M.SET_GAIT, {"run": 1}, 0)
    stop = Goal("SET_GAIT", M.SET_GAIT, {"run": 0}, 0)
    pose = Goal("SET_BODY_POSE", M.SET_BODY_POSE, {}, 0)
    assert _is_motion(go) and not _is_motion(stop) and _is_motion(pose)


# ---- the virtual body's gait, deterministic ------------------------------------------------

class Sim:
    """A VirtualBody driven synchronously with a frozen clock; HEARTBEATs keep its watchdog fed."""

    def __init__(self, **kw):
        self.now = 100.0
        self.vb = VirtualBody(None, clock=lambda: self.now, **kw)
        self.vb.take_sent()
        self.seq = 0

    def send(self, msg):
        self.seq = (self.seq + 1) & 0xFF
        self.vb.process(C.encode_message(msg, self.seq), self.now)
        return self.out()

    def out(self):
        return [(t, P.decode_payload(t, p)) for t, _, p in self.vb.take_sent()]

    def tick(self, n):
        sent = []
        for i in range(n):
            self.now += 0.02
            self.vb._tick(self.now)
            if i % 10 == 9:
                self.vb.process(C.encode_message(P.Heartbeat(), 0), self.now)
            sent += self.out()
        return sent


def gait(epoch=0, run=1, pattern=P.GaitPattern.TROT, period_ms=500, duty=140, stride_mm=35, turn_mm=0,
         step_h_mm=12, body_h_mm=80, flags=7):
    return P.SetGait(epoch=epoch, run=run, pattern=pattern, period_ms=period_ms, duty=duty, stride_mm=stride_mm,
                     turn_mm=turn_mm, step_h_mm=step_h_mm, body_h_mm=body_h_mm, flags=flags)


def evs(out, code=None):
    return [(m.code, m.arg) for t, m in out if t == M.EVENT and (code is None or m.code == code)]


def telem(out):
    return [m for t, m in out if t == M.TELEMETRY]


def test_vbody_gait_start_phase_and_stop():
    s = Sim()
    assert s.send(gait()) == []  # goals are never acked
    vb = s.vb
    # from LIMP: STANCE first (POSTURE_TICKS), mode GAIT from the accepted goal
    assert vb.posture == "STANCE" and vb.mode == P.Mode.GAIT and vb.gait.state == "stance"
    s.tick(vb.params[P.ParamId.POSTURE_TICKS])
    assert vb.gait.state == "run" and vb.gait.phase == 0
    step = 65536 * 20 // 500
    out = s.tick(10)
    assert vb.gait.phase == 10 * step
    t = telem(out)[-1]
    assert t.mode == P.Mode.GAIT and t.flags & P.TelemetryFlags.GAIT and t.owners & 0xF == P.Owner.GAIT
    assert t.gait_phase == (step * (t.tick - vb.tick + 10)) >> 8  # phi >> 8 at that telemetry's tick
    s.tick(20)
    assert vb.gait.amp == 1.0  # ramped up over one period (25 ticks)
    # a refresh with other parameters updates them in place
    s.send(gait(stride_mm=20, period_ms=1000))
    assert vb.gait.msg.stride_mm == 20 and vb.gait.step == 65536 * 20 // 1000
    # run=0: one period of ramp-down (50 ticks at 1000 ms), then STANCE, GAIT_STOPPED once, IDLE
    assert s.send(gait(run=0, period_ms=1000)) == []
    assert vb.gait.state == "stop"
    out = s.tick(49)
    assert vb.gait is not None and evs(out, EV.GAIT_STOPPED) == []
    out = s.tick(1)
    assert vb.gait is None and evs(out, EV.GAIT_STOPPED) == [(EV.GAIT_STOPPED, 0)]
    assert vb.posture == "STANCE" and vb.mode == P.Mode.IDLE
    assert evs(s.tick(60), EV.GAIT_STOPPED) == [] and vb.gait_stops == 1
    assert evs(s.send(gait(run=0)), EV.GAIT_STOPPED) == []  # a stop with nothing walking: nothing


def test_vbody_gait_from_stance_starts_at_once_and_pattern_change():
    s = Sim()
    s.send(P.SetPosture(epoch=0, posture=P.Posture.STANCE))
    s.send(gait(pattern=P.GaitPattern.WALK, period_ms=800))
    assert s.vb.gait.state == "run"
    s.tick(5)
    s.send(gait(pattern=P.GaitPattern.CRAWL, period_ms=1200))  # a stop followed by a start
    assert s.vb.gait.state == "stop" and s.vb.gait.restart is not None
    out = s.tick(41)  # 800 ms ramp-down, then the first crawl tick
    assert s.vb.gait.pattern == P.GaitPattern.CRAWL and s.vb.gait.state == "run" and s.vb.gait.phase > 0
    assert evs(out, EV.GAIT_STOPPED) == []
    # a stop during the move to STANCE ends it at once
    s2 = Sim()
    s2.send(gait())
    assert evs(s2.send(gait(run=0)), EV.GAIT_STOPPED) == [(EV.GAIT_STOPPED, 0)] and s2.vb.gait is None
    # run=1 again while stopping resumes
    s.send(gait(run=0, pattern=P.GaitPattern.CRAWL, period_ms=1200))
    s.tick(3)
    s.send(gait(pattern=P.GaitPattern.CRAWL, period_ms=1200))
    assert s.vb.gait.state == "run"


def test_vbody_gait_refusals():
    s = Sim()
    for bad in (gait(period_ms=299), gait(duty=231), gait(stride_mm=61), gait(turn_mm=41), gait(step_h_mm=41),
                gait(pattern=3), gait(flags=32), gait(run=2)):
        s.now += 1.1  # one GOAL_REFUSED per type per second
        assert evs(s.send(bad)) == [(EV.GOAL_REFUSED, M.SET_GAIT << 8 | R.BAD_ARG)], bad
    s.now += 1.1
    assert evs(s.send(gait(epoch=5))) == [(EV.GOAL_REFUSED, M.SET_GAIT << 8 | R.STALE)]
    assert s.vb.gait is None
    s.now += 1.1
    assert evs(s.send(gait(epoch=5, run=0))) == []  # a stop is never stale
    s.vb.batt_level = P.BatteryLevel.LOW
    s.now += 1.1
    assert evs(s.send(gait())) == [(EV.GOAL_REFUSED, M.SET_GAIT << 8 | R.LOW_BATT)]
    s.vb.batt_level = 0
    u = Sim(config_state=P.ConfigState.DEFAULTED)
    assert evs(u.send(gait())) == [(EV.GOAL_REFUSED, M.SET_GAIT << 8 | R.UNCALIBRATED)]
    f = Sim()
    f.vb.trigger_fall(1)
    f.out()
    assert evs(f.send(gait(epoch=f.vb.epoch))) == [(EV.GOAL_REFUSED, M.SET_GAIT << 8 | R.FALLEN)]
    # the P1 body (cross-check against a firmware without the gait)
    p1 = Sim(p2=False)
    assert evs(p1.send(gait())) == [(EV.GOAL_REFUSED, M.SET_GAIT << 8 | R.UNKNOWN_TYPE)]
    assert evs(p1.send(P.SetPosture(epoch=0, posture=P.Posture.STANCE))) == [
        (EV.GOAL_REFUSED, M.SET_POSTURE << 8 | R.BAD_ARG)]


def test_vbody_gait_ends_on_epoch_bump_and_limp():
    for end in ("watchdog", "fall", "battery", "limp"):
        s = Sim()
        s.send(gait())
        s.tick(30)
        assert s.vb.gait is not None
        e0 = s.vb.epoch
        if end == "watchdog":
            out = []
            for _ in range(30):  # no heartbeat: stage 1 at 0.5 s (tick 25)
                s.now += 0.02
                s.vb._tick(s.now)
                out += s.out()
            # The watchdog's bump ramps the gait down (one period = 25 ticks)
            # instead of cutting it: still walking, no GAIT_STOPPED yet, and
            # stage 2's REST waits for it (the body's §3g behaviour).
            assert s.vb.gait is not None and s.vb.gait.state == "stop"
            assert not [m for t, m in out if t == M.EVENT and m.code == EV.GAIT_STOPPED]
            for _ in range(60):  # ramp done at tick ~50; REST at 1.5 s (tick 75) after it
                s.now += 0.02
                s.vb._tick(s.now)
                out += s.out()
                if s.vb.gait is not None:
                    assert s.vb.posture != "REST"
            assert s.vb.posture == "REST"
        elif end == "fall":
            s.vb.trigger_fall(-1)
            out = s.out()
        elif end == "battery":
            s.vb.set_battery(P.BatteryLevel.LOW)
            out = s.out()
        else:
            out = s.send(P.SetPosture(epoch=0, posture=P.Posture.LIMP))
        assert s.vb.gait is None, end
        stopped = [m for t, m in out if t == M.EVENT and m.code == EV.GAIT_STOPPED]
        assert len(stopped) == 1, end
        assert stopped[0].epoch == (e0 if end == "limp" else e0 + 1), end


def test_vbody_trick_preempts_the_legs_and_the_gait_resumes():
    from kilo.tricks import library

    s = Sim()
    s.send(P.SetPosture(epoch=0, posture=P.Posture.STANCE))
    s.send(gait())
    s.tick(5)
    t = library.load("hi")
    for m in t.upload_messages(7):
        s.send(m)
    s.send(P.TrickPlay(epoch=0, trick_id=7, speed_q6=64, flags=0))
    out = s.tick(10)
    tm = telem(out)[-1]
    assert tm.mode == P.Mode.TRICK and tm.owners & 0xF == P.Owner.TRICK
    assert tm.flags & P.TelemetryFlags.GAIT and tm.flags & P.TelemetryFlags.TRICK
    phase = s.vb.gait.phase
    out = s.tick(400)
    assert not s.vb.trick.running and s.vb.gait is not None and s.vb.gait.phase != phase
    tm = telem(out)[-1]
    assert tm.mode == P.Mode.GAIT and tm.owners & 0xF == P.Owner.GAIT


def test_vbody_body_pose_and_joints():
    s = Sim()
    assert evs(s.send(P.SetBodyPose(epoch=3, pitch=80, roll=0, dheight_mm=0, xshift_mm=0))) == [
        (EV.GOAL_REFUSED, M.SET_BODY_POSE << 8 | R.STALE)]
    assert s.send(P.SetBodyPose(epoch=0, pitch=160, roll=-80, dheight_mm=-5, xshift_mm=4)) == []
    assert s.vb.pose == (160, -80, -5, 4)
    s.send(P.SetStream(telem_div=2, joints_div=5, imu_raw_div=0))
    out = s.tick(5)
    j = [m for t, m in out if t == M.JOINTS]
    assert j and all(a == -32768 for a in j[-1].angle)  # LIMP
    s.send(gait())
    s.tick(25)
    seen = set()
    for _ in range(10):
        j = [m for t, m in s.tick(5) if t == M.JOINTS][-1]
        seen.add(j.angle[3])
        assert all(a != -32768 for a in j.angle)
    assert len(seen) > 3  # the LF shoulder swings with the phase
    s.vb.trigger_fall(1)
    assert s.vb.pose == (0, 0, 0, 0)


# ---- BodyClient against the virtual body ------------------------------------------------------

@pytest.fixture
def rig():
    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="loopback")
    link.mark_alive()
    link.start()
    client = BodyClient(link)
    yield client, vb
    link.stop()
    vb.stop()


def _received(vb, name):
    return [P.MESSAGES[M[name]].unpack(p) for n, _, p in list(vb.received) if n == name]


@pytest.mark.slow
def test_client_walk_stop_and_geometry(rig):
    client, vb = rig
    client.wait_ready(2)
    vb.params[P.ParamId.LEG_L1_MM] = 55
    vb.params[P.ParamId.LEG_L2_MM] = 65
    assert client.load_geometry() == (55, 65)
    client.posture("sit")
    f = client.walk(pattern="walk")
    assert f["body_h_mm"] == 96 and f["stride_mm"] == 30  # 0.8 x 120
    assert "SET_POSTURE" not in client.link.goals()  # the gait replaces the posture goal
    g = client.link.goals()["SET_GAIT"]
    assert g.epoch == client.state.epoch
    assert client.link.wait_until(lambda: client.state.mode == "GAIT", 1)
    n0 = len(_received(vb, "SET_GAIT"))
    client.spin(0.7)
    got = _received(vb, "SET_GAIT")
    assert len(got) - n0 >= 2  # refreshed at 5 Hz
    assert got[-1] == P.SetGait(epoch=g.epoch, **f)
    assert client.link.wait_until(lambda: client.state.snapshot()["gait_phase"] not in (None, 0), 1)
    st = client.gait_state()
    assert st["pattern"] == "walk" and st["run"] == 1 and st["walking"] and 0 <= st["phase"] < 1
    t0 = time.monotonic()
    assert client.stop_walk(wait_s=3.0) is True
    assert 1.5 < time.monotonic() - t0 < 3.0  # one 2000 ms period (the walk default) of ramp-down
    assert _received(vb, "SET_GAIT")[-1].run == 0 and _received(vb, "SET_GAIT")[-1].pattern == P.GaitPattern.WALK
    assert vb.posture == "STANCE" and vb.gait_stops == 1
    assert client.link.wait_until(lambda: "SET_GAIT" not in client.link.goals(), 1)  # GAIT_STOPPED cleared it
    assert client.gait_stopped_since(0) is not None
    # a stop while nothing walks returns once the body shows no gait
    assert client.stop_walk(wait_s=2.0) is True


def test_geometry_params_persist(rig):
    client, vb = rig
    client.wait_ready(2)
    for name in ("LEG_L1_MM", "LEG_L2_MM", "LEG_LB_MM", "LEG_W_MM", "LEG_SH_ZERO_DEG", "LEG_KN_ZERO_DEG"):
        info = client.param_info(name)
        client.param_set(name, info.min, persist=True)
        assert client.link.wait_until(lambda: not vb.save_busy(), 1)
        assert vb.params_saved[info.id] == info.min and client.param_get(name).value == info.min
        with pytest.raises(ValueError):
            client.param_set(name, info.max + 1)


def test_client_walk_goal_cleared_by_a_fall(rig):
    client, vb = rig
    client.wait_ready(2)
    client.walk()
    assert client.link.wait_until(lambda: vb.gait is not None, 1)
    vb.trigger_fall(1)
    assert client.link.wait_until(lambda: "SET_GAIT" not in client.link.goals(), 1)
    assert vb.gait is None and client.gait_stopped_since(0) is not None
    time.sleep(0.5)
    assert vb.gait is None  # nothing replayed


def test_client_walk_until_stopped_and_pose(rig):
    import asyncio

    client, vb = rig
    client.wait_ready(2)
    client.walk(stride_mm=10)
    assert client.link.wait_until(lambda: vb.gait is not None and vb.gait.state == "run", 2)
    n = client.stop_walk()
    assert asyncio.run(client.walk_until_stopped(n, timeout=3)) is True
    f = client.pose(pitch_deg=5, roll_deg=-2.5, dheight_mm=-3, xshift_mm=4)
    assert f == {"pitch": 80, "roll": -40, "dheight_mm": -3, "xshift_mm": 4}
    assert client.link.wait_until(lambda: vb.pose == (80, -40, -3, 4), 1)
    client.release_pose()
    assert client.link.wait_until(lambda: vb.pose == (0, 0, 0, 0), 1)
    with pytest.raises(ValueError):
        client.pose(pitch_deg=3000)


# ---- the daemon --------------------------------------------------------------------------------

@pytest.mark.slow
def test_daemon_walk_ops(start_daemon):  # noqa: F811
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        f = body.walk(stride_mm=-20, pattern="crawl")
        assert f["stride_mm"] == -20 and f["pattern"] == P.GaitPattern.CRAWL
        assert wait_for(lambda: vb.gait is not None and vb.gait.state == "run", 3)
        info = body.conn.call("daemon")
        assert "SET_GAIT" in info["held_goals"] and info["gait"]["pattern"] == "crawl"
        assert info["gait"]["stride_mm"] == -20 and info["gait"]["mode"] == "GAIT"
        assert body.gait_state()["run"] == 1
        with pytest.raises(ValueError):
            body.walk(stride_mm=99)
        assert vb.gait.msg.stride_mm == -20
        assert body.stop_walk(wait_s=4.0) is True
        assert vb.gait is None and vb.posture == "STANCE"
        body.pose(pitch_deg=3)
        assert wait_for(lambda: vb.pose[0] == 48, 1)
        body.release_pose()
        assert wait_for(lambda: vb.pose[0] == 0, 1)
        # a walk held by a session that goes away is stopped, then the idle timeline runs
        body.walk()
        assert wait_for(lambda: vb.gait is not None, 2)
    finally:
        body.close()
    assert wait_for(lambda: vb.gait is None and vb.gait_stops == 2, 4)
    assert wait_for(lambda: vb.posture == "REST", 2)


def test_daemon_walk_under_the_behaviour_runner(start_daemon):  # noqa: F811
    from kilo.link.serial_link import LinkError

    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        d.behaviour_state = "running"  # pretend: the runner owns the body
        with pytest.raises(LinkError, match="behaviour runner active"):
            body.walk()
        with pytest.raises(LinkError, match="behaviour runner active"):
            body.pose(pitch_deg=2)
        assert body.stop_walk() is None  # always allowed
    finally:
        d.behaviour_state = "off"
        body.close()


# ---- the CLI ------------------------------------------------------------------------------------

def test_cli_parsing():
    a = parse_args(["walk"])
    assert (a.direction, a.pattern, a.stride, a.turn, a.hold) == ("forward", "trot", None, None, None)
    a = parse_args(["walk", "left", "--pattern", "crawl", "--turn", "15", "--period", "900", "--duty", "200",
                    "--step-h", "10", "--body-h", "70", "--no-bob", "--no-wag", "--no-balance", "--for", "3"])
    assert (a.direction, a.pattern, a.turn, a.period, a.duty, a.step_h, a.body_h, a.no_bob, a.no_wag,
            a.no_balance, a.hold) == ("left", "crawl", 15, 900, 200, 10, 70, True, True, True, 3)
    assert parse_args(["walk", "stop"]).direction == "stop"
    assert parse_args(["walk", "--speed", "80"]).speed == 80
    with pytest.raises(SystemExit):
        parse_args(["walk", "--pattern", "gallop"])
    a = parse_args(["pose", "--pitch", "5", "--roll", "-2", "--height", "-4", "--xshift", "3", "--for", "1"])
    assert (a.action, a.pitch, a.roll, a.height, a.xshift, a.hold) == ("set", 5, -2, -4, 3, 1)
    assert parse_args(["pose", "release"]).action == "release"
    assert parse_args(["posture", "stance"]).name == "stance"


def test_walk_fields_speed_and_overrides():
    from kilo.cli_walk import walk_fields

    def fields(*argv):
        kw, gait = walk_fields(parse_args(["walk", *argv]))
        return {**kw, "period_ms": gait["period_ms"], "duty": gait["duty"]}

    assert fields() == {"pattern": "trot", "stride_mm": 25, "turn_mm": 0, "period_ms": 600, "duty": 192}
    assert fields("--pattern", "crawl") == {"pattern": "crawl", "stride_mm": 25, "turn_mm": 0,
                                            "period_ms": 2000, "duty": 218}
    assert fields("--speed", "80") == {"pattern": "trot", "stride_mm": 25, "turn_mm": 0, "period_ms": 462,
                                       "duty": 173}
    assert fields("--speed", "10", "back") == {"pattern": "trot", "stride_mm": -13, "turn_mm": 0,
                                               "period_ms": 1477, "duty": 230}
    # explicit fields win, one by one
    assert fields("--speed", "80", "--stride", "20") == {"pattern": "trot", "stride_mm": 20, "turn_mm": 0,
                                                         "period_ms": 462, "duty": 173}
    assert fields("--speed", "80", "--period", "900")["period_ms"] == 900
    assert fields("--speed", "80", "--duty", "200")["duty"] == 200
    assert fields("--period", "1000", "--duty", "179") == {"pattern": "trot", "stride_mm": 25, "turn_mm": 0,
                                                           "period_ms": 1000, "duty": 179}
    # the early-touchdown reflex: on unless --no-catch
    assert walk_fields(parse_args(["walk"]))[1]["catch"] is True
    assert walk_fields(parse_args(["walk", "--no-catch"]))[1]["catch"] is False


def _kilo(path, *args, timeout=20):
    return subprocess.run([sys.executable, "-m", "kilo.cli", "--direct", "--port", path, *args], cwd=MIND,
                          capture_output=True, text=True, timeout=timeout)


@pytest.mark.slow
def test_kilo_walk_direct(vbody_pty):  # noqa: F811
    vb, path = vbody_pty
    r = _kilo(path, "walk", "back", "--pattern", "walk", "--stride", "25", "--for", "1.5")
    assert r.returncode == 0, r.stdout + r.stderr
    assert r.stdout.startswith("SAFETY: Kilo walks on the floor only")
    assert "walking back: walk stride -25 mm, turn 0 mm, period 2000 ms" in r.stdout and "stopped" in r.stdout
    got = [P.SetGait.unpack(p) for n, _, p in list(vb.received) if n == "SET_GAIT"]
    assert got[0].run == 1 and got[0].stride_mm == -25 and got[-1].run == 0
    assert vb.gait is None and vb.gait_stops == 1
    r = _kilo(path, "walk", "stop")
    assert r.returncode == 0 and "stopped" in r.stdout and "SAFETY" not in r.stdout


@pytest.mark.slow
def test_kilo_walk_sigterm_stops(vbody_pty):  # noqa: F811
    vb, path = vbody_pty
    p = subprocess.Popen([sys.executable, "-m", "kilo.cli", "--direct", "--port", path, "walk", "march"], cwd=MIND,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert wait_for(lambda: vb.gait is not None and vb.gait.state == "run", 10)
        p.send_signal(signal.SIGTERM)
        out, err = p.communicate(timeout=10)
    finally:
        if p.poll() is None:
            p.kill()
    assert p.returncode == 0, out + err
    assert "stopped" in out and vb.gait_stops == 1


def test_kilo_walk_refused_by_a_p1_body():
    transport, path, slave = open_pty()
    vb = VirtualBody(transport, p2=False).start()
    try:
        r = _kilo(path, "walk", "--for", "3")
    finally:
        vb.stop()
        transport.close()
        os.close(slave)
    assert r.returncode == 1
    assert "body refused SET_GAIT: UNKNOWN_TYPE (this firmware has no gait generator" in r.stdout, r.stdout


def test_kilo_walk_bad_arg_sends_nothing(vbody_pty):  # noqa: F811
    vb, path = vbody_pty
    r = _kilo(path, "walk", "--stride", "70")
    assert r.returncode == 1 and "stride_mm must be -60..60" in r.stderr
    assert not [n for n, _, _ in list(vb.received) if n == "SET_GAIT"]


@pytest.mark.slow
def test_kilo_pose_direct(vbody_pty):  # noqa: F811
    vb, path = vbody_pty
    p = subprocess.Popen([sys.executable, "-m", "kilo.cli", "--direct", "--port", path, "pose", "--pitch", "4",
                          "--height", "-5", "--posture", "stance", "--for", "1"], cwd=MIND,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    seen = None
    deadline = time.monotonic() + 15
    while p.poll() is None and time.monotonic() < deadline:
        if vb.pose != (0, 0, 0, 0):
            seen = vb.pose
        time.sleep(0.02)
    out, err = p.communicate(timeout=5)
    assert p.returncode == 0, err
    assert "pose pitch 4°, roll 0°, height -5 mm, x-shift +0 mm for 1 s" in out and "released" in out
    assert seen == (64, 0, -5, 0) and vb.posture == "STANCE" and vb.pose == (0, 0, 0, 0)


# ---- renderings and the behaviour adapter ----------------------------------------------------------

def test_monitor_timeline_and_log_render_the_gait():
    snap = {"alive": True, "port": "p", "telemetry_age_s": 0.01, "reboots": 0, "tick": 5, "tick_load_ms": 3.1,
            "rx_err": 0, "last_rx_seq": 1, "mode": "GAIT", "owners": ("GAIT", "GAIT"), "epoch": 0,
            "flags": ["gait"], "balance": False, "pitch_deg": 0.0, "roll_deg": 0.0, "upright": True,
            "fallen": False, "moving": True, "rates_dps": (0, 0, 0), "dist_mm": None, "approach_mm_s": None,
            "batt_mv": 8000, "battery_pct": 80, "gait_phase": 128,
            "events": [(0.5, {"code": EV.GOAL_REFUSED, "arg": M.SET_GAIT << 8 | R.UNKNOWN_TYPE}),
                       (0.2, {"code": EV.GAIT_STOPPED, "arg": 0})],
            "stats": {"frames_rx": 1, "frames_tx": 1, "heartbeats": 0, "decode_errors": {}, "retries": 0,
                      "timeouts": 0, "reconnects": 0}}
    text = render_monitor(snap)
    assert "gait  phase 0.50" in text
    assert "GOAL_REFUSED(SET_GAIT: UNKNOWN_TYPE)" in text and "GAIT_STOPPED(0)" in text
    assert _event_text(EV.GOAL_REFUSED, M.SET_BODY_POSE << 8 | R.STALE) == "GOAL_REFUSED(SET_BODY_POSE: STALE)"
    ss = [TelemetrySample(t=i * 0.04, mode="GAIT", flags={"gait"}, gait_phase=(i * 10) % 256) for i in range(100)]
    ss += [TelemetrySample(t=4.0 + i * 0.04, mode="IDLE", flags=set(), gait_phase=0) for i in range(10)]
    events = [LogEvent(4.0, "GAIT_STOPPED", 0, 1, 0), LogEvent(0.0, "GOAL_REFUSED", M.SET_GAIT << 8 | R.STALE, 0, 0)]
    g = gait_summary(ss, events)
    assert g == {"walking_s": 4.0, "cycles": 3, "stops": 1, "refused": ["SET_GAIT: STALE"]}
    assert gait_summary(ss[100:], []) is None
    text = format_summary({"path": "x", "format": "kilolog", "duration_s": 4.4, "samples": 110, "gait": g,
                           "events": [(0.0, "GOAL_REFUSED", M.SET_GAIT << 8 | R.STALE)]})
    assert "gait: walking 4.0s, 3 cycles, 1 GAIT_STOPPED, refused SET_GAIT: STALE" in text
    assert "GOAL_REFUSED(SET_GAIT: STALE)" in text


def test_timeline_summary_shows_the_phase(rig):
    import io

    from kilo.body.timeline import Timeline

    client, vb = rig
    client.wait_ready(2)
    client.walk()
    assert client.link.wait_until(lambda: client.state.mode == "GAIT", 1)
    out = io.StringIO()
    tl = Timeline(client.link, client.link.clock(), out=out)
    client.spin(0.6)
    tl.poll()
    assert " mode GAIT phase " in out.getvalue()


def test_adapter_maps_walk():
    calls = []
    client = SimpleNamespace(walk=lambda **kw: calls.append(("walk", kw)), stop_walk=lambda: calls.append(("stop",)))
    a = BodyAdapter(client)
    a.walk(stride_mm=20, turn_mm=5)
    a.stop_walk()
    assert calls == [("walk", {"stride_mm": 20, "turn_mm": 5, "pattern": "trot"}), ("stop",)]
    assert not a.unsupported
    bare = BodyAdapter(SimpleNamespace())
    bare.walk()
    assert bare.unsupported["walk"] == 1
    assert set(GAIT_DEFAULTS) == set(consts.GAIT_PATTERNS)
