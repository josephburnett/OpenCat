"""Perception detectors on synthetic traces (docs/architecture.md §5 "Perception").

Noise model of a quiet robot: attitude ±0.3° (uniform) and gyro ±2 °/s at
25 Hz, ultrasonic ±5 mm at 12.5 Hz. Attitude convention: nose down = +pitch,
right side up = +roll.
"""

from __future__ import annotations

import math
import random

import pytest

from kilo.perception import (ImuConfig, ImuDetector, Percepts, TelemetryRing, UltrasonicConfig,
                             UltrasonicDetector, extract)
from kilo.perception import ring as ring_mod

US_DT = 0.08  # 12.5 Hz
IMU_DT = 0.04  # 25 Hz


def names(events):
    return [e.name for e in events]


# ---- ultrasonic ------------------------------------------------------------------

def run_us(dists, det=None, dt=US_DT, t0=0.0):
    det = det or UltrasonicDetector()
    out = []
    for i, d in enumerate(dists):
        out += det.feed(t0 + i * dt, d)
    return det, out


def noisy(v, rng, amp=5.0):
    return v + rng.uniform(-amp, amp)


def test_quiet_far_and_mid_no_events():
    rng = random.Random(1)
    for base in (800.0, 300.0, 1500.0):
        det, ev = run_us([noisy(base, rng) for _ in range(400)])
        assert ev == [], (base, ev)
        assert not det.approaching and not det.near
        assert abs(det.approach_mm_s) < 60


def test_approach_then_near_then_clear():
    rng = random.Random(2)
    seq = [noisy(1200, rng) for _ in range(30)]
    d = 1200.0
    while d > 100:  # 400 mm/s toward the robot
        d -= 400 * US_DT
        seq.append(noisy(d, rng))
    seq += [noisy(100, rng) for _ in range(20)]
    seq += [noisy(800, rng) for _ in range(20)]  # hand withdrawn (a jump)
    det, ev = run_us(seq)
    assert names(ev) == ["approaching", "near", "clear"]
    appr = ev[0]
    assert appr.t < 30 * US_DT + 1.0  # within a second of the start of the approach
    assert 250 < appr.data["speed_mm_s"] < 550
    assert det.clear and not det.near


def test_slow_drift_is_not_approaching():
    rng = random.Random(3)
    seq = [noisy(1000 - 50 * i * US_DT, rng) for i in range(150)]  # 50 mm/s
    _, ev = run_us(seq)
    assert "approaching" not in names(ev)


def test_walk_by_step_is_not_an_approach():
    rng = random.Random(4)
    seq = [noisy(2000, rng) for _ in range(30)] + [noisy(500, rng) for _ in range(8)] \
        + [noisy(2000, rng) for _ in range(30)]
    det, ev = run_us(seq)
    assert "approaching" not in names(ev)
    assert "near" not in names(ev)


def test_single_spike_and_dropouts_ignored():
    rng = random.Random(5)
    seq = []
    for i in range(200):
        if i % 17 == 0:
            seq.append(None)  # dropout
        elif i % 23 == 0:
            seq.append(60.0)  # spurious short echo
        else:
            seq.append(noisy(700, rng))
    _, ev = run_us(seq)
    assert ev == []


def test_dropouts_while_near_hold_near_until_sustained():
    seq = [100.0] * 10 + [None, 100.0, None, None, 100.0] + [None] * 20
    det, ev = run_us(seq)
    assert names(ev) == ["near", "clear"]
    assert ev[1].data["reason"] == "no_echo"
    # the clear came only after >= 1 s without any echo
    assert ev[1].t - (14 * US_DT) >= 1.0 - 1e-9


def test_near_hysteresis_no_chatter():
    rng = random.Random(6)
    seq = [noisy(120, rng) for _ in range(10)] + [noisy(175, rng) for _ in range(100)]
    _, ev = run_us(seq)
    assert names(ev) == ["near"]


def test_configurable_thresholds():
    cfg = UltrasonicConfig(near_mm=300, near_exit_mm=350)
    _, ev = run_us([1000.0] * 5 + [250.0] * 5, UltrasonicDetector(cfg))
    assert names(ev) == ["near"]


def test_approach_cooldown():
    seq = []
    for _ in range(3):
        seq += [1500.0] * 10 + [1500 - 300 * i * US_DT for i in range(1, 20)]
    _, ev = run_us(seq)
    appr = [e for e in ev if e.name == "approaching"]
    assert len(appr) >= 2
    assert all(b.t - a.t >= 2.0 for a, b in zip(appr, appr[1:]))


# ---- IMU ----------------------------------------------------------------------------

class Trace:
    """25 Hz attitude trace builder with realistic noise."""

    def __init__(self, seed=0, pitch=0.9, roll=-2.7, noise=0.3, gyro_noise=2.0):
        self.rng = random.Random(seed)
        self.t = 0.0
        self.p0, self.r0 = pitch, roll
        self.noise, self.gyro_noise = noise, gyro_noise
        self.rows = []

    def add(self, seconds, fp=None, fr=None, flags=("servos_powered",), mode="IDLE"):
        n = int(round(seconds / IMU_DT))
        for i in range(n):
            u = i * IMU_DT
            p = self.p0 + (fp(u) if fp else 0.0) + self.rng.uniform(-self.noise, self.noise)
            r = self.r0 + (fr(u) if fr else 0.0) + self.rng.uniform(-self.noise, self.noise)
            g = tuple(self.rng.uniform(-self.gyro_noise, self.gyro_noise) for _ in range(3))
            self.rows.append((self.t, p, r, g, set(flags), mode))
            self.t += IMU_DT
        return self

    def run(self, det=None):
        det = det or ImuDetector()
        out = []
        for row in self.rows:
            out += det.feed(*row)
        return det, out


def ramp(target, rise):
    return lambda u: target * min(1.0, u / rise)


def test_imu_quiet_trace_no_events():
    for seed in range(5):
        det, ev = Trace(seed).add(60).run()
        assert ev == [], ev
        assert not det.tilted and not det.picked_up and not det.pet


def test_tilt_nose_up_and_left_up():
    # the signcheck5 shape: nose up to -74, back, left side up to -88, back
    tr = Trace(1).add(5)
    tr.add(1.5, fp=ramp(-75, 1.0)).add(3, fp=lambda u: -75).add(1.0, fp=lambda u: -75 + 75 * u).add(4)
    tr.add(1.5, fr=ramp(-85, 1.0)).add(3, fr=lambda u: -85).add(1.0, fr=lambda u: -85 + 85 * u).add(4)
    det, ev = tr.run()
    tilts = [e for e in ev if e.name in ("tilted", "level")]
    assert [(e.name, e.data.get("direction")) for e in tilts] == [
        ("tilted", "nose_up"), ("level", None), ("tilted", "left_up"), ("level", None)]
    assert names(ev).count("picked_up") == 2 and names(ev).count("put_down") == 2
    assert not det.tilted and not det.picked_up
    # the reference was learned from the quiet start (baseline 0.9 / -2.7)
    assert det.ref[0] == pytest.approx(0.9, abs=0.3) and det.ref[1] == pytest.approx(-2.7, abs=0.3)


def test_tilt_vs_posture_reference():
    det = ImuDetector()
    det.set_reference(-20.0, 0.0)  # e.g. a sitting posture, nose up
    tr = Trace(2, pitch=-20.0, roll=0.0).add(10)
    _, ev = tr.run(det)
    assert ev == []
    tr2 = Trace(2, pitch=-20.0, roll=0.0).add(10)
    _, ev = tr2.run(ImuDetector())  # learned reference: no tilt either
    assert "tilted" not in names(ev)


def test_brief_tilt_is_debounced():
    det, ev = Trace(3).add(3).add(0.3, fp=lambda u: 30).add(3).run(ImuDetector(ImuConfig(pickup_deg=90)))
    assert "tilted" not in names(ev)


def tap_shape(u, start=0.0, peak=3.0):
    x = u - start
    if 0 <= x < 0.04:
        return peak * 0.6
    if 0.04 <= x < 0.08:
        return peak
    if 0.08 <= x < 0.12:
        return peak * 0.4
    return 0.0


def test_tap():
    tr = Trace(4).add(3).add(0.2, fp=lambda u: tap_shape(u)).add(2).add(0.2, fr=lambda u: tap_shape(u, peak=-2.5)).add(2)
    det, ev = tr.run()
    assert names(ev) == ["tap", "tap"]
    assert 2.0 <= ev[0].data["peak_deg"] <= 3.6


def test_pet():
    # stroking the back: ±1.5° pitch rocking at 1 Hz for 6 s
    tr = Trace(5).add(3).add(6, fp=lambda u: 1.5 * math.sin(2 * math.pi * 1.0 * u)).add(3)
    det, ev = tr.run()
    assert names(ev).count("pet") >= 1
    assert "tap" not in names(ev) and "picked_up" not in names(ev) and "startle" not in names(ev)
    assert names(ev).count("pet") <= 2  # cooldown


def test_pickup_with_servos_unpowered():
    # limp robot lifted and handled: wandering attitude, then put back down
    def wander(u):
        return 8 * math.sin(2 * math.pi * 0.4 * u) + 4 * math.sin(2 * math.pi * 1.1 * u)
    tr = Trace(6).add(3, flags=()).add(4, fp=wander, fr=lambda u: 0.5 * wander(u + 1), flags=()).add(4, flags=())
    det, ev = tr.run()
    assert names(ev)[0] == "picked_up"
    assert ev[0].data["servos_powered"] is False
    assert "put_down" in names(ev)
    assert not det.picked_up


def test_startle():
    tr = Trace(7).add(3).add(0.5, fr=ramp(20, 0.12)).add(3, fr=lambda u: 20 - 20 * min(1, u / 2)).add(2)
    det, ev = tr.run(ImuDetector(ImuConfig(pickup_deg=90)))
    assert names(ev).count("startle") == 1
    assert "tap" not in names(ev)


def test_startle_by_gyro_rate():
    det = ImuDetector()
    ev = []
    for i in range(80):
        ev += det.feed(i * IMU_DT, 0.0, 0.0, (0.0, 0.0, 300.0 if i == 60 else 0.0), {"servos_powered"}, "IDLE")
    assert names(ev) == ["startle"]


def test_self_motion_suppresses_impulses():
    tr = Trace(8).add(3).add(3, fp=lambda u: 1.5 * math.sin(2 * math.pi * u) + tap_shape(u, 1.0, 4),
                             flags=("servos_powered", "gait"), mode="GAIT").add(0.5)
    _, ev = tr.run()
    assert [n for n in names(ev) if n in ("tap", "pet", "startle", "picked_up")] == []


def test_low_rate_trace_tilt_only():
    det = ImuDetector()
    ev = []
    for i in range(40):  # 1 Hz, signcheck5-like
        p = -74 if 14 <= i <= 19 else 0.9
        r = -88 if 24 <= i <= 28 else -2.7
        ev += det.feed(float(i), p, r, None, None, None)
    assert [(e.name, e.data.get("direction")) for e in ev] == [
        ("tilted", "nose_up"), ("level", None), ("tilted", "left_up"), ("level", None)]


# ---- ring buffer, extract, Percepts ---------------------------------------------------

@pytest.mark.parametrize("use_numpy", [False, True])
def test_ring_buffer(use_numpy):
    if use_numpy and not ring_mod.HAVE_NUMPY:
        pytest.skip("numpy not installed")
    rb = TelemetryRing(seconds=1.0, rate_hz=10, use_numpy=use_numpy)
    for i in range(40):
        rb.append(t=i * 0.1, pitch=i, dist_mm=None)
    assert len(rb) == rb.capacity == 15
    w = rb.window(0.5)
    assert list(w["pitch"]) == [34.0, 35.0, 36.0, 37.0, 38.0, 39.0]
    assert math.isnan(rb.last()["dist_mm"])
    assert list(rb.column("pitch"))[0] == 25.0


def test_extract_shapes():
    from kilo.protocol.kilo_protocol import Telemetry

    tel = Telemetry(tick=5, pitch=16 * 3, roll=-16 * 2, rate_pitch=32, dist_mm=0, mode=2,
                    flags=(1 << 2) | (1 << 11))
    s = extract(tel, 1.0)
    assert (s.pitch, s.roll, s.rates[0], s.dist_mm, s.mode) == (3.0, -2.0, 2.0, None, "IDLE")
    assert s.flags == {"servos_powered", "dist_fresh"} and s.dist_fresh is True
    snap = {"pitch_deg": 1.0, "roll_deg": 2.0, "rates_dps": (0, 0, 0), "dist_mm": 300.0,
            "flags": ["servos_powered"], "mode": "IDLE"}
    s = extract(snap, 2.0)
    assert (s.t, s.dist_mm, s.dist_fresh) == (2.0, 300.0, False)
    assert extract({"pitch_deg": None, "roll_deg": None}) is None


def test_percepts_from_body_state():
    from kilo.body.state import BodyState
    from kilo.protocol.kilo_protocol import Telemetry

    st = BodyState()
    pc = Percepts()
    events = []
    d = 1000
    for i in range(200):
        fresh = i % 2 == 0
        if fresh and i > 50:
            d = max(80, d - 30)  # ≈375 mm/s at 12.5 Hz
        tel = Telemetry(tick=i * 2, pitch=14, roll=-43, dist_mm=d, batt_mv=8000, mode=2,
                        flags=(1 << 2) | ((1 << 11) if fresh else 0))
        with st.lock:
            st.on_telemetry(tel, i * IMU_DT)
        out = pc.update(st)
        events += out["events"]
        # polling the same telemetry twice adds nothing
        assert pc.update(st)["events"] == []
    assert names(events) == ["approaching", "near"]
    out = pc.as_dict()
    assert out["near"] and out["distance_mm"] == 80
    assert set(out) >= {"near", "approaching", "clear", "distance_mm", "approach_mm_s", "tilted",
                        "picked_up", "tap", "pet", "startle", "last_event_times"}
    assert len(pc.ring) == 200 and pc.ring.window(1.0)["dist_mm"][-1] == 80


def test_percepts_snapshot_dicts_and_event_hold():
    pc = Percepts(event_hold_s=1.0)
    for i in range(80):
        out = pc.update({"pitch_deg": 0.0, "roll_deg": 0.0, "rates_dps": (0, 0, 300.0 if i == 60 else 0),
                         "flags": ["servos_powered"], "mode": "IDLE", "dist_mm": 900}, t=i * IMU_DT)
        if i == 60:
            assert out["startle"] and names(out["events"]) == ["startle"]
    assert pc.as_dict()["startle"]  # 0.76 s later, still within the hold
    for i in range(80, 120):
        out = pc.update({"pitch_deg": 0.0, "roll_deg": 0.0, "flags": [], "mode": "IDLE"}, t=i * IMU_DT)
    assert not out["startle"]
    assert out["last_event_times"]["startle"] == pytest.approx(60 * IMU_DT)
