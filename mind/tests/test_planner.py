"""The planner (kilo.planner): the body model's geometry and sign
conventions, the patterns' speed mapping, the planned steps (margins,
slew, lead, prediction, speed / direction / pattern changes, the stop, the
gate and the catch) and the error estimates. Pure computation: no body."""

import math
from dataclasses import replace

import pytest

from kilo.planner import errors
from kilo.planner import patterns as pat
from kilo.planner.model import (LB, LEGS, LF, RB, RF, BodyModel, Mass, fit_attitude, leg_lengths, rotation,
                                stability, world_points)
from kilo.planner.plan import Planner, PlannerConfig
from kilo.protocol import kilo_protocol as P

M = BodyModel()
H = M.stance_height(205)  # 76 mm: STANCE's body height


# ---- the body model ------------------------------------------------------------------------

def test_model_masses_and_reach():
    assert 900 <= M.total_g <= 1100  # Kilo weighs about 1 kg with the battery (estimates)
    cx, cy, cz = M.com([0] * 4, [H] * 4)
    assert abs(cy) < 1e-9 and 0 < cx < 6 and -10 < cz < 5
    assert H == pytest.approx(76.0, abs=0.1)
    assert M.tau_s(80) == pytest.approx(0.090, abs=0.001)  # the 90 ms tip time constant of the doc
    for leg in LEGS:
        for x, z in ((0, H), (20, 70), (-25, 60), (10, 85)):
            t1, t2 = M.ik(leg, x, z)
            assert M.fk(t1, t2) == pytest.approx((x, z), abs=1e-6)
            assert (t2 > 0) == (leg in (LF, RF))  # front knees point back, back knees forward
    assert not M.reachable(0, 95) and M.reachable(0, 90)


def test_leg_lengths_and_attitude_conventions():
    """Pitch + = nose down: the back legs are the longer; roll + = right side
    up: the right legs are the longer (TELEMETRY, the firmware's H_i)."""
    xs = [0.0] * 4
    zs = leg_lengths(M, xs, H, 5.0, 0.0)
    assert zs[RB] > zs[RF] and zs[LB] > zs[LF] and sum(zs) / 4 == pytest.approx(H)
    zs = leg_lengths(M, xs, H, 0.0, 5.0)
    assert zs[RF] > zs[LF] and zs[RB] > zs[LB]
    for p, r in ((3.0, -7.0), (-4.0, 9.0), (0.0, 12.0)):
        zs = leg_lengths(M, [5.0, -3.0, 2.0, -8.0], H, p, r)
        assert fit_attitude(M, [5.0, -3.0, 2.0, -8.0], zs, 0xF) == pytest.approx((p, r), abs=1e-4)
        three = 0xF & ~(1 << RB)
        assert fit_attitude(M, [5.0, -3.0, 2.0, -8.0], zs, three) == pytest.approx((p, r), abs=1e-4)
    # on two feet the plane is free about their line: the previous attitude fills in
    diag = (1 << LF) | (1 << RB)
    assert fit_attitude(M, xs, [H] * 4, diag, prev=(0.0, 0.0)) == pytest.approx((0.0, 0.0), abs=1e-6)
    # the world: a nose-down pitch lowers the front feet, a positive roll the left ones
    R = rotation(10.0, 0.0)
    assert R[2][0] < 0  # height of a point ahead (+x) goes down
    R = rotation(0.0, 10.0)
    assert R[2][1] < 0  # right side up is + roll: a point on the left (+y) goes down
    com, feet = world_points(M, xs, leg_lengths(M, xs, H, 0.0, 10.0), 0.0, 10.0)
    assert feet[RF][2] == pytest.approx(feet[LF][2], abs=1e-6)  # all four on the ground
    feet_y = sum(f[1] for f in feet) / 4
    assert com[1] - feet_y == pytest.approx((H + M.foot_r_mm + M.com(xs, [H] * 4)[2]) * math.sin(math.radians(10)),
                                            abs=0.5)  # right side up: the mass moves left of its feet, h sin(roll)


def test_stability_margin():
    zs = [H] * 4
    four = stability(M, [0.0] * 4, zs, 0xF, 0.0, 0.0)
    assert four == pytest.approx(32.5, abs=0.5)  # the half track
    # on three feet the mass sits on the diagonal's edge (the head makes it a little nose heavy)
    for lifted in LEGS:
        m = stability(M, [0.0] * 4, zs, 0xF & ~(1 << lifted), 0.0, 0.0)
        assert abs(m) < 3.0
    # a diagonal pair has no inside
    assert stability(M, [0.0] * 4, zs, (1 << LF) | (1 << RB), 0.0, 0.0) <= 0


# ---- patterns and speed --------------------------------------------------------------------------

def test_pattern_tables_and_default_timing():
    t = pat.timing("trot")
    assert (t.stride_mm, round(t.period_ms), round(t.shift_ms)) == (25, 600, 150)
    assert t.speed_mm_s == pytest.approx(55.6, abs=0.1)
    assert pat.PATTERNS["trot"].groups == ((LF, RB), (RF, LB))
    assert pat.PATTERNS["walk"].groups == ((LB,), (LF,), (RB,), (RF,)) == pat.PATTERNS["crawl"].groups
    assert [pat.PATTERNS[p].swing_ms for p in ("trot", "walk", "crawl")] == [150, 500, 297]
    w = pat.timing("walk")
    assert (w.stride_mm, round(w.period_ms), round(w.shift_ms)) == (30, 3200, 300)
    assert pat.PATTERNS["crawl"].body_h_256 == 179 and pat.PATTERNS["walk"].body_h_256 == 205


@pytest.mark.parametrize("name", pat.PATTERN_NAMES)
def test_timing_clamps(name):
    lo, hi = pat.speed_range(name)
    p = pat.PATTERNS[name]
    for v in (lo, hi, (lo + hi) / 2):
        t = pat.timing(name, v)
        assert t.speed_mm_s == pytest.approx(v, rel=1e-6)
        assert t.shift_ms >= pat.shift_min_ms(p, 150) - 1e-6 and t.stance_ms <= p.stance_max_ms + 1e-6
    assert pat.timing(name, hi * 3).speed_mm_s == pytest.approx(hi)  # capped
    slow = pat.timing(name, lo / 3)
    assert slow.stride_mm == pat.STRIDE_MIN_MM and slow.speed_mm_s == pytest.approx(lo)
    assert pat.timing(name, -20) == pat.timing(name, 20)  # a magnitude
    # the swing never changes with the speed: the shift does
    assert pat.timing(name, lo).period_ms - pat.timing(name, hi).period_ms > 0


def test_walk_and_crawl_shift_covers_the_lag():
    for name in ("walk", "crawl"):
        assert pat.shift_min_ms(pat.PATTERNS[name], 150) == 250
        assert pat.timing(name, 1e6).shift_ms == pytest.approx(250)
    assert pat.shift_min_ms(pat.PATTERNS["trot"], 150) == 60  # three ticks on four feet


def test_side_strides():
    p = pat.PATTERNS["trot"]
    assert pat.side_strides("forward", 25, p) == (25, 25)
    assert pat.side_strides("backward", 25, p) == (-25, -25)
    assert pat.side_strides("left", 25, p) == (-20, 20)  # turning left: the right side pushes forward
    assert pat.side_strides("right", 12.5, p) == (10, -10)
    assert pat.side_strides("march", 25, p) == (0, 0)
    with pytest.raises(ValueError):
        pat.direction("sideways")
    with pytest.raises(ValueError):
        pat.pattern("gallop")


# ---- the plan --------------------------------------------------------------------------------------

def plan(pattern="walk", ticks=600, config=None, **kw):
    p = Planner(M, pattern, tick=1000, config=config, **kw)
    return p, [p.next() for _ in range(ticks)]


def joint_step_deg(a, b):
    worst = 0.0
    for leg in LEGS:
        t = M.ik(leg, a.x[leg], a.z[leg])
        u = M.ik(leg, b.x[leg], b.z[leg])
        worst = max(worst, abs(math.degrees(t[0] - u[0])), abs(math.degrees(t[1] - u[1])))
    return worst


@pytest.mark.parametrize("name", pat.PATTERN_NAMES)
def test_plan_fits_feet_and_the_slew(name):
    _, ticks = plan(name)
    assert [t.tick for t in ticks] == list(range(1000, 1600))
    lo, hi = [int(v) for v in "-60 60".split()]
    for t in ticks:
        assert all(lo <= x <= hi for x in t.x) and all(30 <= z <= 95 for z in t.z)
        assert all(M.reachable(t.x[i], t.z[i]) for i in LEGS)
        assert bin(t.support).count("1") >= (2 if name == "trot" else 3)
    assert max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:])) < M.slew_deg_per_tick


@pytest.mark.parametrize("name", ("walk", "crawl"))
def test_the_lean_keeps_the_mass_inside_three_feet(name):
    p, ticks = plan(name)
    three = [t for t in ticks if len(t.lifting) == 1]
    assert three and min(t.margin for t in three) > 7.0  # margin_mm 10, linearised
    _, flat = plan(name, config=PlannerConfig(lean=False))
    assert min(t.margin for t in flat if len(t.lifting) == 1) < 0  # the same steps without it: over the edge
    # the lean is in place a servo lag before every lift and held to the touchdown
    lead = p.lead
    assert lead == 8  # 150 ms
    for i, t in enumerate(ticks):
        if t.lifting and not ticks[i - 1].lifting:
            before = ticks[i - lead:i]
            assert max(abs(b.plan_roll - t.plan_roll) for b in before) < 0.6, i
    # the side the mass goes to: away from the lifting leg (roll + = right up = mass left)
    for t in three:
        leg = t.lifting[0]
        assert (t.plan_roll < 0) == (leg in (LF, LB)), (t.tick, leg, t.plan_roll)


def test_trot_stays_level_on_diagonal_pairs():
    _, ticks = plan("trot")
    for t in ticks:
        assert t.lifting in ((), (LF, RB), (RF, LB))
        assert abs(t.pitch) < 0.5 and abs(t.roll) < 0.5
    pairs = [t.lifting for t in ticks if t.lifting]
    assert len(set(pairs)) == 2


def test_prediction_is_the_plan_a_servo_lag_earlier():
    _, ticks = plan("walk", 300)
    lag = M.lag_ticks()  # 7.5
    for i in range(20, 300):
        a, b = ticks[i - 7], ticks[i - 8]
        assert ticks[i].roll == pytest.approx(a.plan_roll + (b.plan_roll - a.plan_roll) * (lag - 7), abs=1e-9)


def test_speed_changes_mid_walk_are_smooth():
    p = Planner(M, "trot", tick=0)
    ticks = [p.next() for _ in range(200)]
    p.set_speed(90)
    ticks += [p.next() for _ in range(200)]
    p.set_speed(10)
    ticks += [p.next() for _ in range(300)]
    assert max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:])) < M.slew_deg_per_tick
    assert max(max(abs(a.x[i] - b.x[i]) for i in LEGS if a.support >> i & 1 and b.support >> i & 1)
               for a, b in zip(ticks, ticks[1:])) <= 2  # feet on the ground never jump
    assert p.timing().speed_mm_s == pytest.approx(10)


def test_direction_and_pattern_changes():
    p = Planner(M, "trot", tick=0)
    for _ in range(100):
        p.next()
    p.set_direction("back")
    for _ in range(100):
        p.next()
    assert p.s_cur == [-25, -25]
    p.set_direction("left")
    for _ in range(100):
        p.next()
    assert p.s_cur == [-20, 20]
    p.set_pattern("walk")
    assert p.pattern.name == "trot" and p.changing  # the legs go home first
    seen = []
    for _ in range(300):
        t = p.next()
        seen.append((p.pattern.name, t.lifting))
    assert ("walk", (LB,)) in seen and all(not (n == "walk" and len(lf) == 2) for n, lf in seen)
    first = next(i for i, (n, _) in enumerate(seen) if n == "walk")
    assert first * 20 < 1500  # out of the trot: the strides ramp down (50 ticks) and each pair steps home
    assert p.s_cur == [-20, 20]  # the direction carried over (turning left)


def check_ticks(ticks, what=""):
    """Every tick inside FEET's ranges and the legs' reach, every joint at
    most 9.5 deg from the tick before (the body's slew is 10)."""
    for t in ticks:
        assert all(-60 <= x <= 60 for x in t.x) and all(30 <= z <= 95 for z in t.z), (what, t)
        assert all(M.reachable(t.x[i], t.z[i]) for i in LEGS), (what, t)
        assert abs(t.pitch) <= 30 and abs(t.roll) <= 30
    worst = max(joint_step_deg(a, b) for a, b in zip(ticks, ticks[1:]))
    assert worst <= 9.5, (what, worst)


@pytest.mark.parametrize("seq", [("walk", "trot", "crawl"), ("trot", "walk", "trot"), ("crawl", "walk", "trot"),
                                 ("walk", "crawl", "walk")])
def test_pattern_changes_one_after_another_stay_in_reach(seq):
    """Two pattern changes in a row (the remote's A3 / C4 / B4 pressed one
    after the other), at several points of the cycle and gaps between them."""
    a, b, c = seq
    for t0 in (100, 157, 211):
        for gap in (10, 60, 140):
            p = Planner(M, a, tick=0)
            ticks = [p.next() for _ in range(t0)]
            p.set_pattern(b)
            ticks += [p.next() for _ in range(gap)]
            p.set_pattern(c)
            ticks += [p.next() for _ in range(500)]
            check_ticks(ticks, (seq, t0, gap))
            assert p.pattern.name == c


def test_the_gate_pauses_the_feet():
    """While the gate holds the lift the plan waits: no foot slides toward the
    end of its range, so the next swing stays inside the slew and the reach
    (the trot at twice its speed slides 2.2 mm a tick)."""
    p = Planner(M, "trot", speed_mm_s=2 * pat.default_speed("trot"), tick=0)
    ticks = [p.next() for _ in range(40)]
    p.observe(6.0, 0.0)
    while p.stats["holds"] == 0:
        ticks.append(p.next())
    held = [ticks[-1]] + [p.next() for _ in range(24)]
    assert p.stats["holds"] == 25 and all(t.x == held[0].x and t.z == held[0].z and not t.lifting for t in held)
    ticks += held[1:]
    ticks.append(p.next())
    assert p.last.lifting  # hold_max: then it lifts anyway
    p.observe(0.0, 0.0)
    ticks += [p.next() for _ in range(100)]
    check_ticks(ticks)


def test_the_stance_time_is_slewed():
    """A speed change moves the feet's slide speed gradually (the stance time
    2 % a tick), not at once."""
    p = Planner(M, "trot", tick=0)
    for _ in range(100):
        p.next()
    before = p.stance_ticks
    p.set_speed(90)
    p.next()
    assert p.stance_ticks == pytest.approx(before / 1.02)
    for _ in range(100):
        p.next()
    assert p.stance_ticks == pytest.approx(p.stance_tgt)


def test_the_predicted_attitude_stays_inside_feet_range():
    """Pitch and roll go out inside FEET's +-30 deg whatever the plan
    predicts (a bad model, a pose offset): the body refuses a FEET beyond it
    (BAD_ARG), which would end the walk."""
    from kilo.planner.walker import pack_feet
    p = Planner(M, "walk", tick=0)
    p._lagged = lambda: (45.0, -31.0)
    t = p.next()
    assert (t.pitch, t.roll) == (30.0, -30.0)
    f = P.Feet.unpack(pack_feet(1, replace(t, pitch=45.0, roll=-45.0)))
    assert (f.pitch, f.roll) == (480, -480)


def test_the_lean_is_solved_during_the_swing_before():
    """The walk's lean for each slot is solved a few units a tick during the
    swing before it, so no tick carries a whole solve, and in a steady walk
    no slot waits for it; a change mid-swing is solved again in time."""
    for name in ("walk", "crawl"):
        p = Planner(M, name, tick=0)
        for _ in range(1500):
            p.next()
        assert p.stats["waits"] == 0, name
    p = Planner(M, "walk", tick=0)
    for i in range(1500):
        if i % 97 == 0:
            p.set_speed(5 + (i % 7))
        p.next()
    assert p.stats["waits"] < 40


def test_the_stop_comes_home_level():
    p = Planner(M, "walk", tick=0)
    for _ in range(300):
        p.next()
    p.stop()
    n = 0
    while not p.done and n < 1000:
        last = p.next()
        n += 1
    assert p.done and n < 600
    assert all(abs(x) <= 1 for x in last.x) and all(abs(z - H) <= 1 for z in last.z)
    assert abs(last.plan_roll) < 0.5 and abs(last.plan_pitch) < 0.5 and last.support == 0xF


def test_gate_holds_the_lift_and_catch_lands_the_foot():
    p = Planner(M, "walk", tick=0)
    p.observe(0.0, 8.0)  # the body is 8 deg off the plan: no lift
    ticks = [p.next() for _ in range(40)]
    assert not any(t.lifting for t in ticks) and p.stats["holds"] > 0
    p.observe(0.0, 0.0)
    while not p.last.lifting:
        p.next()
    leg = p.last.lifting[0]  # LB: tipping toward it is + roll (left side down) and - pitch (tail down)
    assert leg == LB
    p.next()
    p.observe(-3.0, 6.0)
    down = [p.next() for _ in range(4)]
    assert p.stats["catches"] == 1
    assert not down[-1].lifting and down[-1].support == 0xF  # down in three ticks, where it was


# ---- the error estimates ----------------------------------------------------------------------------

def _tel(tick, meas, ref, corr=(0.0, 0.0)):
    return P.Telemetry(tick=tick & 0xFFFF, pitch=round(meas[0] * 16), roll=round(meas[1] * 16),
                       ref_pitch=round(ref[0] * 16), ref_roll=round(ref[1] * 16),
                       corr_pitch=round(corr[0] * 16), corr_roll=round(corr[1] * 16), mode=P.Mode.GAIT)


def test_error_offset_lag_and_spike():
    tr = errors.ErrorTracker()
    for t in range(300):
        ref = 8 * math.sin(2 * math.pi * t / 75)
        meas = 8 * math.sin(2 * math.pi * (t - 2) / 75) + 1.5  # 2 ticks (40 ms) late, 1.5 deg off
        tr.add(t, _tel(t, (0.4, meas), (0.0, ref), (0.0, -0.5)))
    e = tr.estimate()
    assert e["lag_ms"] == 40 and e["lag_quality"] > 0.95
    assert e["offset_roll_deg"] == pytest.approx(1.5, abs=0.2) and e["offset_pitch_deg"] == pytest.approx(0.4, abs=0.05)
    assert e["corr_roll_deg"] == pytest.approx(-0.5, abs=0.05)
    assert tr.offset[0] == pytest.approx(0.4, abs=0.05)
    assert errors.ErrorTracker().estimate() is None
    # a plan that does not move (the trot) gives no lag
    flat = errors.ErrorTracker()
    for t in range(100):
        flat.add(t, _tel(t, (0.0, 0.3), (0.0, 0.0)))
    assert flat.estimate()["lag_ms"] is None
    # from a telemetry history: the newest run in GAIT, ticks unwrapped
    hist = [(t * 0.02, _tel(65500 + t, (0, 8 * math.sin(t / 10)), (0, 8 * math.sin(t / 10)))) for t in range(120)]
    assert errors.from_history(hist, int(P.Mode.GAIT))["lag_ms"] == 0


def test_a_heavier_side_shifts_the_lean():
    """The lean comes from the model's masses: with the battery on the right
    the body needs less roll to the right before a left leg lifts."""
    right = replace(M, battery=Mass(150.0, 0.0, -12.0, -15.0))
    _, a = plan("walk", 200)
    b = Planner(right, "walk", tick=1000)
    b = [b.next() for _ in range(200)]
    assert min(t.plan_roll for t in b) > min(t.plan_roll for t in a)
