"""The step planner: one tick of the whole body at a time.

``Planner.next()`` returns the plan for the next tick: where each foot is
(x, z per leg, as FEET carries them), the feet on the ground, and the
attitude the body is predicted to have at that tick. The streamer
(kilo.planner.walker) calls it once per tick, about five ticks ahead of the
body, and tells it what the body reports (``observe``) in between.

How a tick is made (docs/walking-and-balance.md §10):

- **Slots.** The pattern's swing groups take turns (kilo.planner.patterns):
  each slot is a four-foot shift of ``shift`` ticks, then a swing of
  ``swing`` ticks. Feet on the ground slide back at their side's stride
  speed (stride / stance time); a swinging foot lifts on a cosine profile
  (step height at mid-swing, zero speed at lift-off and touchdown) from
  where it left the ground to half a stride ahead of its shoulder. Speed and
  direction change the stride targets, which move at most 0.5 mm per tick,
  and the stance time, which moves at most 2 % per tick, so any change at
  any tick is smooth. A pattern change goes through the legs' way home: the
  strides go to 0 and each leg steps under its shoulder (as for a stop),
  then the new pattern starts from its first group as a walk starts (from
  the walk, up to ~5.5 s); a second change while that runs just changes
  where it goes.
- **The lean (walk, crawl).** On three feet the body stands on a triangle
  whose long edge runs under its middle, so before each lift the planner
  moves the centre of mass inside the triangle of the three feet that stay
  down, at least ``margin_mm`` from every edge, for the whole swing: it
  solves for the smallest body move (a fore-aft shift of every foot, a roll
  and a little pitch by leg-length differences) that gives that margin at
  lift-off and at touchdown (the body model's centre of mass, linearised
  and solved twice). The move runs during the shift and is complete
  ``lead`` (the servo lag) before the lift; it is held until the foot is
  down again. The solve is spread over the swing before the slot (a few
  units of work per tick, from where the feet will be when the slot
  starts), so no tick carries a whole solve; if something changed since (a
  command, a catch) it is solved again at the slot's start while the plan
  waits. The trot needs none: its diagonal pairs stand on a line,
  briefly (150 ms), and no lean helps there.
- **Prediction.** The attitude for tick t is the plane through the support
  feet the planner commanded a servo lag earlier (the body follows its
  commands that much later). It goes in FEET as the balance's reference:
  the body corrects only what the plan did not predict.
- **From the measured state.** ``observe`` gives the error the body
  reports (measured minus predicted, less the steady offset the walker
  learns). A leg does not lift while that error is above ``gate_deg``: the
  plan waits (up to ``hold_max`` ticks: every foot where it is, nothing
  sliding toward the end of its range) until the body follows the plan
  again. A swinging leg comes down at once (in three ticks, where it is)
  when the body tips toward it by more than ``catch_deg``.
- **Stop.** The strides go to 0; each leg still away from under its
  shoulder takes one more step to it; then the body moves back to level at
  the stance height and ``done`` is set: the walker hands over to the
  STANCE posture.
- **The sent attitude** stays inside FEET's +-30 deg (the body refuses a
  FEET beyond it).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from kilo.planner import patterns as pat
from kilo.planner.model import (FRONT, LEFT, LEGS, BodyModel, _solve3, fit_attitude, hull, leg_lengths, stability,
                                world_points)

X_RANGE = (-60, 60)   # FEET x_mm (spec range)
Z_RANGE = (30, 95)    # FEET z_mm
ATT_MAX_DEG = 30.0    # FEET pitch / roll (the body refuses a FEET beyond it: BAD_ARG ends the stream)
STRIDE_SLEW_MM = 0.5  # per tick
HEIGHT_SLEW_MM = 0.5  # per tick
RECENTRED_MM = 1.5    # a stopping leg this close to its target does not step again
STANCE_SLEW = 0.02    # the stance time (and with it the feet's slide) changes at most 2 % per tick
LEAN_UNITS_PER_TICK = 4  # units of a lean solve (Planner._solve_steps) per tick


@dataclass
class TickPlan:
    tick: int                 # unwrapped body tick
    x: list                   # int mm per leg, as sent
    z: list
    support: int              # bit i: leg i on the ground
    pitch: float              # predicted at this tick (the servo lag included), degrees
    roll: float
    plan_pitch: float         # the attitude these feet give (no lag)
    plan_roll: float
    margin: float             # mm of the centre of mass inside the support polygon (these feet)
    lifting: tuple            # the legs in the air this tick


@dataclass
class Lean:
    dx: float = 0.0           # body forward of its feet, mm
    pitch: float = 0.0        # degrees, nose down +
    roll: float = 0.0         # degrees, right side up +

    def lerp(self, other: "Lean", s: float) -> "Lean":
        return Lean(self.dx + (other.dx - self.dx) * s, self.pitch + (other.pitch - self.pitch) * s,
                    self.roll + (other.roll - self.roll) * s)


@dataclass
class PlannerConfig:
    margin_mm: float = 14.0       # the centre of mass this far inside the three feet, every swing (14: a 10 mm / 50 g model error keeps every direction within 3 deg; 10 was enough only for the exact model)
    dx_max_mm: float = 18.0       # the body's fore-aft shift over its feet
    dx_speed_mm_s: float = 100.0  # ...and its fastest move
    roll_max_deg: float = 14.0
    pitch_max_deg: float = 6.0
    # the lean's cost per unit: a mm of shift, a degree of pitch, a degree of roll
    dx_scale_mm: float = 10.0
    pitch_scale_deg: float = 2.0
    roll_scale_deg: float = 8.0
    gate_deg: float = 5.0         # no lift while the body is off the plan by more than this
    hold_max: int = 25            # ...for at most this many ticks (then it lifts anyway)
    catch_deg: float = 7.0        # a swinging leg lands at once when the body tips toward it this far
    lean: bool = True             # False: the same steps without the body's move (comparisons, tests)
    shift_ms: float | None = None  # a fixed four-foot shift (None: from the speed); 0 = the old tables' walk


def smooth(s: float) -> float:
    """0..1 -> 0..1 with zero slope at both ends."""
    s = min(1.0, max(0.0, s))
    return (1.0 - math.cos(math.pi * s)) / 2.0


class _LeanJob:
    """One slot's lean, solved a few units of work at a time
    (Planner._lean_steps): the slot's inputs it was started from, and the
    result once done: (shift ticks, the lean, its move's ticks)."""

    __slots__ = ("pattern", "k", "xg", "s", "stance", "shift", "lean_from", "gen", "result")

    def matches(self, pattern, k: int, xg, s, stance: float, shift: int, lean_from: Lean) -> bool:
        a, b = self.lean_from, lean_from
        return (self.pattern is pattern and self.k == k and self.shift == shift
                and abs(self.stance - stance) < 1e-6
                and all(abs(u - v) < 1e-6 for u, v in zip(self.xg, xg))
                and all(abs(u - v) < 1e-6 for u, v in zip(self.s, s))
                and abs(a.dx - b.dx) < 1e-6 and abs(a.pitch - b.pitch) < 1e-6 and abs(a.roll - b.roll) < 1e-6)


class Planner:
    def __init__(self, model: BodyModel, pattern: str = pat.DEFAULT_PATTERN, direction: str = "forward",
                 speed_mm_s: float | None = None, tick: int = 0, config: PlannerConfig | None = None,
                 start_height: float | None = None):
        self.model = model
        self.cfg = config or PlannerConfig()
        self.tick = tick
        self.lead = int(math.ceil(model.lag_ticks(pat.TICK_MS) - 1e-9))
        self.pattern = pat.pattern(pattern)
        self.direction = pat.direction(direction)
        self.speed_mm_s = pat.default_speed(self.pattern) if speed_mm_s is None else abs(float(speed_mm_s))
        self._pending_pattern: pat.Pattern | None = None
        self.xg = [0.0] * 4          # foot x in the gait frame (no lean shift)
        self.h = model.stance_height(205) if start_height is None else float(start_height)
        self.s_cur = [0.0, 0.0]      # stride now, left and right
        self.s_tgt = [0.0, 0.0]
        self.stance_ticks = 1.0      # a foot's time on the ground now (slewed toward stance_tgt)
        self.stance_tgt = 1.0
        self.shift_ticks = 1
        self.lean = Lean()
        self.lean_from = Lean()
        self.lean_to = Lean()
        self.tr_ticks = 1            # the lean's move takes this many ticks from the slot's start
        self.k = -1                  # slot index into the groups
        self.t = 0                   # tick within the slot (stands still while the plan waits)
        self.S = 0                   # this slot's shift ticks (may grow: the lean's move, a slower speed)
        self.held = 0                # ticks the gate held this slot
        self.swing = (0.0,) * 4      # per leg: x at lift-off
        self.skip = False            # this slot's group does not step (stopping, already in place)
        self.caught: dict[int, tuple[int, float]] = {}  # leg -> (tick of the catch, lift at the catch)
        self.stopping = False
        self.finishing = False       # stopped: the last move back to level
        self.done = False
        self.err = (0.0, 0.0)        # measured - predicted - offset (pitch, roll), the latest
        self.prev_att = (0.0, 0.0)
        self._lag = model.lag_ticks(pat.TICK_MS)
        self._att_hist: list[tuple[float, float]] = [(0.0, 0.0)] * (int(self._lag) + 3)
        self._job: _LeanJob | None = None   # this slot's lean while it is still being solved (the slot waits)
        self._next: _LeanJob | None = None  # the next slot's, solved during this slot's swing
        self.last: TickPlan | None = None
        self.stats = {"holds": 0, "catches": 0, "waits": 0, "min_margin": math.inf}
        self._apply_timing()
        self.stance_ticks = self.stance_tgt
        self._start_slot()
        if self._job is not None:  # the first slot's lean: solved here, on the caller's thread
            self._run_job(self._job, None)
            self._apply_job(self._job)

    # ---- commands ----------------------------------------------------------------------
    @property
    def swing_ticks(self) -> int:
        return max(2, int(round(self.pattern.swing_ms / pat.TICK_MS)))

    @property
    def changing(self) -> bool:
        """A pattern change is under way (the legs go home first)."""
        return self._pending_pattern is not None

    def timing(self) -> pat.Timing:
        return pat.timing(self.pattern, self.speed_mm_s, self.model.servo_lag_ms)

    def _apply_timing(self) -> None:
        tm = self.timing()
        shift = tm.shift_ms if self.cfg.shift_ms is None else self.cfg.shift_ms
        n = len(self.pattern.groups)
        self.stance_tgt = (n * (shift + self.pattern.swing_ms) - self.pattern.swing_ms) / pat.TICK_MS
        self.shift_ticks = max(0, int(round(shift / pat.TICK_MS)))
        if self.stopping or self._pending_pattern is not None:
            self.s_tgt = [0.0, 0.0]
        else:
            self.s_tgt = list(pat.side_strides(self.direction, tm.stride_mm, self.pattern))
        self._next = None  # the next slot's lean was solved for the old timing

    def set_speed(self, speed_mm_s: float) -> None:
        self.speed_mm_s = abs(float(speed_mm_s))
        self._apply_timing()
        if self._job is not None:  # this slot is still waiting for its lean: solve it for the new shift
            self._job = self._new_job(self.k, self.xg, self.s_cur, self.stance_ticks, self.lean_from, self.h)
        elif self.t < self.S:  # still in the shift: a new length, but never shorter than its lean needs
            self.S = max(self.t + 1, self.tr_ticks + (self.lead if self._leaning() else 0), self.shift_ticks)

    def set_direction(self, direction: str) -> None:
        self.direction = pat.direction(direction)
        self._apply_timing()

    def set_pattern(self, name: str) -> None:
        """Change the pattern: the strides go to 0 and every leg steps home
        (as for a stop, without the level-out), then the new pattern starts
        from its first group the way a walk starts. Asking for the pattern
        that is walking cancels a change still under way; the last pattern
        asked for wins. Ignored once stopping."""
        p = pat.pattern(name)
        if self.stopping:
            return
        if p is self.pattern:
            if self._pending_pattern is not None:
                self._pending_pattern = None
                self._apply_timing()
            return
        self._pending_pattern = p
        self._apply_timing()

    def stop(self) -> None:
        self.stopping = True
        self._pending_pattern = None
        self._apply_timing()

    def observe(self, err_pitch: float, err_roll: float) -> None:
        """The body's latest error: measured - predicted - the steady offset (degrees)."""
        self.err = (err_pitch, err_roll)

    # ---- the slot machine ----------------------------------------------------------------
    def _leaning(self) -> bool:
        return self.pattern.lean and self.cfg.lean

    def _group(self) -> tuple:
        return self.pattern.groups[self.k]

    def _home(self) -> bool:
        return (all(abs(self.s_cur[i]) < 0.05 for i in (0, 1))
                and all(abs(self.xg[leg]) < RECENTRED_MM for leg in LEGS))

    def _start_slot(self) -> None:
        self._job = None
        if self._pending_pattern is not None and self._home():
            # every foot is home and still: the new pattern starts from its first group
            self.pattern, self._pending_pattern = self._pending_pattern, None
            self.k = -1
            self._apply_timing()
            self.stance_ticks = self.stance_tgt
        self.k = (self.k + 1) % len(self.pattern.groups)
        self.t = 0
        self.held = 0
        self.caught = {}
        g = self._group()
        self.skip = (self.stopping or self.changing) and all(
            abs(self.s_cur[0]) < 0.05 and abs(self.s_cur[1]) < 0.05 and abs(self.xg[leg]) < RECENTRED_MM for leg in g)
        self.lean_from = Lean(self.lean.dx, self.lean.pitch, self.lean.roll)
        nxt, self._next = self._next, None
        if self.stopping and self._home():
            self.finishing = True
        if self.finishing:
            self.lean_to = Lean()
            self.tr_ticks = max(self.lead + 5, self._move_ticks(self.lean_from, self.lean_to, self.h))
            self.S = self.tr_ticks + self.lead
            return
        self.S = self.shift_ticks
        if self._leaning() and not self.skip:
            if nxt is not None and nxt.matches(self.pattern, self.k, self.xg, self.s_cur, self.stance_ticks,
                                               self.shift_ticks, self.lean_from):
                job = nxt
            else:  # something changed since it was solved (a command, a catch): solved now, the slot waits
                job = self._new_job(self.k, self.xg, self.s_cur, self.stance_ticks, self.lean_from, self.h)
            if job.result is not None:
                self._apply_job(job)
            else:
                self._job = job
        else:
            self.lean_to = Lean() if not self._leaning() else self.lean_from
            tr = 0
            if self.lean_to != self.lean_from:  # out of a leaning pattern: level first, as fast as gravity lets
                tr = self._move_ticks(self.lean_from, self.lean_to, self.h)
                self.S = max(self.S, tr + self.lead)
            self.tr_ticks = max(1, tr, self.S - (self.lead if tr else 0))

    def _move_ticks(self, a: Lean, b: Lean, height: float) -> int:
        """Ticks the body needs to move from lean a to lean b on a smooth
        (cosine) profile: a side or an end that drops falls no faster than
        gravity lets it (half of BodyModel.drop_accel, a margin), and the
        fore-aft shift goes at most dx_speed_mm_s."""
        m, cfg = self.model, self.cfg
        h = height + m.foot_r_mm
        need = 0.0
        for d_deg, lever in ((b.roll - a.roll, m.track_mm / 2), (b.pitch - a.pitch, m.length_mm / 2)):
            amp = abs(math.radians(d_deg))
            if amp > 0:
                need = max(need, math.pi * math.sqrt(amp / (0.5 * m.drop_accel(lever, h))))
        need = max(need, math.pi / 2 * abs(b.dx - a.dx) / cfg.dx_speed_mm_s)
        return max(1, int(math.ceil(need / (pat.TICK_MS / 1000.0))))

    def _side_speed(self, leg: int) -> float:
        """mm per tick a foot on the ground slides back."""
        return self.s_cur[0 if LEFT[leg] else 1] / self.stance_ticks

    def _target_x(self, leg: int) -> float:
        return self.s_cur[0 if LEFT[leg] else 1] / 2.0

    def _slewed(self, s, stance: float, h: float):
        """One tick of the strides, the stance time and the height toward
        their targets (the stance time by at most STANCE_SLEW of itself, so
        a speed change never jumps the feet's slide either)."""
        s = [s[i] + max(-STRIDE_SLEW_MM, min(STRIDE_SLEW_MM, self.s_tgt[i] - s[i])) for i in (0, 1)]
        stance = min(stance * (1.0 + STANCE_SLEW), max(stance / (1.0 + STANCE_SLEW), self.stance_tgt))
        h += max(-HEIGHT_SLEW_MM, min(HEIGHT_SLEW_MM, self._height_target() - h))
        return s, stance, h

    # ---- the lean -----------------------------------------------------------------------------
    def _new_job(self, k: int, xg, s, stance: float, lean_from: Lean, h_now: float) -> _LeanJob:
        job = _LeanJob()
        job.pattern, job.k, job.xg, job.s, job.stance = self.pattern, k, tuple(xg), tuple(s), stance
        job.shift, job.lean_from, job.result = self.shift_ticks, lean_from, None
        group = self.pattern.groups[k]
        speed = [s[0 if LEFT[leg] else 1] / stance for leg in LEGS]
        tgt = [s[0 if LEFT[leg] else 1] / 2.0 for leg in LEGS]
        h = self.model.stance_height(self.pattern.body_h_256)
        job.gen = self._lean_steps(group, list(xg), speed, tgt, h, h_now, self.shift_ticks, self.swing_ticks,
                                   lean_from)
        return job

    @staticmethod
    def _run_job(job: _LeanJob, units: int | None) -> bool:
        """At most ``units`` units of the job's work (None: all of it). True once it is done."""
        if job.result is None:
            try:
                if units is None:
                    while True:
                        next(job.gen)
                for _ in range(units):
                    next(job.gen)
            except StopIteration as e:
                job.result = e.value
        return job.result is not None

    def _apply_job(self, job: _LeanJob) -> None:
        self.S, lean_to, self.tr_ticks = job.result
        self.lean_to = Lean(lean_to.dx, lean_to.pitch, lean_to.roll)
        self._job = None

    def _lean_steps(self, group: tuple, xg, speed, tgt, h: float, h_now: float, shift: int, swing: int,
                    lean_from: Lean):
        """The slot's shift, lean and move ticks (a generator: one unit of
        work per step, the result as its return value). The move must be
        done a servo lag before the lift: when it needs longer than the
        speed's shift gives, the shift grows and the lean is solved again
        for the later lift-off."""
        S = shift
        lean_to = yield from self._solve_steps(group, xg, speed, tgt, h, S, swing)
        tr = self._move_ticks(lean_from, lean_to, h_now)
        if tr + self.lead > S:
            S = tr + self.lead
            lean_to = yield from self._solve_steps(group, xg, speed, tgt, h, S, swing)
            tr = self._move_ticks(lean_from, lean_to, h_now)
        tr_ticks = max(1, S - self.lead, tr)
        return max(S, tr_ticks + self.lead), lean_to, tr_ticks

    def _solve_steps(self, group: tuple, xg, speed, tgt, h: float, shift: int, swing: int):
        """The smallest body move that keeps the centre of mass margin_mm
        inside the feet that stay down, at lift-off (``shift`` ticks on) and
        at touchdown (``swing`` later), for feet now at ``xg`` (gait frame)
        sliding back ``speed`` mm per tick, the group landing at ``tgt``. A
        generator: one margin evaluation or one part of the quadratic
        program per step."""
        cfg = self.cfg
        support = tuple(leg for leg in LEGS if leg not in group)
        configs = []
        for ahead in (shift, shift + swing):
            xs = [xg[leg] - speed[leg] * ahead for leg in LEGS]
            for leg in group:
                xs[leg] = xg[leg] - speed[leg] * shift if ahead == shift else tgt[leg]
            configs.append(xs)
        scale = (cfg.dx_scale_mm, cfg.pitch_scale_deg, cfg.roll_scale_deg)
        lim = (cfg.dx_max_mm, cfg.pitch_max_deg, cfg.roll_max_deg)
        u = [0.0, 0.0, 0.0]
        for _ in range(2):
            rows = []  # (gradient in u, margin at u) per edge per config
            for xs in configs:
                base = self._edge_margins(xs, h, support, u)
                yield
                grads = []
                for j in range(3):
                    du = list(u)
                    du[j] += 0.5
                    m = self._edge_margins(xs, h, support, du)
                    yield
                    grads.append([(m[e] - base[e]) / 0.5 for e in range(len(base))])
                for e in range(len(base)):
                    rows.append(((grads[0][e], grads[1][e], grads[2][e]), base[e]))
            # minimise sum((d_j / scale_j)^2) s.t. m_e + g_e . d >= margin, d = the step from u
            want = cfg.margin_mm
            step = None
            while step is None and want >= -1e-9:
                step = yield from _qp_steps(rows, want, scale)
                if step is None:
                    want = want - 2.0 if want > 2.0 else (0.0 if want > 0 else -1.0)
            if step is None:
                break
            u = [min(lim[j], max(-lim[j], u[j] + step[j])) for j in range(3)]
        return Lean(u[0], u[1], u[2])

    def _edge_margins(self, xs_gait, h, support: tuple, u) -> list:
        """Signed distance (mm, + inside) of the centre of mass from each edge
        of the support polygon, for the gait-frame feet shifted by lean u
        (three support feet: their triangle, counter-clockwise)."""
        xs = [x - u[0] for x in xs_gait]
        zs = leg_lengths(self.model, xs, h, u[1], u[2])
        com, feet = world_points(self.model, xs, zs, u[1], u[2])
        if len(support) == 3:
            a, b, c = (feet[leg] for leg in support)
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) < 0:
                b, c = c, b
            poly = (a, b, c)
        else:
            poly = hull([(feet[leg][0], feet[leg][1]) for leg in support])
        out = []
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            ex, ey = b[0] - a[0], b[1] - a[1]
            L = math.hypot(ex, ey) or 1.0
            out.append((ex * (com[1] - a[1]) - ey * (com[0] - a[0])) / L)
        return out

    def _height_target(self) -> float:
        if self.finishing:
            return self.model.stance_height(205)
        return self.model.stance_height(self.pattern.body_h_256)

    def _predict_slot_end(self):
        """The feet, strides, stance time and height at the end of this
        slot's swing if nothing changes (the same arithmetic as next())."""
        W = self.swing_ticks
        xg = list(self.xg)
        s, st, h = self.s_cur, self.stance_ticks, self.h
        g = self._group()
        for t in range(self.t, self.S + W):
            s, st, h = self._slewed(s, st, h)
            sw = smooth((t - self.S + 1) / W)
            for leg in LEGS:
                side = 0 if LEFT[leg] else 1
                if leg in g:
                    x0 = self.swing[leg]
                    xg[leg] = x0 + (s[side] / 2.0 - x0) * sw
                else:
                    xg[leg] -= s[side] / st
        return xg, s, st, h

    # ---- one tick ---------------------------------------------------------------------------------
    def next(self) -> TickPlan:
        """Plan the next tick (``self.tick``) and advance."""
        cfg = self.cfg
        W = self.swing_ticks
        if self.t >= self.S + W or (self.skip and self.t >= self.S) or (self.finishing and self.t >= self.S):
            if self.finishing:
                self.done = True
            else:
                self._start_slot()
        # The plan waits (every foot where it is, the lean held, nothing
        # slewed, the slot's clock stopped) while this slot's lean is still
        # being solved, and while the gate holds the lift: a leg lifts only
        # while the body follows the plan.
        wait = False
        if self._job is not None:
            if self._run_job(self._job, LEAN_UNITS_PER_TICK):
                self._apply_job(self._job)
            else:
                wait = True
                self.stats["waits"] += 1
        if not wait and self.t == self.S and not self.skip and not self.finishing and self.held < cfg.hold_max \
                and max(abs(self.err[0]), abs(self.err[1])) > cfg.gate_deg:
            wait = True
            self.held += 1
            self.stats["holds"] += 1

        g = self._group()
        lifting: tuple = ()
        lift = [0.0, 0.0, 0.0, 0.0]
        swinging = False
        if not wait:
            self.s_cur, self.stance_ticks, self.h = self._slewed(self.s_cur, self.stance_ticks, self.h)
            if self.t < self.S or self.skip or self.finishing:
                for leg in LEGS:
                    self.xg[leg] -= self._side_speed(leg)
            else:
                swinging = True
                j = self.t - self.S + 1          # 1..swing
                s = j / W
                if j == 1:
                    self.swing = tuple(self.xg)  # where the feet leave the ground
                for leg in LEGS:
                    if leg in g:
                        continue
                    self.xg[leg] -= self._side_speed(leg)
                for leg in g:
                    if leg in self.caught:
                        continue
                    if self._tip_toward(leg) > cfg.catch_deg and j < W:
                        prev_lift = self.pattern.step_h_mm * (1 - math.cos(2 * math.pi * (j - 1) / W)) / 2
                        self.caught[leg] = (j - 1, prev_lift)
                        self.stats["catches"] += 1
                        self._next = None  # it lands elsewhere: the next slot's lean is solved when it starts
                for leg in g:
                    if leg in self.caught:
                        jc, lc = self.caught[leg]
                        k = j - jc
                        lift[leg] = lc * (1 - smooth(k / 3.0))
                    else:
                        x0 = self.swing[leg]
                        self.xg[leg] = x0 + (self._target_x(leg) - x0) * smooth(s)
                        lift[leg] = self.pattern.step_h_mm * (1 - math.cos(2 * math.pi * s)) / 2
                if self.caught and all(lift[leg] < 1e-6 for leg in g):
                    self.t = self.S + W  # every caught foot is down: the slot is over
                    for leg in g:
                        lift[leg] = 0.0
                lifting = tuple(leg for leg in g if lift[leg] > 1e-6)
            # the lean: moves from the slot's start, complete tr_ticks later
            if self.t < self.tr_ticks:
                self.lean = self.lean_from.lerp(self.lean_to, smooth((self.t + 1) / self.tr_ticks))
            else:
                self.lean = Lean(self.lean_to.dx, self.lean_to.pitch, self.lean_to.roll)

        lean = self.lean
        xs = [self.xg[leg] - lean.dx for leg in LEGS]
        zs = leg_lengths(self.model, xs, self.h, lean.pitch, lean.roll)
        xi = [int(round(min(X_RANGE[1], max(X_RANGE[0], xs[leg])))) for leg in LEGS]
        zi = [int(round(min(Z_RANGE[1], max(Z_RANGE[0], zs[leg] - lift[leg])))) for leg in LEGS]
        support = 0
        for leg in LEGS:
            if lift[leg] <= 1e-6:
                support |= 1 << leg
        att = fit_attitude(self.model, xi, zi, support, self.prev_att)
        self.prev_att = att
        self._att_hist.append(att)
        del self._att_hist[0]
        pp, pr = self._lagged()
        pp = min(ATT_MAX_DEG, max(-ATT_MAX_DEG, pp))
        pr = min(ATT_MAX_DEG, max(-ATT_MAX_DEG, pr))
        m = stability(self.model, xi, zi, support, att[0], att[1])
        if len(lifting) == 1:
            self.stats["min_margin"] = min(self.stats["min_margin"], m)
        out = TickPlan(self.tick, xi, zi, support, pp, pr, att[0], att[1], m, lifting)
        self.last = out
        self.tick += 1
        if not wait:
            self.t += 1
        # The next slot's lean, solved a little every tick of this swing (so
        # no tick carries a whole solve), from where the feet will be when
        # that slot starts; used if nothing changes before then.
        if swinging and not self.caught and self._leaning() and not self.changing and not self.finishing:
            if self._next is None:
                xg, s, st, h = self._predict_slot_end()
                self._next = self._new_job((self.k + 1) % len(self.pattern.groups), xg, s, st,
                                           Lean(self.lean_to.dx, self.lean_to.pitch, self.lean_to.roll), h)
            self._run_job(self._next, LEAN_UNITS_PER_TICK)
        return out

    def _lagged(self) -> tuple[float, float]:
        """The planned attitude a servo lag ago (linear between ticks)."""
        hist = self._att_hist
        n = len(hist) - 1            # hist[n] is this tick
        lag = self._lag
        i = int(math.floor(lag))
        f = lag - i
        a = hist[max(0, n - i)]
        b = hist[max(0, n - i - 1)]
        return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f

    def _tip_toward(self, leg: int) -> float:
        """How far (degrees) the body is tipping toward this leg beyond the plan."""
        ep, er = self.err
        side = 1.0 if LEFT[leg] else -1.0    # a left leg: the left side going down is +roll
        end = 1.0 if FRONT[leg] else -1.0    # a front leg: nose going down is +pitch
        return side * er + end * ep


def _qp_steps(rows, want: float, scale):
    """min sum((d_j / scale_j)^2) subject to m_e + g_e . d >= want for every
    row (g_e, m_e): enumerate active sets of up to three constraints (the
    problem is three-dimensional, the rows few). A generator (one step per
    active-set size); returns d, or None if infeasible."""
    s0, s1, s2 = scale
    # whiten: d = S w, so the cost is |w|^2 and a row's gradient becomes g S
    W = [(g[0] * s0, g[1] * s1, g[2] * s2, want - m) for g, m in rows]  # a . w >= b

    def feasible(w0, w1, w2) -> bool:
        for a0, a1, a2, b in W:
            if a0 * w0 + a1 * w1 + a2 * w2 < b - 1e-6:
                return False
        return True

    if feasible(0.0, 0.0, 0.0):
        return [0.0, 0.0, 0.0]
    best = None
    best_n = math.inf
    n = len(W)
    for size in (1, 2, 3):
        yield
        for idx in itertools.combinations(range(n), size):
            w = _min_norm_eq([W[i] for i in idx])
            if w is None:
                continue
            nn = w[0] * w[0] + w[1] * w[1] + w[2] * w[2]
            if nn < best_n and feasible(w[0], w[1], w[2]):
                best, best_n = w, nn
    if best is None:
        return None
    return [best[0] * s0, best[1] * s1, best[2] * s2]


def _min_norm_eq(A):
    """The least-norm w with a . w = b for each row (a0, a1, a2, b) of A (1..3 rows)."""
    k = len(A)
    if k == 1:
        a0, a1, a2, b = A[0]
        g = a0 * a0 + a1 * a1 + a2 * a2
        if g < 1e-12:
            return None
        lam = b / g
        return (a0 * lam, a1 * lam, a2 * lam)
    if k == 2:
        (p0, p1, p2, pb), (q0, q1, q2, qb) = A
        g00 = p0 * p0 + p1 * p1 + p2 * p2
        g01 = p0 * q0 + p1 * q1 + p2 * q2
        g11 = q0 * q0 + q1 * q1 + q2 * q2
        det = g00 * g11 - g01 * g01
        if abs(det) < 1e-9:
            return None
        l0 = (pb * g11 - g01 * qb) / det
        l1 = (g00 * qb - g01 * pb) / det
        return (p0 * l0 + q0 * l1, p1 * l0 + q1 * l1, p2 * l0 + q2 * l1)
    G = [[A[i][0] * A[j][0] + A[i][1] * A[j][1] + A[i][2] * A[j][2] for j in range(3)] for i in range(3)]
    lam = _solve3(G, (A[0][3], A[1][3], A[2][3]))
    if lam is None:
        return None
    return tuple(A[0][c] * lam[0] + A[1][c] * lam[1] + A[2][c] * lam[2] for c in range(3))
