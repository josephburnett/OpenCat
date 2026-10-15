"""Gait patterns and the speed -> timing mapping (docs/walking-and-balance.md
§3, §10; mind/README.md "Walking").

A pattern is a sequence of swing groups: the legs that lift together, in
order. Each group gets a slot of the cycle: a four-foot phase (the "shift":
every foot down, the body moving over the feet that will stay down), then
the swing. So

    period = groups x (shift + swing),   stance = period - swing.

The swing time is fixed per pattern (short enough for the body's tip time
constant on two feet, long enough for the 10 deg/tick servo slew at the
pattern's stride and step height). Speed comes from the stride and the
stance time: stance = stride / speed; a slower speed lengthens the shift,
never the swing. Slower than the longest stance allows, the stride shrinks
(down to STRIDE_MIN_MM) instead; faster than the default stride at the
shortest shift allows, the speed is capped.

The shortest shift: the trot's three ticks of four-foot phase (60 ms; the
body's balance needs three feet down to correct at all); the one-leg-at-a-
time patterns (walk, crawl) must move the body over the next three feet
before each lift, and finish that move a servo lag ahead of the lift
(kilo.planner.plan), so their shift is at least the lag plus a 100 ms move.
"""

from __future__ import annotations

from dataclasses import dataclass

from kilo.planner.model import LB, LF, RB, RF

TICK_MS = 20.0
STRIDE_MIN_MM = 8.0
DEFAULT_TURN_MM = 20.0   # the per-side stride of a turn in place at the pattern's default stride
LEAN_MOVE_MS = 100.0     # the shortest move of the body between two swings (walk, crawl)


@dataclass(frozen=True)
class Pattern:
    name: str
    groups: tuple            # tuple of tuples of legs: who swings, in order
    swing_ms: float          # fixed swing time
    stride_mm: float         # default stride
    step_h_mm: float         # foot lift at mid-swing
    body_h_256: int          # body height, /256 of L1 + L2
    period_ms: float         # the cycle at the default speed
    shift_min_ms: float      # the shortest four-foot phase before a swing (without the lean's needs)
    stance_max_ms: float     # the longest stance: slower than that, the stride shrinks
    lean: bool               # one leg at a time: the body moves over the three feet that stay down


# Walk and crawl lift LB, LF, RB, RF (the lateral sequence; the phase
# offsets of the old tables, 128 0 64 192 /256 for LF RF RB LB); the trot
# the diagonal pairs LF+RB, RF+LB. Swing times, strides, step heights and
# body heights are the old tables' (walk 500 ms, crawl 297 ms, trot
# 150 ms). The walk and the crawl now have a four-foot shift before every
# swing (the old duty 192/256 at four legs left none), so their periods are
# longer than the old 2000 ms.
PATTERNS: dict[str, Pattern] = {
    "walk": Pattern("walk", ((LB,), (LF,), (RB,), (RF,)), swing_ms=500.0, stride_mm=30.0, step_h_mm=15.0,
                    body_h_256=205, period_ms=3200.0, shift_min_ms=0.0, stance_max_ms=4000.0, lean=True),
    "trot": Pattern("trot", ((LF, RB), (RF, LB)), swing_ms=150.0, stride_mm=25.0, step_h_mm=8.0,
                    body_h_256=205, period_ms=600.0, shift_min_ms=60.0, stance_max_ms=1327.0, lean=False),
    "crawl": Pattern("crawl", ((LB,), (LF,), (RB,), (RF,)), swing_ms=297.0, stride_mm=25.0, step_h_mm=18.0,
                     body_h_256=179, period_ms=2800.0, shift_min_ms=0.0, stance_max_ms=4000.0, lean=True),
}
PATTERN_NAMES = tuple(PATTERNS)
DEFAULT_PATTERN = "trot"
DIRECTIONS = ("forward", "back", "left", "right", "march")
_ALIASES = {"backward": "back", "backwards": "back"}


def pattern(name: str) -> Pattern:
    try:
        return PATTERNS[name.lower()]
    except KeyError:
        raise ValueError(f"unknown gait pattern {name!r} (known: {', '.join(PATTERN_NAMES)})") from None


def direction(name: str) -> str:
    d = _ALIASES.get(name.lower(), name.lower())
    if d not in DIRECTIONS:
        raise ValueError(f"unknown direction {name!r} (known: {', '.join(DIRECTIONS)})")
    return d


@dataclass(frozen=True)
class Timing:
    stride_mm: float   # magnitude
    shift_ms: float
    stance_ms: float
    period_ms: float
    speed_mm_s: float  # what this timing walks at with the stride


def shift_min_ms(p: Pattern, lag_ms: float) -> float:
    if p.lean:
        return max(p.shift_min_ms, lag_ms + LEAN_MOVE_MS)
    return p.shift_min_ms


def _stance_limits(p: Pattern, lag_ms: float) -> tuple[float, float]:
    n = len(p.groups)
    lo = n * (shift_min_ms(p, lag_ms) + p.swing_ms) - p.swing_ms
    return lo, max(lo, p.stance_max_ms)


def default_speed(p: Pattern | str) -> float:
    """The speed (mm/s) of the pattern's table: its stride over its default stance."""
    p = pattern(p) if isinstance(p, str) else p
    return p.stride_mm / (p.period_ms - p.swing_ms) * 1000.0


def speed_range(p: Pattern | str, lag_ms: float = 150.0) -> tuple[float, float]:
    """(slowest, fastest) speed in mm/s that timing() gives."""
    p = pattern(p) if isinstance(p, str) else p
    lo, hi = _stance_limits(p, lag_ms)
    return STRIDE_MIN_MM / hi * 1000.0, p.stride_mm / lo * 1000.0


def timing(p: Pattern | str, speed_mm_s: float | None = None, lag_ms: float = 150.0) -> Timing:
    """Stride, shift and period for walking at ``speed_mm_s`` (default: the
    pattern's default speed), clamped to speed_range()."""
    p = pattern(p) if isinstance(p, str) else p
    n = len(p.groups)
    st_lo, st_hi = _stance_limits(p, lag_ms)
    v = default_speed(p) if speed_mm_s is None else abs(float(speed_mm_s))
    stride = p.stride_mm
    stance = stride / v * 1000.0 if v > 0 else float("inf")
    if stance > st_hi:
        stance = st_hi
        stride = max(STRIDE_MIN_MM, v * st_hi / 1000.0)
    stance = max(stance, st_lo)
    period = stance + p.swing_ms
    return Timing(stride, period / n - p.swing_ms, stance, period, stride / stance * 1000.0)


def side_strides(direction_name: str, stride: float, p: Pattern) -> tuple[float, float]:
    """(left, right) stride in mm (+ = the feet push the body forward) for a direction.

    forward / back: the stride on both sides; left / right: turn in place,
    one side backward and the other forward by DEFAULT_TURN_MM scaled with
    the stride (a slower turn takes shorter steps); march: in place."""
    d = direction(direction_name)
    if d == "forward":
        return stride, stride
    if d == "back":
        return -stride, -stride
    if d == "march":
        return 0.0, 0.0
    t = DEFAULT_TURN_MM * stride / p.stride_mm
    return (-t, t) if d == "left" else (t, -t)
