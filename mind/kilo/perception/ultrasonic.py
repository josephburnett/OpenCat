"""Ultrasonic percepts: near / approaching / clear (docs/architecture.md §5 "Perception").

Fed with ``(t, distance_mm)`` samples, one per fresh range (the body ranges at
12.5 Hz, every other 25 Hz TELEMETRY, and flags the fresh ones with
``DIST_FRESH``). ``None``, 0 (the body's "no echo") and anything beyond
``max_range_mm`` count as a dropout.

Pipeline:

1. **Median of 3** over the last three valid echoes no older than
   ``stale_s``: a single spike or a single dropout changes nothing.
2. **Range rate** (mm/s, negative = getting closer) from a least-squares line
   over the filtered values of the last ``speed_window_s`` (at least
   ``min_speed_points``). A filtered jump larger than ``jump_mm`` between
   consecutive samples (somebody walking across the beam, the beam moving to
   another object) restarts the window, so a step is never read as an approach.
3. **Zones with hysteresis**, each change confirmed by ``confirm_samples``
   consecutive filtered samples:
   ``near``  enter < ``near_mm`` (150), leave > ``near_exit_mm`` (200);
   ``clear`` enter > ``clear_mm`` (400) or no echo at all for
   ``no_echo_clear_s`` (1 s: the HC-SR04 gives no echo when nothing is in
   range), leave < ``clear_exit_mm`` (350); between the two it is ``mid``.
4. **Approaching**: range rate < ``approach_mm_s`` (-100) for
   ``approach_samples`` (3) consecutive samples within ``approach_max_mm``;
   released after ``approach_samples`` samples above ``approach_release_mm_s``
   (-40), or on a sustained no-echo.

Events (``PerceptEvent``): ``near`` and ``clear`` on entering the zone (the
very first zone after start-up is silent unless it is ``near``),
``approaching`` on the rising edge, at most once per ``approach_cooldown_s``.
"""

from __future__ import annotations

import statistics
from collections import deque
from dataclasses import dataclass
from typing import Any

from kilo.perception.events import PerceptEvent


@dataclass
class UltrasonicConfig:
    near_mm: float = 150.0
    near_exit_mm: float = 200.0
    clear_mm: float = 400.0
    clear_exit_mm: float = 350.0
    confirm_samples: int = 2
    max_range_mm: float = 4000.0
    stale_s: float = 0.5
    no_echo_clear_s: float = 1.0
    speed_window_s: float = 0.8
    min_speed_points: int = 4
    jump_mm: float = 200.0
    approach_mm_s: float = -100.0
    approach_release_mm_s: float = -40.0
    approach_samples: int = 3
    approach_max_mm: float = 2000.0
    approach_cooldown_s: float = 2.0


def _slope(pts: Any) -> float:
    n = len(pts)
    tm = sum(p[0] for p in pts) / n
    vm = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - tm) ** 2 for p in pts)
    if sxx <= 0:
        return 0.0
    return sum((p[0] - tm) * (p[1] - vm) for p in pts) / sxx


class UltrasonicDetector:
    def __init__(self, config: UltrasonicConfig | None = None):
        self.cfg = config or UltrasonicConfig()
        self.reset()

    def reset(self) -> None:
        self._raw: deque[tuple[float, float]] = deque(maxlen=3)
        self._hist: deque[tuple[float, float]] = deque()
        self.distance_mm: float | None = None  # filtered
        self.range_rate_mm_s: float | None = None  # negative = closer
        self.zone: str | None = None  # near / mid / clear
        self.approaching = False
        self.last_valid_t: float | None = None
        self.first_t: float | None = None
        self._pending: str | None = None
        self._pending_n = 0
        self._neg_n = 0
        self._rel_n = 0
        self._last_approach_ev: float | None = None

    @property
    def near(self) -> bool:
        return self.zone == "near"

    @property
    def clear(self) -> bool:
        return self.zone == "clear"

    @property
    def approach_mm_s(self) -> float | None:
        """Closing speed, positive when something gets closer (as BodyState)."""
        return None if self.range_rate_mm_s is None else -self.range_rate_mm_s

    def feed(self, t: float, distance_mm: float | None) -> list[PerceptEvent]:
        c = self.cfg
        ev: list[PerceptEvent] = []
        if self.first_t is None:
            self.first_t = t
        valid = distance_mm is not None and 0 < distance_mm <= c.max_range_mm
        if not valid:
            since = self.last_valid_t if self.last_valid_t is not None else self.first_t
            if t - since >= c.no_echo_clear_s:
                self._raw.clear()
                self._hist.clear()
                self.distance_mm = None
                self.range_rate_mm_s = None
                self.approaching = False
                self._neg_n = self._rel_n = 0
                self._pending, self._pending_n = None, 0
                if self.zone != "clear":
                    if self.zone is not None:
                        ev.append(PerceptEvent(t, "clear", {"reason": "no_echo"}))
                    self.zone = "clear"
            return ev

        self.last_valid_t = t
        self._raw.append((t, float(distance_mm)))
        while self._raw and t - self._raw[0][0] > c.stale_s:
            self._raw.popleft()
        f = float(statistics.median(v for _, v in self._raw))
        if self._hist and abs(f - self._hist[-1][1]) > c.jump_mm:
            self._hist.clear()
        self._hist.append((t, f))
        while self._hist and t - self._hist[0][0] > c.speed_window_s:
            self._hist.popleft()
        self.distance_mm = f
        if len(self._hist) >= c.min_speed_points:
            self.range_rate_mm_s = _slope(self._hist)
        else:
            self.range_rate_mm_s = None

        ev.extend(self._zone_step(t, f))
        ev.extend(self._approach_step(t, f))
        return ev

    def _zone_step(self, t: float, f: float) -> list[PerceptEvent]:
        c = self.cfg
        z = self.zone
        if f < c.near_mm or (z == "near" and f <= c.near_exit_mm):
            cand = "near"
        elif f > c.clear_mm or (z == "clear" and f >= c.clear_exit_mm):
            cand = "clear"
        else:
            cand = "mid"
        if cand == z:
            self._pending, self._pending_n = None, 0
            return []
        if cand == self._pending:
            self._pending_n += 1
        else:
            self._pending, self._pending_n = cand, 1
        if self._pending_n < c.confirm_samples:
            return []
        self.zone = cand
        self._pending, self._pending_n = None, 0
        if cand == "mid" or (z is None and cand != "near"):
            return []
        return [PerceptEvent(t, cand, {"distance_mm": round(f, 1)})]

    def _approach_step(self, t: float, f: float) -> list[PerceptEvent]:
        c = self.cfg
        r = self.range_rate_mm_s
        if r is not None and r < c.approach_mm_s and f <= c.approach_max_mm:
            self._neg_n += 1
        else:
            self._neg_n = 0
        if not self.approaching:
            if self._neg_n >= c.approach_samples:
                self.approaching = True
                self._rel_n = 0
                last = self._last_approach_ev
                if last is None or t - last >= c.approach_cooldown_s:
                    self._last_approach_ev = t
                    return [PerceptEvent(t, "approaching", {"distance_mm": round(f, 1),
                                                            "speed_mm_s": round(-r, 1)})]
            return []
        if r is None or r > c.approach_release_mm_s:
            self._rel_n += 1
            if self._rel_n >= c.approach_samples:
                self.approaching = False
        else:
            self._rel_n = 0
        return []
