"""IMU percepts from TELEMETRY attitude: tilted, picked_up, tap, pet, startle.

Fed with ``(t, pitch_deg, roll_deg, rates_dps, flags, mode)`` per TELEMETRY
(25 Hz by default). Attitude convention as measured on Kilo (legacy ypr):
**nose down = +pitch, right side up = +roll**, degrees. ``rates_dps`` is
(rate_pitch, rate_roll, rate_yaw) from the raw gyro, or None.

What telemetry offers, and so what these detectors assume:

- DMP attitude (pitch, roll) and raw gyro rates. **No accelerometer**: a lift
  straight up, or being held still in the air, does not change the attitude
  and is invisible. Everything below is attitude-shape heuristics.
- ``servos_powered`` flag and ``mode``/``gait``/``trick`` flags tell whether the
  body could have moved itself. While it does (GAIT, TRICK, RECOVERING,
  CALIB, BOOT, the gait/trick flags), and for ``settle_s`` after any mode
  change (posture moves are IDLE and invisible, so a mode change is the best
  hint), tap/pet/startle/pickup are suppressed. The behaviour layer can call
  ``notify_self_motion(t, seconds)`` when it starts a posture move.

Detectors (thresholds in ``ImuConfig``):

- **tilted**: max(|pitch - ref_pitch|, |roll - ref_roll|) > ``tilt_deg`` (25)
  for ``tilt_confirm_s`` (0.5 s), released below ``tilt_exit_deg`` (18) for
  the same time; events ``tilted`` (with ``direction``) and ``level``. The
  reference is the posture's expected attitude: ``set_reference()``, else
  learned after start-up and after every mode change once the attitude has
  been calm for ``settle_s`` (if within ``max_ref_deg`` of level), else level.
- **picked_up**: attitude excursion (max - min over ``pickup_window_s``, 1 s)
  >= ``pickup_deg`` (12°; ``pickup_unpowered_deg`` 6° with servos off) for
  ``pickup_confirm_s`` (0.5 s) while the body is not moving itself. Released
  (``put_down``) once calm (excursion < ``calm_deg`` over ``put_down_calm_s``)
  and back within ``tilt_deg`` of the reference.
- **tap**: a short impulse on a still body (the preceding ``pre_window`` moved
  less than ``tap_quiet_deg``), measured vs that window's median:
  rises from < half of ``tap_min_deg`` to >= ``tap_min_deg`` (1.5°) within
  ``tap_rise_s`` and returns below ``tap_return_deg`` within ``tap_max_s``
  (0.35 s), peak < ``startle_deg``. Cooldown ``tap_cooldown_s``.
- **pet**: gentle periodic rocking over ``pet_window_s`` (4 s): peak-to-peak
  ``pet_min_pp``..``pet_max_pp`` (1.5..10°), >= ``pet_min_half_cycles``
  alternating crossings of a band around the mean, half-periods 0.2..1.5 s,
  never faster than ``pet_max_rate_dps``. Cooldown ``pet_cooldown_s``; the
  ``pet`` percept stays true for ``pet_hold_s`` after the last match.
- **startle** (candidate): a sudden large change, >= ``startle_deg`` (10°)
  from calm within ``startle_rise_s`` (0.2 s), or a gyro rate >=
  ``startle_rate_dps``. Cooldown ``startle_cooldown_s``.

Tap, pet and startle need >= ~10 Hz telemetry and pickup >= 5 Hz (they are
silent on a 1 Hz monitor log); tilt works at any rate.
"""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass
from typing import Any, Iterable

from kilo.perception.events import PerceptEvent

SELF_MOTION_MODES = frozenset({"GAIT", "TRICK", "RECOVERING", "CALIB", "BOOT"})
SELF_MOTION_FLAGS = frozenset({"gait", "trick"})


@dataclass
class ImuConfig:
    # tilt
    tilt_deg: float = 25.0
    tilt_exit_deg: float = 18.0
    tilt_confirm_s: float = 0.5
    max_ref_deg: float = 20.0
    auto_reference: bool = True
    # self motion
    settle_s: float = 1.0
    calm_deg: float = 2.0
    # pickup
    pickup_window_s: float = 1.0
    pickup_deg: float = 12.0
    pickup_unpowered_deg: float = 6.0
    pickup_confirm_s: float = 0.5
    pickup_min_samples: int = 5  # in pickup_window_s: needs >= 5 Hz telemetry
    put_down_calm_s: float = 1.5
    # impulses
    pre_window_s: float = 1.0
    pre_gap_s: float = 0.25
    tap_min_deg: float = 1.5
    tap_quiet_deg: float = 1.5
    tap_rise_s: float = 0.12
    tap_return_deg: float = 0.8
    tap_max_s: float = 0.35
    tap_cooldown_s: float = 0.5
    startle_deg: float = 10.0
    startle_rise_s: float = 0.2
    startle_rate_dps: float = 250.0
    startle_cooldown_s: float = 3.0
    # pet
    pet_window_s: float = 4.0
    pet_min_pp: float = 1.5
    pet_max_pp: float = 10.0
    pet_band_frac: float = 0.25
    pet_min_band: float = 0.5
    pet_min_half_cycles: int = 4
    pet_half_period_s: tuple[float, float] = (0.2, 1.5)
    pet_max_rate_dps: float = 30.0
    pet_cooldown_s: float = 3.0
    pet_hold_s: float = 1.5


def _direction(dp: float, dr: float) -> str:
    if abs(dp) >= abs(dr):
        return "nose_down" if dp > 0 else "nose_up"
    return "right_up" if dr > 0 else "left_up"


def _smooth3(x: list[float]) -> list[float]:
    if len(x) < 3:
        return list(x)
    return [x[0]] + [(x[i - 1] + x[i] + x[i + 1]) / 3 for i in range(1, len(x) - 1)] + [x[-1]]


class ImuDetector:
    def __init__(self, config: ImuConfig | None = None):
        self.cfg = config or ImuConfig()
        self.reset()

    def reset(self) -> None:
        c = self.cfg
        keep = max(c.pet_window_s, c.pre_window_s, c.put_down_calm_s, c.pickup_window_s) + 0.5
        self._keep_s = keep
        self._buf: deque[tuple[float, float, float]] = deque()  # t, pitch, roll
        self._d: deque[tuple[float, float]] = deque()  # t, impulse deviation
        self.ref: tuple[float, float] = (0.0, 0.0)
        self._ref_fixed = False
        self._relearn_from: float | None = None  # learn once calm after this time
        self._mode: str | None = None
        self._quiet_from = -math.inf  # detectors active from this time
        self._self_moving = False
        self.pitch: float | None = None
        self.roll: float | None = None
        # tilt
        self.tilted = False
        self.tilt_direction: str | None = None
        self._tilt_since: float | None = None
        # pickup
        self.picked_up = False
        self._pick_since: float | None = None
        # tap
        self._tap_state = "idle"
        self._tap_t0 = 0.0
        self._tap_peak = 0.0
        self._pre_quiet = False
        # pet
        self._pet_seen: float | None = None
        # last event times
        self._last: dict[str, float] = {}

    # ---- inputs -------------------------------------------------------------
    def set_reference(self, pitch: float | None, roll: float | None = None) -> None:
        """Expected attitude of the current posture; None = learn it again."""
        if pitch is None:
            self._ref_fixed = False
            self._relearn_from = self._buf[-1][0] if self._buf else None
        else:
            self.ref = (float(pitch), float(roll or 0.0))
            self._ref_fixed = True

    def notify_self_motion(self, t: float, seconds: float = 0.0) -> None:
        """The mind is moving the body (posture move...): suppress until t + seconds + settle."""
        self._quiet_from = max(self._quiet_from, t + seconds + self.cfg.settle_s)
        if not self._ref_fixed:
            self._relearn_from = t + seconds

    @property
    def active(self) -> bool:
        return not self._self_moving and bool(self._buf) and self._buf[-1][0] >= self._quiet_from

    @property
    def pet(self) -> bool:
        return (self._pet_seen is not None and bool(self._buf)
                and self._buf[-1][0] - self._pet_seen <= self.cfg.pet_hold_s)

    def feed(self, t: float, pitch: float | None, roll: float | None,
             rates: Iterable[float] | None = None, flags: Iterable[str] | None = None,
             mode: str | None = None) -> list[PerceptEvent]:
        if pitch is None or roll is None:
            return []
        c = self.cfg
        flags = None if flags is None else set(flags)
        ev: list[PerceptEvent] = []
        if not self._buf:
            self._relearn_from = t if not self._ref_fixed else None
        # self motion
        moving = mode in SELF_MOTION_MODES or bool(flags and flags & SELF_MOTION_FLAGS)
        if mode is not None and self._mode is not None and mode != self._mode and mode != "FALLEN":
            self._quiet_from = max(self._quiet_from, t + c.settle_s)
            if not self._ref_fixed:
                self._relearn_from = t
        if self._self_moving and not moving:
            self._quiet_from = max(self._quiet_from, t + c.settle_s)
            if not self._ref_fixed:
                self._relearn_from = t
        if mode is not None:
            self._mode = mode
        self._self_moving = moving

        self.pitch, self.roll = float(pitch), float(roll)
        self._buf.append((t, self.pitch, self.roll))
        while self._buf and t - self._buf[0][0] > self._keep_s:
            self._buf.popleft()

        self._learn_reference(t)
        ev += self._tilt(t)
        rate_mag = None
        if rates is not None:
            r = [float(x) for x in rates if x is not None]
            if r:
                rate_mag = math.sqrt(sum(x * x for x in r))
        powered = True if flags is None else "servos_powered" in flags
        ev += self._pickup(t, powered)
        d = self._impulse(t)
        ev += self._startle(t, d, rate_mag)
        ev += self._tap(t, d)
        ev += self._pet(t)
        for e in ev:
            self._last[e.name] = e.t
        return ev

    # ---- helpers --------------------------------------------------------------
    def _window(self, t: float, seconds: float) -> list[tuple[float, float, float]]:
        return [s for s in self._buf if s[0] >= t - seconds - 1e-9]

    def _excursion(self, w: list[tuple[float, float, float]]) -> float:
        if len(w) < 2:
            return 0.0
        ps = [s[1] for s in w]
        rs = [s[2] for s in w]
        return max(max(ps) - min(ps), max(rs) - min(rs))

    def _cooldown_ok(self, name: str, t: float, cooldown: float) -> bool:
        last = self._last.get(name)
        return last is None or t - last >= cooldown

    # ---- detectors -------------------------------------------------------------
    def _learn_reference(self, t: float) -> None:
        c = self.cfg
        if self._ref_fixed or self._relearn_from is None or self.picked_up:
            return
        if t < max(self._quiet_from, self._relearn_from + c.settle_s) or self._self_moving:
            return
        w = self._window(t, c.settle_s)
        if not w or w[-1][0] - w[0][0] < c.settle_s * 0.8 - 1e-9 or self._excursion(w) >= c.calm_deg:
            return
        p = statistics.fmean(s[1] for s in w)
        r = statistics.fmean(s[2] for s in w)
        if max(abs(p), abs(r)) <= c.max_ref_deg:
            self.ref = (p, r)
        self._relearn_from = None

    def _tilt(self, t: float) -> list[PerceptEvent]:
        c = self.cfg
        dp = self.pitch - self.ref[0]
        dr = self.roll - self.ref[1]
        over = max(abs(dp), abs(dr))
        want = over < c.tilt_exit_deg if self.tilted else over > c.tilt_deg
        if not want:
            self._tilt_since = None
            if self.tilted:
                self.tilt_direction = _direction(dp, dr)
            return []
        if self._tilt_since is None:
            self._tilt_since = t
            return []
        if t - self._tilt_since < c.tilt_confirm_s:
            return []
        self._tilt_since = None
        self.tilted = not self.tilted
        data = {"pitch": round(self.pitch, 1), "roll": round(self.roll, 1)}
        if self.tilted:
            self.tilt_direction = _direction(dp, dr)
            return [PerceptEvent(t, "tilted", {"direction": self.tilt_direction, **data})]
        self.tilt_direction = None
        return [PerceptEvent(t, "level", data)]

    def _pickup(self, t: float, powered: bool) -> list[PerceptEvent]:
        c = self.cfg
        if not self.picked_up:
            thr = c.pickup_deg if powered else c.pickup_unpowered_deg
            w = self._window(t, c.pickup_window_s)
            if not self.active or len(w) < c.pickup_min_samples or self._excursion(w) < thr:
                self._pick_since = None
                return []
            if self._pick_since is None:
                self._pick_since = t
            if t - self._pick_since < c.pickup_confirm_s:
                return []
            self.picked_up = True
            self._pick_since = None
            self._tap_state = "idle"
            return [PerceptEvent(t, "picked_up", {"pitch": round(self.pitch, 1),
                                                  "roll": round(self.roll, 1),
                                                  "servos_powered": powered})]
        w = self._window(t, c.put_down_calm_s)
        span_ok = len(w) >= 2 and w[-1][0] - w[0][0] >= c.put_down_calm_s * 0.9
        near_ref = max(abs(self.pitch - self.ref[0]), abs(self.roll - self.ref[1])) < c.tilt_deg
        if span_ok and near_ref and self._excursion(w) < c.calm_deg:
            self.picked_up = False
            if not self._ref_fixed:
                self._relearn_from = t
            return [PerceptEvent(t, "put_down", {"pitch": round(self.pitch, 1),
                                                 "roll": round(self.roll, 1)})]
        return []

    def _impulse(self, t: float) -> float | None:
        """Deviation from the median of [t - pre_window - gap, t - gap], or None."""
        c = self.cfg
        pre = [s for s in self._buf if t - c.pre_gap_s - c.pre_window_s <= s[0] <= t - c.pre_gap_s]
        if len(pre) < 5:
            self._d.clear()
            return None
        pp = statistics.median(s[1] for s in pre)
        pr = statistics.median(s[2] for s in pre)
        d = max(abs(self.pitch - pp), abs(self.roll - pr))
        self._pre_quiet = max(max(s[1] for s in pre) - min(s[1] for s in pre),
                              max(s[2] for s in pre) - min(s[2] for s in pre)) < c.tap_quiet_deg
        self._d.append((t, d))
        while self._d and t - self._d[0][0] > max(c.tap_rise_s, c.startle_rise_s) + 0.05:
            self._d.popleft()
        return d

    def _rose_from(self, t: float, below: float, within: float) -> bool:
        return any(dt < below for tt, dt in self._d if t - within - 1e-9 <= tt < t)

    def _startle(self, t: float, d: float | None, rate: float | None) -> list[PerceptEvent]:
        c = self.cfg
        if not self.active or not self._cooldown_ok("startle", t, c.startle_cooldown_s):
            return []
        by_angle = d is not None and d >= c.startle_deg and self._rose_from(t, c.calm_deg, c.startle_rise_s)
        by_rate = rate is not None and rate >= c.startle_rate_dps
        if not (by_angle or by_rate):
            return []
        return [PerceptEvent(t, "startle", {"delta_deg": None if d is None else round(d, 1),
                                            "rate_dps": None if rate is None else round(rate, 1)})]

    def _tap(self, t: float, d: float | None) -> list[PerceptEvent]:
        c = self.cfg
        if d is None or not self.active or self.picked_up:
            self._tap_state = "idle"
            return []
        if self._tap_state == "idle":
            if d >= c.tap_min_deg and self._pre_quiet and self._rose_from(t, c.tap_min_deg / 2, c.tap_rise_s):
                self._tap_state, self._tap_t0, self._tap_peak = "cand", t, d
            elif d >= c.tap_min_deg:
                self._tap_state = "busy"
            return []
        if self._tap_state == "busy":
            if d <= c.tap_return_deg:
                self._tap_state = "idle"
            return []
        # candidate
        self._tap_peak = max(self._tap_peak, d)
        if d <= c.tap_return_deg:
            self._tap_state = "idle"
            if self._tap_peak < c.startle_deg and self._cooldown_ok("tap", t, c.tap_cooldown_s):
                return [PerceptEvent(t, "tap", {"onset": round(self._tap_t0, 3),
                                                "peak_deg": round(self._tap_peak, 1),
                                                "duration_s": round(t - self._tap_t0, 3)})]
            return []
        if t - self._tap_t0 > c.tap_max_s:
            self._tap_state = "busy"
        return []

    def _pet(self, t: float) -> list[PerceptEvent]:
        c = self.cfg
        if not self.active or self.picked_up:
            return []
        w = self._window(t, c.pet_window_s)
        if len(w) < 20 or w[-1][0] - w[0][0] < c.pet_window_s * 0.9:
            return []
        ts = [s[0] for s in w]
        best = None
        for k in (1, 2):
            x = _smooth3([s[k] for s in w])
            pp = max(x) - min(x)
            if best is None or pp > best[0]:
                best = (pp, x)
        pp, x = best
        if not (c.pet_min_pp <= pp <= c.pet_max_pp):
            return []
        rate = max(abs(x[i + 1] - x[i]) / max(ts[i + 1] - ts[i], 1e-3) for i in range(len(x) - 1))
        if rate > c.pet_max_rate_dps:
            return []
        m = statistics.fmean(x)
        band = max(c.pet_min_band, c.pet_band_frac * pp)
        side = 0
        cross: list[float] = []
        for ti, xi in zip(ts, x):
            s = 1 if xi > m + band else -1 if xi < m - band else 0
            if s and s != side:
                if side:
                    cross.append(ti)
                side = s
        if len(cross) < c.pet_min_half_cycles:
            return []
        halves = [b - a for a, b in zip(cross, cross[1:])]
        lo, hi = c.pet_half_period_s
        if halves and not (lo <= statistics.median(halves) <= hi):
            return []
        self._pet_seen = t
        if not self._cooldown_ok("pet", t, c.pet_cooldown_s):
            return []
        return [PerceptEvent(t, "pet", {"pp_deg": round(pp, 1), "strokes": len(cross) // 2})]

    def state(self) -> dict[str, Any]:
        return {
            "tilted": self.tilted,
            "tilt_direction": self.tilt_direction,
            "picked_up": self.picked_up,
            "pet": self.pet,
            "reference": (round(self.ref[0], 2), round(self.ref[1], 2)),
            "imu_active": self.active,
        }
