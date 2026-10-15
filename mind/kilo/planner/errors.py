"""The model's error, from what the body reports every tick while it walks.

In GAIT the body's TELEMETRY carries the attitude it measured, the
reference it balanced toward (the planner's prediction for that tick, from
FEET) and the correction it applied. Their difference is the model's
error (docs/walking-and-balance.md §9 item 4, §10):

- a **steady offset** (the mean of measured - predicted): a mass or the
  IMU placed differently from the model; learn it once;
- a **lag** (the shift that best lines the predicted roll up with the
  measured one, by cross-correlation over the last seconds): how much
  slower the servos are than the model's servo lag (+) or faster (-); it
  drifts with the battery voltage;
- a **spike** (the largest error once the offset is taken out): a bump, a
  slope, a slipped foot.

No model parameter is adjusted from these yet; they are reported
(``kilo walk``, ``kilo status``, the daemon's walk status) so that they can
be compared with the scale and servo measurements.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Iterable

A16 = 16.0
TICK_MS = 20.0
WINDOW_TICKS = 200       # 4 s of 50 Hz telemetry
MIN_SAMPLES = 50
MAX_LAG_TICKS = 10       # +-200 ms around the model's lag
MIN_SWING_DEG = 1.0      # the predicted roll must move this much (sd) for a lag estimate


class ErrorTracker:
    """Error samples from TELEMETRY in GAIT (``add``), estimates on demand."""

    def __init__(self, window: int = WINDOW_TICKS, offset_s: float = 2.0):
        self.samples: deque = deque(maxlen=window)  # (tick, meas p, meas r, ref p, ref r, corr p, corr r)
        self.offset = (0.0, 0.0)  # a running mean of measured - predicted (pitch, roll)
        self._alpha = TICK_MS / 1000.0 / offset_s
        self._n = 0

    def add(self, tick: int, msg: Any) -> tuple[float, float]:
        """One TELEMETRY in GAIT (``tick`` unwrapped). Returns its error less the offset."""
        mp, mr = msg.pitch / A16, msg.roll / A16
        rp, rr = msg.ref_pitch / A16, msg.ref_roll / A16
        self.samples.append((tick, mp, mr, rp, rr, msg.corr_pitch / A16, msg.corr_roll / A16))
        ep, er = mp - rp, mr - rr
        self._n += 1
        a = max(self._alpha, 1.0 / self._n)
        op, orr = self.offset
        self.offset = (op + (ep - op) * a, orr + (er - orr) * a)
        return ep - self.offset[0], er - self.offset[1]

    def estimate(self) -> dict[str, Any] | None:
        return estimate(self.samples)


def estimate(samples: Iterable[tuple]) -> dict[str, Any] | None:
    """Offset, lag and spike from (tick, meas p, meas r, ref p, ref r, corr
    p, corr r) samples; None with too few."""
    s = list(samples)
    n = len(s)
    if n < MIN_SAMPLES:
        return None
    off_p = sum(x[1] - x[3] for x in s) / n
    off_r = sum(x[2] - x[4] for x in s) / n
    spike = max(max(abs(x[1] - x[3] - off_p), abs(x[2] - x[4] - off_r)) for x in s)
    corr_p = sum(x[5] for x in s) / n
    corr_r = sum(x[6] for x in s) / n
    lag_ms, quality = _lag([(x[0], x[4]) for x in s], {x[0]: x[2] for x in s})
    return {"samples": n, "offset_pitch_deg": round(off_p, 2), "offset_roll_deg": round(off_r, 2),
            "lag_ms": lag_ms, "lag_quality": quality, "spike_deg": round(spike, 2),
            "corr_pitch_deg": round(corr_p, 2), "corr_roll_deg": round(corr_r, 2)}


def _lag(ref: list, meas: dict) -> tuple[float | None, float | None]:
    """The shift k (ticks) maximising the correlation of ref[t - k] with
    meas[t]: + = the body is later than the prediction."""
    vals = [v for _, v in ref]
    m_ref = sum(vals) / len(vals)
    sd = math.sqrt(sum((v - m_ref) ** 2 for v in vals) / len(vals))
    if sd < MIN_SWING_DEG:
        return None, None
    mv = list(meas.values())
    m_meas = sum(mv) / len(mv)
    sd_meas = math.sqrt(sum((v - m_meas) ** 2 for v in mv) / len(mv)) or 1e-9
    best_k, best_c = None, -2.0
    for k in range(-MAX_LAG_TICKS, MAX_LAG_TICKS + 1):
        acc, cnt = 0.0, 0
        for t, v in ref:
            w = meas.get(t + k)
            if w is not None:
                acc += (v - m_ref) * (w - m_meas)
                cnt += 1
        if cnt < MIN_SAMPLES // 2:
            continue
        c = acc / cnt / (sd * sd_meas)
        if c > best_c:
            best_k, best_c = k, c
    if best_k is None:
        return None, None
    return best_k * TICK_MS, round(best_c, 2)


def from_history(history: Iterable[tuple[float, Any]], gait_mode: int) -> dict[str, Any] | None:
    """Estimate from BodyState.history ((time, TELEMETRY) pairs): the newest
    unbroken run in GAIT (``kilo status``)."""
    run: list = []
    last_raw = None
    tick = 0
    for _, m in history:
        if m.mode != gait_mode:
            run, last_raw = [], None
            continue
        if last_raw is not None:
            tick += (m.tick - last_raw) & 0xFFFF
        last_raw = m.tick
        run.append((tick, m.pitch / A16, m.roll / A16, m.ref_pitch / A16, m.ref_roll / A16,
                    m.corr_pitch / A16, m.corr_roll / A16))
    return estimate(run[-WINDOW_TICKS:])
