"""Percepts: the perception aggregator the behaviour layer reads (§5 "Perception").

``Percepts.update(snapshot, t)`` takes one telemetry sample in any of these
duck-typed shapes:

- a ``BodyState`` (anything with a ``telemetry`` attribute): its latest raw
  TELEMETRY is used, so the ultrasonic detector sees raw echoes and dropouts;
- a decoded TELEMETRY message (attributes ``tick``, ``pitch`` in 1/16°...) or
  a dict of its raw fields (as in the JSONL log's ``rx`` records);
- a ``BodyState.snapshot()``-style dict (``pitch_deg``, ``roll_deg``,
  ``rates_dps``, ``dist_mm``, ``flags`` as names, ``mode`` as a name), or any
  object with those attributes (``kilo.logs`` samples).

``t`` defaults to the sample's own time (``t`` / ``telemetry_at``), else
``time.monotonic()``. It returns the percept dict (``as_dict()``), with the
events produced by this update under ``events``. Every sample also goes into
a 10 s ``TelemetryRing`` (numpy when available) for detectors that want a
window, e.g. ``percepts.ring.window(2.0)["pitch"]``.

The ultrasonic detector is fed once per fresh range: only samples with the
``dist_fresh`` flag once the body has been seen to set it, every sample
otherwise.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from typing import Any

from kilo.link import consts
from kilo.perception.events import PerceptEvent
from kilo.perception.imu import ImuConfig, ImuDetector
from kilo.perception.ring import TelemetryRing
from kilo.perception.ultrasonic import UltrasonicConfig, UltrasonicDetector

A16 = 16.0
_MODE_CODES = {v: k for k, v in consts.MODES.items()}
_FLAG_MASKS = {name: 1 << bit for bit, name in consts.FLAG_BITS.items()}


@dataclass
class Sample:
    t: float | None
    pitch: float | None
    roll: float | None
    rates: tuple[float, float, float] | None
    dist_mm: float | None  # None = no echo / unknown
    dist_fresh: bool | None  # None = unknown
    batt_mv: float | None
    mode: str | None
    flags: set[str] | None


def _flag_names(v: Any) -> set[str] | None:
    if v is None:
        return None
    if isinstance(v, int):
        return {n for n, m in _FLAG_MASKS.items() if v & m}
    return {str(x).lower() for x in v}


def _mode_name(v: Any) -> str | None:
    if v is None or isinstance(v, str):
        return v
    return consts.MODES.get(int(v), f"?{v}")


def _get(obj: Any, name: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _from_raw(m: Any, t: float | None) -> Sample:
    """A TELEMETRY message (object or dict of raw fields)."""
    flags = _get(m, "flags")
    names = _flag_names(flags)
    dist = _get(m, "dist_mm")
    rates = None
    if _get(m, "rate_pitch") is not None:
        rates = tuple(_get(m, k) / A16 for k in ("rate_pitch", "rate_roll", "rate_yaw"))
    return Sample(
        t=t,
        pitch=_get(m, "pitch") / A16,
        roll=_get(m, "roll") / A16,
        rates=rates,
        dist_mm=float(dist) if dist else None,
        dist_fresh=None if names is None else "dist_fresh" in names,
        batt_mv=_get(m, "batt_mv"),
        mode=_mode_name(_get(m, "mode")),
        flags=names,
    )


def extract(snapshot: Any, t: float | None = None) -> Sample | None:
    """Normalise a duck-typed telemetry sample; None when it has no attitude."""
    if snapshot is None:
        return None
    if not isinstance(snapshot, dict) and hasattr(snapshot, "telemetry") and hasattr(snapshot, "lock"):
        with snapshot.lock:
            tel = snapshot.telemetry
            at = snapshot.telemetry_at
        if tel is None:
            return None
        return _from_raw(tel, at if t is None else t)
    if _get(snapshot, "pitch_deg") is None and _get(snapshot, "pitch") is not None \
            and _get(snapshot, "tick") is not None:
        return _from_raw(snapshot, _get(snapshot, "t") if t is None else t)
    pitch = _get(snapshot, "pitch_deg")
    roll = _get(snapshot, "roll_deg")
    if pitch is None or roll is None:
        return None
    flags = _flag_names(_get(snapshot, "flags"))
    rates = _get(snapshot, "rates_dps")
    if rates is None and _get(snapshot, "rate_pitch_dps") is not None:
        rates = (_get(snapshot, "rate_pitch_dps"), _get(snapshot, "rate_roll_dps"),
                 _get(snapshot, "rate_yaw_dps"))
    if rates is not None and any(x is None for x in rates):
        rates = None
    dist = _get(snapshot, "dist_raw_mm", _get(snapshot, "dist_mm"))
    fresh = _get(snapshot, "dist_fresh")
    if fresh is None and flags is not None:
        fresh = "dist_fresh" in flags
    return Sample(
        t=_get(snapshot, "t") if t is None else t,
        pitch=float(pitch), roll=float(roll),
        rates=None if rates is None else tuple(float(x) for x in rates),
        dist_mm=float(dist) if dist else None,
        dist_fresh=fresh,
        batt_mv=_get(snapshot, "batt_mv"),
        mode=_mode_name(_get(snapshot, "mode")),
        flags=flags,
    )


class Percepts:
    def __init__(self, ultrasonic: UltrasonicConfig | None = None, imu: ImuConfig | None = None,
                 ring_seconds: float = 10.0, rate_hz: float = 25.0, event_hold_s: float = 1.0,
                 history: int = 256, clock=time.monotonic):
        self.us = UltrasonicDetector(ultrasonic)
        self.imu = ImuDetector(imu)
        self.ring = TelemetryRing(ring_seconds, rate_hz)
        self.event_hold_s = event_hold_s
        self.events: deque[PerceptEvent] = deque(maxlen=history)
        self.last_event_times: dict[str, float] = {}
        self.clock = clock
        self.t: float | None = None
        self.sample: Sample | None = None
        self._seen_fresh = False

    def reset(self) -> None:
        self.us.reset()
        self.imu.reset()
        self.ring.clear()
        self.events.clear()
        self.last_event_times.clear()
        self.t = None
        self._seen_fresh = False

    def notify_self_motion(self, t: float | None = None, seconds: float = 0.0) -> None:
        """The mind started a posture move or similar: IMU detectors stand down."""
        self.imu.notify_self_motion(self.clock() if t is None else t, seconds)

    def update(self, snapshot: Any, t: float | None = None) -> dict[str, Any]:
        s = extract(snapshot, t)
        if s is None:
            return self.as_dict([])
        now = s.t if s.t is not None else self.clock()
        if self.t is not None and now <= self.t and self.sample is not None and s == self.sample:
            return self.as_dict([])  # same telemetry polled twice
        self.t, self.sample = now, s
        self.ring.append(
            t=now, pitch=s.pitch, roll=s.roll,
            rate_pitch=None if s.rates is None else s.rates[0],
            rate_roll=None if s.rates is None else s.rates[1],
            rate_yaw=None if s.rates is None else s.rates[2],
            dist_mm=s.dist_mm, batt_mv=s.batt_mv,
            mode=_MODE_CODES.get(s.mode) if s.mode else None,
            flags=None if s.flags is None else sum(_FLAG_MASKS.get(f, 0) for f in s.flags),
        )
        new: list[PerceptEvent] = []
        if s.dist_fresh:
            self._seen_fresh = True
        if s.dist_fresh or not self._seen_fresh:
            new += self.us.feed(now, s.dist_mm)
        new += self.imu.feed(now, s.pitch, s.roll, s.rates, s.flags, s.mode)
        for e in new:
            self.events.append(e)
            self.last_event_times[e.name] = e.t
        return self.as_dict(new)

    def _recent(self, name: str) -> bool:
        last = self.last_event_times.get(name)
        return last is not None and self.t is not None and self.t - last <= self.event_hold_s

    def as_dict(self, new: list[PerceptEvent] | None = None) -> dict[str, Any]:
        us, imu = self.us, self.imu
        return {
            "t": self.t,
            "near": us.near,
            "approaching": us.approaching,
            "clear": us.clear,
            "distance_mm": us.distance_mm,
            "approach_mm_s": us.approach_mm_s,
            "range_rate_mm_s": us.range_rate_mm_s,
            "pitch_deg": imu.pitch,
            "roll_deg": imu.roll,
            "tilted": imu.tilted,
            "tilt_direction": imu.tilt_direction,
            "picked_up": imu.picked_up,
            "tap": self._recent("tap"),
            "pet": imu.pet,
            "startle": self._recent("startle"),
            "last_event_times": dict(self.last_event_times),
            "events": list(new or []),
        }
