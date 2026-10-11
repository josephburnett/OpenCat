"""BodyState: the mind's view of the body (docs/architecture.md §5 "Body state").

Updated by SerialLink from its reader thread under ``state.lock``; readers on
other threads should take ``snapshot()`` or hold the lock while reading
several fields. Holds:

- the latest TELEMETRY, with the u16 tick unwrapped;
- link health: transport connected, telemetry alive (300 ms), counters;
- derived values: upright, moving, fallen, filtered distance with approach
  speed, battery percentage from a resting 2S curve;
- owner/mode/flags decoded; the goal epoch;
- the last HELLO (versions, reset cause), reboot count, last events;
- a 10 s ring buffer of telemetry for detectors (a deque; the numpy ring
  buffer in §5 arrives with the detectors in P3, numpy is not a P0 dependency).
"""

from __future__ import annotations

import statistics
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from typing import Any

from kilo.link import consts
from kilo.link.messages import fields_of

A16 = 16.0  # a16: 1/16 degree
TELEMETRY_TIMEOUT_S = 0.300
UPRIGHT_DEG = 30.0

# Resting 2S Li-ion curve, pack mV -> percent. (est.) Replace with a measured
# curve for Kilo's ZCF503060 pack.
BATTERY_CURVE: tuple[tuple[int, float], ...] = (
    (6600, 0.0), (7000, 5.0), (7200, 12.0), (7300, 20.0), (7400, 30.0),
    (7500, 40.0), (7600, 50.0), (7800, 65.0), (8000, 80.0), (8200, 90.0),
    (8400, 100.0),
)


def battery_percent(mv: int | None) -> float | None:
    if not mv:
        return None
    pts = BATTERY_CURVE
    if mv <= pts[0][0]:
        return pts[0][1]
    if mv >= pts[-1][0]:
        return pts[-1][1]
    for (v0, p0), (v1, p1) in zip(pts, pts[1:]):
        if v0 <= mv <= v1:
            return p0 + (p1 - p0) * (mv - v0) / (v1 - v0)
    return None  # unreachable


def decode_flags(flags: int) -> set[str]:
    return {name for bit, name in consts.FLAG_BITS.items() if flags >> bit & 1}


@dataclass
class LinkStats:
    frames_rx: int = 0
    frames_tx: int = 0
    bytes_rx: int = 0
    bytes_tx: int = 0
    decode_errors: Counter = field(default_factory=Counter)
    undecodable: int = 0  # valid frame, layout unknown or wrong length
    heartbeats: int = 0
    goal_sends: int = 0
    commands: int = 0
    retries: int = 0
    timeouts: int = 0
    nacks: int = 0  # ACK with status != OK
    dup_acks: int = 0  # re-ACK of an already completed command (de-dup at work)
    unmatched_acks: int = 0
    tx_dropped: int = 0  # frames discarded while disconnected
    tx_silenced: int = 0  # frames not sent because the link is silent (passive)
    open_failures: int = 0
    reconnects: int = 0
    disconnects: int = 0
    goals_refused: Counter = field(default_factory=Counter)
    goals_cleared: int = 0
    hello_polls: int = 0

    def as_dict(self) -> dict[str, Any]:
        d = dict(self.__dict__)
        d["decode_errors"] = dict(self.decode_errors)
        d["goals_refused"] = dict(self.goals_refused)
        return d


class BodyState:
    def __init__(self, clock=time.monotonic) -> None:
        self.lock = threading.RLock()
        self.clock = clock
        # connection
        self.port: str | None = None
        self.connected = False
        self.connected_since: float | None = None
        self.last_rx_at: float | None = None
        self.stats = LinkStats()
        # telemetry
        self.telemetry: Any = None
        self.telemetry_at: float | None = None
        self.tick: int | None = None  # unwrapped
        self._tick_raw: int | None = None
        self.history: deque[tuple[float, Any]] = deque(maxlen=250)  # 10 s at 25 Hz
        self._dist: deque[int] = deque(maxlen=3)
        self._dist_hist: deque[tuple[float, float]] = deque(maxlen=8)
        self.epoch: int | None = None
        # boot / identity
        self.hello: Any = None
        self.hello_at: float | None = None
        self.reboots = 0
        self.proto_ok: bool | None = None
        # events and replies
        self.events: deque[tuple[float, Any]] = deque(maxlen=64)
        self.last_stats: Any = None

    # ---- updates (called by the link with the lock held) -------------------
    def on_telemetry(self, msg: Any, now: float) -> tuple[bool, int | None]:
        """Record telemetry. Returns (tick_went_backwards, previous_epoch)."""
        backwards = False
        raw = msg.tick & 0xFFFF
        if self._tick_raw is None or self.tick is None:
            self.tick = raw
        else:
            d = (raw - self._tick_raw) & 0xFFFF
            if d >= 0x8000:  # more than half a wrap backwards: the body restarted
                backwards = True
                self.tick = raw
            else:
                self.tick += d
        self._tick_raw = raw
        self.telemetry = msg
        self.telemetry_at = now
        self.history.append((now, msg))
        if msg.dist_mm:
            self._dist.append(msg.dist_mm)
            self._dist_hist.append((now, float(statistics.median(self._dist))))
        prev_epoch = self.epoch
        self.epoch = msg.epoch
        return backwards, prev_epoch

    def on_hello(self, msg: Any, now: float, expected_major: int | None) -> None:
        self.hello = msg
        self.hello_at = now
        self.epoch = msg.epoch
        self.proto_ok = None if expected_major is None else msg.proto_major == expected_major

    def reset_session(self) -> None:
        """Forget tick history after a body reboot."""
        self.tick = None
        self._tick_raw = None
        self._dist.clear()
        self._dist_hist.clear()

    # ---- derived ------------------------------------------------------------
    @property
    def alive(self) -> bool:
        t = self.telemetry_at
        return self.connected and t is not None and self.clock() - t < TELEMETRY_TIMEOUT_S

    def _tf(self, name: str) -> Any:
        return getattr(self.telemetry, name, None) if self.telemetry is not None else None

    @property
    def pitch_deg(self) -> float | None:
        v = self._tf("pitch")
        return None if v is None else v / A16

    @property
    def roll_deg(self) -> float | None:
        v = self._tf("roll")
        return None if v is None else v / A16

    @property
    def mode(self) -> str | None:
        v = self._tf("mode")
        return None if v is None else consts.MODES.get(v, f"?{v}")

    @property
    def flags(self) -> set[str]:
        v = self._tf("flags")
        return set() if v is None else decode_flags(v)

    @property
    def owners(self) -> tuple[str, str] | None:
        v = self._tf("owners")
        if v is None:
            return None
        return (consts.OWNERS.get(v & 0xF, str(v & 0xF)), consts.OWNERS.get(v >> 4, str(v >> 4)))

    @property
    def batt_mv(self) -> int | None:
        return self._tf("batt_mv")

    @property
    def battery_pct(self) -> float | None:
        if "no_battery" in self.flags:
            return None
        return battery_percent(self.batt_mv)

    @property
    def fallen(self) -> bool:
        return self.mode in ("FALLEN", "RECOVERING")

    @property
    def upright(self) -> bool | None:
        p, r = self.pitch_deg, self.roll_deg
        if p is None or r is None:
            return None
        return abs(p) < UPRIGHT_DEG and abs(r) < UPRIGHT_DEG and not self.fallen

    @property
    def moving(self) -> bool:
        return bool(self.flags & {"gait", "trick"}) or self.mode in ("GAIT", "TRICK", "RECOVERING")

    @property
    def distance_mm(self) -> float | None:
        """Median of the last 3 echoes (0 = no echo is skipped)."""
        return float(statistics.median(self._dist)) if self._dist else None

    @property
    def approach_mm_s(self) -> float | None:
        """Positive when something is getting closer."""
        h = self._dist_hist
        if len(h) < 2 or h[-1][0] - h[0][0] <= 0:
            return None
        return (h[0][1] - h[-1][1]) / (h[-1][0] - h[0][0])

    @property
    def reset_cause(self) -> str | None:
        if self.hello is None:
            return None
        return consts.RESET_CAUSES.get(self.hello.reset_cause, str(self.hello.reset_cause))

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            now = self.clock()
            tel = self.telemetry
            return {
                "port": self.port,
                "connected": self.connected,
                "alive": self.alive,
                "telemetry_age_s": None if self.telemetry_at is None else now - self.telemetry_at,
                "tick": self.tick,
                "epoch": self.epoch,
                "mode": self.mode,
                "owners": self.owners,
                "flags": sorted(self.flags),
                "balance": "balance_active" in self.flags,
                "pitch_deg": self.pitch_deg,
                "roll_deg": self.roll_deg,
                "rates_dps": None if tel is None else tuple(
                    getattr(tel, k) / A16 for k in ("rate_pitch", "rate_roll", "rate_yaw")),
                "dist_mm": self.distance_mm,
                "approach_mm_s": self.approach_mm_s,
                "batt_mv": self.batt_mv,
                "battery_pct": self.battery_pct,
                "upright": self.upright,
                "fallen": self.fallen,
                "moving": self.moving,
                "tick_load_ms": None if tel is None else tel.tick_load / 10.0,
                "rx_err": None if tel is None else tel.rx_err,
                "last_rx_seq": None if tel is None else tel.last_rx_seq,
                "hello": None if self.hello is None else _fields(self.hello),
                "reset_cause": self.reset_cause,
                "reboots": self.reboots,
                "proto_ok": self.proto_ok,
                "events": [(round(now - t, 2), _fields(e)) for t, e in list(self.events)[-8:]],
                "stats": self.stats.as_dict(),
            }


def _fields(m: Any) -> dict[str, Any]:
    if hasattr(m, "__dataclass_fields__"):
        return fields_of(m)
    return dict(vars(m))
