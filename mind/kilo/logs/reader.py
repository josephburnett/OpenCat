"""Read Kilo logs into telemetry samples and events.

Formats, detected from the content:

- **link JSONL** (``kilo --log`` / ``--log-file``, written by ``kilo.log``):
  one JSON object per line with ``t`` (monotonic s), ``wall``, ``kind``.
  ``rx``/``tx`` records carry ``type``, ``seq``, ``payload`` (hex) and, for rx,
  the decoded raw ``fields`` (pitch in 1/16°...); also ``event``, ``link``,
  ``goal``, ``cmd``, ``rx_error`` and ``log_dropped`` records. TELEMETRY is
  taken from ``fields``, else decoded from ``payload`` through the codec.
- **monitor JSONL** (``kilo monitor`` with stdout not a tty, 1 Hz): objects
  with ``tick``, ``mode``, ``pitch_deg``, ``roll_deg``, ``batt_mv``... and no
  ``kind``. Time is ``t`` if present, else ``tick / TICK_HZ``.
- **.kilolog** raw frames (``kilo.logs.kilolog``), optionally gzipped.

Times in the result are seconds relative to the first record.
"""

from __future__ import annotations

import gzip
import json
import os
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from kilo.link import consts
from kilo.logs import kilolog
from kilo.protocol import kilo_protocol as P

A16 = 16.0
_FLAG_MASKS = {name: 1 << bit for bit, name in consts.FLAG_BITS.items()}


@dataclass
class TelemetrySample:
    t: float
    tick: int | None = None
    pitch_deg: float | None = None
    roll_deg: float | None = None
    rate_pitch_dps: float | None = None
    rate_roll_dps: float | None = None
    rate_yaw_dps: float | None = None
    dist_mm: float | None = None  # raw echo; None = no echo / not logged
    batt_mv: int | None = None
    mode: str | None = None
    flags: set[str] | None = None
    owners: int | None = None
    epoch: int | None = None
    tick_load_ms: float | None = None
    rx_err: int | None = None
    gait_phase: int | None = None
    trick_frame: int | None = None
    balance: bool | None = None

    def get(self, name: str) -> Any:
        return getattr(self, name, None)


@dataclass
class LogEvent:
    t: float
    name: str
    arg: int | None = None
    tick: int | None = None
    epoch: int | None = None


@dataclass
class LogData:
    path: Path
    format: str  # jsonl-link, jsonl-monitor, kilolog
    meta: dict[str, Any] = field(default_factory=dict)
    samples: list[TelemetrySample] = field(default_factory=list)
    events: list[LogEvent] = field(default_factory=list)
    notes: list[tuple[float, str, dict[str, Any]]] = field(default_factory=list)  # link/goal/cmd
    kinds: Counter = field(default_factory=Counter)
    rx_types: Counter = field(default_factory=Counter)
    tx_types: Counter = field(default_factory=Counter)
    rx_errors: Counter = field(default_factory=Counter)
    bad_lines: int = 0
    dropped: int = 0
    t0: float = 0.0  # absolute time of the first record

    @property
    def duration_s(self) -> float:
        ts = [s.t for s in self.samples] + [e.t for e in self.events]
        return max(ts) - min(ts) if ts else 0.0


def _flag_names(v: int) -> set[str]:
    return {n for n, m in _FLAG_MASKS.items() if v & m}


def sample_from_fields(t: float, f: dict[str, Any] | Any) -> TelemetrySample:
    """Raw TELEMETRY fields (dict or decoded dataclass) -> TelemetrySample."""
    g = f.get if isinstance(f, dict) else (lambda k, d=None: getattr(f, k, d))
    flags = g("flags")
    dist = g("dist_mm")
    names = None if flags is None else _flag_names(int(flags))
    return TelemetrySample(
        t=t, tick=g("tick"),
        pitch_deg=g("pitch") / A16, roll_deg=g("roll") / A16,
        rate_pitch_dps=None if g("rate_pitch") is None else g("rate_pitch") / A16,
        rate_roll_dps=None if g("rate_roll") is None else g("rate_roll") / A16,
        rate_yaw_dps=None if g("rate_yaw") is None else g("rate_yaw") / A16,
        dist_mm=float(dist) if dist else None,
        batt_mv=g("batt_mv"),
        mode=None if g("mode") is None else consts.MODES.get(int(g("mode")), f"?{g('mode')}"),
        flags=names, owners=g("owners"), epoch=g("epoch"),
        tick_load_ms=None if g("tick_load") is None else g("tick_load") / 10.0,
        rx_err=g("rx_err"), gait_phase=g("gait_phase"), trick_frame=g("trick_frame"),
        balance=None if names is None else "balance_active" in names,
    )


def _decode(type_name: str, payload_hex: str | None) -> Any:
    if not payload_hex:
        return None
    try:
        return P.decode_payload(int(P.MsgId[type_name]), bytes.fromhex(payload_hex))
    except (KeyError, ValueError, P.PayloadError):
        return None


def _open_text(path: Path):
    if path.suffix == ".gz":
        return gzip.open(path, "rt", encoding="utf-8", errors="replace")
    return open(path, encoding="utf-8", errors="replace")


def _read_jsonl(path: Path) -> LogData:
    data = LogData(path, "jsonl-link")
    rx_events: list[LogEvent] = []
    first_t: float | None = None
    monitor = 0
    tick_base: int | None = None
    with _open_text(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                data.bad_lines += 1
                continue
            if not isinstance(rec, dict):
                data.bad_lines += 1
                continue
            kind = rec.get("kind")
            if kind is None and "pitch_deg" in rec:  # monitor line
                monitor += 1
                t = rec.get("t")
                if t is None:
                    tick = rec.get("tick")
                    if tick is None:
                        t = float(monitor - 1)
                    else:
                        tick_base = tick if tick_base is None else tick_base
                        t = (tick - tick_base) / P.TICK_HZ
                if first_t is None:
                    first_t = t
                data.kinds["monitor"] += 1
                if rec.get("pitch_deg") is None or rec.get("roll_deg") is None:
                    continue
                flags = rec.get("flags")
                rates = rec.get("rates_dps") or (None, None, None)
                data.samples.append(TelemetrySample(
                    t=t, tick=rec.get("tick"), pitch_deg=rec["pitch_deg"], roll_deg=rec["roll_deg"],
                    rate_pitch_dps=rates[0], rate_roll_dps=rates[1], rate_yaw_dps=rates[2],
                    dist_mm=rec.get("dist_mm") or None, batt_mv=rec.get("batt_mv"), mode=rec.get("mode"),
                    flags=None if flags is None else set(flags), epoch=rec.get("epoch"),
                    balance=rec.get("balance")))
                continue
            t = rec.get("t")
            if t is None or kind is None:
                data.bad_lines += 1
                continue
            if first_t is None:
                first_t = t
            data.kinds[kind] += 1
            if kind == "rx":
                typ = rec.get("type")
                data.rx_types[typ] += 1
                fields = rec.get("fields") or None
                if typ == "TELEMETRY":
                    src = fields if fields else _decode(typ, rec.get("payload"))
                    if src is not None:
                        data.samples.append(sample_from_fields(t, src))
                elif typ == "EVENT":
                    src = fields if fields else _decode(typ, rec.get("payload"))
                    if src is not None:
                        g = src.get if isinstance(src, dict) else (lambda k: getattr(src, k, None))
                        rx_events.append(LogEvent(t, consts.EVENTS.get(g("code"), str(g("code"))),
                                                  g("arg"), g("tick"), g("epoch")))
            elif kind == "tx":
                data.tx_types[rec.get("type")] += 1
            elif kind == "event":
                data.events.append(LogEvent(t, str(rec.get("name")), rec.get("arg"), rec.get("tick"),
                                            rec.get("epoch")))
            elif kind == "rx_error":
                data.rx_errors[rec.get("error")] += 1
            elif kind == "log_dropped":
                data.dropped += int(rec.get("count", 0))
            elif kind in ("link", "goal", "cmd"):
                data.notes.append((t, kind, {k: v for k, v in rec.items() if k not in ("t", "wall", "kind")}))
    if monitor and not data.kinds.keys() - {"monitor"}:
        data.format = "jsonl-monitor"
    if not data.events:
        data.events = rx_events
    data.t0 = first_t or 0.0
    _rebase(data)
    return data


def _read_kilolog(path: Path) -> LogData:
    meta, recs = kilolog.read_kilolog(path)
    data = LogData(path, "kilolog", meta=meta)
    for r in recs:
        data.kinds[r.direction] += 1
        if r.frame is None:
            data.rx_errors[r.error] += 1
            continue
        try:
            name = P.MsgId(r.frame.type).name
        except ValueError:
            name = f"0x{r.frame.type:02X}"
        if r.direction == "tx":
            data.tx_types[name] += 1
            continue
        data.rx_types[name] += 1
        if name not in ("TELEMETRY", "EVENT"):
            continue
        try:
            msg = P.decode_payload(r.frame.type, r.frame.payload)
        except (KeyError, ValueError, P.PayloadError):
            data.rx_errors["undecodable"] += 1
            continue
        if name == "TELEMETRY":
            data.samples.append(sample_from_fields(r.t, msg))
        else:
            data.events.append(LogEvent(r.t, consts.EVENTS.get(msg.code, str(msg.code)), msg.arg,
                                        msg.tick, msg.epoch))
    data.t0 = float(meta.get("start_mono", 0.0))
    return data


def _rebase(data: LogData) -> None:
    t0 = data.t0
    for s in data.samples:
        s.t -= t0
    for e in data.events:
        e.t -= t0
    data.notes = [(t - t0, k, d) for t, k, d in data.notes]


def load(path: str | os.PathLike) -> LogData:
    """Read a log of any supported format."""
    p = Path(path)
    if kilolog.is_kilolog(p):
        return _read_kilolog(p)
    return _read_jsonl(p)


def fields_available(samples: Iterable[TelemetrySample]) -> list[str]:
    s = next(iter(samples), None)
    if s is None:
        return []
    return [k for k, v in vars(s).items() if k not in ("t", "flags") and v is not None]
