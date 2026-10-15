"""Offline analysis of Kilo logs: summary, perception replay, plots."""

from __future__ import annotations

import csv
import math
import statistics
from collections import Counter
from pathlib import Path
from typing import Any

from kilo.logs.reader import LogData, TelemetrySample
from kilo.perception import PerceptEvent, Percepts
from kilo.protocol import kilo_protocol as P

# --fields names -> TelemetrySample attributes
FIELD_ALIASES: dict[str, str] = {
    "pitch": "pitch_deg", "roll": "roll_deg",
    "rate_pitch": "rate_pitch_dps", "rate_roll": "rate_roll_dps", "rate_yaw": "rate_yaw_dps",
    "gyro_pitch": "rate_pitch_dps", "gyro_roll": "rate_roll_dps", "gyro_yaw": "rate_yaw_dps",
    "dist": "dist_mm", "distance": "dist_mm", "batt": "batt_mv", "battery": "batt_mv",
    "load": "tick_load_ms", "tick_load": "tick_load_ms",
    "ref_pitch": "ref_pitch_deg", "ref_roll": "ref_roll_deg", "corr_pitch": "corr_pitch_deg",
    "corr_roll": "corr_roll_deg",
}
NUMERIC = ("pitch_deg", "roll_deg", "rate_pitch_dps", "rate_roll_dps", "rate_yaw_dps", "dist_mm",
           "batt_mv", "tick_load_ms", "rx_err", "trick_frame", "tick", "epoch", "ref_pitch_deg", "ref_roll_deg",
           "corr_pitch_deg", "corr_roll_deg", "feet_queued", "support")
_MODE_CODES = {m.name: int(m) for m in P.Mode}


def _stats(xs: list[float]) -> dict[str, float] | None:
    if not xs:
        return None
    return {"min": min(xs), "max": max(xs), "mean": statistics.fmean(xs),
            "std": statistics.pstdev(xs) if len(xs) > 1 else 0.0}


def summary(data: LogData) -> dict[str, Any]:
    ss = data.samples
    out: dict[str, Any] = {
        "path": str(data.path), "format": data.format,
        "duration_s": round(data.duration_s, 3), "samples": len(ss),
    }
    if len(ss) > 1 and ss[-1].t > ss[0].t:
        out["telemetry_hz"] = round((len(ss) - 1) / (ss[-1].t - ss[0].t), 2)
    ticks = [s.tick for s in ss if s.tick is not None]
    if len(ticks) > 1:
        diffs = [(b - a) % 0x10000 for a, b in zip(ticks, ticks[1:])]
        step = statistics.median(diffs) or 1
        out["tick_step"] = step
        out["telemetry_gaps"] = sum(1 for d in diffs if d > step * 1.5)
        out["telemetry_missed"] = int(sum(round(d / step) - 1 for d in diffs if d > step * 1.5))
    for name in ("pitch_deg", "roll_deg", "rate_pitch_dps", "rate_roll_dps", "rate_yaw_dps", "tick_load_ms"):
        st = _stats([s.get(name) for s in ss if s.get(name) is not None])
        if st:
            out[name] = {k: round(v, 2) for k, v in st.items()}
    batt = [s.batt_mv for s in ss if s.batt_mv]
    if batt:
        out["battery_mv"] = {"first": batt[0], "last": batt[-1], "min": min(batt), "max": max(batt)}
    with_dist = [s for s in ss if s.flags is None or "dist_fresh" in s.flags]
    echoes = [s.dist_mm for s in with_dist if s.dist_mm]
    if with_dist and any(s.dist_mm is not None for s in ss):
        out["distance_mm"] = {"ranges": len(with_dist), "echo_fraction": round(len(echoes) / len(with_dist), 3),
                              "min": min(echoes) if echoes else None,
                              "median": statistics.median(echoes) if echoes else None}
    timeline: list[tuple[float, str]] = []
    time_in: Counter = Counter()
    for i, s in enumerate(ss):
        if s.mode is None:
            continue
        if not timeline or timeline[-1][1] != s.mode:
            timeline.append((round(s.t, 3), s.mode))
        if i + 1 < len(ss):
            time_in[s.mode] += ss[i + 1].t - s.t
    if timeline:
        out["modes"] = timeline
        out["time_in_mode_s"] = {k: round(v, 2) for k, v in time_in.items()}
    if any(s.flags for s in ss):
        fc: Counter = Counter(f for s in ss for f in (s.flags or ()))
        out["flags_fraction"] = {k: round(v / len(ss), 3) for k, v in sorted(fc.items())}
    gait = gait_summary(ss, data.events)
    if gait:
        out["gait"] = gait
    out["events"] = [(round(e.t, 3), e.name, e.arg) for e in data.events]
    out["event_counts"] = dict(Counter(e.name for e in data.events))
    if data.format != "jsonl-monitor":
        out["link"] = {"kinds": dict(data.kinds), "rx": dict(data.rx_types), "tx": dict(data.tx_types),
                       "rx_errors": dict(data.rx_errors), "log_dropped": data.dropped}
    if data.bad_lines:
        out["bad_lines"] = data.bad_lines
    return out


def _walking(s: TelemetrySample) -> bool:
    return s.mode == "GAIT" or "gait" in (s.flags or ())


def gait_summary(ss: list[TelemetrySample], events: list[Any]) -> dict[str, Any] | None:
    """Time walking (mode GAIT or the GAIT flag), GAIT_STOPPED (by arg: 0 a
    posture or an epoch bump, 1 settled after an underrun), FEET_UNDERRUN,
    FEET refusals, and the model's error over the walking samples
    (kilo.planner.errors: offset, lag, spike); None if the log has no gait."""
    from kilo.planner.errors import estimate

    stops = [e.arg for e in events if e.name == "GAIT_STOPPED"]
    underruns = sum(1 for e in events if e.name == "FEET_UNDERRUN")
    refused = [goal_refused_text(e.arg) for e in events if e.name == "GOAL_REFUSED" and e.arg is not None
               and (int(e.arg) & 0xFFFF) >> 8 == int(P.MsgId.FEET)]
    walking_s = 0.0
    for a, b in zip(ss, ss[1:]):
        if _walking(a):
            walking_s += b.t - a.t
    if not walking_s and not stops and not refused:
        return None
    errs = []
    tick, last = 0, None
    for i, s in enumerate(ss):
        if not (_walking(s) and s.ref_pitch_deg is not None and s.pitch_deg is not None):
            continue
        if s.tick is None:
            tick = i
        else:
            tick = s.tick if last is None else tick + ((s.tick - last) & 0xFFFF)
            last = s.tick
        errs.append((tick, s.pitch_deg, s.roll_deg, s.ref_pitch_deg, s.ref_roll_deg, s.corr_pitch_deg or 0.0,
                     s.corr_roll_deg or 0.0))
    return {"walking_s": round(walking_s, 2), "stops": len(stops), "stopped_underrun": sum(1 for a in stops if a == 1),
            "underruns": underruns, "refused": refused, "error": estimate(errs)}


def goal_refused_text(arg: Any) -> str:
    """EVENT GOAL_REFUSED arg -> 'FEET: STALE'."""
    from kilo.link import consts

    a = int(arg) & 0xFFFF
    try:
        name = P.MsgId(a >> 8).name
    except ValueError:
        name = f"0x{a >> 8:02X}"
    return f"{name}: {consts.REFUSE_REASON.get(a & 0xFF, a & 0xFF)}"


def _event_arg(name: str, arg: Any) -> str:
    if not arg:
        return ""
    return f"({goal_refused_text(arg)})" if name == "GOAL_REFUSED" else f"({arg})"


def format_summary(s: dict[str, Any]) -> str:
    L = [f"{s['path']} ({s['format']})",
         f"  duration {s['duration_s']:.1f} s, {s['samples']} telemetry samples"
         + (f" at {s['telemetry_hz']:.1f} Hz" if "telemetry_hz" in s else "")]
    if "telemetry_gaps" in s:
        L.append(f"  tick step {s['tick_step']}, {s['telemetry_gaps']} gaps, ≈{s['telemetry_missed']} missed")
    for name, label in (("pitch_deg", "pitch °"), ("roll_deg", "roll °"), ("rate_pitch_dps", "gyro pitch °/s"),
                        ("rate_roll_dps", "gyro roll °/s"), ("rate_yaw_dps", "gyro yaw °/s"),
                        ("tick_load_ms", "tick load ms")):
        st = s.get(name)
        if st:
            L.append(f"  {label:15s} min {st['min']:8.2f} max {st['max']:8.2f} "
                     f"mean {st['mean']:8.2f} std {st['std']:6.2f}")
    b = s.get("battery_mv")
    if b:
        L.append(f"  battery mV      first {b['first']} last {b['last']} min {b['min']} max {b['max']}")
    d = s.get("distance_mm")
    if d:
        L.append(f"  distance        {d['ranges']} ranges, echo {d['echo_fraction']:.0%}, "
                 f"min {d['min']} median {d['median']}")
    if s.get("modes"):
        L.append("  modes: " + ", ".join(f"{t:.1f}s {m}" for t, m in s["modes"]))
        L.append("  time in mode: " + ", ".join(f"{m} {v:.1f}s" for m, v in s["time_in_mode_s"].items()))
    if s.get("flags_fraction"):
        L.append("  flags (share of samples): "
                 + ", ".join(f"{k} {v:.0%}" for k, v in s["flags_fraction"].items()))
    g = s.get("gait")
    if g:
        L.append(f"  gait: walking {g['walking_s']:.1f}s, {g['stops']} GAIT_STOPPED "
                 f"({g['stopped_underrun']} after an underrun), {g['underruns']} FEET_UNDERRUN"
                 + (f", refused {', '.join(sorted(set(g['refused'])))}" if g["refused"] else ""))
        e = g.get("error")
        if e:
            lag = e.get("lag_ms")
            L.append(f"  model error: offset pitch {e['offset_pitch_deg']:+.1f}° roll {e['offset_roll_deg']:+.1f}°, "
                     f"servo lag {'-' if lag is None else f'{lag:+.0f} ms vs the model'}, "
                     f"largest {e['spike_deg']:.1f}°")
    if s["events"]:
        L.append(f"  events ({len(s['events'])}): "
                 + ", ".join(f"{t:.1f}s {n}" + _event_arg(n, a) for t, n, a in s["events"][:40])
                 + (" ..." if len(s["events"]) > 40 else ""))
    else:
        L.append("  events: none")
    lk = s.get("link")
    if lk:
        L.append(f"  link: rx {lk['rx']} tx {lk['tx']}")
        if lk["rx_errors"] or lk["log_dropped"]:
            L.append(f"  rx errors {lk['rx_errors']}, log records dropped {lk['log_dropped']}")
    if s.get("bad_lines"):
        L.append(f"  {s['bad_lines']} unparsable lines skipped")
    return "\n".join(L)


def replay(data: LogData, percepts: Percepts | None = None) -> list[PerceptEvent]:
    """Feed every telemetry sample through Percepts; the events it would have produced."""
    pc = percepts or Percepts()
    out: list[PerceptEvent] = []
    for s in data.samples:
        out += pc.update(s, s.t)["events"]
    return out


def resolve_fields(fields: list[str], samples: list[TelemetrySample]) -> list[str]:
    attrs = []
    for f in fields:
        f = f.strip()
        if not f:
            continue
        a = FIELD_ALIASES.get(f, f)
        if a not in NUMERIC and a not in ("mode", "balance"):
            raise ValueError(f"unknown field {f!r}; known: {', '.join(sorted(set(FIELD_ALIASES) | {'mode', 'balance'}))}")
        attrs.append(a)
    return attrs


def _value(s: TelemetrySample, a: str) -> float:
    v = s.get(a)
    if a == "mode":
        return float(_MODE_CODES.get(v, math.nan)) if v is not None else math.nan
    if v is None:
        return math.nan
    return float(v)


def write_csv(data: LogData, fields: list[str], out: str | Path) -> Path:
    attrs = resolve_fields(fields, data.samples)
    out = Path(out)
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t"] + attrs)
        for s in data.samples:
            w.writerow([f"{s.t:.3f}"] + ["" if math.isnan(v := _value(s, a)) else f"{v:g}" for a in attrs])
    return out


def plot(data: LogData, fields: list[str], out: str | Path | None = None,
         force_csv: bool = False) -> tuple[Path, str]:
    """PNG with one panel per field (matplotlib), else CSV. Returns (path, note)."""
    attrs = resolve_fields(fields, data.samples)
    if not attrs:
        raise ValueError("no fields")
    if out is None:
        base = str(data.path.with_name(f"{data.path.name}.{'-'.join(attrs)}"))
        csv_path, png_path = Path(base + ".csv"), Path(base + ".png")
    else:
        out = Path(out)
        force_csv = force_csv or out.suffix == ".csv"
        csv_path = out if out.suffix == ".csv" else out.with_suffix(".csv")
        png_path = out if out.suffix == ".png" else out.with_suffix(".png")
    plt = None
    if not force_csv:
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            plt = None
    if plt is None:
        path = write_csv(data, fields, csv_path)
        why = "CSV requested" if force_csv else "matplotlib is not installed (sudo apt install python3-matplotlib)"
        return path, f"wrote CSV {path} ({why})"
    path = png_path
    ts = [s.t for s in data.samples]
    fig, axes = plt.subplots(len(attrs), 1, sharex=True, figsize=(10, 2.2 * len(attrs) + 0.8), squeeze=False)
    for ax, a in zip(axes[:, 0], attrs):
        ax.plot(ts, [_value(s, a) for s in data.samples], lw=1.0, color="#3b6ea5")
        ax.set_ylabel(a)
        ax.grid(True, alpha=0.3)
        for e in data.events:
            ax.axvline(e.t, color="#c0392b", lw=0.8, alpha=0.6)
    axes[0, 0].set_title(f"{data.path.name}" + (f"  (events: {', '.join(sorted({e.name for e in data.events}))})"
                                                  if data.events else ""), fontsize=9)
    axes[-1, 0].set_xlabel("t (s)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path, f"wrote {path}"
