"""python -m kilo.logs: offline log tools (until ``kilo log ...`` is wired into cli.py).

    python -m kilo.logs summary FILE [--json]
    python -m kilo.logs replay FILE [--json] [--with-body-events]
    python -m kilo.logs plot FILE --fields pitch,roll[,dist,batt,rate_pitch,mode,balance...]
                              [--out PATH.png|PATH.csv] [--csv]

FILE is a link JSONL log (``kilo --log``), a ``kilo monitor`` JSONL capture,
or a ``.kilolog`` (optionally ``.gz``).
"""

from __future__ import annotations

import argparse
import json
import sys

from kilo.logs import analysis, reader


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m kilo.logs", description="Kilo log tools")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("summary", help="duration, rates, attitude, battery, modes, events")
    s.add_argument("file")
    s.add_argument("--json", action="store_true")
    r = sub.add_parser("replay", help="feed the log through Percepts and print its events")
    r.add_argument("file")
    r.add_argument("--json", action="store_true")
    r.add_argument("--with-body-events", action="store_true", help="interleave the body's EVENTs")
    pl = sub.add_parser("plot", help="plot fields over time (CSV without matplotlib)")
    pl.add_argument("file")
    pl.add_argument("--fields", default="pitch,roll", help="comma-separated (default pitch,roll)")
    pl.add_argument("--out", help="output .png or .csv (default next to the log)")
    pl.add_argument("--csv", action="store_true", help="write CSV even if matplotlib is available")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        data = reader.load(args.file)
    except (OSError, ValueError) as e:
        print(f"cannot read {args.file}: {e}", file=sys.stderr)
        return 1
    if args.cmd == "summary":
        s = analysis.summary(data)
        print(json.dumps(s, indent=1) if args.json else analysis.format_summary(s))
        return 0
    if args.cmd == "replay":
        evs = analysis.replay(data)
        rows = [(e.t, "percept", e.name, e.data) for e in evs]
        if args.with_body_events:
            rows += [(e.t, "body", e.name, {"arg": e.arg}) for e in data.events]
            rows.sort(key=lambda r: r[0])
        if args.json:
            for t, src, name, d in rows:
                print(json.dumps({"t": round(t, 3), "src": src, "name": name, **d}))
        else:
            print(f"{data.path}: {len(data.samples)} samples, {len(evs)} percept events")
            for t, src, name, d in rows:
                extra = " ".join(f"{k}={v}" for k, v in d.items() if v is not None)
                print(f"{t:9.3f}  {src:7s} {name:12s} {extra}")
        return 0
    try:
        path, note = analysis.plot(data, args.fields.split(","), args.out, force_csv=args.csv)
    except ValueError as e:
        print(f"plot: {e}", file=sys.stderr)
        return 2
    print(note)
    return 0


if __name__ == "__main__":
    sys.exit(main())
