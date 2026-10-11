"""The `kilo` subcommands and options that wire the mind's pieces together
(kept out of kilo/cli.py, which only registers them):

    kilo log summary|replay|plot FILE ...     kilo.logs (python -m kilo.logs)
    kilo --kilolog [FILE] COMMAND ...         raw wire frames to a .kilolog(.gz)
    kilo monitor --percepts                   percept events and flags, live
    kilo daemon                               which socket paths were tried
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any, Iterable, Sequence

from kilo.perception.shared import FLAG_NAMES


# ---- kilo log ---------------------------------------------------------------------
def add_log_parser(sub: Any) -> None:
    s = sub.add_parser("log", help="log tools: summary|replay|plot FILE ... (kilo.logs)", add_help=False)
    s.add_argument("log_argv", nargs=argparse.REMAINDER, metavar="summary|replay|plot FILE ...")


def cmd_log(args: argparse.Namespace) -> int:
    from kilo.logs.__main__ import build_parser, main

    if not args.log_argv or args.log_argv[0] in ("-h", "--help"):
        build_parser().print_help()
        return 0 if args.log_argv else 2
    return main(list(args.log_argv))


# ---- --kilolog ----------------------------------------------------------------------
def fix_kilolog_argv(argv: Sequence[str], commands: Iterable[str]) -> list[str]:
    """``--kilolog`` takes an optional FILE: make sure a bare ``--kilolog``
    followed by the command (``kilo --kilolog status``) does not swallow it."""
    commands = set(commands)
    out = list(argv)
    for i, a in enumerate(out):
        if a == "--kilolog":
            nxt = out[i + 1] if i + 1 < len(out) else None
            if nxt is None or nxt in commands or nxt.startswith("-"):
                out[i] = "--kilolog="
    return out


def open_kilolog(value: str, port: str | None, source: str = "kilo") -> Any:
    """A KilologWriter for ``--kilolog [FILE]`` ("" = the default path)."""
    from kilo.logs.kilolog import KilologWriter, default_path

    w = KilologWriter(Path(value).expanduser() if value else default_path(), meta={"source": source, "port": port})
    print(f"raw frame log {w.path}", file=sys.stderr)
    return w


# ---- kilo monitor --percepts ------------------------------------------------------------
def _wall_of(t: float | None) -> float:
    """A monotonic stamp (system-wide on Linux) as wall time."""
    if t is None:
        return time.time()
    return time.time() - (time.monotonic() - t)


def monitor_percepts(args: argparse.Namespace, client: Any) -> int:
    """``kilo monitor --percepts``: the monitor table (or JSON lines) plus the
    percept flags, and every percept event as it happens. Direct: a local
    SharedPercepts fed on every TELEMETRY. Through the daemon: its
    ``percepts`` stream (events) and ``percepts`` op (flags)."""
    from kilo.cli import monitor_record, render_monitor
    from kilo.remote import RemoteBody

    events: queue.Queue[dict[str, Any]] = queue.Queue()
    if isinstance(client, RemoteBody):
        conn = client.conn

        def on_stream(msg: dict[str, Any]) -> None:
            if msg.get("stream") == "percepts":
                for e in (msg.get("data") or {}).get("events", []):
                    events.put(e)
        conn.subscribe(on_stream, rate_hz=max(min(args.rate, 50.0), 0.5), kinds=("percepts",))

        def flags() -> dict[str, bool]:
            return conn.call("percepts")["flags"]
    else:
        from kilo.perception.shared import SharedPercepts

        sp = SharedPercepts()
        sp.add_listener(lambda evs, d: [events.put(e.as_dict()) for e in evs])
        client.link.subscribe("telemetry", lambda msg: sp.update(client.state))
        flags = sp.flags
    table = sys.stdout.isatty() and not args.json
    period = 1.0 / max(args.rate, 0.5)
    end = None if args.duration is None else time.monotonic() + args.duration
    recent: deque[dict[str, Any]] = deque(maxlen=6)
    next_at = time.monotonic()
    try:
        while end is None or time.monotonic() < end:
            client.link.mark_alive()
            snap = client.state.snapshot()
            f = flags()
            new = []
            while True:
                try:
                    new.append(events.get_nowait())
                except queue.Empty:
                    break
            on = [k for k in FLAG_NAMES if f.get(k)]
            if table:
                recent.extend(new)
                rows = render_monitor(snap)
                rows += "\n   percepts  " + (" ".join(on) or "-")
                rows += "\n  p-events  " + ("  ".join(
                    f"{time.strftime('%H:%M:%S', time.localtime(_wall_of(e.get('t'))))} {e.get('name')}"
                    for e in reversed(recent)) or "-")
                sys.stdout.write("\x1b[H\x1b[2J" + rows + "\n(Ctrl-C to quit)\n")
                sys.stdout.flush()
            else:
                for e in new:
                    rec = {k: v for k, v in e.items() if k not in ("t", "name")}
                    print(json.dumps({"time": round(_wall_of(e.get("t")), 3), "percept": e.get("name"), **rec},
                                     default=str), flush=True)
                print(json.dumps({**monitor_record(snap), "percepts": on}), flush=True)
            next_at += period
            delay = next_at - time.monotonic()
            if delay < 0:
                next_at, delay = time.monotonic(), 0.0
            time.sleep(delay)
    except KeyboardInterrupt:
        pass
    return 0


# ---- kilo daemon -------------------------------------------------------------------------
def daemon_link_lines(info: dict[str, Any]) -> list[str]:
    """Extra `kilo daemon` lines: the port's hot-plug state, percepts, raw log."""
    out = []
    if not info.get("port_released"):
        bits = []
        if not info.get("connected"):
            bits.append("device present" if info.get("port_present") else "device MISSING (unplugged?)")
            if info.get("open_error"):
                bits.append(f"last error: {info['open_error']}")
            if info.get("retry_in_s"):
                bits.append(f"retrying every {info['retry_in_s']:g} s")
        elif info.get("connected_for_s") is not None:
            bits.append(f"up {info['connected_for_s']:.0f} s")
        bits.append(f"reconnects {info.get('reconnects', 0)}  disconnects {info.get('disconnects', 0)}  "
                    f"failed opens {info.get('open_failures', 0)}")
        out.append("link        " + "  ".join(bits))
    p = info.get("percepts")
    if p is not None:
        out.append("percepts    " + (" ".join(k for k, v in p.items() if v) or "-"))
    out.append(f"raw log     {info.get('kilolog') or '-'}")
    return out


def print_tried(tried: list[tuple[Path, str]]) -> None:
    for p, what in tried:
        print(f"  tried {p}: {what}")
