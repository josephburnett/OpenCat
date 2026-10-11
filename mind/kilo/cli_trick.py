"""`kilo trick ...`: the keyframe tricks (docs/architecture.md §3e, §5 "Tricks").

    kilo trick list                          the library: frames, duration, loop, end posture
    kilo trick show NAME                     header, notes and every keyframe
    kilo trick upload NAME [--force]         upload (cached by trick_id and CRC)
    kilo trick play NAME [--mirror] [--speed X] [--repeats N] [--for S]
                         [--posture P] [--then-watch T]
    kilo trick stop [--transition TICKS]

``list`` and ``show`` read mind/kilo/tricks/library and need no body: they run
while the arguments are parsed, before cli.main opens the port. ``play``
uploads the trick if the body does not hold it, sends TRICK_PLAY and waits
for EVENT TRICK_DONE (BodyClient.play). ``--repeats 255`` loops the loop
section until stopped (Ctrl-C or ``--for S``; H9). ``--then-watch T`` then
goes silent at once and prints the watchdog timeline, like ``kilo posture
--then-watch``. A trick cannot start from LIMP: pass ``--posture stand``
(or set one first).

Registered in cli.py with one line (``add_parser`` adds the subcommand and
its handler to LINK_COMMANDS).
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time
from dataclasses import replace
from typing import Any

from kilo.body.client import SPEED_MAX, SPEED_MIN, BodyClient, TrickError, trick_id_for
from kilo.tricks import library
from kilo.tricks.model import CANONICAL, JOINT_NAMES, Trick
from kilo.tricks.player import FOREVER, TICK_S

EXIT_OK, EXIT_ERR = 0, 1
OFFLINE = ("list", "show")
SHORT = {"head_pan": "pan", "head_tilt": "tilt", "tail": "tail", "lf_shoulder": "LFs", "rf_shoulder": "RFs",
         "rb_shoulder": "RBs", "lb_shoulder": "LBs", "lf_knee": "LFk", "rf_knee": "RFk", "rb_knee": "RBk",
         "lb_knee": "LBk"}


class _OfflineFirst(argparse._SubParsersAction):
    """Run `trick list` / `trick show` during parsing: they need no port."""

    def __call__(self, parser, namespace, values, option_string=None):
        super().__call__(parser, namespace, values, option_string)
        cmd = getattr(namespace, "trick_cmd", None)
        if cmd in OFFLINE:
            rc = (cmd_list if cmd == "list" else cmd_show)(namespace)
            sys.stdout.flush()
            parser.exit(rc)


def add_parser(sub: Any, commands: dict) -> None:
    """Add `kilo trick` to the CLI's subparsers and its handler to ``commands``."""
    s = sub.add_parser("trick", help="keyframe tricks: list, show, upload, play, stop")
    ts = s.add_subparsers(dest="trick_cmd", required=True, metavar="ACTION", action=_OfflineFirst)
    ts.add_parser("list", help="the trick library (no body needed)")
    c = ts.add_parser("show", help="one trick's header and keyframes (no body needed)")
    c.add_argument("name", help="library name or legacy token (hi, wsf, pu, ...)")
    c = ts.add_parser("upload", help="upload a trick to the body (skipped if it already holds it)")
    c.add_argument("name")
    c.add_argument("--force", action="store_true", help="upload even if the body holds it")
    c = ts.add_parser("play", help="play a trick and wait for TRICK_DONE")
    c.add_argument("name")
    c.add_argument("--mirror", action="store_true", help="left/right mirror (pan, tail and roll negated)")
    c.add_argument("--speed", type=float, default=1.0,
                   help=f"playback speed, {SPEED_MIN:g}..{SPEED_MAX:.2f} (default 1)")
    c.add_argument("--repeats", type=int, default=None, metavar="N",
                   help="total plays of the loop section (255 = until stopped); default: the trick's")
    c.add_argument("--for", dest="hold", type=float, default=None, metavar="S",
                   help="stop the trick after S seconds if it is still playing")
    c.add_argument("--posture", choices=("rest", "sit", "stand"), default=None,
                   help="take this posture first (needed when the body is LIMP)")
    c.add_argument("--then-watch", type=float, default=None, metavar="T",
                   help="after the trick, stop sending at once and print the passive timeline for "
                        "T seconds (the watchdog timeline)")
    c = ts.add_parser("stop", help="stop the playing trick (crossfade to its end posture)")
    c.add_argument("--transition", type=int, default=0, metavar="TICKS",
                   help="crossfade length in 20 ms ticks (0 = XFADE_TICKS, 10)")
    commands["trick"] = cmd_trick


# ---- offline -----------------------------------------------------------------------
def _seconds(trick: Trick, speed: float = 1.0) -> str:
    t = BodyClient.expected_ticks(trick, speed)
    return "until stopped" if t is None else f"{t * TICK_S:5.2f} s"


def _loop(trick: Trick) -> str:
    if trick.repeats == FOREVER:
        return f"{trick.loop_start}..{trick.loop_end} forever"
    if max(trick.repeats, 1) == 1:
        return "-"
    return f"{trick.loop_start}..{trick.loop_end} x{trick.repeats}"


def cmd_list(args: argparse.Namespace) -> int:
    print(f"{'name':<13} {'legacy':<7} {'frames':>6} {'1x':>8}  {'loop':<12} {'end':<6} flags")
    for name in library.names():
        t = library.load(name)
        flags = ",".join(x for x, on in (("balance", t.balance), ("fall_ok", t.fall_ok)) if on) or "-"
        print(f"{name:<13} {library.FIRST_VERSION[name]:<7} {len(t.frames):>6} {_seconds(t):>8}  {_loop(t):<12} "
              f"{t.end_posture.name.lower():<6} {flags}")
    return EXIT_OK


def cmd_show(args: argparse.Namespace) -> int:
    try:
        t = library.load(args.name)
    except KeyError as e:
        print(f"kilo trick: {e.args[0]}", file=sys.stderr)
        return EXIT_ERR
    print(f"{t.name}  (legacy {t.source.get('legacy_name', '?')}, {t.source.get('file', '')})")
    print(f"  frames {len(t.frames)}, {_seconds(t)} at 1x, loop {_loop(t)}, end posture "
          f"{t.end_posture.name.lower()}, balance {t.balance}, fall_ok {t.fall_ok}")
    print(f"  expected roll {t.expected_roll:g}°, pitch {t.expected_pitch:g}°; joint_mask 0x{t.joint_mask:04X}; "
          f"trick_id {trick_id_for(t)}, crc 0x{t.crc16():04X}")
    for n in t.notes:
        print(f"  note: {n}")
    cols = [JOINT_NAMES[j] for j in CANONICAL]
    print("  " + f"{'#':>2} {'dur':>4} {'hold':>4} {'ease':<7}" + "".join(f"{SHORT[c]:>5}" for c in cols))
    for i, f in enumerate(t.frames):
        ang = "".join(f"{'-' if f.angles[j] is None else round(f.angles[j]):>5}" for j in CANONICAL)
        loop = "*" if t.loop_start <= i <= t.loop_end and max(t.repeats, 1) > 1 else " "
        print(f" {loop}{i:>2} {f.dur:>4} {f.hold:>4} {f.easing.name.lower():<7}{ang}" + ("  balance" if f.balance else ""))
    return EXIT_OK


# ---- with the body ----------------------------------------------------------------
def _load(name: str, repeats: int | None) -> Trick:
    t = library.load(name)
    if repeats is not None:
        if not 0 <= repeats <= 255:
            raise ValueError("--repeats must be 0..255")
        t = replace(t, repeats=repeats)
    return t


def cmd_trick(args: argparse.Namespace, client: BodyClient) -> int:
    if args.trick_cmd == "upload":
        client.wait_ready(args.timeout, require_telemetry=False)
        t = _load(args.name, None)
        tid, uploaded = client.upload_trick(t, force=args.force)
        print(f"{t.name}: trick_id {tid}, crc 0x{t.crc16():04X}, {len(t.frames)} frames "
              + ("uploaded" if uploaded else "already on the body (cached)"))
        return EXIT_OK
    if args.trick_cmd == "stop":
        client.wait_ready(args.timeout, require_telemetry=False)
        client.stop_trick(args.transition)
        print("stopped")
        return EXIT_OK
    return _play(args, client)


def _play(args: argparse.Namespace, client: BodyClient) -> int:
    t = _load(args.name, args.repeats)
    client.speed_q6(args.speed)  # range check before anything moves
    if args.repeats == FOREVER and args.hold is None:
        print("note: --repeats 255 plays until stopped: Ctrl-C (or --for S) stops it", flush=True)
    client.wait_ready(args.timeout)
    if args.posture:
        client.posture(args.posture)
        client.spin(1.0)  # the default posture move is 0.5 s
    elif client.state.mode == "LIMP":
        print("kilo trick: the body is LIMP and a trick cannot start from LIMP; add --posture stand",
              file=sys.stderr)
        return EXIT_ERR
    tid, uploaded = client.upload_trick(t)
    print(f"{t.name}: trick_id {tid} {'uploaded' if uploaded else 'cached'}; playing"
          + (" mirrored" if args.mirror else "") + (f" at x{args.speed:g}" if args.speed != 1 else "")
          + f", expected {_seconds(t, args.speed)}", flush=True)
    stopped_by_us = False

    async def run() -> Any:
        nonlocal stopped_by_us
        task = asyncio.ensure_future(client.play(t, mirror=args.mirror, speed=args.speed))
        if args.hold is not None:
            done, _ = await asyncio.wait({task}, timeout=args.hold)
            if not done:
                stopped_by_us = True
                client.stop_trick()
        return await task

    try:
        res = asyncio.run(run())
    except KeyboardInterrupt:
        client.stop_trick()
        print("interrupted: trick stopped")
        return EXIT_OK
    except TrickError as e:
        print(f"kilo trick: {e}", file=sys.stderr)
        return EXIT_ERR
    exp = "" if res.expected_ticks is None else f" (expected {(res.expected_ticks + 1) * TICK_S:.2f} s)"
    print(f"TRICK_DONE {res.reason} after {res.elapsed_s:.2f} s{exp}; end posture "
          f"{t.end_posture.name.lower()}, epoch {res.epoch}", flush=True)
    ok = res.reason == "DONE" or (stopped_by_us and res.reason == "STOPPED")
    if args.then_watch is not None:
        _then_watch(client, args.then_watch)
    return EXIT_OK if ok else EXIT_ERR


def _then_watch(client: BodyClient, seconds: float) -> None:
    from kilo.cli import _wall, watch_timeline

    last = client.link.go_silent()
    hb = client.link.last_heartbeat
    if last is None:
        t0 = client.link.clock()
        print(f"silent from {_wall(time.time())} (t=0); nothing was sent", flush=True)
    else:
        t0, wall, name = last
        print(f"silent: last frame {name} sent at {_wall(wall)} (t=0)"
              + (f", last HEARTBEAT at {_wall(hb[1])} (t={hb[0] - t0:+.2f})" if hb else "")
              + f"; watching {seconds:g} s", flush=True)
    watch_timeline(client, t0, seconds)

