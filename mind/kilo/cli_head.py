"""`kilo look ...`: hold the head (SET_HEAD, docs/architecture.md §3c layer 1).

    kilo look PAN TILT [--tail T] [--speed DPS] [--for S] [--posture P]

Latches SET_HEAD mode=HOLD (refreshed at 5 Hz) and keeps it for ``--for S``
seconds, or until Ctrl-C; then releases it (SET_HEAD mode=RELEASE) so the
head crossfades back to the posture. The head follows only while its owner
is IDLE or GAIT: from LIMP nothing moves (pass ``--posture stand``), a trick
that moves the head wins, and a fall or watchdog epoch bump drops the hold
(the command then exits with status 1). Angles in degrees, joint convention
of the postures; the body clamps them to the joint limits (pan and tail
±120°, tilt -80..45°). TELEMETRY shows the hold as head owner PI_HOLD and
the HEAD_HELD flag.

Registered in cli.py with one line (``add_parser`` adds the subcommand and
its handler to LINK_COMMANDS).
"""

from __future__ import annotations

import argparse
import time
from typing import Any

from kilo.body.client import HEAD_SPEED_DPS, BodyClient

EXIT_OK, EXIT_ERR = 0, 1


def add_parser(sub: Any, commands: dict) -> None:
    """Add `kilo look` to the CLI's subparsers and its handler to ``commands``."""
    s = sub.add_parser("look", help="hold the head at pan/tilt (and the tail) for a while")
    s.add_argument("pan", type=float, help="head pan, degrees")
    s.add_argument("tilt", type=float, help="head tilt, degrees")
    s.add_argument("--tail", type=float, default=None, metavar="T", help="also hold the tail at T degrees")
    s.add_argument("--speed", type=float, default=HEAD_SPEED_DPS, metavar="DPS",
                   help=f"slew limit in °/s (default {HEAD_SPEED_DPS}; 0 = the body's 500°/s safety slew)")
    s.add_argument("--for", dest="hold", type=float, default=None, metavar="S",
                   help="hold S seconds, then release (default: until Ctrl-C)")
    s.add_argument("--posture", choices=("rest", "sit", "stand", "balance"), default=None,
                   help="take this posture first (the head does not move from LIMP)")
    commands["look"] = cmd_look


def cmd_look(args: argparse.Namespace, client: BodyClient) -> int:
    client.wait_ready(args.timeout)
    if args.posture:
        client.posture(args.posture)
        client.spin(1.0)  # the default posture move is 0.5 s
    elif client.state.mode == "LIMP":
        print("note: the body is LIMP, so the head will not move; add --posture stand", flush=True)
    f = client.look(args.pan, args.tilt, args.tail, speed_dps=args.speed)
    tail = "" if args.tail is None else f", tail {args.tail:g}°"
    print(f"looking at pan {args.pan:g}°, tilt {args.tilt:g}°{tail} at "
          + (f"{f['max_speed_dps']}°/s" if f["max_speed_dps"] else "full speed")
          + (f" for {args.hold:g} s" if args.hold is not None else " (Ctrl-C to release)"), flush=True)
    end = None if args.hold is None else time.monotonic() + args.hold
    rc = EXIT_OK
    try:
        while end is None or time.monotonic() < end:
            client.spin(0.25 if end is None else max(0.0, min(0.25, end - time.monotonic())))
            if "SET_HEAD" not in client.link.goals():
                st = client.state
                print(f"head goal cleared by the body (mode {st.mode}, epoch {st.epoch}); exiting")
                rc = EXIT_ERR
                break
    except KeyboardInterrupt:
        pass
    client.release_head()
    client.spin(0.1)  # let the release go out before the link closes
    if rc == EXIT_OK:
        print("released")
    return rc
