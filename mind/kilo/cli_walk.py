"""`kilo walk ...` and `kilo pose ...`: the gait and the body pose (SET_GAIT,
SET_BODY_POSE; docs/architecture.md §3d, P2).

    kilo walk [forward|back|left|right|march|stop] [--stride MM] [--turn MM]
              [--pattern trot|walk|crawl] [--period MS] [--duty N] [--step-h MM]
              [--body-h MM] [--no-bob] [--no-wag] [--no-balance] [--for S]
    kilo pose [set|release] [--pitch DEG] [--roll DEG] [--height MM] [--xshift MM]
              [--posture stance|...] [--for S]

``kilo walk`` latches SET_GAIT run=1 (refreshed at 5 Hz) and walks until
Ctrl-C (or SIGTERM) or ``--for S``; then it sends the stop (run=0) and waits
for EVENT GAIT_STOPPED: the body ramps the stride down over one period and
hands over to STANCE. Directions: forward / back with the pattern's default
stride (``--stride``), ``--turn`` (+ = left) bends the path; left / right
turn in place (``--turn`` magnitude, default 20 mm; ``--stride`` makes an
arc); march steps in place. Fields left out take the §3d defaults of the
pattern. ``kilo walk stop`` stops whoever walks (a client of the daemon, the
IR remote) and waits for GAIT_STOPPED. The body moves to STANCE first from
any other posture. A refusal (EVENT GOAL_REFUSED for SET_GAIT: UNKNOWN_TYPE
from a firmware without the gait, STALE, FALLEN, LOW_BATT, UNCALIBRATED) or
the goal cleared by an epoch bump (fall, watchdog, low battery) ends the
command with status 1.

SAFETY (the owner's rule): Kilo walks on the floor only, never on a desk or
a table; every walk prints that reminder first.

``kilo pose`` latches SET_BODY_POSE (pitch + = nose down, roll + = right side
up, height delta, x-shift) on top of the gait and the STANCE posture until
Ctrl-C or ``--for``, then latches the neutral pose. ``kilo pose release``
latches the neutral pose at once.

Registered in cli.py with one line (``add_parser`` adds the subcommands and
their handlers to LINK_COMMANDS).
"""

from __future__ import annotations

import argparse
import contextlib
import signal
import threading
import time
from typing import Any, Iterator

from kilo.body.client import DEFAULT_PATTERN, DEFAULT_TURN_MM, GAIT_PATTERN_CHOICES, BodyClient, walk_args
from kilo.link import consts
from kilo.protocol import kilo_protocol as P

EXIT_OK, EXIT_ERR = 0, 1
SAFETY = "SAFETY: Kilo walks on the floor only, never on a desk or a table (it fell off a desk once)."
DIRECTIONS = ("forward", "back", "left", "right", "march", "stop")
REFUSAL_HINTS = {
    "UNKNOWN_TYPE": "this firmware has no gait generator: flash the P2 body (kilo flash)",
    "UNCALIBRATED": "the body has no calibration (kilo cal)",
    "STALE": "the body's epoch changed (a fall, the watchdog, a reboot)",
    "FALLEN": "Kilo is down: the fall reflex holds the joints",
    "LOW_BATT": "the battery is low",
    "BAD_ARG": "a field is out of range for the body",
}


def add_parser(sub: Any, commands: dict) -> None:
    """Add `kilo walk` and `kilo pose` to the CLI's subparsers and their handlers to ``commands``."""
    s = sub.add_parser("walk", help="walk (SET_GAIT) until Ctrl-C or --for, then stop; ON THE FLOOR ONLY")
    s.add_argument("direction", nargs="?", choices=DIRECTIONS, default="forward",
                   help="forward (default), back, left / right (turn in place), march (in place), "
                        "or stop (stop any walk and wait for GAIT_STOPPED)")
    s.add_argument("--pattern", choices=GAIT_PATTERN_CHOICES, default=DEFAULT_PATTERN,
                   help=f"gait pattern (default {DEFAULT_PATTERN})")
    s.add_argument("--stride", type=float, default=None, metavar="MM",
                   help="stride in mm (default: the pattern's; forward/back take its magnitude)")
    s.add_argument("--turn", type=float, default=None, metavar="MM",
                   help=f"turn in mm, + = left (forward/back: an arc; left/right: the magnitude, "
                        f"default {DEFAULT_TURN_MM})")
    s.add_argument("--period", type=int, default=None, metavar="MS", help="cycle period in ms")
    s.add_argument("--duty", type=int, default=None, metavar="N", help="stance fraction in 1/256")
    s.add_argument("--step-h", type=float, default=None, metavar="MM", help="foot lift in mm")
    s.add_argument("--body-h", type=float, default=None, metavar="MM",
                   help="body height in mm (default: a fraction of LEG_L1_MM + LEG_L2_MM)")
    s.add_argument("--no-bob", action="store_true", help="no head bob")
    s.add_argument("--no-wag", action="store_true", help="no tail wag")
    s.add_argument("--no-balance", action="store_true", help="no balance corrections while walking")
    s.add_argument("--for", dest="hold", type=float, default=None, metavar="S",
                   help="walk S seconds, then stop (default: until Ctrl-C)")
    s.add_argument("--wait", type=float, default=None, metavar="S",
                   help="how long to wait for GAIT_STOPPED after the stop (default: one period + 1.5 s)")
    commands["walk"] = cmd_walk

    s = sub.add_parser("pose", help="body pitch/roll/height/x-shift (SET_BODY_POSE) over STANCE and the gait")
    s.add_argument("action", nargs="?", choices=("set", "release"), default="set",
                   help="set (default: hold until Ctrl-C or --for) or release (neutral pose now)")
    s.add_argument("--pitch", type=float, default=0.0, metavar="DEG", help="body pitch, + = nose down")
    s.add_argument("--roll", type=float, default=0.0, metavar="DEG", help="body roll, + = right side up")
    s.add_argument("--height", type=float, default=0.0, metavar="MM", help="height delta in mm")
    s.add_argument("--xshift", type=float, default=0.0, metavar="MM", help="forward shift in mm")
    s.add_argument("--posture", choices=("stance",), default=None,
                   help="take STANCE first (the pose applies to STANCE and the gait only)")
    s.add_argument("--for", dest="hold", type=float, default=None, metavar="S",
                   help="hold S seconds, then release (default: until Ctrl-C)")
    commands["pose"] = cmd_pose


@contextlib.contextmanager
def _sigterm_interrupts() -> Iterator[None]:
    """SIGTERM ends the hold like Ctrl-C (a background `kilo walk` over ssh
    ignores SIGINT; see README "Background clients over ssh"), so the walk
    is stopped, not left to the watchdog."""
    if threading.current_thread() is not threading.main_thread():
        yield
        return

    def raise_interrupt(signum: int, frame: Any) -> None:
        raise KeyboardInterrupt

    old = signal.signal(signal.SIGTERM, raise_interrupt)
    try:
        yield
    finally:
        signal.signal(signal.SIGTERM, old)


def refusal(client: Any, goal: str, since_s: float) -> str | None:
    """The reason of an EVENT GOAL_REFUSED for ``goal`` in the last ``since_s`` seconds."""
    gtype = int(P.MsgId[goal])
    for age, e in reversed(client.state.snapshot()["events"]):
        if age > since_s:
            break
        if e.get("code") == consts.EV["GOAL_REFUSED"]:
            arg = int(e.get("arg", 0)) & 0xFFFF
            if arg >> 8 == gtype:
                return consts.REFUSE_REASON.get(arg & 0xFF, str(arg & 0xFF))
    return None


def _print_refusal(goal: str, reason: str) -> None:
    hint = REFUSAL_HINTS.get(reason)
    print(f"body refused {goal}: {reason}" + (f" ({hint})" if hint else ""), flush=True)


def _stop(client: Any, wait: float | None) -> int:
    timeout = client.stop_timeout() if wait is None else wait
    t0 = time.monotonic()
    if client.stop_walk(wait_s=timeout):
        print(f"stopped ({time.monotonic() - t0:.1f} s)", flush=True)
        return EXIT_OK
    print(f"no GAIT_STOPPED within {timeout:.1f} s (mode {client.state.mode}); the stop stays latched",
          flush=True)
    return EXIT_ERR


def cmd_walk(args: argparse.Namespace, client: BodyClient) -> int:
    client.wait_ready(args.timeout)
    if args.direction == "stop":
        with _sigterm_interrupts():
            return _stop(client, args.wait)
    kw = walk_args(args.direction, args.pattern, args.stride, args.turn)
    gait = dict(period_ms=args.period, duty=args.duty, step_h_mm=args.step_h, body_h_mm=args.body_h,
                bob=not args.no_bob, wag=not args.no_wag, balance=not args.no_balance)
    print(SAFETY, flush=True)  # out-of-range fields raise ValueError in walk(), before anything is sent
    client.load_geometry()
    rc = EXIT_OK
    with _sigterm_interrupts():
        t0 = time.monotonic()
        try:
            f = client.walk(**kw, **gait)
            print(f"walking {args.direction}: {kw['pattern']} stride {f['stride_mm']} mm, turn {f['turn_mm']} mm, "
                  f"period {f['period_ms']} ms, duty {f['duty']}/256, step {f['step_h_mm']} mm, body "
                  f"{f['body_h_mm']} mm" + (f" for {args.hold:g} s" if args.hold is not None
                                             else " (Ctrl-C to stop)"), flush=True)
            end = None if args.hold is None else t0 + args.hold
            next_line = t0 + 1.0
            while end is None or time.monotonic() < end:
                client.spin(0.25 if end is None else max(0.0, min(0.25, end - time.monotonic())))
                why = refusal(client, "SET_GAIT", time.monotonic() - t0 + 0.05)
                if why is not None:
                    _print_refusal("SET_GAIT", why)
                    rc = EXIT_ERR
                    break
                if "SET_GAIT" not in client.link.goals():
                    st = client.state
                    print(f"gait goal cleared by the body (mode {st.mode}, epoch {st.epoch}); exiting", flush=True)
                    return EXIT_ERR
                now = time.monotonic()
                if now >= next_line:
                    next_line += 1.0
                    s = client.state.snapshot()
                    phase = s.get("gait_phase")
                    print(f"t={now - t0:5.1f}s  mode {s['mode']}  phase "
                          + ("-" if phase is None else f"{phase / 256:.2f}")
                          + f"  pitch {_f(s['pitch_deg'])}°  roll {_f(s['roll_deg'])}°", flush=True)
        except KeyboardInterrupt:
            pass
        try:
            stop_rc = _stop(client, args.wait)
        except KeyboardInterrupt:
            stop_rc = EXIT_ERR
    return rc if rc != EXIT_OK else stop_rc


def cmd_pose(args: argparse.Namespace, client: BodyClient) -> int:
    client.wait_ready(args.timeout)
    if args.action == "release":
        client.release_pose()
        client.spin(0.3)  # let it go out (and be refreshed once) before the link closes
        print("pose released (neutral)")
        return EXIT_OK
    if args.posture:
        client.posture(args.posture)
        client.spin(1.0)  # the default posture move is 0.5 s
    elif client.state.mode != "GAIT":
        print("note: the pose applies to STANCE and the gait only; add --posture stance if Kilo stands "
              "otherwise", flush=True)
    rc = EXIT_OK
    with _sigterm_interrupts():
        t0 = time.monotonic()
        try:
            f = client.pose(args.pitch, args.roll, args.height, args.xshift)
            print(f"pose pitch {f['pitch'] / 16:g}°, roll {f['roll'] / 16:g}°, height {f['dheight_mm']:+d} mm, "
                  f"x-shift {f['xshift_mm']:+d} mm" + (f" for {args.hold:g} s" if args.hold is not None
                                                       else " (Ctrl-C to release)"), flush=True)
            end = None if args.hold is None else t0 + args.hold
            while end is None or time.monotonic() < end:
                client.spin(0.25 if end is None else max(0.0, min(0.25, end - time.monotonic())))
                why = refusal(client, "SET_BODY_POSE", time.monotonic() - t0 + 0.05)
                if why is not None:
                    _print_refusal("SET_BODY_POSE", why)
                    rc = EXIT_ERR
                    break
                if "SET_BODY_POSE" not in client.link.goals():
                    st = client.state
                    print(f"pose goal cleared by the body (mode {st.mode}, epoch {st.epoch}); exiting")
                    return EXIT_ERR
        except KeyboardInterrupt:
            pass
        client.release_pose()
        client.spin(0.3)
    if rc == EXIT_OK:
        print("released")
    return rc


def _f(v: Any) -> str:
    return "-" if v is None else f"{v:.1f}"
