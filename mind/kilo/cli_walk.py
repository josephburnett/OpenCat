"""`kilo walk ...` and `kilo pose ...`: walking (the planner's FEET stream)
and the body pose (SET_BODY_POSE; docs/architecture.md §3d).

    kilo walk [forward|back|left|right|march|stop] [--pattern trot|walk|crawl]
              [--speed MM_S] [--no-balance] [--for S] [--wait S]
    kilo pose [set|release] [--pitch DEG] [--roll DEG] [--height MM] [--xshift MM]
              [--posture stance|...] [--for S]

``kilo walk`` walks until Ctrl-C (or SIGTERM) or ``--for S``, then stops:
each leg steps back under its shoulder, the body levels, and Kilo stands in
STANCE. The steps are planned in the mind (kilo.planner: the walk and the
crawl move the body over the three feet that stay down before each lift)
and streamed to the body every 20 ms. Directions: forward, back, left /
right (turn in place), march (step in place). ``--speed`` in mm/s (the
pattern's range is in ``--help``); ``--no-balance`` turns the body's
balance off for this walk (SET_BALANCE enable 0) and restores it after.
Once a second it prints the attitude, the error the body reports (measured
minus predicted) and the model's error estimates (steady offset, extra
servo lag; kilo.planner.errors). ``kilo walk stop`` stops whoever walks: the
daemon's walk, or, without a daemon, a walk of another ``kilo`` process
(the STANCE posture ends it on the body). A walk the body ends (a fall, a
refusal, the stream late, a reboot) ends the command with status 1.

SAFETY (the owner's rule): Kilo walks on the floor only, never on a desk or
a table; every walk prints that reminder first.

``kilo pose`` latches SET_BODY_POSE (pitch + = nose down, roll + = right side
up, height delta, x-shift) on top of the STANCE posture until Ctrl-C or
``--for``, then latches the neutral pose. ``kilo pose release`` latches the
neutral pose at once.

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

from kilo.body.client import BodyClient
from kilo.link import consts
from kilo.planner import patterns as pat
from kilo.planner.walker import stop_timeout
from kilo.protocol import kilo_protocol as P

EXIT_OK, EXIT_ERR = 0, 1
SAFETY = "SAFETY: Kilo walks on the floor only, never on a desk or a table (it fell off a desk once)."
DIRECTIONS = ("forward", "back", "left", "right", "march", "stop")
STOP_WAIT_S = 2.0  # `kilo walk stop` without a daemon: how long GAIT may take to end after STANCE
REFUSAL_HINTS = {
    "UNKNOWN_TYPE": "this firmware does not play FEET: flash the protocol 2 body (kilo flash)",
    "UNCALIBRATED": "the body has no calibration (kilo cal)",
    "STALE": "the body's epoch changed (a fall, the watchdog, a reboot)",
    "FALLEN": "Kilo is down: the fall reflex holds the joints",
    "LOW_BATT": "the battery is low",
    "BAD_ARG": "a field is out of range for the body",
}


def add_parser(sub: Any, commands: dict) -> None:
    """Add `kilo walk` and `kilo pose` to the CLI's subparsers and their handlers to ``commands``."""
    s = sub.add_parser("walk", help="walk (the planner streams FEET) until Ctrl-C or --for, then stop; "
                                    "ON THE FLOOR ONLY")
    s.add_argument("direction", nargs="?", choices=DIRECTIONS, default="forward",
                   help="forward (default), back, left / right (turn in place), march (in place), "
                        "or stop (stop any walk)")
    s.add_argument("--pattern", choices=pat.PATTERN_NAMES, default=pat.DEFAULT_PATTERN,
                   help=f"gait pattern (default {pat.DEFAULT_PATTERN})")
    s.add_argument("--speed", type=float, default=None, metavar="MM_S",
                   help="speed in mm/s (default: the pattern's, " + ", ".join(
                       f"{p} {pat.default_speed(p):.0f} ({lo:.1f}..{hi:.0f})"
                       for p in pat.PATTERN_NAMES for lo, hi in [pat.speed_range(p)]) + ")")
    s.add_argument("--no-balance", action="store_true",
                   help="the body's balance off for this walk (SET_BALANCE enable 0), restored after")
    s.add_argument("--for", dest="hold", type=float, default=None, metavar="S",
                   help="walk S seconds, then stop (default: until Ctrl-C)")
    s.add_argument("--wait", type=float, default=None, metavar="S",
                   help="how long the stop may take (default: two cycles + 1.5 s)")
    commands["walk"] = cmd_walk

    s = sub.add_parser("pose", help="body pitch/roll/height/x-shift (SET_BODY_POSE) over STANCE")
    s.add_argument("action", nargs="?", choices=("set", "release"), default="set",
                   help="set (default: hold until Ctrl-C or --for) or release (neutral pose now)")
    s.add_argument("--pitch", type=float, default=0.0, metavar="DEG", help="body pitch, + = nose down")
    s.add_argument("--roll", type=float, default=0.0, metavar="DEG", help="body roll, + = right side up")
    s.add_argument("--height", type=float, default=0.0, metavar="MM", help="height delta in mm")
    s.add_argument("--xshift", type=float, default=0.0, metavar="MM", help="forward shift in mm")
    s.add_argument("--posture", choices=("stance",), default=None,
                   help="take STANCE first (the pose applies to STANCE only)")
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


def error_text(est: dict[str, Any] | None) -> str:
    """The model's error estimates in one line (kilo.planner.errors)."""
    if not est:
        return "model error: not enough walking yet (needs 1 s of 50 Hz telemetry)"
    lag = est.get("lag_ms")
    return (f"model error: offset pitch {est['offset_pitch_deg']:+.1f}° roll {est['offset_roll_deg']:+.1f}°  "
            f"servo lag {'-' if lag is None else f'{lag:+.0f} ms vs the model'}  "
            f"largest {est['spike_deg']:.1f}°  balance corr {est['corr_pitch_deg']:+.1f}° "
            f"{est['corr_roll_deg']:+.1f}°")


def _stop_other(client: Any, wait: float | None) -> int:
    """`kilo walk stop`: the daemon's walk (its walker), or, without a
    daemon, whoever streams: the STANCE posture ends GAIT on the body."""
    t0 = time.monotonic()
    st = client.walk_status() or {}
    if client.stop_walk(wait_s=stop_timeout(st.get("period_ms")) if wait is None else wait) and not client.walking():
        print(f"stopped ({time.monotonic() - t0:.1f} s)", flush=True)
        return EXIT_OK
    if not client.walking():
        print("not walking", flush=True)
        return EXIT_OK
    client.posture("stance")
    end = time.monotonic() + STOP_WAIT_S
    while client.walking() and time.monotonic() < end:
        client.spin(0.05)
    if not client.walking():
        print(f"stopped: STANCE ended the walk ({time.monotonic() - t0:.1f} s)", flush=True)
        return EXIT_OK
    print(f"still walking after STANCE (mode {client.state.mode})", flush=True)
    return EXIT_ERR


def _line(t: float, snap: dict[str, Any]) -> str:
    err = snap.get("err_deg")
    return (f"t={t:5.1f}s  mode {snap['mode']}  pitch {_f(snap['pitch_deg'])}° roll {_f(snap['roll_deg'])}°"
            + ("" if err is None else f"  error {err[0]:+.1f}° {err[1]:+.1f}°"))


def cmd_walk(args: argparse.Namespace, client: BodyClient) -> int:
    client.wait_ready(args.timeout)
    if args.direction == "stop":
        with _sigterm_interrupts():
            return _stop_other(client, args.wait)
    pat.pattern(args.pattern)  # ValueError before anything is sent
    print(SAFETY, flush=True)
    rc = EXIT_OK
    with _sigterm_interrupts():
        t0 = time.monotonic()
        try:
            st = client.walk(args.direction, args.pattern, args.speed, balance=not args.no_balance)
            print(f"walking {args.direction}: {st['pattern']} {st['speed_mm_s']:.1f} mm/s (stride "
                  f"{st['stride_mm']:.0f} mm, cycle {st['period_ms']} ms)"
                  + (", balance off" if args.no_balance else "")
                  + (f" for {args.hold:g} s" if args.hold is not None else " (Ctrl-C to stop)"), flush=True)
            end = None if args.hold is None else t0 + args.hold
            next_line = t0 + 1.0
            while end is None or time.monotonic() < end:
                client.spin(0.25 if end is None else max(0.0, min(0.25, end - time.monotonic())))
                st = client.walk_status() or {}
                if st.get("ended"):
                    why = refusal(client, "FEET", time.monotonic() - t0 + 0.05)
                    if why is not None:
                        _print_refusal("FEET", why)
                    print(f"the walk ended: {st['ended']}", flush=True)
                    print(error_text(st.get("error")), flush=True)
                    return EXIT_OK if st.get("ok") else EXIT_ERR
                now = time.monotonic()
                if now >= next_line:
                    next_line += 1.0
                    print(_line(now - t0, client.state.snapshot()), flush=True)
        except KeyboardInterrupt:
            pass
        try:
            wait = stop_timeout((client.walk_status() or {}).get("period_ms")) if args.wait is None else args.wait
            stopped = client.stop_walk(wait_s=wait)
        except KeyboardInterrupt:
            stopped = False
        st = client.walk_status() or {}
        if stopped and st.get("ok", True):
            print(f"stopped ({time.monotonic() - t0:.1f} s): sent {st.get('sent', 0)} ticks, "
                  f"{st.get('skipped', 0)} too late, {st.get('underruns', 0)} underruns, "
                  f"smallest margin {st.get('min_margin_mm')} mm", flush=True)
        else:
            print(f"the walk ended: {st.get('ended') or 'no end within the wait'}", flush=True)
            rc = EXIT_ERR
        print(error_text(st.get("error")), flush=True)
    return rc


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
    else:
        print("note: the pose applies to STANCE only; add --posture stance if Kilo stands otherwise", flush=True)
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
