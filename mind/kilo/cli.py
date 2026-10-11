"""The `kilo` command line (docs/architecture.md Appendix G), P0 subset.

    kilo [--port P] [--log | --log-file FILE] [-v] status [--json]
    kilo monitor [--rate HZ] [--duration S] [--json] [--passive]
    kilo posture {limp,rest,stand,sit,balance,...} [--transition TICKS] [--for S [--then-watch T]]
    kilo balance on|off [--kp X] [--kd S] [--deadband D] [--clamp C] [--posture stand|sit] [--for S]
    kilo cal show | set JOINT DEGREES | save | revert
    kilo param list | get ID | set ID VALUE [--persist]
    kilo flash [--legacy] [--no-build] [--hex FILE] [--reset auto|gpio|dtr|none] [--verify-timeout S]
    kilo vbody [--fall-after S] [--no-keys]
    kilo run [--port P] [--socket PATH] [--log] [--kilolog [FILE]] [--behaviour]
    kilo daemon [status|stop]
    kilo log summary|replay|plot FILE ...
    kilo --kilolog [FILE] COMMAND ...    (also: kilo monitor --percepts; kilo.cli_tools)

Every command first tries the daemon socket (``kilo run``, kilo.daemon; path
from kilo.remote.socket_path()); when no daemon listens, or ``--direct`` is
given, or ``--port`` names another port than the daemon's, it opens the port
itself. Because goals are only held while the mind is alive, ``kilo posture``
keeps running and holding the posture until Ctrl-C or ``--for``; when it exits
the body's watchdog takes over (stop, then REST after 1.5 s), or, through the
daemon, the daemon ends the hold (REST at once, LIMP 10 s later).

Every command but ``monitor --passive`` feeds the body's link watchdog while
it runs. ``monitor --passive`` writes nothing at all to the port (the body
streams TELEMETRY and EVENTs unprompted) and prints a timeline, so it shows
what the body does on its own. ``posture --for S --then-watch T`` holds, then
goes silent at once and prints the same timeline for T seconds, relative to
the last frame it sent: the watchdog timeline (H5) in one command.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Any, Sequence

from kilo import __version__
from kilo.body.client import (BALANCE_CLAMP, BALANCE_DEADBAND, BALANCE_KD_S, BALANCE_KP, POSTURE_CHOICES,
                              BodyClient, ProtocolMismatch)
from kilo.cli_tools import add_log_parser, cmd_log, fix_kilolog_argv
from kilo.link import consts
from kilo.link.messages import fields_of
from kilo.link.ports import detect_port
from kilo.link.serial_link import CommandError, LinkError, SerialLink
from kilo.link.transport import DEFAULT_BAUD, open_port
from kilo.protocol import kilo_protocol as P
from kilo.remote import DaemonError

EXIT_OK, EXIT_ERR, EXIT_USAGE = 0, 1, 2


# ---- argument parsing -------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="kilo", description="Kilo mind command line")
    p.add_argument("--version", action="version", version=f"kilo {__version__}")
    p.add_argument("--port", "-p", help="serial port (default: auto-detect; see README 'Port rules')")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="link baud rate (default 250000)")
    p.add_argument("--log", action="store_true",
                   help="write a JSON-lines frame/event log to ~/kilo-logs/kilo-<time>.jsonl")
    p.add_argument("--log-file", metavar="FILE", help="write the JSON-lines log to FILE")
    p.add_argument("--kilolog", nargs="?", const="", default=None, metavar="FILE",
                   help="write the raw wire frames to FILE (default ~/kilo-logs/kilo-<time>.kilolog.gz)")
    p.add_argument("--timeout", type=float, default=3.0, help="seconds to wait for the body")
    p.add_argument("-v", "--verbose", action="count", default=0)
    p.add_argument("--socket", metavar="PATH",
                   help="daemon socket (default $KILO_SOCKET, $XDG_RUNTIME_DIR/kilo/kilo.sock, "
                        "/run/kilo/kilo.sock as root, else ~/.kilo/kilo.sock)")
    p.add_argument("--direct", action="store_true", help="never use the daemon: open the port directly")
    sub = p.add_subparsers(dest="cmd", required=True, metavar="COMMAND")

    s = sub.add_parser("status", help="HELLO, state and link statistics")
    s.add_argument("--json", action="store_true")
    s.add_argument("--no-stats", action="store_true", help="do not request STATS from the body")

    s = sub.add_parser("monitor", help="live telemetry table")
    s.add_argument("--rate", type=float, default=10.0, help="refresh rate in Hz (table and JSON lines)")
    s.add_argument("--duration", type=float, default=None, help="stop after S seconds")
    s.add_argument("--json", action="store_true",
                   help="JSON lines instead of the table (the default when stdout is not a terminal)")
    s.add_argument("--passive", action="store_true",
                   help="send nothing (no HELLO, heartbeat or goal; the body's watchdog is not fed) "
                        "and print a timeline: events and mode/epoch/owner/flag changes as they "
                        "happen, a telemetry line every second")
    s.add_argument("--percepts", action="store_true",
                   help="also show the percept flags and print percept events (tap, pet, near...) live")

    s = sub.add_parser("posture", help="hold a posture (until Ctrl-C or --for)")
    s.add_argument("name", choices=POSTURE_CHOICES)
    s.add_argument("--transition", type=int, default=0, metavar="TICKS",
                   help="transition length in 20 ms ticks (0 = body default, 25)")
    s.add_argument("--for", dest="hold", type=float, default=None, metavar="S",
                   help="hold for S seconds, then exit")
    s.add_argument("--pca", action="store_true",
                   help="print the PCA9685 counts read back at the end of the hold (H2)")
    s.add_argument("--then-watch", type=float, default=None, metavar="T",
                   help="after the --for hold, stop sending at once (no heartbeat, no goal "
                        "refresh) and print the passive timeline for T seconds, relative to the "
                        "last frame sent (the watchdog timeline, H5)")

    s = sub.add_parser("balance", help="balance on/off with live gains (SET_BALANCE), holding a posture "
                                        "until Ctrl-C or --for; prints pitch/roll once a second")
    s.add_argument("state", choices=("on", "off"))
    s.add_argument("--kp", type=float, default=BALANCE_KP, help=f"proportional gain (default {BALANCE_KP})")
    s.add_argument("--kd", type=float, default=BALANCE_KD_S, metavar="S",
                   help=f"rate gain in seconds (default {BALANCE_KD_S})")
    s.add_argument("--deadband", type=float, default=BALANCE_DEADBAND, metavar="DEG",
                   help=f"dead band in degrees (default {BALANCE_DEADBAND})")
    s.add_argument("--clamp", type=float, default=BALANCE_CLAMP, metavar="DEG",
                   help=f"correction limit in degrees (default {BALANCE_CLAMP})")
    s.add_argument("--posture", choices=("stand", "sit"), default="stand", help="posture to hold (default stand)")
    s.add_argument("--transition", type=int, default=0, metavar="TICKS",
                   help="transition length in 20 ms ticks (0 = body default, 25)")
    s.add_argument("--for", dest="hold", type=float, default=None, metavar="S",
                   help="hold for S seconds, then exit")

    s = sub.add_parser("cal", help="joint calibration")
    cs = s.add_subparsers(dest="cal_cmd", metavar="ACTION")
    cs.add_parser("show", help="read the 16 offsets (default)")
    c = cs.add_parser("set", help="set one joint's absolute offset")
    c.add_argument("joint", type=int)
    c.add_argument("degrees", type=float, help="absolute offset in degrees (±25)")
    c = cs.add_parser("save", help="queue a calibration slot write")
    c.add_argument("--no-wait", action="store_true", help="do not wait for EEPROM_SAVED")
    cs.add_parser("revert", help="reload calibration from the slot")

    s = sub.add_parser("param", help="live tuning parameters")
    ps = s.add_subparsers(dest="param_cmd", required=True, metavar="ACTION")
    ps.add_parser("list", help="every param: value, bounds, default, unit")
    c = ps.add_parser("get")
    c.add_argument("id", help="param id (number) or name")
    c = ps.add_parser("set")
    c.add_argument("id", help="param id (number) or name")
    c.add_argument("value", type=int)
    c.add_argument("--persist", action="store_true", help="also write to EEPROM")

    s = sub.add_parser("pca", help="read back the 16 PCA9685 OFF counts (PCA_READ)")

    s = sub.add_parser("eeprom", help="read the body's EEPROM (EEPROM_READ)")
    es = s.add_subparsers(dest="eeprom_cmd", required=True, metavar="ACTION")
    c = es.add_parser("dump", help="hex dump of START..END (END exclusive), e.g. `eeprom dump 0 283`")
    c.add_argument("start", type=lambda x: int(x, 0))
    c.add_argument("end", type=lambda x: int(x, 0))

    s = sub.add_parser("flash", help="build and flash the body (Appendix F)")
    s.add_argument("--legacy", action="store_true", help="flash mind/firmware/opencat-legacy.hex")
    s.add_argument("--no-build", action="store_true", help="flash the last build")
    s.add_argument("--hex", type=Path, help="flash this hex file (implies --no-build)")
    s.add_argument("--sketch", type=Path, help="sketch directory (default: <repo>/body)")
    s.add_argument("--build-dir", type=Path, help="arduino-cli output dir (default ~/.cache/kilo-build)")
    s.add_argument("--reset", choices=("auto", "gpio", "dtr", "none"), default="auto")
    s.add_argument("--reset-ms", type=float, default=float(P.RESET_PULSE_MS),
                   help=f"GPIO4 low time in ms (default {P.RESET_PULSE_MS})")
    s.add_argument("--no-size-check", action="store_true")
    s.add_argument("--no-verify", action="store_true", help="skip the HELLO FLASHED check")
    s.add_argument("--verify-timeout", type=float, default=8.0, metavar="S",
                   help="seconds to wait for the HELLO after flashing (default 8)")
    s.add_argument("--settle-cycles", type=int, default=3, metavar="N",
                   help="open/close cycles that settle DTR/HUPCL after avrdude (default 3)")
    s.add_argument("--dry-run", action="store_true", help="print the commands only")

    s = sub.add_parser("vbody", help="run a virtual body on a pty")
    s.add_argument("--fall-after", type=float, default=None, metavar="S",
                   help="simulate a fall S seconds after start")
    s.add_argument("--no-keys", action="store_true", help="do not read keys from the terminal")
    s.add_argument("--quiet", action="store_true")

    s = sub.add_parser("run", help="run the daemon: own the port, serve the socket (normally via systemd)")
    # Repeated here (SUPPRESS: only set when given) so `kilo run --port P` works too.
    s.add_argument("--port", "-p", default=argparse.SUPPRESS, help="serial port")
    s.add_argument("--socket", default=argparse.SUPPRESS, metavar="PATH", help="socket path")
    s.add_argument("--log", action="store_true", default=argparse.SUPPRESS,
                   help="write the JSON-lines frame log to --log-dir")
    s.add_argument("--kilolog", nargs="?", const="", default=argparse.SUPPRESS, metavar="FILE",
                   help="write the raw wire frames (default <log-dir>/kilo-<time>.kilolog.gz)")
    s.add_argument("--daemon-log", default=None, metavar="FILE",
                   help="the daemon's own rotating log (default <log-dir>/daemon.log; 'none' = off)")
    s.add_argument("--log-dir", default=None, metavar="DIR",
                   help="telemetry/behaviour log dir (default ~/kilo-logs)")
    s.add_argument("--behaviour", action="store_true",
                   help="start kilo.behaviour.runner.run_behaviour if it is installed")
    s.add_argument("--idle-posture", choices=("rest", "limp", "none"), default="rest",
                   help="what the daemon latches when a client's hold ends (default rest, then limp)")
    s.add_argument("--idle-limp", type=float, default=10.0, metavar="S",
                   help="seconds in the idle REST before LIMP (default 10)")

    s = sub.add_parser("daemon", help="status of a running daemon, or stop it")
    s.add_argument("action", choices=("status", "stop"), nargs="?", default="status")
    s.add_argument("--json", action="store_true")
    __import__("kilo.cli_trick").cli_trick.add_parser(sub, LINK_COMMANDS)  # kilo trick (cli_trick.py)
    add_log_parser(sub)
    return p


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = build_parser()
    argv = fix_kilolog_argv(sys.argv[1:] if argv is None else argv, {*LINK_COMMANDS, *PORT_COMMANDS})
    args = parser.parse_args(argv)
    if args.cmd == "cal" and args.cal_cmd is None:
        args.cal_cmd = "show"
    if args.cmd == "posture" and args.then_watch is not None and args.hold is None:
        parser.error("--then-watch needs --for")
    return args


# ---- helpers ---------------------------------------------------------------------
def _setup_logging(verbose: int, daemon: bool = False) -> None:
    level = logging.WARNING if verbose == 0 else logging.INFO if verbose == 1 else logging.DEBUG
    fmt = "%(asctime)s %(name)s %(levelname)s %(message)s"
    if daemon:
        level = min(level, logging.INFO)
        if os.environ.get("JOURNAL_STREAM"):  # under systemd: the journal adds the time
            fmt = "%(name)s %(levelname)s %(message)s"
    logging.basicConfig(level=level, format=fmt)


def _daemon_for(args: argparse.Namespace) -> Any:
    """A DaemonConnection when a daemon serves this command's port, else None.

    ``--direct`` never uses the daemon; an explicit ``--port`` other than the
    daemon's opens that port directly."""
    if getattr(args, "direct", False):
        return None
    from kilo.remote import DaemonError, connect

    try:
        conn = connect(args.socket)
    except DaemonError as e:
        print(f"warning: {e}; opening the port directly", file=sys.stderr)
        return None
    if conn is None:
        return None
    if args.port and os.path.realpath(args.port) != os.path.realpath(conn.info.get("port") or ""):
        conn.close()
        return None
    if args.verbose:
        print(f"daemon pid {conn.info.get('pid')} on {conn.path}, port {conn.info.get('port')}", file=sys.stderr)
    return conn


def _refuse_port_busy(args: argparse.Namespace, conn: Any, what: str) -> int:
    print(f"kilo {args.cmd}: {what} needs the serial port to itself, but the daemon (`kilo run`, pid "
          f"{conn.info.get('pid')}) holds {conn.info.get('port')} and feeds the body's watchdog. Stop it "
          f"first (`kilo daemon stop`, or `systemctl --user stop kilo` under systemd), or pass --port "
          f"for another port.", file=sys.stderr)
    return EXIT_ERR


def _resolve_port(args: argparse.Namespace) -> str:
    choice = detect_port(args.port)
    for w in choice.warnings:
        print(f"warning: {w}", file=sys.stderr)
    if choice.port is None:
        raise SystemExit("kilo: no serial port found; use --port (or run `kilo vbody`)")
    if args.verbose:
        print(f"port {choice.port} ({choice.reason})", file=sys.stderr)
    return choice.port


def open_link(args: argparse.Namespace, passive: bool = False) -> tuple[SerialLink, BodyClient]:
    port = _resolve_port(args)
    logger = None
    if args.log or args.log_file:
        from kilo.log import JsonlLogger, default_path

        logger = JsonlLogger(args.log_file or default_path())
    wire = None
    if getattr(args, "kilolog", None) is not None:
        from kilo.cli_tools import open_kilolog

        wire = open_kilolog(args.kilolog, port, f"kilo {args.cmd}")
    link = SerialLink(lambda: open_port(port, args.baud), logger=logger, port_name=port,
                      passive=passive, wire_logger=wire)
    link.mark_alive()
    link.start()
    return link, BodyClient(link)


def close_link(link: SerialLink) -> None:
    link.stop()
    if link.logger is not None:
        link.logger.close()
    if link.wire_logger is not None:
        link.wire_logger.close()


def _fmt(v: Any, f: str = "{:7.2f}") -> str:
    return "      -" if v is None else f.format(v)


# ---- commands --------------------------------------------------------------------
def cmd_status(args: argparse.Namespace, client: BodyClient) -> int:
    client.wait_ready(args.timeout)
    stats = None
    if not args.no_stats:
        try:
            stats = client.stats()
        except LinkError as e:
            stats = f"unavailable: {e}"
    _fresh_telemetry(client)
    snap = client.state.snapshot()
    snap["body_stats"] = fields_of(stats) if hasattr(stats, "__dataclass_fields__") else stats
    if args.json:
        print(json.dumps(snap, default=str, indent=2))
        return EXIT_OK
    h = snap["hello"] or {}
    print(f"port        {snap['port']}  connected={snap['connected']} alive={snap['alive']}")
    print(f"firmware    proto {h.get('proto_major')}.{h.get('proto_minor')}  build "
          f"{h.get('fw_build', 0):08x}  reset {snap['reset_cause']}  last_alive_tick "
          f"{h.get('last_alive_tick')}  config {consts.CONFIG_STATES.get(h.get('config_state'), h.get('config_state'))}")
    print(f"mode        {snap['mode']}  owners {snap['owners']}  epoch {snap['epoch']}  "
          f"flags {','.join(snap['flags']) or '-'}")
    print(f"attitude    pitch {_fmt(snap['pitch_deg'])}°  roll {_fmt(snap['roll_deg'])}°  "
          f"upright={snap['upright']}")
    print(f"balance     {'ACTIVE' if snap['balance'] else 'off'}")
    print(f"battery     {snap['batt_mv']} mV  ({_fmt(snap['battery_pct'], '{:.0f}')}%)")
    print(f"distance    {_fmt(snap['dist_mm'], '{:.0f}')} mm")
    bs = snap["body_stats"]
    if isinstance(bs, dict):
        print(f"body stats  uptime {bs['uptime_s']} s  max tick {bs['max_tick_us']} us  overruns "
              f"{bs['overruns']}  stack free {bs['stack_free_min']} B  dmp {bs['dmp_rate_hz']} Hz")
        print(f"            rx_crc_err {bs['rx_crc_err']}  rx_overflow {bs['rx_overflow']}  "
              f"i2c_err {bs['i2c_err']}  imu_stale {bs['imu_stale']}")
    else:
        print(f"body stats  {bs}")
    ls = snap["stats"]
    print("link stats  " + " ".join(f"{k}={v}" for k, v in ls.items() if v))
    return EXIT_OK


def _fresh_telemetry(client: BodyClient, timeout: float = 0.3) -> None:
    """Wait for a TELEMETRY that arrives after this point (after the STATS
    reply, so the body has heard this session): right after the port opens
    the reader may still be chewing through telemetry the body sent before
    (buffered in the port), whose flags (link_ok...) predate us. A daemon's
    link has been up all along: nothing to wait for."""
    link, st = client.link, client.state
    if not hasattr(link, "wait_until") or not hasattr(st, "telemetry_at"):
        return
    since = link.clock()
    link.wait_until(lambda: st.telemetry_at is not None and st.telemetry_at > since, timeout)


def render_monitor(snap: dict[str, Any]) -> str:
    rates = snap["rates_dps"] or (None, None, None)
    rows = [
        ("link", f"{'ALIVE' if snap['alive'] else 'LOST'}  port {snap['port']}  "
                 f"age {_fmt(snap['telemetry_age_s'], '{:.3f}')} s  reboots {snap['reboots']}"),
        ("tick", f"{snap['tick']}  load {_fmt(snap['tick_load_ms'], '{:.1f}')} ms  "
                 f"rx_err {snap['rx_err']}  last_rx_seq {snap['last_rx_seq']}"),
        ("mode", f"{snap['mode']}  owners {snap['owners']}  epoch {snap['epoch']}"),
        ("flags", ",".join(snap["flags"]) or "-"),
        ("balance", "ACTIVE" if snap["balance"] else "off"),
        ("pitch/roll", f"{_fmt(snap['pitch_deg'])}° {_fmt(snap['roll_deg'])}°  "
                       f"upright={snap['upright']} fallen={snap['fallen']} moving={snap['moving']}"),
        ("rates", " ".join(_fmt(r) for r in rates) + " °/s"),
        ("distance", f"{_fmt(snap['dist_mm'], '{:.0f}')} mm  approach "
                     f"{_fmt(snap['approach_mm_s'], '{:.0f}')} mm/s"),
        ("battery", f"{snap['batt_mv']} mV  {_fmt(snap['battery_pct'], '{:.0f}')}%"),
    ]
    out = [f"{k:>11}  {v}" for k, v in rows]
    out.append("     events  " + ("  ".join(
        f"-{age:.1f}s {consts.EVENTS.get(e.get('code'), e.get('code'))}({e.get('arg')})"
        for age, e in reversed(snap["events"][-5:])) or "-"))
    st = snap["stats"]
    out.append(f"      frames  rx {st['frames_rx']} tx {st['frames_tx']} hb {st['heartbeats']} "
               f"decode_err {sum(st['decode_errors'].values())} retries {st['retries']} "
               f"timeouts {st['timeouts']} reconnects {st['reconnects']}")
    return "\n".join(out)


def _wall(t: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t % 1 * 1000):03d}"


def watch_timeline(client: BodyClient, t0: float, duration: float | None) -> int:
    """Print the passive timeline (kilo.body.timeline) until ``duration`` or Ctrl-C."""
    from kilo.body.timeline import Timeline

    st = client.link.state.stats
    sent_before = st.frames_tx
    tl = Timeline(client.link, t0)
    end = None if duration is None else t0 + duration
    try:
        while end is None or client.link.clock() < end:
            tl.poll()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    print(f"done: {st.frames_tx - sent_before} frames sent while watching "
          f"({st.frames_tx} in total)")
    return EXIT_OK


def cmd_monitor(args: argparse.Namespace, client: BodyClient) -> int:
    if args.percepts and not args.passive:
        from kilo.cli_tools import monitor_percepts

        return monitor_percepts(args, client)
    if args.passive:
        t0 = client.link.clock()
        print(f"passive monitor on {client.state.port} from {_wall(time.time())} (t=0): sending "
              f"nothing, the body's watchdog is not fed"
              + (f"; {args.duration:g} s" if args.duration is not None else "; Ctrl-C to quit"),
              flush=True)
        return watch_timeline(client, t0, args.duration)
    table = sys.stdout.isatty() and not args.json
    period = 1.0 / max(args.rate, 0.5)
    end = None if args.duration is None else time.monotonic() + args.duration
    next_at = time.monotonic()
    try:
        while end is None or time.monotonic() < end:
            client.link.mark_alive()
            snap = client.state.snapshot()
            if table:
                sys.stdout.write("\x1b[H\x1b[2J" + render_monitor(snap) + "\n(Ctrl-C to quit)\n")
                sys.stdout.flush()
            else:
                print(json.dumps(monitor_record(snap)), flush=True)
            # fixed-rate schedule: the snapshot and print time do not stretch the period
            next_at += period
            delay = next_at - time.monotonic()
            if delay < 0:
                next_at, delay = time.monotonic(), 0.0
            time.sleep(delay)
    except KeyboardInterrupt:
        pass
    return EXIT_OK


MONITOR_JSON_KEYS = ("tick", "mode", "epoch", "pitch_deg", "roll_deg", "balance", "batt_mv", "alive")


def monitor_record(snap: dict[str, Any]) -> dict[str, Any]:
    """One `kilo monitor --json` line: wall time plus the main fields."""
    return {"time": round(time.time(), 3), **{k: snap[k] for k in MONITOR_JSON_KEYS}}


def cmd_posture(args: argparse.Namespace, client: BodyClient) -> int:
    client.wait_ready(args.timeout)
    client.posture(args.name, transition_ticks=args.transition)
    print(f"holding {args.name}" + (f" for {args.hold:g} s" if args.hold else " (Ctrl-C to release)"))
    end = None if args.hold is None else time.monotonic() + args.hold
    try:
        while end is None or time.monotonic() < end:
            client.spin(0.25)
            if "SET_POSTURE" not in client.link.goals():
                st = client.state
                print(f"posture goal cleared by the body (mode {st.mode}, epoch {st.epoch}); exiting")
                return EXIT_ERR
    except KeyboardInterrupt:
        if args.then_watch is not None:
            print("interrupted: not watching")
        args.then_watch = None
    if args.pca:
        _print_pca(client.pca_counts())
    if args.then_watch is None:
        return EXIT_OK
    # Silence first, print after: nothing (no goal refresh, no heartbeat) goes
    # out once go_silent() returns.
    last = client.link.go_silent()
    hb = client.link.last_heartbeat
    if last is None:
        t0 = client.link.clock()
        print(f"silent from {_wall(time.time())} (t=0); nothing was sent", flush=True)
    else:
        t0, wall, name = last
        print(f"silent: last frame {name} sent at {_wall(wall)} (t=0)"
              + (f", last HEARTBEAT at {_wall(hb[1])} (t={hb[0] - t0:+.2f})" if hb else "")
              + f"; watching {args.then_watch:g} s", flush=True)
    return watch_timeline(client, t0, args.then_watch)


def cmd_balance(args: argparse.Namespace, client: BodyClient) -> int:
    on = args.state == "on"
    client.wait_ready(args.timeout)
    f = client.set_balance(on, kp=args.kp, kd_s=args.kd, deadband_deg=args.deadband, clamp_deg=args.clamp)
    client.posture(args.posture, transition_ticks=args.transition, balance=on)
    print(f"balance {args.state}: {args.posture}, Kp {f['kp_q8'] / 256:.3f}  Kd {f['kd_ms'] / 1000:.3f} s  "
          f"dead band {f['deadband_deg']}°  clamp {f['max_corr_deg']}°"
          + (f"; holding {args.hold:g} s" if args.hold else "; Ctrl-C to release"), flush=True)
    t0 = time.monotonic()
    end = None if args.hold is None else t0 + args.hold
    try:
        while end is None or time.monotonic() < end:
            client.spin(1.0)
            if "SET_POSTURE" not in client.link.goals():
                st = client.state
                print(f"posture goal cleared by the body (mode {st.mode}, epoch {st.epoch}); exiting")
                return EXIT_ERR
            s = client.state.snapshot()
            rates = s["rates_dps"] or (None, None, None)
            print(f"t={time.monotonic() - t0:5.1f}s  pitch {_fmt(s['pitch_deg'])}°  roll {_fmt(s['roll_deg'])}°  "
                  f"rate p/r {_fmt(rates[0])} {_fmt(rates[1])} °/s  balance "
                  f"{'ACTIVE' if s['balance'] else 'off'}  mode {s['mode']}", flush=True)
    except KeyboardInterrupt:
        pass
    return EXIT_OK


def cmd_cal(args: argparse.Namespace, client: BodyClient) -> int:
    client.wait_ready(args.timeout, require_telemetry=False)
    if args.cal_cmd == "show":
        offs = client.cal_get()
        for j, v in enumerate(offs):
            print(f"joint {j:2d}  {v / 16:+7.2f}°  ({v:+d} a16)")
    elif args.cal_cmd == "set":
        client.cal_set(args.joint, int(round(args.degrees * 16)))
        print(f"joint {args.joint} offset {args.degrees:+.2f}° (not saved; `kilo cal save`)")
    elif args.cal_cmd == "save":
        ok = client.cal_save(None if args.no_wait else 2.0)
        print("saved" if ok else "queued, but no EEPROM_SAVED within 2 s")
        return EXIT_OK if ok else EXIT_ERR
    elif args.cal_cmd == "revert":
        client.cal_revert()
        print("reverted to the saved calibration")
    return EXIT_OK


def cmd_param(args: argparse.Namespace, client: BodyClient) -> int:
    client.wait_ready(args.timeout, require_telemetry=False)
    if args.param_cmd == "list":
        for v in client.param_list():
            print(_param_line(v))
    elif args.param_cmd == "get":
        print(_param_line(client.param_get(args.id)))
    else:
        info = client.param_info(args.id)
        client.param_set(args.id, args.value, args.persist)
        print(f"{info.name} = {args.value} {info.unit}" + (" (persist queued)" if args.persist else ""))
    return EXIT_OK


def _param_line(v: Any) -> str:
    info = consts.PARAMS.get(v.id)
    name = info.name if info else f"#{v.id}"
    unit = info.unit if info else ""
    return (f"{v.id:2d} {name:<20} {v.value:>6} {unit:<9} [{v.min}..{v.max}] default {v.default}"
            + ("" if v.value == v.default else "  *"))


def cmd_pca(args: argparse.Namespace, client: BodyClient) -> int:
    client.wait_ready(args.timeout, require_telemetry=False)
    _print_pca(client.pca_counts())
    return EXIT_OK


def _print_pca(counts: list[int]) -> None:
    for ch, c in enumerate(counts):
        print(f"channel {ch:2d}  " + ("off" if c & P.PCA_FULL_OFF_BIT else f"{c:5d}"))


def cmd_eeprom(args: argparse.Namespace, client: BodyClient) -> int:
    if not 0 <= args.start < args.end <= 1024:
        raise ValueError("need 0 <= START < END <= 1024")
    client.wait_ready(args.timeout, require_telemetry=False)
    data = bytearray()
    addr = args.start
    while addr < args.end:
        n = min(P.EEPROM_READ_MAX, args.end - addr)
        data += client.eeprom_read(addr, n)
        addr += n
    for off in range(0, len(data), 16):
        row = data[off:off + 16]
        print(f"{args.start + off:04d}: " + " ".join(f"{b:02x}" for b in row))
    return EXIT_OK


def cmd_flash(args: argparse.Namespace) -> int:
    from kilo.tools.flash import (DEFAULT_BUILD_DIR, DEFAULT_SKETCH, FlashError, FlashOptions, VerifyResult,
                                  flash, is_usb_port, verify_hello)

    conn = None if args.dry_run else _daemon_for(args)
    port = conn.info["port"] if conn is not None else _resolve_port(args)
    opts = FlashOptions(
        port=port, legacy=args.legacy, build=not (args.no_build or args.hex), hex_path=args.hex,
        sketch=args.sketch or DEFAULT_SKETCH, build_dir=args.build_dir or DEFAULT_BUILD_DIR,
        reset=args.reset, reset_ms=args.reset_ms, size_check=not args.no_size_check,
        verify=not args.no_verify, dry_run=args.dry_run, verify_timeout=args.verify_timeout,
        settle_cycles=args.settle_cycles, baud=args.baud, verbose=bool(args.verbose))
    # Over a USB adapter the settle reset (DTR) makes the HELLO say WARM_UNEXPECTED.
    any_cause = is_usb_port(port)
    holder: dict[str, SerialLink] = {}

    def verify(since: float, expected_build: int | None, timeout: float) -> Any:
        if conn is not None:
            r = conn.call("verify_hello", timeout=timeout + 10, since=since, expected_build=expected_build,
                          any_cause=any_cause, wait_s=timeout)
            return VerifyResult(bool(r["ok"]), r.get("hello"))
        link = SerialLink(lambda: open_port(port, args.baud), port_name=port)
        holder["link"] = link
        link.mark_alive()
        link.start()
        return verify_hello(link, since, expected_build, timeout=timeout, any_cause=any_cause)

    lease = DaemonPortLease(conn, port) if conn is not None else None
    try:
        flash(opts, link=lease, verify=verify)
    except FlashError as e:
        print(f"kilo flash: {e}", file=sys.stderr)
        return EXIT_ERR
    finally:
        if "link" in holder:
            holder["link"].stop()
        if conn is not None:
            if lease.released:
                try:
                    lease.start()
                except Exception:
                    pass
            conn.close()
    return EXIT_OK


class DaemonPortLease:
    """flash()'s ``link`` when a daemon owns the port: stop() asks it to
    release the port (with a lease, so a crashed flash cannot keep it; the
    daemon also reacquires when this connection drops), start() to reacquire."""

    LEASE_S = 300.0

    def __init__(self, conn: Any, port: str):
        self.conn = conn
        self.port = port
        self.released = False

    @property
    def running(self) -> bool:
        return not self.released

    def stop(self) -> None:
        self.conn.call("release_port", lease_s=self.LEASE_S)
        self.released = True
        print(f"daemon released {self.port}", flush=True)

    def start(self) -> None:
        self.conn.call("reacquire_port")
        self.released = False
        print(f"daemon reacquired {self.port}", flush=True)


def cmd_vbody(args: argparse.Namespace) -> int:
    from kilo.vbody.sim import VirtualBody, open_pty

    transport, path, slave_fd = open_pty()
    vb = VirtualBody(transport, fall_after=args.fall_after, verbose=not args.quiet)
    print(path, flush=True)
    keys = not args.no_keys and sys.stdin.isatty()
    if not args.quiet:
        print(f"virtual body on {path}: run `kilo --port {path} monitor`"
              + ("; keys: f/F fall right/left, r reboot, q quit" if keys else "; Ctrl-C to quit"),
              file=sys.stderr, flush=True)
    vb.start()
    old = None
    try:
        if keys:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            while True:
                r, _, _ = select.select([fd], [], [], 0.2)
                if not r:
                    continue
                ch = os.read(fd, 1).decode(errors="ignore")
                if ch == "f":
                    vb.trigger_fall(1)
                elif ch == "F":
                    vb.trigger_fall(-1)
                elif ch == "r":
                    vb.reboot(3)
                elif ch in ("q", "\x04"):
                    break
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        if old is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)
        vb.stop()
        transport.close()
        os.close(slave_fd)
    return EXIT_OK


def cmd_run(args: argparse.Namespace) -> int:
    from kilo.daemon import run_daemon

    return run_daemon(args)


def cmd_daemon(args: argparse.Namespace) -> int:
    from kilo.cli_tools import daemon_link_lines, print_tried
    from kilo.remote import DaemonError, connect, socket_path

    tried: list[Any] = []
    try:
        conn = connect(args.socket, tried=tried)
    except DaemonError as e:
        print(f"kilo daemon: {e}", file=sys.stderr)
        print_tried(tried)
        return EXIT_ERR
    if conn is None:
        print(f"no daemon listening on {socket_path(args.socket)}")
        print_tried(tried)
        return EXIT_ERR if args.action == "status" else EXIT_OK
    path = conn.path
    with conn:
        info = conn.info
        if args.action == "stop":
            conn.call("shutdown")
            end = time.monotonic() + 10
            while path.exists() and time.monotonic() < end:
                time.sleep(0.1)
            print(f"stopped daemon pid {info.get('pid')}" if not path.exists()
                  else f"daemon pid {info.get('pid')} did not stop within 10 s")
            return EXIT_OK if not path.exists() else EXIT_ERR
    if args.json:
        print(json.dumps(info, indent=2))
        return EXIT_OK
    print(f"daemon      pid {info['pid']}  kilo {info['version']}  up {info['uptime_s']:.0f} s  "
          f"clients {info['clients']}")
    print(f"socket      {info['socket']}")
    print(f"port        {info['port']}  " + ("RELEASED (flashing)" if info["port_released"] else
                                             f"connected={info['connected']} alive={info['alive']}"))
    for line in daemon_link_lines(info):
        print(line)
    print(f"holds       {', '.join(info['held_goals']) or '-'}  idle {info['idle'] or '-'}")
    print(f"behaviour   {info['behaviour']}")
    print(f"frame log   {info['log_file'] or '-'}")
    return EXIT_OK


LINK_COMMANDS = {"status": cmd_status, "monitor": cmd_monitor, "posture": cmd_posture, "balance": cmd_balance,
                 "cal": cmd_cal, "param": cmd_param, "pca": cmd_pca, "eeprom": cmd_eeprom}
# Commands that manage the port (or the daemon) themselves: cmd(args).
PORT_COMMANDS = {"flash": cmd_flash, "vbody": cmd_vbody, "run": cmd_run, "daemon": cmd_daemon,
                 "log": cmd_log}
# Link-command modes that must send nothing, so they cannot share the daemon's port.
SILENT_MODES = {"monitor": ("passive", "--passive"), "posture": ("then_watch", "--then-watch")}


def _silent_mode(args: argparse.Namespace) -> str | None:
    attr, flag = SILENT_MODES.get(args.cmd, (None, None))
    return flag if attr is not None and getattr(args, attr, None) not in (None, False) else None


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    _setup_logging(args.verbose, daemon=args.cmd == "run")
    if args.cmd in PORT_COMMANDS:
        return PORT_COMMANDS[args.cmd](args)
    conn = _daemon_for(args)
    if conn is not None:
        silent = _silent_mode(args)
        if silent is not None:
            conn.close()
            return _refuse_port_busy(args, conn, f"`kilo {args.cmd} {silent}`")
        from kilo.remote import RemoteBody

        if args.log or args.log_file or args.kilolog is not None:
            print("note: through the daemon, frames are logged by the daemon (`kilo run --log`/`--kilolog`)",
                  file=sys.stderr)
        link, client = None, RemoteBody(conn)
    else:
        link, client = open_link(args, passive=args.cmd == "monitor" and args.passive)
    try:
        return LINK_COMMANDS[args.cmd](args, client)
    except ProtocolMismatch as e:
        print(f"kilo {args.cmd}: {e}", file=sys.stderr)
        return EXIT_ERR
    except CommandError as e:
        print(f"kilo {args.cmd}: body refused: {e.status_name}", file=sys.stderr)
        return EXIT_ERR
    except (LinkError, KeyError, ValueError) as e:
        print(f"kilo {args.cmd}: {e}", file=sys.stderr)
        return EXIT_ERR
    except DaemonError as e:
        print(f"kilo {args.cmd}: daemon: {e}", file=sys.stderr)
        return EXIT_ERR
    finally:
        if link is not None:
            close_link(link)
        else:
            client.close()


if __name__ == "__main__":
    sys.exit(main())
