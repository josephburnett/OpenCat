"""``kilo run``: from kilo.cli's argparse namespace to a running Daemon."""

from __future__ import annotations

import logging
import os
import time
from pathlib import Path
from typing import Any

from kilo.daemon.core import Daemon
from kilo.daemon.logs import add_file_log

log = logging.getLogger("kilo.daemon")


def run_daemon(args: Any) -> int:
    """``kilo run``: argparse namespace from kilo.cli."""
    from kilo.link.ports import detect_port

    log_dir = Path(args.log_dir).expanduser() if getattr(args, "log_dir", None) else Path.home() / "kilo-logs"
    daemon_log = getattr(args, "daemon_log", None)
    if daemon_log != "none":
        add_file_log(Path(daemon_log).expanduser() if daemon_log else log_dir / "daemon.log")
    choice = detect_port(args.port)
    for w in choice.warnings:
        log.warning("%s", w)
    explicit = choice.reason in ("--port", "$KILO_PORT")
    if choice.port is None:
        log.warning("no serial port found yet: waiting for one to appear (set KILO_PORT in "
                    "~/.config/kilo/env or pass --port to pin it)")
    else:
        log.info("port %s (%s%s)", choice.port, choice.reason, "" if explicit else "; re-detected on reconnect")
        if not os.path.exists(choice.port):
            log.warning("%s does not exist yet (robot unplugged?): waiting for it", choice.port)
    logger = None
    if args.log or args.log_file:
        from kilo.log import JsonlLogger

        logger = JsonlLogger(args.log_file or log_dir / time.strftime("kilo-%Y%m%d-%H%M%S.jsonl"))
        log.info("frame log %s", logger.path)
    wire = None
    kl = getattr(args, "kilolog", None)
    if kl is not None:
        from kilo.logs.kilolog import KilologWriter

        wire = KilologWriter(Path(kl).expanduser() if kl else log_dir / time.strftime("kilo-%Y%m%d-%H%M%S.kilolog.gz"),
                             meta={"source": "kilo run", "port": choice.port})
        log.info("raw frame log %s", wire.path)
    keep = [log_dir / "daemon.log"] if daemon_log != "none" and not daemon_log else []
    d = Daemon(choice.port, baud=args.baud, socket=args.socket, logger=logger,
               idle_posture=args.idle_posture, idle_limp_s=args.idle_limp, behaviour=args.behaviour,
               remote=getattr(args, "remote", False), balance_kp=getattr(args, "balance", None),
               log_dir=log_dir, wire_logger=wire,
               auto_port=not explicit, keep_logs=keep)
    return d.run()
