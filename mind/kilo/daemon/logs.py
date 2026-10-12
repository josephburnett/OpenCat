"""The daemon's logs: its own log file and the log directory's size cap.

``run_daemon`` writes the daemon's own log to ``<log-dir>/daemon.log`` (1 MB
x 3, rotating; ``--daemon-log FILE|none``) as well as to stderr, because the
user journal is not always readable or persistent on the Pi. The file is
written through a QueueHandler by a QueueListener thread, so a slow disk
never stalls the link's reader and writer threads, which log too; a full
queue drops the record rather than block.

``--kilolog [FILE]`` hands a ``KilologWriter`` to the link (raw wire frames
both ways; its own thread writes them, flushed every 5 s by the pump).
``--log`` (the JSON-lines frame log) is off unless asked for.

The log directory is kept under ``log_cap_bytes`` (500 MB): at start and
hourly (from the pump, in the pool) ``prune_logs`` deletes the oldest kilo
logs beyond it (``kilo.log.prune_logs``: never an open one, nor one in
``keep_logs``).
"""

from __future__ import annotations

import logging
import logging.handlers
from pathlib import Path
from typing import Any

from kilo.log import prune_logs

log = logging.getLogger("kilo.daemon")

DAEMON_LOG_BYTES = 1 << 20
DAEMON_LOG_BACKUPS = 3


class LogsMixin:
    def prune_logs(self) -> list[Path]:
        """Keep the log directory under ``log_cap_bytes`` (pool thread)."""
        keep = set(self.keep_logs)
        for lg in (self.logger, self.wire_logger):
            if lg is not None and getattr(lg, "path", None) is not None:
                keep.add(Path(lg.path))
        try:
            gone = prune_logs(self.log_dir, self.log_cap_bytes, keep)
        except Exception:
            log.exception("pruning %s", self.log_dir)
            return []
        if gone:
            log.warning("log directory %s over %d MB: deleted %d oldest files (%s)", self.log_dir,
                        self.log_cap_bytes >> 20, len(gone), ", ".join(p.name for p in gone[:5])
                        + (" ..." if len(gone) > 5 else ""))
        return gone


def add_file_log(path: Path) -> logging.Handler | None:
    """Also write the daemon's log to ``path`` (rotating, small): there is
    always something to read even when the user journal is not.

    The file is written by a QueueListener thread: the link's reader and
    writer threads log too, and must never wait for the disk. Returns the
    QueueHandler on the root logger (its ``listener`` is stopped at exit)."""
    import atexit
    import queue
    from logging.handlers import QueueListener, RotatingFileHandler

    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fh = RotatingFileHandler(path, maxBytes=DAEMON_LOG_BYTES, backupCount=DAEMON_LOG_BACKUPS)
    except OSError as e:
        log.warning("cannot write the daemon log %s: %s", path, e)
        return None
    fh.setFormatter(logging.Formatter("%(asctime)s %(name)s %(levelname)s %(message)s"))
    q: queue.Queue[Any] = queue.Queue(10000)
    h = _DroppingQueueHandler(q)
    h.setLevel(logging.INFO)
    h.listener = QueueListener(q, fh, respect_handler_level=False)  # type: ignore[attr-defined]
    h.listener.start()  # type: ignore[attr-defined]
    atexit.register(h.listener.stop)  # type: ignore[attr-defined]
    root = logging.getLogger()
    root.addHandler(h)
    if root.level > logging.INFO or root.level == logging.NOTSET:
        root.setLevel(logging.INFO)
    log.info("daemon log %s", path)
    return h


class _DroppingQueueHandler(logging.handlers.QueueHandler):
    """A QueueHandler that drops a record rather than block when its queue is full."""

    def enqueue(self, record: logging.LogRecord) -> None:
        try:
            self.queue.put_nowait(record)
        except Exception:  # queue.Full: the disk is that slow
            pass
