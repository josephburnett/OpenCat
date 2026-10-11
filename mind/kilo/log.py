"""Structured JSON-lines logging off the control path (§5 "Logger").

``JsonlLogger.log(kind, **fields)`` only enqueues; a writer thread serialises
and writes, so a disk stall never delays the link or the behaviour loop. When
the queue is full, records are dropped and counted (``dropped``), and a
``log_dropped`` record is written once space frees up.

Each line is one JSON object with ``t`` (time.monotonic()), ``wall``
(time.time()) and ``kind``. The link writes ``rx``/``tx`` records with the
frame's type, seq and payload hex (so a log can be replayed through the codec)
and, when decodable, the fields; plus ``event``, ``link`` and ``goal`` records.
Files rotate at ``max_bytes`` (``name.jsonl`` -> ``name.jsonl.1`` ...).
"""

from __future__ import annotations

import json
import os
import queue
import threading
import time
from pathlib import Path
from typing import Any

DEFAULT_DIR = Path.home() / "kilo-logs"


def default_path() -> Path:
    return DEFAULT_DIR / time.strftime("kilo-%Y%m%d-%H%M%S.jsonl")


class JsonlLogger:
    def __init__(self, path: str | os.PathLike, max_bytes: int = 64 << 20,
                 backups: int = 5, queue_size: int = 10000):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.max_bytes = max_bytes
        self.backups = backups
        self.dropped = 0
        self._q: queue.Queue[dict[str, Any] | None] = queue.Queue(queue_size)
        self._f = open(self.path, "a", encoding="utf-8")
        self._thread = threading.Thread(target=self._run, name="kilo-log", daemon=True)
        self._thread.start()

    def log(self, kind: str, **fields: Any) -> None:
        rec = {"t": round(time.monotonic(), 6), "wall": round(time.time(), 6),
               "kind": kind, **fields}
        try:
            self._q.put_nowait(rec)
        except queue.Full:
            self.dropped += 1

    def close(self, timeout: float = 2.0) -> None:
        try:
            self._q.put(None, timeout=timeout)
        except queue.Full:
            pass
        self._thread.join(timeout)

    def _rotate(self) -> None:
        self._f.close()
        for i in range(self.backups - 1, 0, -1):
            src = self.path.with_name(f"{self.path.name}.{i}")
            if src.exists():
                src.replace(self.path.with_name(f"{self.path.name}.{i + 1}"))
        self.path.replace(self.path.with_name(f"{self.path.name}.1"))
        self._f = open(self.path, "a", encoding="utf-8")

    def _run(self) -> None:
        reported = 0
        while True:
            rec = self._q.get()
            if rec is None:
                break
            try:
                if self.dropped != reported:
                    self._f.write(json.dumps({"t": rec["t"], "kind": "log_dropped",
                                              "count": self.dropped - reported}) + "\n")
                    reported = self.dropped
                self._f.write(json.dumps(rec, default=_default, separators=(",", ":")) + "\n")
                if self._f.tell() >= self.max_bytes:
                    self._rotate()
                elif self._q.empty():
                    self._f.flush()
            except OSError:
                self.dropped += 1
        self._f.flush()
        self._f.close()


# ---- the log directory's size --------------------------------------------------------
LOG_DIR_MAX_BYTES = 500 << 20   # ~/kilo-logs is pruned to this (oldest files first)
PRUNE_PREFIXES = ("kilo-", "behaviour-", "daemon.log.")
PRUNE_MIN_AGE_S = 600.0          # never a file written in the last 10 minutes


def prune_logs(log_dir: str | os.PathLike, max_bytes: int = LOG_DIR_MAX_BYTES,
               keep: "set[Path] | frozenset[Path]" = frozenset(), now: float | None = None) -> list[Path]:
    """Delete the oldest log files in ``log_dir`` (not recursive) until the
    total is at most ``max_bytes``. Only kilo's own logs (``kilo-*``,
    ``behaviour-*``, rotated ``daemon.log.N``) are candidates; the files in
    ``keep`` (open logs) and anything modified in the last ``PRUNE_MIN_AGE_S``
    are never touched (they still count towards the total). Returns the
    deleted paths."""
    d = Path(log_dir)
    now = time.time() if now is None else now
    keep = {Path(p).resolve() for p in keep}
    files: list[tuple[float, int, Path, bool]] = []
    try:
        entries = list(d.iterdir())
    except OSError:
        return []
    total = 0
    for p in entries:
        try:
            st = p.stat()
        except OSError:
            continue
        if not p.is_file():
            continue
        total += st.st_size
        ok = (p.name.startswith(PRUNE_PREFIXES) and p.resolve() not in keep
              and now - st.st_mtime >= PRUNE_MIN_AGE_S)
        files.append((st.st_mtime, st.st_size, p, ok))
    deleted: list[Path] = []
    for _mtime, size, p, ok in sorted(files):
        if total <= max_bytes:
            break
        if not ok:
            continue
        try:
            p.unlink()
        except OSError:
            continue
        total -= size
        deleted.append(p)
    return deleted


def _default(o: Any) -> Any:
    if isinstance(o, (bytes, bytearray)):
        return o.hex()
    if hasattr(o, "as_dict"):
        return o.as_dict()
    return repr(o)
