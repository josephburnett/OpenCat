"""The ``.kilolog`` raw frame log (docs/architecture.md §5 "Logging").

A binary file of wire frames exactly as they crossed the UART, so a session
can be replayed through the codec. ``kilo --kilolog [FILE]`` and ``kilo run
--kilolog [FILE]`` hand a ``KilologWriter`` to the SerialLink, which calls
``write_wire(wire, "rx"|"tx")`` once per frame from its reader and writer
paths (default file ``~/kilo-logs/kilo-<time>.kilolog.gz``). ``kilo --log``
writes JSONL (``kilo.log``), which ``kilo.logs.reader`` reads too.

Format, version 1 (all integers little-endian)::

    header   8 B   magic b"KILOLOG\\n"
             2 B   u16 meta_len
             n B   meta: UTF-8 JSON object, at least
                   {"version": 1, "proto": [major, minor], "start_wall": time.time(),
                    "start_mono": time.monotonic(), "tick_hz": 50}
    record   1 B   u8 flags: bit 0 direction (0 = rx, body -> Pi; 1 = tx, Pi -> body)
             4 B   u32 t_ms since start_mono (wraps after 49 days)
             1 B   u8 n
             n B   the wire frame: COBS(type | seq | payload | crc16 LE) | 0x00

A record is 6 B + the frame: a TELEMETRY frame is 32-33 B on the wire, so
25 Hz telemetry is ≈39 B x 25 x 3600 ≈ 3.5 MB/h raw. Gzip brings that to
≈1 MB/h (the §5 estimate); the writer and reader handle ``*.kilolog.gz``
transparently. A truncated last record (crash mid-write) is ignored.
"""

from __future__ import annotations

import gzip
import json
import os
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO, Iterator

from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P

MAGIC = b"KILOLOG\n"
VERSION = 1
_REC = struct.Struct("<BIB")
DIR_RX, DIR_TX = 0, 1


def default_path() -> Path:
    """``~/kilo-logs/kilo-YYYYmmdd-HHMMSS.kilolog.gz``."""
    return Path.home() / "kilo-logs" / time.strftime("kilo-%Y%m%d-%H%M%S.kilolog.gz")


def _open(path: Path, mode: str) -> BinaryIO:
    if path.suffix == ".gz":
        return gzip.open(path, mode)  # type: ignore[return-value]
    return open(path, mode)


def is_kilolog(path: str | os.PathLike) -> bool:
    try:
        with _open(Path(path), "rb") as f:
            return f.read(len(MAGIC)) == MAGIC
    except (OSError, EOFError, gzip.BadGzipFile):
        return False


class KilologWriter:
    """Append wire frames to a .kilolog. Thread-safe; ``close()`` when done."""

    def __init__(self, path: str | os.PathLike, meta: dict[str, Any] | None = None,
                 clock=time.monotonic):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.clock = clock
        self.start_mono = clock()
        self.meta = {"version": VERSION, "proto": [P.PROTO_MAJOR, P.PROTO_MINOR],
                     "start_wall": time.time(), "start_mono": self.start_mono,
                     "tick_hz": P.TICK_HZ, **(meta or {})}
        m = json.dumps(self.meta, separators=(",", ":")).encode()
        self._lock = threading.Lock()
        self._f = _open(self.path, "wb")
        self._f.write(MAGIC + struct.pack("<H", len(m)) + m)
        self.records = 0

    def write_wire(self, wire: bytes, direction: str = "rx", t: float | None = None) -> None:
        wire = bytes(wire)
        if len(wire) > 255:
            raise ValueError("frame longer than 255 B")
        t = self.clock() if t is None else t
        t_ms = int(round((t - self.start_mono) * 1000)) & 0xFFFFFFFF
        rec = _REC.pack(DIR_TX if direction == "tx" else DIR_RX, t_ms, len(wire)) + wire
        with self._lock:
            if self._f.closed:
                return  # frames racing the close are dropped
            self._f.write(rec)
            self.records += 1

    def write_frame(self, type: int, seq: int, payload: bytes, direction: str = "rx",
                    t: float | None = None) -> None:
        self.write_wire(C.build_frame(type, seq, payload), direction, t)

    def write_message(self, msg: Any, seq: int = 0, direction: str = "rx", t: float | None = None) -> None:
        """A generated kilo_protocol dataclass instance."""
        self.write_wire(C.encode_message(msg, seq), direction, t)

    def flush(self) -> None:
        """Push buffered records to disk (a sync point in a .gz, so a killed
        writer leaves a readable file up to here)."""
        with self._lock:
            if not self._f.closed:
                self._f.flush()

    def close(self) -> None:
        with self._lock:
            if not self._f.closed:
                self._f.close()

    def __enter__(self) -> "KilologWriter":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()


@dataclass
class KilologRecord:
    t: float  # seconds since start_mono
    direction: str  # "rx" or "tx"
    wire: bytes
    frame: C.Frame | None
    error: str | None  # FrameError kind when the frame does not parse


def read_kilolog(path: str | os.PathLike) -> tuple[dict[str, Any], Iterator[KilologRecord]]:
    """(meta, records). Records are parsed through the reference codec."""
    path = Path(path)
    f = _open(path, "rb")
    head = f.read(len(MAGIC) + 2)
    if head[:len(MAGIC)] != MAGIC or len(head) < len(MAGIC) + 2:
        f.close()
        raise ValueError(f"{path}: not a kilolog")
    (n,) = struct.unpack("<H", head[len(MAGIC):])
    meta = json.loads(f.read(n).decode())
    if meta.get("version") != VERSION:
        f.close()
        raise ValueError(f"{path}: kilolog version {meta.get('version')} not supported")

    def gen() -> Iterator[KilologRecord]:
        wraps, last_ms = 0, 0
        with f:
            while True:
                try:
                    h = f.read(_REC.size)
                    if len(h) < _REC.size:
                        return
                    flags, t_ms, n = _REC.unpack(h)
                    wire = f.read(n)
                except (EOFError, gzip.BadGzipFile, OSError):
                    return  # a .gz cut short (crash or kill mid-write): keep what was read
                if len(wire) < n:
                    return
                if t_ms < last_ms - (1 << 31):
                    wraps += 1
                last_ms = t_ms
                t = (t_ms + (wraps << 32)) / 1000.0
                try:
                    fr, err = C.parse_frame(wire), None
                except C.FrameError as e:
                    fr, err = None, e.kind
                yield KilologRecord(t, "tx" if flags & 1 else "rx", wire, fr, err)

    return meta, gen()
