"""Kilo log replay and analysis (docs/architecture.md §5 "Logging").

- ``reader.load(path)``: link JSONL (``kilo --log``), monitor JSONL, or a raw
  ``.kilolog`` -> ``LogData`` (telemetry samples, events, link counters);
- ``kilolog``: the raw frame log format, ``KilologWriter`` and its reader;
- ``analysis``: ``summary``, ``replay`` through ``kilo.perception.Percepts``,
  ``plot`` (matplotlib, else CSV).

Command line: ``python -m kilo.logs {summary,replay,plot} FILE ...``.
"""

from kilo.logs.kilolog import KilologWriter, read_kilolog
from kilo.logs.reader import LogData, LogEvent, TelemetrySample, load

__all__ = ["KilologWriter", "LogData", "LogEvent", "TelemetrySample", "load", "read_kilolog"]
