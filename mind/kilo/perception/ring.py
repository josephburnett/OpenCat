"""A fixed-capacity ring buffer of telemetry rows for detectors (§5 "Body state").

Columns are floats; missing values are NaN. Uses numpy when it is importable
(an apt dependency on the Pi, ``python3-numpy``), else ``array('d')`` columns,
with the same API. ``window(seconds)`` returns the rows of the last
``seconds`` (by the ``t`` column), oldest first, as a dict of column ->
sequence (numpy arrays or lists).
"""

from __future__ import annotations

import math
from array import array
from typing import Any, Sequence

try:  # numpy is optional
    import numpy as _np
except ImportError:  # pragma: no cover - depends on the environment
    _np = None

HAVE_NUMPY = _np is not None

COLUMNS: tuple[str, ...] = (
    "t", "pitch", "roll", "rate_pitch", "rate_roll", "rate_yaw",
    "dist_mm", "batt_mv", "mode", "flags",
)

NAN = float("nan")


class TelemetryRing:
    """Last ``seconds`` of telemetry. ``capacity`` defaults to seconds x rate x 1.5."""

    def __init__(self, seconds: float = 10.0, rate_hz: float = 25.0,
                 capacity: int | None = None, columns: Sequence[str] = COLUMNS,
                 use_numpy: bool | None = None):
        self.seconds = seconds
        self.columns = tuple(columns)
        self._col = {c: i for i, c in enumerate(self.columns)}
        self.capacity = capacity or max(8, int(math.ceil(seconds * rate_hz * 1.5)))
        self.numpy = HAVE_NUMPY if use_numpy is None else bool(use_numpy and HAVE_NUMPY)
        if self.numpy:
            self._data = _np.full((self.capacity, len(self.columns)), _np.nan)
        else:
            self._data = [array("d", [NAN] * self.capacity) for _ in self.columns]
        self._next = 0
        self._n = 0

    def __len__(self) -> int:
        return self._n

    def clear(self) -> None:
        self._next = 0
        self._n = 0

    def append(self, **values: Any) -> None:
        i = self._next
        for c, k in self._col.items():
            v = values.get(c)
            v = NAN if v is None else float(v)
            if self.numpy:
                self._data[i, k] = v
            else:
                self._data[k][i] = v
        self._next = (i + 1) % self.capacity
        self._n = min(self._n + 1, self.capacity)

    def _order(self) -> list[int]:
        start = (self._next - self._n) % self.capacity
        return [(start + j) % self.capacity for j in range(self._n)]

    def column(self, name: str) -> Any:
        """All stored values of one column, oldest first."""
        k = self._col[name]
        idx = self._order()
        if self.numpy:
            return self._data[idx, k] if idx else _np.empty(0)
        col = self._data[k]
        return [col[i] for i in idx]

    def last(self) -> dict[str, float] | None:
        if not self._n:
            return None
        i = (self._next - 1) % self.capacity
        if self.numpy:
            return {c: float(self._data[i, k]) for c, k in self._col.items()}
        return {c: self._data[k][i] for c, k in self._col.items()}

    def window(self, seconds: float | None = None) -> dict[str, Any]:
        """Rows of the last ``seconds`` (default: the buffer's span), oldest first."""
        seconds = self.seconds if seconds is None else seconds
        idx = self._order()
        if not idx:
            return {c: (_np.empty(0) if self.numpy else []) for c in self.columns}
        tk = self._col["t"]
        if self.numpy:
            rows = self._data[idx]
            rows = rows[rows[:, tk] >= rows[-1, tk] - seconds]
            return {c: rows[:, k] for c, k in self._col.items()}
        tcol = self._data[tk]
        t_end = tcol[idx[-1]]
        idx = [i for i in idx if tcol[i] >= t_end - seconds]
        return {c: [self._data[k][i] for i in idx] for c, k in self._col.items()}
