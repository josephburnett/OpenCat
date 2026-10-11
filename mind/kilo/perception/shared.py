"""SharedPercepts: one Percepts used from several threads (daemon, runner, monitor).

``Percepts`` itself has no locking. The daemon feeds one on every TELEMETRY
(on its asyncio loop) for ``kilo monitor --percepts`` and the ``percepts``
stream, and hands the same object to the behaviour runner, which calls
``update(state)`` every tick and ``notify_self_motion()`` when it moves the
body. A sample already seen is ignored by ``Percepts.update`` (same
telemetry polled twice), so the two feeders do not double count.

Listeners (``add_listener(fn)``) are called with ``(events, percepts_dict)``
after an update that produced events, outside the lock, on the updating
thread.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Any, Callable

from kilo.perception.events import PerceptEvent
from kilo.perception.percepts import Percepts

# The boolean percepts (as_dict keys) shown by `kilo monitor --percepts`.
FLAG_NAMES = ("near", "approaching", "clear", "tilted", "picked_up", "tap", "pet", "startle")


class SharedPercepts:
    def __init__(self, percepts: Percepts | None = None, history: int = 64):
        self.percepts = percepts or Percepts()
        self.lock = threading.Lock()
        self.recent: deque[PerceptEvent] = deque(maxlen=history)
        self._listeners: list[Callable[[list[PerceptEvent], dict[str, Any]], None]] = []

    def add_listener(self, fn: Callable[[list[PerceptEvent], dict[str, Any]], None]) -> None:
        self._listeners.append(fn)

    def update(self, snapshot: Any, t: float | None = None) -> dict[str, Any]:
        with self.lock:
            d = self.percepts.update(snapshot, t)
            self.recent.extend(d["events"])
        if d["events"]:
            for fn in list(self._listeners):
                try:
                    fn(d["events"], d)
                except Exception:
                    pass
        return d

    def notify_self_motion(self, t: float | None = None, seconds: float = 0.0) -> None:
        with self.lock:
            self.percepts.notify_self_motion(t, seconds)

    def as_dict(self) -> dict[str, Any]:
        with self.lock:
            return self.percepts.as_dict([])

    def flags(self) -> dict[str, bool]:
        d = self.as_dict()
        return {k: bool(d.get(k)) for k in FLAG_NAMES}

    def state(self) -> dict[str, Any]:
        """JSON-able: the flags, distance/approach/tilt values and recent events."""
        d = self.as_dict()
        with self.lock:
            events = [e.as_dict() for e in self.recent]
        return {"t": d["t"], "flags": {k: bool(d.get(k)) for k in FLAG_NAMES},
                "distance_mm": d["distance_mm"], "approach_mm_s": d["approach_mm_s"],
                "tilt_direction": d["tilt_direction"], "events": events}
