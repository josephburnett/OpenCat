"""BodyEvents: body events for the behaviour layer, from the snapshot alone.

BodyClient has no event callback yet (docs/architecture.md §5 "Mind
integration backlog"). Until it does, the runner watches what every
``BodyState.snapshot()`` already carries: the epoch and the last 8 EVENTs.
``update(snapshot)`` returns what is new since the previous call:

- ``{"type": "epoch", "from": 3, "to": 4}`` on an epoch change (the body
  dropped every motion goal: re-assert);
- ``{"type": "event", "name": "FELL", "arg": 1, "epoch": 4, "tick": 1234}``
  for each new EVENT whose name is in ``names`` (default FELL, WATCHDOG,
  LOW_BATTERY, BATTERY_OK, BOOT, LINK_RESTORED, GOAL_REFUSED);
- ``{"type": "connection", "connected": bool, "alive": bool}`` when the
  link's connected/alive flags change (the port went away or came back).

Events already in the first snapshot are not reported (they happened before
the watcher started). At 10 Hz polling an EVENT cannot fall out of the
snapshot's window of 8 unless the body sends more than 80 a second.
"""

from __future__ import annotations

from collections import deque
from typing import Any

from kilo.link import consts

WATCHED = frozenset({"FELL", "WATCHDOG", "LOW_BATTERY", "BATTERY_OK", "BOOT", "LINK_RESTORED",
                     "GOAL_REFUSED"})


class BodyEvents:
    def __init__(self, names: frozenset[str] | set[str] = WATCHED, memory: int = 64):
        self.names = frozenset(names)
        self._seen: deque[tuple] = deque(maxlen=memory)
        self._epoch: Any = None
        self._link: tuple[Any, Any] | None = None
        self._primed = False

    def update(self, snap: dict[str, Any]) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        link = (snap.get("connected"), snap.get("alive"))
        if self._link is not None and link != self._link and None not in link:
            out.append({"type": "connection", "connected": bool(link[0]), "alive": bool(link[1])})
        self._link = link
        epoch = snap.get("epoch")
        if epoch is not None:
            if self._epoch is not None and epoch != self._epoch:
                out.append({"type": "epoch", "from": self._epoch, "to": epoch})
            self._epoch = epoch
        for item in snap.get("events") or ():
            e = item[1] if isinstance(item, (list, tuple)) and len(item) == 2 else item
            if not isinstance(e, dict):
                continue
            key = (e.get("tick"), e.get("code"), e.get("arg"), e.get("epoch"))
            if key in self._seen:
                continue
            self._seen.append(key)
            name = consts.EVENTS.get(e.get("code"), str(e.get("code")))
            if self._primed and name in self.names:
                out.append({"type": "event", "name": name, "arg": e.get("arg"), "epoch": e.get("epoch"),
                            "tick": e.get("tick")})
        self._primed = True
        return out
