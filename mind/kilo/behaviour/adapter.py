"""BodyAdapter: the behaviour layer's body interface over a real BodyClient.

BodyClient (P0/P1) has postures and balance. Head targets, tricks and the
buzzer are not in the body yet, so ``look``, ``play`` and ``beep`` call the
client's method when it has one and otherwise record the call in
``unsupported`` (``play`` then resolves at once). When BodyClient grows
``look``/``play``/``beep``, nothing here needs to change.

The adapter also gives the runner ``snapshot()`` (BodyState.snapshot()) and
``on_tick()``, which stamps the link's liveness from the asyncio loop: the
link's writer only refreshes goals and heartbeats while the mind's loop is
alive (§5 "Serial link").
"""

from __future__ import annotations

import asyncio
import inspect
from collections import Counter, deque
from typing import Any


class BodyAdapter:
    def __init__(self, client: Any):
        self.client = client
        self.unsupported: Counter[str] = Counter()
        self.calls: deque[tuple[str, tuple]] = deque(maxlen=1000)

    # ---- runner hooks -----------------------------------------------------------
    def snapshot(self) -> dict[str, Any]:
        return self.client.state.snapshot()

    def on_tick(self) -> None:
        link = getattr(self.client, "link", None)
        if link is not None:
            link.mark_alive()

    # ---- the body interface ---------------------------------------------------------
    def posture(self, name: str, balance: bool | None = None) -> None:
        self.calls.append(("posture", (name, balance)))
        self.client.posture(name, balance=balance)

    def set_balance(self, enable: bool) -> None:
        self.calls.append(("set_balance", (enable,)))
        self.client.set_balance(enable)

    def look(self, pan: float, tilt: float) -> None:
        self._maybe("look", pan, tilt)

    def beep(self, notes: list[tuple[int, int]]) -> None:
        self._maybe("beep", notes)

    def play(self, trick: str, mirror: bool = False) -> Any:
        r = self._maybe("play", trick, mirror=mirror)
        if inspect.isawaitable(r):
            return r
        return _done()

    def _maybe(self, name: str, *args: Any, **kw: Any) -> Any:
        self.calls.append((name, args))
        fn = getattr(self.client, name, None)
        if fn is None:
            self.unsupported[name] += 1
            return None
        return fn(*args, **kw)


async def _done() -> None:
    await asyncio.sleep(0)
