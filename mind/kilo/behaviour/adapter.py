"""BodyAdapter: the behaviour layer's body interface over a real BodyClient.

BodyClient has postures and balance. ``look``, ``play``, ``beep``, ``walk``
and ``stop_walk`` call the client's method when it has one (BodyClient has
all but ``beep``) and otherwise record the call in ``unsupported``
(``play`` then resolves at once). No behaviour walks yet: the owner decides
when autonomous walking is allowed.

The adapter also gives the runner ``snapshot()`` (BodyState.snapshot()) and
``on_tick()``, which stamps the link's liveness from the asyncio loop: the
link's writer only refreshes goals and heartbeats while the mind's loop is
alive (§5 "Serial link").

With ``percepts`` (a ``Percepts`` or ``SharedPercepts``), every move it
starts (a posture change, a head look, a trick) first calls
``percepts.notify_self_motion(seconds=...)`` so the IMU detectors (tap, pet,
picked_up, startle) do not mistake Kilo's own motion for a touch.
"""

from __future__ import annotations

import asyncio
import inspect
from collections import Counter, deque
from typing import Any


POSTURE_MOVE_S = 0.5  # the body's default posture transition: 25 ticks of 20 ms
LOOK_MOVE_S = 0.3
TRICK_MOVE_S = 2.0
WALK_MOVE_S = 2.0  # renewed by each walk() call; the IMU detectors stay quiet meanwhile


class BodyAdapter:
    def __init__(self, client: Any, percepts: Any = None):
        self.client = client
        self.percepts = percepts
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
    def _self_motion(self, seconds: float) -> None:
        if self.percepts is not None:
            self.percepts.notify_self_motion(seconds=seconds)

    def posture(self, name: str, balance: bool | None = None) -> None:
        self.calls.append(("posture", (name, balance)))
        self._self_motion(POSTURE_MOVE_S)
        self.client.posture(name, balance=balance)

    def set_balance(self, enable: bool) -> None:
        self.calls.append(("set_balance", (enable,)))
        self.client.set_balance(enable)

    def look(self, pan: float, tilt: float) -> None:
        if getattr(self.client, "look", None) is not None:
            self._self_motion(LOOK_MOVE_S)
        self._maybe("look", pan, tilt)

    def walk(self, direction: str = "forward", pattern: str = "trot", speed_mm_s: float | None = None) -> Any:
        """BodyClient.walk (the planner's stream; blocks ~0.6 s on a start).
        NOTE: no behaviour calls this yet, on purpose: the owner decides when
        Kilo may walk autonomously (explore stays the stand-and-look-around
        stub until then). Walking is on the floor only, never on a desk."""
        if getattr(self.client, "walk", None) is not None:
            self._self_motion(WALK_MOVE_S)
        return self._maybe("walk", direction, pattern, speed_mm_s)

    def stop_walk(self) -> Any:
        return self._maybe("stop_walk")

    def beep(self, notes: list[tuple[int, int]]) -> None:
        self._maybe("beep", notes)

    def play(self, trick: str, mirror: bool = False) -> Any:
        if getattr(self.client, "play", None) is not None:
            self._self_motion(TRICK_MOVE_S)
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
