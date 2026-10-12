"""StepClock: a monotonic clock that only moves when a test says so.

Production code takes ``clock=time.monotonic`` and waits with
``threading.Event.wait(timeout)``. A test can pass a ``StepClock`` instead
(to ``SerialLink``, ``BodyState`` through it, and ``VirtualBody``, so both
ends of an in-process link share one time) and drive time with
``step(dt)``:

- ``clock()`` returns the current time (seconds, float). Time is kept in
  integer nanoseconds, so ``step(0.01)`` ten times is exactly 0.1 s.
- ``clock.wait(event, timeout)`` is ``event.wait(timeout)`` in clock time:
  True as soon as ``event`` is set, False once ``timeout`` clock seconds have
  passed. ``SerialLink`` uses it (when the clock has it) for the ACK timeout
  and the reconnect backoff. A real-time guard (``real_guard_s``) ends a wait
  that nobody steps, so a broken test fails instead of hanging.
- ``step(dt)`` advances time, then returns only once the components have
  caught up with the new time: every ``wait`` whose deadline has passed has
  returned, and every hook registered with ``attach`` (``SerialLink.sync``,
  ``VirtualBody.sync``, a test peer's ``sync``) has returned. After
  ``step()`` the link's writer has run its housekeeping at the new time and
  written what that housekeeping sends, and the attached readers have
  consumed it. Assertions that something did *not* happen are therefore
  exact; assertions that something did happen on another thread (the link's
  reader handling a frame) still wait for it with a bounded real-time wait.
- ``deadlines()`` / ``wait_for_waiters()`` show who is waiting until when,
  so a test can check a timeout's length exactly (the ACK timeout, the
  reconnect backoff) instead of measuring wall time.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

_NS = 1_000_000_000


class StepClock:
    def __init__(self, start: float = 1000.0, real_guard_s: float = 10.0):
        self._ns = int(round(start * _NS))
        self._cond = threading.Condition()
        self._waiters: dict[int, int] = {}  # waiter id -> deadline (ns)
        self._next_id = 0
        self._hooks: list[Callable[[], Any]] = []
        self.real_guard_s = real_guard_s

    # ---- reading -------------------------------------------------------------------
    def __call__(self) -> float:
        return self._ns / _NS

    def now(self) -> float:
        return self()

    # ---- driving ---------------------------------------------------------------------
    def attach(self, sync: Callable[[], Any]) -> None:
        """Call ``sync()`` after every step (it blocks until its component caught up)."""
        self._hooks.append(sync)

    def detach(self, sync: Callable[[], Any]) -> None:
        if sync in self._hooks:
            self._hooks.remove(sync)

    def step(self, dt: float, *, settle: bool = True) -> float:
        """Advance by ``dt`` seconds; with ``settle``, wait for everyone to catch up."""
        if dt < 0:
            raise ValueError("time does not go backwards")
        with self._cond:
            self._ns += int(round(dt * _NS))
            now_ns = self._ns
            self._cond.notify_all()
        if settle:
            self.settle(now_ns)
        return self()

    def run(self, seconds: float, dt: float = 0.01, each: Callable[[], Any] | None = None) -> None:
        """``step(dt)`` until ``seconds`` have passed, calling ``each()`` before every step."""
        end = self._ns + int(round(seconds * _NS))
        while self._ns < end:
            if each is not None:
                each()
            self.step(min(dt, (end - self._ns) / _NS))

    def settle(self, now_ns: int | None = None, timeout: float = 5.0) -> None:
        now_ns = self._ns if now_ns is None else now_ns
        with self._cond:
            ok = self._cond.wait_for(lambda: all(d > now_ns for d in self._waiters.values()), timeout)
        if not ok:
            raise TimeoutError("StepClock: a waiter past its deadline did not return")
        for sync in list(self._hooks):
            sync()

    # ---- waiting ---------------------------------------------------------------------
    def wait(self, event: threading.Event, timeout: float | None) -> bool:
        """``event.wait(timeout)`` with ``timeout`` in clock seconds."""
        if event.is_set():
            return True
        with self._cond:
            wid = self._next_id
            self._next_id += 1
            deadline = None if timeout is None else self._ns + int(round(timeout * _NS))
            if deadline is not None:
                self._waiters[wid] = deadline
                self._cond.notify_all()
        real_end = time.monotonic() + self.real_guard_s
        try:
            while True:
                if event.is_set():
                    return True
                if deadline is not None and self._ns >= deadline:
                    return False
                if time.monotonic() >= real_end:
                    return event.is_set()
                with self._cond:
                    # an Event.set() does not notify this condition: poll it
                    self._cond.wait(0.002)
        finally:
            with self._cond:
                self._waiters.pop(wid, None)
                self._cond.notify_all()

    def deadlines(self) -> list[float]:
        """Pending ``wait`` deadlines, in clock seconds, sorted."""
        with self._cond:
            return sorted(d / _NS for d in self._waiters.values())

    def wait_for_waiters(self, n: int = 1, timeout: float = 2.0) -> list[float]:
        """Block (real time) until at least ``n`` waits are pending; return their deadlines."""
        with self._cond:
            self._cond.wait_for(lambda: len(self._waiters) >= n, timeout)
        return self.deadlines()
