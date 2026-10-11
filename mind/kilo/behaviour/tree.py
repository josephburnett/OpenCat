"""A minimal behaviour-tree engine (docs/architecture.md §5 "Behaviour").

Every node has ``tick(ctx) -> Status`` (RUNNING, SUCCESS or FAILURE) and
``halt()``, which resets a node (and cancels a running coroutine) when a
parent abandons it. Composites:

- ``Sequence``: with memory by default (resumes at the running child); fails
  on the first failure, succeeds when all children have. ``memory=False``
  makes it reactive: leading Conditions are re-checked every tick as guards.
- ``Selector``: memory-less; re-checks children from the first every tick, so a
  higher-priority child preempts (and halts) a lower one that was running.
- ``Parallel``: ticks all unfinished children; policy ``"all"`` succeeds when all
  succeed and fails on the first failure, ``"any"`` succeeds on the first
  success and fails when all fail.

Decorators: ``Cooldown``, ``Timeout``, ``Chance``, ``Inverter``, ``Repeat``,
``ForceSuccess``. Leaves: ``Action`` (sync function or coroutine; a coroutine
runs as an asyncio task and the node is RUNNING until it finishes),
``Condition`` and ``Wait``.

The ``Context`` (blackboard) carries the injectable clock, the body interface,
the latest body snapshot, percepts, drives, a seeded RNG and scratch data. Each
tick records the active path: the node names down to the last running leaf (or
the last leaf ticked when nothing is running).
"""

from __future__ import annotations

import asyncio
import enum
import inspect
import random
import time
from dataclasses import dataclass, field
from typing import Any, Callable


class Status(enum.Enum):
    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"


RUNNING, SUCCESS, FAILURE = Status.RUNNING, Status.SUCCESS, Status.FAILURE


@dataclass
class Context:
    """The blackboard handed to every node."""

    body: Any = None
    clock: Callable[[], float] = time.monotonic
    rng: random.Random = field(default_factory=lambda: random.Random(0))
    state: dict[str, Any] = field(default_factory=dict)  # BodyState.snapshot()
    percepts: dict[str, Any] = field(default_factory=dict)
    drives: Any = None
    data: dict[str, Any] = field(default_factory=dict)  # scratch shared by behaviours
    personality: Any = None
    now: float = 0.0
    ticks: int = 0
    errors: list[tuple[str, str]] = field(default_factory=list)
    # per-tick trace
    path: list[str] = field(default_factory=list)
    _stack: list[str] = field(default_factory=list)
    _running_path: list[str] | None = None

    def begin_tick(self) -> None:
        self.now = self.clock()
        self.ticks += 1
        self.path = []
        self._stack = []
        self._running_path = None

    def end_tick(self) -> list[str]:
        if self._running_path is not None:
            self.path = self._running_path
        return self.path

    def _leaf(self, status: Status) -> None:
        p = list(self._stack)
        self.path = p
        if status is RUNNING:
            self._running_path = p


class Node:
    def __init__(self, name: str | None = None):
        self.name = name or type(self).__name__
        self.status: Status | None = None

    def tick(self, ctx: Context) -> Status:
        ctx._stack.append(self.name)
        try:
            st = self._tick(ctx)
        finally:
            ctx._stack.pop()
        self.status = st
        return st

    def _tick(self, ctx: Context) -> Status:  # pragma: no cover - abstract
        raise NotImplementedError

    def halt(self) -> None:
        """Abandon: reset state, cancel anything running."""
        self.status = None

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.name!r})"


# ---- composites -----------------------------------------------------------------

class Sequence(Node):
    """``memory=True`` (default): resume at the running child. ``memory=False``
    (reactive): start from the first child every tick, so leading Conditions
    act as guards; when one fails, the running child is halted. Only put
    idempotent children before the running one in a reactive sequence."""

    def __init__(self, name: str | None, children: list[Node], memory: bool = True):
        super().__init__(name)
        self.children = list(children)
        self.memory = memory
        self.idx = 0
        self.running: int | None = None  # index of the child left RUNNING last tick

    def _tick(self, ctx: Context) -> Status:
        running, self.running = self.running, None
        if not self.memory:
            self.idx = 0
        while self.idx < len(self.children):
            st = self.children[self.idx].tick(ctx)
            if self.idx == running and st is not RUNNING:
                running = None  # it finished by itself
            if st is SUCCESS:
                self.idx += 1
                continue
            if running is not None and running != self.idx:
                self.children[running].halt()  # a guard before it failed or took over
            if st is RUNNING:
                self.running = self.idx
                return RUNNING
            self.idx = 0
            return FAILURE
        self.idx = 0
        return SUCCESS

    def halt(self) -> None:
        if self.running is not None:
            self.children[self.running].halt()
        self.running = None
        self.idx = 0
        super().halt()


class Selector(Node):
    def __init__(self, name: str | None, children: list[Node]):
        super().__init__(name)
        self.children = list(children)
        self.running: int | None = None

    def _tick(self, ctx: Context) -> Status:
        for i, child in enumerate(self.children):
            st = child.tick(ctx)
            if st is FAILURE:
                if self.running == i:
                    self.running = None
                continue
            if self.running is not None and self.running != i:
                self.children[self.running].halt()  # preempted
            self.running = i if st is RUNNING else None
            return st
        self.running = None
        return FAILURE

    def halt(self) -> None:
        if self.running is not None:
            self.children[self.running].halt()
        self.running = None
        super().halt()


class Parallel(Node):
    def __init__(self, name: str | None, children: list[Node], policy: str = "all"):
        super().__init__(name)
        if policy not in ("all", "any"):
            raise ValueError("policy must be 'all' or 'any'")
        self.children = list(children)
        self.policy = policy
        self.done: dict[int, Status] = {}

    def _tick(self, ctx: Context) -> Status:
        for i, child in enumerate(self.children):
            if i in self.done:
                continue
            st = child.tick(ctx)
            if st is not RUNNING:
                self.done[i] = st
            if self.policy == "all" and st is FAILURE:
                return self._finish(FAILURE)
            if self.policy == "any" and st is SUCCESS:
                return self._finish(SUCCESS)
        if len(self.done) == len(self.children):
            return self._finish(SUCCESS if self.policy == "all" else FAILURE)
        return RUNNING

    def _finish(self, st: Status) -> Status:
        for i, c in enumerate(self.children):
            if i not in self.done:
                c.halt()
        self.done = {}
        return st

    def halt(self) -> None:
        self._finish(FAILURE)
        super().halt()


# ---- decorators -----------------------------------------------------------------

class Decorator(Node):
    def __init__(self, name: str | None, child: Node):
        super().__init__(name)
        self.child = child

    def halt(self) -> None:
        self.child.halt()
        super().halt()


class Inverter(Decorator):
    def __init__(self, child: Node, name: str | None = None):
        super().__init__(name or "Inverter", child)

    def _tick(self, ctx: Context) -> Status:
        st = self.child.tick(ctx)
        return {SUCCESS: FAILURE, FAILURE: SUCCESS}.get(st, st)


class ForceSuccess(Decorator):
    def __init__(self, child: Node, name: str | None = None):
        super().__init__(name or "ForceSuccess", child)

    def _tick(self, ctx: Context) -> Status:
        st = self.child.tick(ctx)
        return RUNNING if st is RUNNING else SUCCESS


class Cooldown(Decorator):
    """After the child finishes (either way, or is abandoned while running),
    FAILURE without ticking it for ``seconds``."""

    def __init__(self, seconds: float, child: Node, name: str | None = None):
        super().__init__(name or f"Cooldown({seconds:g}s)", child)
        self.seconds = seconds
        self.ready_at = float("-inf")
        self._last_now = float("-inf")
        self._running = False

    def _tick(self, ctx: Context) -> Status:
        self._last_now = ctx.now
        if ctx.now < self.ready_at:
            return FAILURE
        st = self.child.tick(ctx)
        self._running = st is RUNNING
        if st is not RUNNING:
            self.ready_at = ctx.now + self.seconds
        return st

    def halt(self) -> None:
        if self._running:  # an abandoned run still counts as a use
            self.ready_at = self._last_now + self.seconds
            self._running = False
        super().halt()


class Timeout(Decorator):
    """FAILURE (and halt the child) if it is still RUNNING ``seconds`` after it started."""

    def __init__(self, seconds: float, child: Node, name: str | None = None):
        super().__init__(name or f"Timeout({seconds:g}s)", child)
        self.seconds = seconds
        self.started: float | None = None

    def _tick(self, ctx: Context) -> Status:
        if self.started is None:
            self.started = ctx.now
        if ctx.now - self.started >= self.seconds:
            self.child.halt()
            self.started = None
            return FAILURE
        st = self.child.tick(ctx)
        if st is not RUNNING:
            self.started = None
        return st

    def halt(self) -> None:
        self.started = None
        super().halt()


class Chance(Decorator):
    """Roll ``ctx.rng`` once per run: with probability ``p`` run the child, else FAILURE."""

    def __init__(self, p: float, child: Node, name: str | None = None):
        if not 0.0 <= p <= 1.0:
            raise ValueError("p must be in [0, 1]")
        super().__init__(name or f"Chance({p:g})", child)
        self.p = p
        self.active = False

    def _tick(self, ctx: Context) -> Status:
        if not self.active:
            if ctx.rng.random() >= self.p:
                return FAILURE
            self.active = True
        st = self.child.tick(ctx)
        if st is not RUNNING:
            self.active = False
        return st

    def halt(self) -> None:
        self.active = False
        super().halt()


class Repeat(Decorator):
    """Re-run the child after each SUCCESS; ``times=None`` forever (RUNNING).

    A child FAILURE fails the repeat. With ``times`` set, SUCCESS after that
    many successes. At most one child completion per tick.
    """

    def __init__(self, child: Node, times: int | None = None, name: str | None = None):
        super().__init__(name or ("Forever" if times is None else f"Repeat({times})"), child)
        self.times = times
        self.count = 0

    def _tick(self, ctx: Context) -> Status:
        st = self.child.tick(ctx)
        if st is FAILURE:
            self.count = 0
            return FAILURE
        if st is SUCCESS:
            self.count += 1
            if self.times is not None and self.count >= self.times:
                self.count = 0
                return SUCCESS
        return RUNNING

    def halt(self) -> None:
        self.count = 0
        super().halt()


# ---- leaves ---------------------------------------------------------------------

def _as_status(v: Any) -> Status:
    if isinstance(v, Status):
        return v
    if v is None or v is True:
        return SUCCESS
    if v is False:
        return FAILURE
    raise TypeError(f"action returned {v!r}, expected Status, bool or None")


class Action(Node):
    """Run ``fn(ctx)``.

    ``fn`` may return a Status, a bool or None (SUCCESS), or an awaitable. An
    awaitable runs as an asyncio task on the running loop; the node is RUNNING
    until the task finishes, then SUCCESS (or the Status/bool it returned) or
    FAILURE if it raised. ``halt()`` cancels the task. A sync function that
    raises makes the node FAIL (the error is kept in ``ctx.errors``).
    """

    def __init__(self, name: str, fn: Callable[[Context], Any]):
        super().__init__(name)
        self.fn = fn
        self.task: asyncio.Future | None = None

    def _tick(self, ctx: Context) -> Status:
        if self.task is None:
            try:
                r = self.fn(ctx)
                if inspect.isawaitable(r):
                    self.task = asyncio.ensure_future(r)
                else:
                    st = _as_status(r)
                    ctx._leaf(st)
                    return st
            except Exception as e:  # an action failing must not kill the tree
                ctx.errors.append((self.name, repr(e)))
                ctx._leaf(FAILURE)
                return FAILURE
        if not self.task.done():
            ctx._leaf(RUNNING)
            return RUNNING
        task, self.task = self.task, None
        if task.cancelled():
            st = FAILURE
        elif task.exception() is not None:
            ctx.errors.append((self.name, repr(task.exception())))
            st = FAILURE
        else:
            try:
                st = _as_status(task.result())
            except TypeError as e:
                ctx.errors.append((self.name, repr(e)))
                st = FAILURE
        ctx._leaf(st)
        return st

    def halt(self) -> None:
        if self.task is not None and not self.task.done():
            self.task.cancel()
        self.task = None
        super().halt()


class Condition(Node):
    def __init__(self, name: str, pred: Callable[[Context], Any]):
        super().__init__(name)
        self.pred = pred

    def _tick(self, ctx: Context) -> Status:
        try:
            st = SUCCESS if self.pred(ctx) else FAILURE
        except Exception as e:
            ctx.errors.append((self.name, repr(e)))
            st = FAILURE
        ctx._leaf(st)
        return st


class Wait(Node):
    """RUNNING for ``seconds`` (a number or ``fn(ctx)``) from the first tick, then SUCCESS."""

    def __init__(self, seconds: float | Callable[[Context], float], name: str | None = None):
        super().__init__(name or (f"Wait({seconds:g}s)" if not callable(seconds) else "Wait"))
        self.seconds = seconds
        self.until: float | None = None

    def _tick(self, ctx: Context) -> Status:
        if self.until is None:
            s = self.seconds(ctx) if callable(self.seconds) else self.seconds
            self.until = ctx.now + s
        if ctx.now >= self.until:
            self.until = None
            ctx._leaf(SUCCESS)
            return SUCCESS
        ctx._leaf(RUNNING)
        return RUNNING

    def halt(self) -> None:
        self.until = None
        super().halt()


def tick_root(root: Node, ctx: Context) -> Status:
    """One tick: stamp time, tick, record the active path in ``ctx.path``."""
    ctx.begin_tick()
    st = root.tick(ctx)
    ctx.end_tick()
    return st
