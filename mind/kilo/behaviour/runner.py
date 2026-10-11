"""The behaviour runner: a 10 Hz asyncio ticker (docs/architecture.md §5).

Each tick:

1. ``body.on_tick()`` (if present: stamps the link's liveness) and
   ``body.snapshot()`` (a BodyState.snapshot()-shaped dict);
2. on an epoch change the cached posture/balance goals are forgotten (the
   body dropped them), so behaviours re-assert them;
3. percepts: the ``percepts()`` source (camera, IMU detectors, later) merged
   with ultrasonic ones from the snapshot (``near``, ``approaching``) and a
   ``startle`` on a new fall;
4. drives are advanced by the elapsed time, with the activity of the
   behaviour that ran last tick;
5. the root (library.build_root) is ticked;
6. a record goes to the JSONL behaviour log when the choice or active path
   changes, and at least every ``log_every_s``. Writes go through
   kilo.log.JsonlLogger (queue + writer thread), never blocking the loop.

``run()`` returns when ``stop()`` is called, ``duration`` has elapsed, or the
wind-down branch has asked for poweroff (then ``on_poweroff`` is called; the
runner itself never powers anything off).
"""

from __future__ import annotations

import asyncio
import random
import time
from collections import deque
from pathlib import Path
from typing import Any, Awaitable, Callable

from .drives import DriveInputs, Drives
from .library import build_root
from .personality import load as load_personality
from .tree import Context

DEFAULT_LOG_DIR = Path.home() / "kilo-logs"


def default_log_path() -> Path:
    return DEFAULT_LOG_DIR / time.strftime("behaviour-%Y%m%d-%H%M%S.jsonl")


class Runner:
    def __init__(self, body: Any, personality: dict[str, Any] | None = None, *,
                 percepts: Callable[[], dict[str, Any]] | None = None,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], Awaitable[Any]] = asyncio.sleep,
                 seed: int | None = None, hz: float = 10.0,
                 log_path: str | Path | None = None, log: bool = True, logger: Any = None,
                 log_every_s: float = 1.0, history: int = 0,
                 on_poweroff: Callable[[], Any] | None = None):
        self.cfg = personality if personality is not None else load_personality()
        self.body = body
        self.percept_source = percepts
        self.clock = clock
        self.sleep = sleep
        self.hz = hz
        self.seed = seed if seed is not None else random.SystemRandom().randrange(1 << 32)
        self.ctx = Context(body=body, clock=clock, rng=random.Random(self.seed),
                           drives=Drives.from_personality(self.cfg), personality=self.cfg)
        self.root = build_root(self.cfg)
        self.on_poweroff = on_poweroff
        self.log_every_s = log_every_s
        self._own_logger = False
        self.log_path: Path | None = None
        if logger is not None:
            self.logger = logger
        elif log:
            from kilo.log import JsonlLogger
            self.log_path = Path(log_path) if log_path is not None else default_log_path()
            self.logger = JsonlLogger(self.log_path)
            self._own_logger = True
        else:
            self.logger = None
        self.history: deque[dict[str, Any]] | None = deque(maxlen=history) if history else None
        self.last_record: dict[str, Any] | None = None
        self.ticks = 0
        self.overruns = 0
        self.stopped_by: str | None = None
        self._stop = False
        self._last_now: float | None = None
        self._last_epoch: Any = None
        self._was_fallen = False
        self._last_logged: tuple[float, Any] | None = None
        self._poweroff_done = False

    # ---- control ---------------------------------------------------------------------
    def stop(self) -> None:
        self._stop = True

    def request_shutdown(self) -> None:
        """Ask for a clean wind-down (rest, then poweroff)."""
        self.ctx.data["shutdown_requested"] = True

    # ---- one tick --------------------------------------------------------------------
    def _snapshot(self) -> dict[str, Any]:
        hook = getattr(self.body, "on_tick", None)
        if hook is not None:
            hook()
        snap_fn = getattr(self.body, "snapshot", None)
        if snap_fn is None:
            return {}
        try:
            return snap_fn() or {}
        except Exception as e:
            self.ctx.errors.append(("snapshot", repr(e)))
            return {}

    def _percepts(self, snap: dict[str, Any]) -> dict[str, Any]:
        pc = self.cfg["percepts"]
        p: dict[str, Any] = {}
        dist = snap.get("dist_mm")
        p["near"] = dist is not None and 0 < dist < pc["near_mm"]
        appr = snap.get("approach_mm_s")
        p["approaching"] = bool(p["near"] and appr is not None and appr > pc["approach_mm_s"])
        fallen = bool(snap.get("fallen"))
        if fallen and not self._was_fallen:
            p["startle"] = 1.0
        self._was_fallen = fallen
        if self.percept_source is not None:
            try:
                p.update(self.percept_source() or {})
            except Exception as e:
                self.ctx.errors.append(("percepts", repr(e)))
        return p

    def step(self) -> dict[str, Any]:
        ctx = self.ctx
        snap = self._snapshot()
        epoch = snap.get("epoch")
        if epoch != self._last_epoch:
            ctx.data.pop("posture", None)
            ctx.data.pop("balance", None)
            self._last_epoch = epoch
        ctx.state = snap
        ctx.percepts = self._percepts(snap)
        ctx.begin_tick()
        now = ctx.now
        dt = 0.0 if self._last_now is None else now - self._last_now
        self._last_now = now
        p = ctx.percepts
        ctx.drives.update(dt, DriveInputs(
            activity=self.root.last.activity, moving=bool(snap.get("moving")),
            battery_pct=snap.get("battery_pct"), social=bool(p.get("face") or p.get("pet")),
            startle=float(p.get("startle") or 0.0), approaching=bool(p.get("approaching"))),
            self.cfg, ctx.rng)
        self.root.tick(ctx)
        path = ctx.end_tick()
        self.ticks += 1
        last = self.root.last
        rec = {
            "t": round(now, 3),
            "tick": self.ticks,
            "chosen": last.chosen,
            "reason": last.reason,
            "status": None if last.status is None else last.status.value,
            "path": path,
            "drives": ctx.drives.as_dict(),
            "scores": {k: round(v, 4) for k, v in last.scores.items()},
            "percepts": {k: v for k, v in p.items() if v},
            "mode": snap.get("mode"),
            "battery_pct": None if snap.get("battery_pct") is None else round(snap["battery_pct"], 1),
        }
        if ctx.errors:
            rec["errors"] = ctx.errors[-5:]
            ctx.errors.clear()
        self.last_record = rec
        if self.history is not None:
            self.history.append(rec)
        self._log(rec, now)
        if ctx.data.get("poweroff") and not self._poweroff_done:
            self._poweroff_done = True
            self.stopped_by = "poweroff"
            self._stop = True
            if self.logger is not None:
                self.logger.log("poweroff", t=round(now, 3), seed=self.seed)
            if self.on_poweroff is not None:
                self.on_poweroff()
        return rec

    def _log(self, rec: dict[str, Any], now: float) -> None:
        if self.logger is None:
            return
        key = (rec["chosen"], tuple(rec["path"]))
        if (self._last_logged is None or self._last_logged[1] != key
                or now - self._last_logged[0] >= self.log_every_s or "errors" in rec):
            self._last_logged = (now, key)
            self.logger.log("behaviour", **rec)

    # ---- the loop ----------------------------------------------------------------------
    async def run(self, duration: float | None = None) -> str:
        """Tick at ``hz`` until stopped; returns why it stopped."""
        period = 1.0 / self.hz
        start = self.clock()
        if self.logger is not None:
            self.logger.log("start", t=round(start, 3), seed=self.seed, hz=self.hz,
                            personality=self.cfg.get("name"))
        k = 0
        try:
            while not self._stop:
                self.step()
                if self._stop:
                    break
                if duration is not None and self.clock() - start >= duration:
                    self.stopped_by = "duration"
                    break
                k += 1
                delay = start + k * period - self.clock()
                if delay < -1.0:  # fell far behind: resync rather than burst
                    self.overruns += 1
                    start, k = self.clock(), 0
                    delay = 0.0
                await self.sleep(max(0.0, delay))
            if self.stopped_by is None:
                self.stopped_by = "stop"
        finally:
            self.root.halt()  # cancels running coroutine actions
            if self.logger is not None:
                self.logger.log("stop", t=round(self.clock(), 3), reason=self.stopped_by,
                                ticks=self.ticks, overruns=self.overruns)
            if self._own_logger:
                await asyncio.get_running_loop().run_in_executor(None, self.logger.close)
        return self.stopped_by or "stop"
