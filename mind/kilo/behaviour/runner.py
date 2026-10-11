"""The behaviour runner: a 10 Hz asyncio ticker (docs/architecture.md §5).

Each tick:

1. ``body.on_tick()`` (if present: stamps the link's liveness) and
   ``body.snapshot()`` (a BodyState.snapshot()-shaped dict);
2. on an epoch change, a body reboot (``reboots``) or a connection change
   (``connected``) the cached posture/balance goals are forgotten (the body
   or the link dropped them), so behaviours re-assert them. While
   ``paused()`` is true (the daemon's ``--override``: a CLI hold owns the
   body) only steps 1-2 run: the running behaviour is halted (a trick is
   stopped), nothing is ticked, and the cache is forgotten again on resume;
3. body events: ``BodyEvents`` turns the snapshot's epoch, EVENTs and link
   flags into ``body_event`` log records and the ``on_body_event`` callback
   (epoch change, FELL, WATCHDOG, LOW_BATTERY, connection...);
4. percepts: ultrasonic ones from the snapshot (``near``, ``approaching``,
   using the personality's ``percepts`` thresholds) and a ``startle`` on a new
   fall, overridden by the ``percepts()`` source when there is one (a
   ``startle`` is kept if either says so). ``run_behaviour`` uses
   ``PerceptFeed``: ``kilo.perception.Percepts.update(state)`` each tick;
5. drives are advanced by the elapsed time, with the activity of the
   behaviour that ran last tick;
6. the root (library.build_root) is ticked;
7. a record goes to the JSONL behaviour log when the choice or active path
   changes, and at least every ``log_every_s``. Writes go through
   kilo.log.JsonlLogger (queue + writer thread), never blocking the loop.

``run()`` returns when ``stop()`` is called, ``duration`` has elapsed, or the
wind-down branch has asked for poweroff (then ``on_poweroff`` is called; the
runner itself never powers anything off).

Behaviour percepts (``ctx.percepts`` keys, the selector's percept terms) and
where ``PerceptFeed`` gets them from ``Percepts.as_dict()``:

- ``face``: always False until a camera exists (term ``face``; greet);
- ``pet``: Percepts ``pet``, gentle rocking (term ``pet``; greet; satisfies
  sociability);
- ``near``: Percepts ``near``, ultrasonic < 150 mm with hysteresis (term ``near``);
- ``approaching``: Percepts ``approaching``, range closing > 100 mm/s (term
  ``approaching``; arousal spikes);
- ``alone``: not a percept; the selector derives it as not (face or pet);
- ``startle``: 1.0 while Percepts ``startle`` is held (1 s after the event),
  or on a new fall (arousal spikes; the startle behaviour);
- ``tap``, ``tilted``, ``picked_up``, ``clear``: the Percepts flags of the same
  name, in ``ctx.percepts`` and the log, no selector term yet;
- ``percept_events``: names of the PerceptEvents produced this tick.

``run_behaviour(client, state, stop_event, log_dir)`` is the daemon's entry
point (``kilo run --behaviour``).
"""

from __future__ import annotations

import asyncio
import random
import time
from collections import deque
from pathlib import Path
from typing import Any, Awaitable, Callable

from .body_events import BodyEvents
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
                 on_poweroff: Callable[[], Any] | None = None,
                 on_body_event: Callable[[dict[str, Any]], Any] | None = None,
                 paused: Callable[[], bool] | None = None):
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
        self.on_body_event = on_body_event
        self.body_events = BodyEvents()
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
        self._last_body_key: Any = None
        self.paused = paused
        self._was_paused = False
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
                src = self.percept_source() or {}
            except Exception as e:
                self.ctx.errors.append(("percepts", repr(e)))
                src = {}
            startle = max(float(p.get("startle") or 0.0), float(src.get("startle") or 0.0))
            p.update(src)
            if startle:
                p["startle"] = startle
            else:
                p.pop("startle", None)
        return p

    def _body_events(self, snap: dict[str, Any], now: float) -> list[dict[str, Any]]:
        evs = self.body_events.update(snap)
        for ev in evs:
            if self.logger is not None:
                self.logger.log("body_event", t=round(now, 3), **ev)
            if self.on_body_event is not None:
                try:
                    self.on_body_event(ev)
                except Exception as e:
                    self.ctx.errors.append(("on_body_event", repr(e)))
        return evs

    def step(self) -> dict[str, Any]:
        ctx = self.ctx
        snap = self._snapshot()
        key = (snap.get("epoch"), snap.get("reboots"), snap.get("connected"))
        if key != self._last_body_key:
            ctx.data.pop("posture", None)
            ctx.data.pop("balance", None)
            self._last_body_key = key
            self._last_epoch = key[0]
        paused = bool(self.paused is not None and self.paused())
        if paused != self._was_paused:
            self._was_paused = paused
            self.root.halt()  # pause: stop what runs; resume: start afresh
            ctx.data.pop("posture", None)
            ctx.data.pop("balance", None)
            if self.logger is not None:
                self.logger.log("paused" if paused else "resumed", t=round(self.clock(), 3))
        if paused:
            ctx.state = snap
            self.ticks += 1
            return {"t": round(self.clock(), 3), "tick": self.ticks, "paused": True}
        ctx.state = snap
        ctx.percepts = self._percepts(snap)
        ctx.begin_tick()
        now = ctx.now
        ctx.data["body_events"] = self._body_events(snap, now)
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


# ---- the daemon's entry point ------------------------------------------------------------

def behaviour_percepts(d: dict[str, Any]) -> dict[str, Any]:
    """Map a ``Percepts.as_dict()``/``update()`` result to behaviour percepts
    (the list in the module docstring)."""
    out: dict[str, Any] = {
        "face": False,  # no camera yet
        "pet": bool(d.get("pet")),
        "near": bool(d.get("near")),
        "approaching": bool(d.get("approaching")),
        "clear": bool(d.get("clear")),
        "tilted": bool(d.get("tilted")),
        "tap": bool(d.get("tap")),
        "picked_up": bool(d.get("picked_up")),
        "startle": 1.0 if d.get("startle") else 0.0,
    }
    evs = d.get("events") or []
    if evs:
        out["percept_events"] = [getattr(e, "name", str(e)) for e in evs]
    return out


class PerceptFeed:
    """The runner's percept source over kilo.perception: ``update(state)``
    once per tick (a sample already seen is ignored), mapped by
    ``behaviour_percepts``. ``percepts`` may be shared with the daemon."""

    def __init__(self, state: Any, percepts: Any = None):
        if percepts is None:
            from kilo.perception import SharedPercepts
            percepts = SharedPercepts()
        self.state = state
        self.percepts = percepts

    def __call__(self) -> dict[str, Any]:
        return behaviour_percepts(self.percepts.update(self.state))


def run_behaviour(client: Any, state: Any, stop_event: Any, log_dir: str | Path, *,
                  percepts: Any = None, personality: str | Path | None = None, seed: int | None = None,
                  hz: float = 10.0, paused: Any = None) -> str:
    """Run the behaviour layer until ``stop_event`` is set (the daemon's
    ``kilo run --behaviour`` calls this in its own thread).

    ``client`` is a BodyClient, ``state`` its BodyState, ``stop_event`` a
    threading.Event, ``log_dir`` where ``behaviour-<time>.jsonl`` goes.
    ``percepts`` (the daemon's SharedPercepts) is used instead of a private
    one when given; ``personality`` (else ``$KILO_PERSONALITY``) picks the
    YAML; ``paused`` (a threading.Event, the daemon's ``--override``) pauses
    the behaviours while set. Returns why it stopped (``stop``, ``poweroff``...)."""
    import logging
    import os

    from .adapter import BodyAdapter

    log = logging.getLogger("kilo.behaviour")
    cfg = load_personality(personality or os.environ.get("KILO_PERSONALITY") or None)
    feed = PerceptFeed(state, percepts)
    body = BodyAdapter(client, percepts=feed.percepts)
    log_path = Path(log_dir) / time.strftime("behaviour-%Y%m%d-%H%M%S.jsonl")
    runner = Runner(body, cfg, percepts=feed, seed=seed, hz=hz, log_path=log_path,
                    on_body_event=lambda ev: log.info("body event %s", ev),
                    paused=paused.is_set if paused is not None else None)
    log.info("behaviour runner: log %s, seed %s, personality %s", log_path, runner.seed, cfg.get("name"))

    async def main() -> str:
        async def watch() -> None:
            while not stop_event.is_set():
                await asyncio.sleep(0.05)
            runner.stop()
        w = asyncio.ensure_future(watch())
        try:
            return await runner.run()
        finally:
            w.cancel()

    why = asyncio.run(main())
    log.info("behaviour runner stopped: %s after %d ticks (unsupported calls: %s)", why, runner.ticks,
             dict(body.unsupported) or "-")
    return why
