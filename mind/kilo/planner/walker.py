"""Walking: the planner's ticks streamed to the body as FEET, 50 Hz.

``Walker`` (one per BodyClient: ``client.walker``) starts, steers and stops
a walk; ``Stream`` is one walk's producer, which the walker's thread wakes
every 20 ms (``Stream.wake``).

Start (``Walker.start``, blocking for about 0.6 s): TELEMETRY at 50 Hz
(SET_STREAM telem_div 1), the balance off for this walk if asked
(SET_BALANCE enable 0, the previous goal restored after), the STANCE
posture and its transition, then the posture goal is dropped (a SET_POSTURE
refresh would end GAIT on the body) and the stream starts. The first FEET
the body accepts puts it in GAIT.

Every wake (docs/walking-and-balance.md §10 "Stream timing"):

1. New TELEMETRY (BodyState.history, by count) updates the body's clock
   estimate: body time = local time + the largest (tick x 20 ms - arrival
   time) of the last two seconds (the least delayed frame is the best
   estimate; the window follows the drift between the body's crystal and
   the Pi's clock). In GAIT each one also gives the model's error to the
   ErrorTracker (offset, lag, spike) and to the planner (``observe``).
2. New EVENTs: GAIT_STOPPED (arg 0: a posture or an epoch bump took the
   legs; arg 1: the body settled after an underrun), FEET_UNDERRUN
   (counted), GOAL_REFUSED for FEET, FELL end the stream; so do a reboot, an
   epoch change, and the mind's loop stalling (``mind_alive``) for 0.5 s.
3. Every planned tick from the last one sent up to the body's tick + 5
   (link.feet_lookahead_ticks, 100 ms) goes out at once, so a late wake
   (a garbage-collector pause, an SD-card stall) is caught up, and ticks
   the body has already played are skipped (counted, not sent).

The stream never writes to disk: its per-tick records go to the frame
log's queue (kilo.log.JsonlLogger.log only enqueues) and are written by
that logger's thread. While a stream runs the garbage collector's young
generation threshold is raised and the objects that exist are frozen
(gc.freeze), so a collection pass stays short.

Stop: ``stop()`` asks the planner to stop (each leg steps back under its
shoulder, the body levels at the stance height); when the plan is done the
walker latches SET_POSTURE STANCE, which ends GAIT on the body (EVENT
GAIT_STOPPED arg 0), sends no FEET after it, and clears that posture goal
again once the body has left GAIT. ``abort()`` ends the stream without the
planned stop (the body holds, then settles on its own).
"""

from __future__ import annotations

import gc
import logging
import math
import threading
import time
from collections import deque
from typing import Any, Callable

from kilo.link import consts
from kilo.link.serial_link import LinkError
from kilo.planner import patterns as pat
from kilo.planner.errors import ErrorTracker
from kilo.planner.model import BodyModel
from kilo.planner.plan import Planner, PlannerConfig, TickPlan
from kilo.protocol import kilo_protocol as P

log = logging.getLogger("kilo.walk")

TICK_S = P.TICK_MS / 1000.0
LOOKAHEAD = P.FEET_LOOKAHEAD_TICKS
CLOCK_WINDOW = 100            # telemetry frames in the body-clock estimate (2 s at 50 Hz)
ENTER_TIMEOUT_S = 0.6         # the body must be in GAIT this soon after the first FEET
HANDOVER_TIMEOUT_S = 1.0      # ...and out of it this soon after the STANCE posture
MIND_STALL_S = 0.5
STANCE_WAIT_S = 0.6           # POSTURE_TICKS (25 ticks, 0.5 s) and a margin
GC_THRESHOLD0 = 20000
_FEET = P.Feet._FMT           # the FEET payload layout (generated)
_A16 = 16.0


class WalkError(LinkError):
    pass


def stop_timeout(period_ms: float | None = None) -> float:
    """How long a planned stop may take: the stride's ramp-down, a cycle to
    step each leg home, the level-out and a margin (default: the slowest
    pattern's cycle)."""
    period = period_ms or max(x.period_ms for x in pat.PATTERNS.values())
    return 2 * period / 1000.0 + 1.5


class BodyClock:
    """The body's tick, estimated from TELEMETRY arrival times."""

    def __init__(self, window: int = CLOCK_WINDOW):
        self._offs: deque = deque(maxlen=window)
        self._raw: int | None = None
        self.tick: int | None = None   # unwrapped tick of the newest TELEMETRY

    def add(self, raw_tick: int, rx_time: float) -> int:
        if self._raw is None:
            self.tick = raw_tick
        else:
            d = (raw_tick - self._raw) & 0xFFFF
            if d >= 0x8000:  # backwards: a restart (the stream ends on the reboot anyway)
                self._offs.clear()
                self.tick = raw_tick
            else:
                self.tick += d
        self._raw = raw_tick
        self._offs.append(self.tick * TICK_S - rx_time)
        return self.tick

    def now(self, t: float) -> float | None:
        """The body's tick at local time t (fractional), None before any TELEMETRY."""
        if not self._offs:
            return None
        return (t + max(self._offs)) / TICK_S


def pack_feet(epoch: int, tp: TickPlan) -> bytes:
    x, z = tp.x, tp.z
    return _FEET.pack(epoch, tp.tick & 0xFFFF, x[0], x[1], x[2], x[3], z[0], z[1], z[2], z[3],
                      int(round(tp.pitch * _A16)), int(round(tp.roll * _A16)), tp.support)


class Stream:
    """One walk: plan, send, watch. ``wake(now)`` is one 20 ms step; it
    returns False once the stream is over (``ended`` says why, ``ok`` whether
    it was the planned stop)."""

    def __init__(self, link: Any, planner: Planner, epoch: int, clock: BodyClock,
                 on_done: Callable[[], None], tracker: ErrorTracker | None = None):
        self.link = link
        self.state = link.state
        self.planner = planner
        self.epoch = epoch
        self.clock = clock
        self.on_done = on_done
        self.tracker = tracker or ErrorTracker()
        st = self.state
        with st.lock:
            self._tel_seen = st.telemetry_count
            self._ev_seen = st.event_count
            self._reboots = st.reboots
        self.sent = 0
        self.skipped = 0            # planned ticks already past when planned: not sent
        self.underruns = 0          # EVENT FEET_UNDERRUN seen
        self.max_behind = 0         # the most ticks one wake had to catch up
        self.last_sent: int | None = None
        self.gait_seen = False
        self.handover_at: float | None = None
        self.started_at: float | None = None
        self.stalled_since: float | None = None
        self.ended: str | None = None
        self.ok = False
        self.last_tel: Any = None

    # ---- what the body reports ----------------------------------------------------------
    def _new_items(self) -> tuple[list, list]:
        st = self.state
        with st.lock:
            nt = st.telemetry_count - self._tel_seen
            ne = st.event_count - self._ev_seen
            tel = list(st.history)[-nt:] if nt > 0 else []
            evs = list(st.events)[-ne:] if ne > 0 else []
            self._tel_seen = st.telemetry_count
            self._ev_seen = st.event_count
        return tel, evs

    def _end(self, why: str, ok: bool = False) -> bool:
        if self.ended is None:
            self.ended, self.ok = why, ok
            (log.info if ok else log.warning)("walk ended: %s", why)
            self._log("walk_end", reason=why, ok=ok, sent=self.sent, skipped=self.skipped,
                      underruns=self.underruns)
        return False

    def _log(self, kind: str, **fields: Any) -> None:
        lg = self.link.logger
        if lg is not None:
            lg.log(kind, **fields)

    def ingest(self) -> None:
        tel, evs = self._new_items()
        EV = consts.EV
        for _, e in evs:
            if e.code == EV["GAIT_STOPPED"]:
                if self.handover_at is not None:
                    self._end("stopped", ok=True)
                elif e.arg == 1:
                    self._end("the body settled after a FEET underrun (the stream was late)")
                else:
                    self._end("the body stopped the gait (a posture or an epoch bump took the legs)")
            elif e.code == EV["FEET_UNDERRUN"]:
                self.underruns += 1
                self._log("walk_underrun", ticks=e.arg)
            elif e.code == EV["GOAL_REFUSED"]:
                gtype, reason = (e.arg & 0xFFFF) >> 8, e.arg & 0xFF
                if gtype == int(P.MsgId.FEET):
                    self._end(f"the body refused FEET: {consts.REFUSE_REASON.get(reason, reason)}")
            elif e.code == EV["FELL"]:
                self._end("Kilo fell")
        gait = int(P.Mode.GAIT)
        for rx, m in tel:
            tick = self.clock.add(m.tick, rx)
            self.last_tel = m
            if m.mode == gait:
                self.gait_seen = True
                ep, er = self.tracker.add(tick, m)
                self.planner.observe(ep, er)
            elif self.gait_seen and self.handover_at is None and self.ended is None:
                self._end(f"the body left GAIT (mode {consts.MODES.get(m.mode, m.mode)})")

    # ---- one wake ------------------------------------------------------------------------------
    def wake(self, now: float) -> bool:
        if self.ended is not None:
            return False
        self.ingest()
        st = self.state
        if self.ended is not None:
            return False
        if st.reboots != self._reboots:
            return self._end("the body rebooted")
        if st.epoch != self.epoch:
            return self._end(f"the body's epoch changed ({self.epoch} -> {st.epoch})")
        if self.handover_at is not None:
            if self.gait_seen and self.last_tel is not None and self.last_tel.mode != int(P.Mode.GAIT):
                return self._end("stopped", ok=True)
            if now - self.handover_at > HANDOVER_TIMEOUT_S:
                return self._end("stopped (no GAIT_STOPPED seen after the STANCE posture)", ok=True)
            return True
        if not self.link.mind_alive():
            if self.stalled_since is None:
                self.stalled_since = now
            elif now - self.stalled_since > MIND_STALL_S:
                return self._end("the mind's main loop stalled")
            return True  # nothing goes out while the mind looks dead: the body holds, then settles
        self.stalled_since = None
        body = self.clock.now(now)
        if body is None:
            return True
        cur = int(math.floor(body))
        if self.started_at is None:
            self.started_at = now
        elif not self.gait_seen and now - self.started_at > ENTER_TIMEOUT_S:
            return self._end("the body did not enter GAIT")
        p = self.planner
        target = cur + LOOKAHEAD
        behind = 0
        while p.tick <= target and not p.done:
            tp = p.next()
            if tp.tick <= cur:
                self.skipped += 1
                continue
            behind += 1
            if self.link.send_stream("FEET", pack_feet(self.epoch, tp)):
                self.sent += 1
                self.last_sent = tp.tick
            if self.link.logger is not None:
                self._log("plan", tick=tp.tick, x=tp.x, z=tp.z, pitch=round(tp.pitch, 2), roll=round(tp.roll, 2),
                          support=tp.support, margin=round(tp.margin, 1))
        self.max_behind = max(self.max_behind, behind)
        if p.done and self.handover_at is None:
            # every planned tick is out; the STANCE posture ends GAIT after them
            self.handover_at = now
            self.on_done()
        return True


class Walker:
    """Starts, steers and stops walks on one BodyClient (one at a time)."""

    def __init__(self, client: Any, model: BodyModel | None = None, config: PlannerConfig | None = None,
                 clock: Callable[[], float] = time.monotonic, stance_wait_s: float = STANCE_WAIT_S):
        self.client = client
        self.link = client.link
        self.model = model or BodyModel()
        self.config = config or PlannerConfig()
        self.clock = clock
        self.stance_wait_s = stance_wait_s
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._stop_thread = threading.Event()
        self.stream: Stream | None = None
        self.planner: Planner | None = None
        self.tracker = ErrorTracker()
        self._restore_balance: dict | None = None
        self._balance_changed = False
        self.last_result: dict[str, Any] | None = None
        self._done = threading.Event()
        self._done.set()

    # ---- state -----------------------------------------------------------------------------
    @property
    def active(self) -> bool:
        t = self._thread
        return t is not None and t.is_alive()

    def status(self) -> dict[str, Any] | None:
        """What the walk is doing, or how the last one ended; None if there was none."""
        s, p = self.stream, self.planner
        if s is None or p is None:
            return None
        tm = p.timing()
        return {"active": self.active, "pattern": p.pattern.name, "direction": p.direction,
                "speed_mm_s": round(tm.speed_mm_s, 1), "stride_mm": round(tm.stride_mm, 1),
                "period_ms": round(tm.period_ms), "stopping": p.stopping, "ended": s.ended, "ok": s.ok,
                "sent": s.sent, "skipped": s.skipped, "underruns": s.underruns, "max_behind": s.max_behind,
                "holds": p.stats["holds"], "catches": p.stats["catches"],
                "min_margin_mm": None if p.stats["min_margin"] == math.inf else round(p.stats["min_margin"], 1),
                "error": self.tracker.estimate()}

    # ---- commands -------------------------------------------------------------------------------
    def start(self, pattern: str = pat.DEFAULT_PATTERN, direction: str = "forward",
              speed_mm_s: float | None = None, balance: bool = True) -> dict[str, Any]:
        """Walk (or, while walking, change pattern, direction and speed).
        Blocks for the STANCE transition on a start. Returns status()."""
        pattern, direction = pat.pattern(pattern).name, pat.direction(direction)
        with self._lock:
            if self.active and self.planner is not None and not self.planner.stopping:
                return self.update(pattern, direction, speed_mm_s)
            if self.active:  # a stop in progress: let it finish first
                self.wait(5.0)
            self._setup(balance)
            try:
                self._begin(pattern, direction, speed_mm_s)
            except BaseException:
                self._teardown()
                raise
        return self.status() or {}

    def update(self, pattern: str | None = None, direction: str | None = None,
               speed_mm_s: float | None = None) -> dict[str, Any]:
        with self._lock:
            p = self.planner
            if p is None or not self.active:
                raise WalkError("not walking")
            if pattern is not None:
                p.set_pattern(pattern)
            if direction is not None:
                p.set_direction(direction)
            if speed_mm_s is not None:
                p.set_speed(speed_mm_s)
        return self.status() or {}

    def stop(self, wait_s: float | None = None) -> bool | None:
        """The planned stop. Without ``wait_s`` returns None at once; with it,
        whether the walk ended within ``wait_s`` (any end counts)."""
        with self._lock:
            if self.planner is not None and self.active:
                self.planner.stop()
        if wait_s is None:
            return None
        return self.wait(wait_s)

    def abort(self) -> None:
        """End the stream now, without the planned stop: the body holds the
        last pose, then settles on its own (or a posture takes the legs)."""
        s = self.stream
        if s is not None and self.active:
            s._end("aborted")
        self.wait(1.0)

    def wait(self, timeout: float | None = None) -> bool:
        """Until the walk has ended (True) or ``timeout``. The CLI's main
        thread stamps the link's liveness while it waits here (the stream
        stops sending when the mind looks dead)."""
        if not getattr(self.client, "stamp_liveness", False):
            return self._done.wait(timeout)
        end = None if timeout is None else time.monotonic() + timeout
        while True:
            self.link.mark_alive()
            left = 0.05 if end is None else min(0.05, end - time.monotonic())
            if left <= 0:
                return self._done.is_set()
            if self._done.wait(left):
                return True


    # ---- setup and teardown -------------------------------------------------------------------------
    def _setup(self, balance: bool) -> None:
        c, st = self.client, self.link.state
        c._check_drivable()
        if st.epoch is None or not st.alive:
            raise WalkError("no telemetry from the body")
        if st.mode in ("FALLEN", "RECOVERING"):
            raise WalkError("Kilo is down: the fall reflex holds the joints")
        self._balance_changed = False
        c.set_stream(telem_div=1)
        if not balance:
            g = self.link.goals().get("SET_BALANCE")
            self._restore_balance = dict(g.fields) if g is not None else None
            c.set_balance(False)
            self._balance_changed = True
        if st.mode != "GAIT":
            c.posture("stance")
            self._sleep(self.stance_wait_s)
        self.link.clear_goal("SET_POSTURE")

    def _begin(self, pattern: str, direction: str, speed_mm_s: float | None) -> None:
        st = self.link.state
        bc = BodyClock()
        with st.lock:
            hist = list(st.history)[-CLOCK_WINDOW:]
            epoch = st.epoch
        for rx, m in hist:
            bc.add(m.tick, rx)
        now_tick = bc.now(self.clock())
        if now_tick is None:
            raise WalkError("no telemetry from the body")
        self.tracker = ErrorTracker()
        planner = Planner(self.model, pattern, direction, speed_mm_s, tick=int(math.floor(now_tick)) + LOOKAHEAD,
                          config=self.config)
        self.planner = planner
        self.stream = Stream(self.link, planner, epoch, bc, self._handover, self.tracker)
        self._stop_thread.clear()
        self._done.clear()
        self._thread = threading.Thread(target=self._run, name="kilo-walk", daemon=True)
        self._thread.start()
        log.info("walk %s %s at %.1f mm/s", pattern, direction, planner.timing().speed_mm_s)

    def _handover(self) -> None:
        try:
            self.client.posture("stance")
        except (LinkError, ValueError) as e:
            log.warning("walk: STANCE at the end: %s", e)

    def _run(self) -> None:
        s = self.stream
        old = gc.get_threshold()
        gc.freeze()
        gc.set_threshold(GC_THRESHOLD0, old[1], old[2])
        try:
            nxt = self.clock()
            while s.wake(self.clock()):
                nxt += TICK_S
                d = nxt - self.clock()
                if d < -2 * TICK_S:
                    nxt = self.clock()  # far behind (the wake caught up already): no burst of wakes
                elif d > 0:
                    time.sleep(d)
        except Exception as e:  # never leave a dead stream thread silently
            log.exception("walk stream failed")
            s._end(f"the stream failed: {e}")
        finally:
            gc.set_threshold(*old)
            gc.unfreeze()
            self._teardown()
            self.last_result = self.status()
            self._done.set()

    def _teardown(self) -> None:
        if self.stream is not None and self.stream.handover_at is not None:
            self.link.clear_goal("SET_POSTURE", only_if=lambda g: g.fields.get("posture") == consts.POSTURES["stance"])
        try:
            self.client.set_stream(telem_div=P.TELEMETRY_DEFAULT_DIV)
        except LinkError as e:
            log.warning("walk: telemetry rate not restored: %s", e)
        if self._balance_changed:
            self._balance_changed = False
            if self._restore_balance is not None:
                try:
                    self.link.set_goal("SET_BALANCE", **self._restore_balance)
                except ValueError:
                    self.link.clear_goal("SET_BALANCE")
            else:
                self.link.clear_goal("SET_BALANCE")
            self._restore_balance = None

    def _sleep(self, s: float) -> None:
        end = time.monotonic() + s
        while True:
            if getattr(self.client, "stamp_liveness", False):
                self.link.mark_alive()
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(0.05, left))
