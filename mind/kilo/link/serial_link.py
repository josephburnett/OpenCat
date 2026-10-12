"""SerialLink: the mind's end of the body link (docs/architecture.md §4, §5).

Threads:

- **reader**: owns (re)connection with backoff, reads the transport, feeds the
  codec's streaming decoder, unpacks messages, and updates BodyState. ACK
  matching, epoch handling and reboot detection happen here so that a
  ``request()`` blocked on another thread is answered even while the mind's
  main loop is busy. User callbacks are handed to ``dispatch`` (inline by
  default; ``attach_asyncio`` routes them through ``call_soon_threadsafe``).
- **writer**: drains the outbound queue, re-sends latched goals at 5 Hz,
  polls HELLO_REQ at 2 Hz while the body is lost (and once on every
  (re)connect), and sends HEARTBEAT after 100 ms of silence. All of this
  autonomous traffic is sent only while the mind's liveness stamp
  (``mark_alive()``) is < 1 s old, so a stalled mind trips the body's watchdog
  instead of being masked by this thread. Goals and commands issued by the
  mind itself are sent immediately regardless.
- **logger**: ``kilo.log.JsonlLogger`` (optional) writes every frame off-thread.
- **wire_logger**: a ``kilo.logs.kilolog.KilologWriter`` (optional): every
  frame's wire bytes as sent (writer path) and as received (reader path, from
  the decoder's ``feed_wire``), for ``kilo --kilolog``.

Reconnection: the reader thread reopens the port (calls the factory) after a
failed open or a read/write error, backing off from ``reconnect_min_s`` to
``reconnect_max_s``; the first failure of a streak is logged as a warning,
``last_open_error``/``open_fail_streak`` say what is going on, and
``connection`` subscribers hear every connect and disconnect.

Silence (``passive=True`` or ``go_silent()``): the link keeps reading and
decoding, but writes nothing at all to the transport: no HELLO_REQ, no
heartbeat, no goal refresh, no commands (``request()`` raises). The body
streams TELEMETRY from boot at its TELEM_DIV rate and sends EVENTs and HELLO
unprompted, so a silent link still sees everything the body does on its own;
it just does not feed (or arm) the body's link watchdog. Every write holds
``_tx_lock`` and checks the flag under it, so once ``go_silent()`` returns no
further byte is written (a write in progress finishes first and is the
``last_tx`` it returns). A ``request()`` waiting for its reply is woken with
``LinkError`` (its frame may have been dropped), not left to time out.

Goals and the epoch rule (§3c, §4, §5):

- ``set_goal(name, **fields)`` latches a goal. Epoch-bearing goals (SET_POSTURE,
  SET_GAIT, SET_HEAD, SET_BODY_POSE) are stamped with the epoch current when
  the mind set them, not re-stamped later, so a goal can never outlive an
  epoch bump it did not see.
- On any epoch change seen in TELEMETRY, EVENT (every EVENT carries the
  post-bump epoch) or HELLO, on EVENT FELL / WATCHDOG / LOW_BATTERY, and on a
  body reboot, desired motion goals from older epochs are cleared and
  ``goals_cleared`` subscribers are told. Nothing is replayed.
- EVENT GOAL_REFUSED with reason STALE drops the refused goal and logs it.
- Contact (a body that may just have rebooted): after every (re)connect, and
  after the body was lost (no TELEMETRY for 300 ms), no motion goal
  (SET_POSTURE other than REST/LIMP, a SET_BALANCE enable, SET_GAIT other
  than a stop, the other epoch goals) is sent or refreshed, and TRICK_PLAY is refused, until the body has
  been heard on this connection: a HELLO that proves a reboot (motion goals
  are cleared first, under the same lock the writer packs goals under), or a
  TELEMETRY (its tick is checked for a restart first). After a reconnect or a
  loss a HELLO that looks like the last one is not enough: only the tick of
  the next TELEMETRY tells a quiet reboot apart. A HELLO is a reboot when it
  is unsolicited, or when its identity (reset cause, last_alive_tick,
  fw_build) differs from the previous HELLO's. Goals set meanwhile stay
  latched and go out once contact is back. (A body that reboots without the
  mind noticing within 300 ms still sees the refreshes sent meanwhile; it
  boots at a new epoch, so they are STALE: docs §3c "Boot epoch".)

Commands: ``request()`` assigns a seq, waits 50 ms for the matching ACK (or
the data reply, for queries), and retries up to 3 times. Non-query commands
retry with the *same* seq so the body's de-dup re-acks without re-executing;
queries (HELLO_REQ, CAL_GET, ...) are answered by their data reply only (an
ACK only on failure), are never de-duplicated, and retry with a fresh seq.
One request is in flight at a time. The TX seq starts at a random value per
link, so a fresh CLI process does not repeat the (seq, type) of the previous
one inside the body's 500 ms de-dup window.
"""

from __future__ import annotations

import contextlib
import logging
import queue
import random
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

from kilo.body.state import BodyState
from kilo.link import consts
from kilo.link.codec import Codec, DecodeError, Frame, default_codec
from kilo.link.messages import Messages, default_messages, fields_of
from kilo.link.transport import Transport, TransportFactory

log = logging.getLogger("kilo.link")

_DEFAULT = object()


class LinkError(Exception):
    pass


class NotConnected(LinkError):
    pass


class LinkTimeout(LinkError):
    pass


class CommandError(LinkError):
    def __init__(self, name: str, status: int):
        self.name = name
        self.status = status
        self.status_name = consts.ACK_STATUS.get(status, str(status))
        super().__init__(f"{name} refused: {self.status_name}")


@dataclass
class LinkConfig:
    heartbeat_idle_s: float = 0.100   # §4: HEARTBEAT after 100 ms of silence
    liveness_window_s: float = 1.0    # §4/§5: only while the mind proved alive < 1 s ago
    goal_refresh_s: float = 0.200     # §4: active goals re-sent at 5 Hz
    ack_timeout_s: float = 0.050      # §4: retry after 50 ms
    retries: int = 3                  # §4: up to 3 times
    hello_poll_s: float = 0.500       # §4: HELLO_REQ at 2 Hz while lost
    hello_solicit_window_s: float = 1.0  # a HELLO within this of a HELLO_REQ is solicited
    reconnect_min_s: float = 0.1
    reconnect_max_s: float = 2.0
    read_size: int = 256
    housekeeping_s: float = 0.010


@dataclass
class Goal:
    name: str
    type: int
    fields: dict[str, Any]
    epoch: int | None  # epoch-bearing goals only: the epoch they were set under
    dirty: bool = True
    sends: int = 0


class _Pending:
    def __init__(self, name: str, type: int, reply: str | None):
        self.name = name
        self.type = type
        self.reply = reply
        self.seq = -1
        self.done = threading.Event()
        self.status: int | None = None
        self.reply_msg: Any = None
        self.acked = False
        self.silenced = False  # go_silent() ended the wait: its frame may never have gone out


class SerialLink:
    def __init__(
        self,
        transport_factory: TransportFactory,
        *,
        codec: Codec | None = None,
        messages: Messages | None = None,
        state: BodyState | None = None,
        config: LinkConfig | None = None,
        logger: Any = None,
        dispatch: Callable[..., Any] | None = None,
        port_name: str | None = None,
        clock: Callable[[], float] = time.monotonic,
        passive: bool = False,
        wire_logger: Any = None,
    ):
        self._factory = transport_factory
        self.codec = codec or default_codec()
        self.messages = messages or default_messages()
        self.state = state or BodyState(clock=clock)
        self.cfg = config or LinkConfig()
        self.logger = logger
        self.wire_logger = wire_logger
        self.last_open_error: str | None = None
        self.open_fail_streak = 0
        self.retry_in_s: float | None = None  # current reconnect backoff while the port is missing
        self._dispatch_fn = dispatch
        self.clock = clock
        self.state.port = port_name

        self._lock = self.state.lock
        self._changed = threading.Condition(self._lock)
        self._txq: queue.Queue[tuple[int, int | None, bytes, str]] = queue.Queue()
        self._transport: Transport | None = None
        self._broken = threading.Event()
        self._running = threading.Event()
        self._threads: list[threading.Thread] = []
        self._seq = random.randrange(256)
        self._hello_on_connect = False
        self._goals: dict[str, Goal] = {}
        self._req_lock = threading.Lock()
        self._pending: _Pending | None = None
        self._last_done: tuple[int, int] | None = None
        self._subs: dict[str, list[Callable[..., Any]]] = {}
        self._mind_alive_at = -1e9
        self._last_tx_at = -1e9
        self._last_refresh_at = -1e9
        self._last_hello_poll = -1e9
        self._hello_solicited_at = -1e9
        self._last_reboot_at = -1e9
        self._was_alive = False
        self._ever_connected = False
        self._ever_contact = False
        # Contact with the body on this connection (see "Contact" above).
        self._contact = False
        self._contact_needs_telemetry = False
        self.alive_stamps: dict[str, float] = {}
        self._deferred: tuple[int, list[tuple[str, tuple]]] | None = None
        self._tx_lock = threading.Lock()
        self._silent = passive
        self.last_tx: tuple[float, float, str] | None = None  # (clock, wall time, type) of the last frame
        self.last_heartbeat: tuple[float, float] | None = None  # (clock, wall time)
        # Completed writer passes (housekeeping included): a stepped clock
        # (kilo.vbody.core) waits for one to start and end after each step.
        self.writer_passes = 0

    # ---- lifecycle -------------------------------------------------------------
    def start(self) -> "SerialLink":
        if self._running.is_set():
            return self
        self._running.set()
        self._threads = [
            threading.Thread(target=self._reader, name="kilo-link-rx", daemon=True),
            threading.Thread(target=self._writer, name="kilo-link-tx", daemon=True),
        ]
        for t in self._threads:
            t.start()
        return self

    def stop(self, timeout: float = 1.0) -> None:
        """Stop both threads and close the transport (e.g. before flashing)."""
        self._running.clear()
        for t in self._threads:
            t.join(timeout)
        self._threads = []
        self._close_transport()

    @property
    def running(self) -> bool:
        return self._running.is_set()

    def __enter__(self) -> "SerialLink":
        return self.start()

    def __exit__(self, *exc: Any) -> None:
        self.stop()

    # ---- silence -------------------------------------------------------------------
    @property
    def silent(self) -> bool:
        return self._silent

    def go_silent(self) -> tuple[float, float, str] | None:
        """Stop writing to the transport, at once and for good.

        Drops every latched goal and queued frame without sending them (no
        final refresh). Returns ``last_tx``: (clock, wall time, type) of the
        last frame written, which is when the body's watchdog last got fed.
        """
        with self._tx_lock:
            self._silent = True
            with self._lock:
                self._goals.clear()
                # A request() waiting for its reply would otherwise sit out all
                # its retries for a frame that is now dropped: wake it.
                p = self._pending
                if p is not None and not p.done.is_set():
                    p.silenced = True
                    p.done.set()
            while True:
                try:
                    self._txq.get_nowait()
                except queue.Empty:
                    break
            last = self.last_tx
        self._log("link", silent=True)
        return last

    # ---- mind liveness ----------------------------------------------------------
    def mark_alive(self, source: str = "mind") -> None:
        """Called by the mind's main loop each iteration (≥ 1 Hz).

        ``source`` names who stamped (kept in ``alive_stamps``): the daemon's
        pump stamps as "daemon", so it can tell whether the behaviour runner
        (which stamps as "mind") is still ticking."""
        now = self.clock()
        self._mind_alive_at = now
        self.alive_stamps[source] = now

    def mind_alive(self) -> bool:
        return self.clock() - self._mind_alive_at < self.cfg.liveness_window_s

    def attach_asyncio(self, loop: Any, period_s: float = 0.2) -> None:
        """Deliver callbacks on ``loop`` and stamp liveness from a task on it (§5)."""
        import asyncio

        self._dispatch_fn = loop.call_soon_threadsafe

        async def pump() -> None:
            while self.running:
                self.mark_alive()
                await asyncio.sleep(period_s)

        loop.call_soon_threadsafe(lambda: loop.create_task(pump()))

    # ---- subscriptions ----------------------------------------------------------
    def subscribe(self, kind: str, fn: Callable[..., Any]) -> None:
        """Kinds and arguments:

        ``telemetry(msg)``, ``event(msg)``, ``hello(msg, solicited)``,
        ``reboot(hello_or_None, why)``, ``goals_cleared(reason, names)``,
        ``refused(goal_name, reason_name)``, ``connection(connected, alive)``,
        ``frame(frame, msg_or_None)`` (every valid frame).
        """
        self._subs.setdefault(kind, []).append(fn)

    def _emit(self, kind: str, *args: Any) -> None:
        d = self._deferred
        if d is not None and d[0] == threading.get_ident():
            d[1].append((kind, args))  # inside _locked_then_emit(): after the lock
            return
        for fn in self._subs.get(kind, ()):
            if self._dispatch_fn is not None:
                self._dispatch_fn(self._safe_call, fn, args)
            else:
                self._safe_call(fn, args)

    @contextlib.contextmanager
    def _locked_then_emit(self) -> Any:
        """Hold the state lock; callbacks emitted inside run after it is released."""
        with self._lock:
            self._deferred = (threading.get_ident(), [])
            try:
                yield
            finally:
                held, self._deferred = self._deferred[1], None
        for kind, args in held:
            self._emit(kind, *args)

    @staticmethod
    def _safe_call(fn: Callable[..., Any], args: tuple) -> None:
        try:
            fn(*args)
        except Exception:
            log.exception("link callback %r failed", fn)

    def _log(self, kind: str, **fields: Any) -> None:
        if self.logger is not None:
            self.logger.log(kind, **fields)

    # ---- goals ------------------------------------------------------------------
    def set_goal(self, name: str, **fields: Any) -> Goal:
        """Latch a desired-state goal. Sent now, then refreshed at 5 Hz.

        Raises ValueError if the message cannot be
        packed, so errors surface at the call site and not in the writer.
        """
        if name not in consts.GOAL_TYPES or name == "HEARTBEAT":
            raise ValueError(f"{name} is not a goal")
        epoch_goal = name in consts.EPOCH_GOALS
        with self._lock:
            epoch = self.state.epoch if epoch_goal else None
            # validate now
            self._pack_goal(name, fields, epoch if epoch is not None else 0, epoch_goal)
            g = Goal(name, self.messages.type_of(name), dict(fields), epoch)
            self._goals[name] = g
        self._log("goal", action="set", name=name, fields=fields, epoch=epoch)
        return g

    def clear_goal(self, name: str, only_if: Callable[[Goal], bool] | None = None) -> bool:
        """Drop a latched goal (with ``only_if``: only when it says so,
        checked under the lock). Returns whether one was dropped."""
        with self._lock:
            g = self._goals.get(name)
            if only_if is not None and (g is None or not only_if(g)):
                return False
            self._goals.pop(name, None)
        self._log("goal", action="clear", name=name)
        return g is not None

    def goals(self) -> dict[str, Goal]:
        with self._lock:
            return dict(self._goals)

    def clear_motion_goals(self, reason: str, keep_epoch: int | None = None) -> list[str]:
        """Drop desired motion goals (all, or only those not from ``keep_epoch``)."""
        with self._lock:
            names = [n for n, g in self._goals.items() if n in consts.MOTION_GOALS
                     and (keep_epoch is None or (g.epoch is not None and g.epoch != keep_epoch))]
            if keep_epoch is not None:
                # goals still waiting for a first epoch adopt this one
                for g in self._goals.values():
                    if g.name in consts.EPOCH_GOALS and g.epoch is None:
                        g.epoch = keep_epoch
            for n in names:
                del self._goals[n]
            self.state.stats.goals_cleared += len(names)
        if names:
            log.info("cleared goals %s (%s)", names, reason)
            self._log("goal", action="cleared", names=names, reason=reason)
            self._emit("goals_cleared", reason, names)
        return names

    def _pack_goal(self, name: str, fields: dict[str, Any], epoch: int | None, epoch_goal: bool) -> bytes:
        if epoch_goal:
            return self.messages.pack(name, epoch=epoch, **fields)
        return self.messages.pack(name, **fields)

    # ---- raw sends and commands ---------------------------------------------
    def _next_seq(self) -> int:
        with self._lock:
            s = self._seq
            self._seq = (self._seq + 1) & 0xFF
            return s

    def send(self, name: str, **fields: Any) -> None:
        """Fire-and-forget (queued). For goals prefer ``set_goal``."""
        payload = self.messages.pack(name, **fields)
        self._txq.put((self.messages.type_of(name), None, payload, name))

    def request(self, name: str, fields: dict[str, Any] | None = None, *,
                reply: Any = _DEFAULT, timeout: float | None = None,
                retries: int | None = None) -> Any:
        """Send a command and wait for its ACK or data reply.

        Returns the reply message (queries) or the ACK message. Raises
        NotConnected, LinkTimeout, or CommandError (ACK status != OK).
        """
        reply_name = consts.REPLIES.get(name) if reply is _DEFAULT else reply
        # Queries are retried with a fresh seq (the body never de-dups them);
        # everything else with the same seq.
        fresh_seq = name in consts.QUERIES or reply_name is not None
        payload = self.messages.pack(name, **(fields or {}))
        typ = self.messages.type_of(name)
        timeout = self.cfg.ack_timeout_s if timeout is None else timeout
        retries = self.cfg.retries if retries is None else retries
        if self._silent:
            raise LinkError(f"{name}: the link is passive (sends nothing)")
        if self._transport is None:
            raise NotConnected(f"{name}: not connected")
        if name == "TRICK_PLAY" and not self._contact:
            raise NotConnected(f"{name}: the body has not been heard on this connection yet")
        with self._req_lock:
            p = _Pending(name, typ, reply_name)
            self.state.stats.commands += 1
            ok = False
            for attempt in range(retries + 1):
                with self._lock:
                    # checked under the lock go_silent() wakes waiters under: a
                    # request is either refused here or woken by go_silent()
                    if self._silent:
                        p.silenced = True
                        break
                    if attempt == 0 or fresh_seq:
                        p.seq = self._next_seq()
                    self._pending = p
                    if name == "HELLO_REQ":
                        self._hello_solicited_at = self.clock()
                self._txq.put((typ, p.seq, payload, name))
                if p.done.wait(timeout):
                    ok = not p.silenced
                    break
                if attempt < retries:
                    self.state.stats.retries += 1
                    log.debug("%s seq=%d: no answer, retry %d", name, p.seq, attempt + 1)
            with self._lock:
                self._pending = None
                if ok:
                    self._last_done = (p.seq, p.type)
            if p.silenced:
                self._log("cmd", name=name, seq=p.seq, result="silenced")
                raise LinkError(f"{name}: the link went silent (sends nothing)")
            if not ok:
                self.state.stats.timeouts += 1
                self._log("cmd", name=name, seq=p.seq, result="timeout")
                raise LinkTimeout(f"{name}: no answer after {retries + 1} tries")
        if p.status is not None and p.status != consts.OK:
            self.state.stats.nacks += 1
            self._log("cmd", name=name, seq=p.seq, result=consts.ACK_STATUS.get(p.status, p.status))
            raise CommandError(name, p.status)
        return p.reply_msg

    def wait_until(self, pred: Callable[[], bool], timeout: float, *, alive: bool = False) -> bool:
        """Wait until ``pred()`` is true (checked on every received frame).

        Waiting does not stamp liveness: a pool thread waiting for a reply
        must not keep a stalled mind looking alive. Only a caller that *is*
        the mind's main loop (a CLI's main thread) passes ``alive=True``.
        """
        deadline = self.clock() + timeout
        with self._changed:
            while True:
                if alive:
                    self.mark_alive()
                if pred():
                    return True
                left = deadline - self.clock()
                if left <= 0:
                    return False
                self._changed.wait(min(left, 0.1))

    # ---- writer thread -------------------------------------------------------
    def _writer(self) -> None:
        while self._running.is_set():
            try:
                item = self._txq.get(timeout=self.cfg.housekeeping_s)
            except queue.Empty:
                item = None
            if item is not None:
                typ, seq, payload, name = item
                self._send_frame(typ, self._next_seq() if seq is None else seq, payload, name)
            try:
                self._housekeeping()
            except Exception:
                log.exception("link housekeeping failed")
            self.writer_passes += 1

    def _send_frame(self, typ: int, seq: int, payload: bytes, name: str) -> bool:
        with self._tx_lock:
            return self._send_frame_locked(typ, seq, payload, name)

    def _send_frame_locked(self, typ: int, seq: int, payload: bytes, name: str) -> bool:
        t = self._transport
        st = self.state.stats
        if self._silent:
            st.tx_silenced += 1
            return False
        if t is None or self._broken.is_set():
            st.tx_dropped += 1
            return False
        try:
            data = self.codec.encode(typ, seq, payload)
        except ValueError:
            log.exception("cannot encode %s", name)
            return False
        try:
            t.write(data)
        except OSError as e:
            log.warning("write failed: %s", e)
            self._broken.set()
            st.tx_dropped += 1
            return False
        self._last_tx_at = self.clock()
        self.last_tx = (self._last_tx_at, time.time(), name)
        if self.wire_logger is not None:
            self._wire(data, "tx", self._last_tx_at)
        if name == "HEARTBEAT":
            self.last_heartbeat = self.last_tx[:2]
        st.frames_tx += 1
        st.bytes_tx += len(data)
        if self.logger is not None and name != "HEARTBEAT":
            self._log("tx", type=name, seq=seq, payload=payload.hex())
        return True

    def _housekeeping(self) -> None:
        now = self.clock()
        alive = self.state.alive
        if alive != self._was_alive:
            self._was_alive = alive
            if not alive:
                with self._lock:
                    # it may be rebooting: no motion goal until it is heard again
                    self._contact = False
                    self._contact_needs_telemetry = True
            log.info("body %s", "alive" if alive else "lost (no telemetry for 300 ms)")
            self._log("link", alive=alive, connected=self.state.connected)
            self._emit("connection", self.state.connected, alive)
        if self._transport is None or self._broken.is_set() or self._silent:
            return
        st = self.state.stats
        # goals changed by the mind: send immediately
        with self._lock:
            have_epoch = self.state.epoch
            todo = []
            refresh = self.mind_alive() and now - self._last_refresh_at >= self.cfg.goal_refresh_s
            if refresh:
                self._last_refresh_at = now
            contact = self._contact
            for g in self._goals.values():
                if not (g.dirty or refresh):
                    continue
                if not contact and _is_motion(g):
                    continue  # stays dirty: sent once the body is heard on this connection
                epoch_goal = g.name in consts.EPOCH_GOALS
                if epoch_goal and g.epoch is None:
                    if have_epoch is None:
                        continue  # wait for the first epoch from HELLO/telemetry
                    g.epoch = have_epoch
                try:
                    payload = self._pack_goal(g.name, g.fields, g.epoch, epoch_goal)
                except ValueError:
                    log.exception("cannot pack goal %s", g.name)
                    continue
                g.dirty = False
                g.sends += 1
                todo.append((g.type, payload, g.name))
        for typ, payload, name in todo:
            if self._send_frame(typ, self._next_seq(), payload, name):
                st.goal_sends += 1
        if not self.mind_alive():
            return  # a stalled mind sends nothing on its own: let the body's watchdog trip
        if self._hello_on_connect or (not self.state.alive
                                      and now - self._last_hello_poll >= self.cfg.hello_poll_s):
            self._hello_on_connect = False
            self._last_hello_poll = now
            with self._lock:
                self._hello_solicited_at = now
            st.hello_polls += 1
            self._send_frame(self.messages.type_of("HELLO_REQ"), self._next_seq(), b"", "HELLO_REQ")
        if now - self._last_tx_at >= self.cfg.heartbeat_idle_s:
            if self._send_frame(self.messages.type_of("HEARTBEAT"), self._next_seq(), b"", "HEARTBEAT"):
                st.heartbeats += 1

    # ---- reader thread ---------------------------------------------------------
    def _close_transport(self) -> None:
        t, self._transport = self._transport, None
        if t is not None:
            try:
                t.close()
            except Exception:
                pass
        with self._lock:
            was = self.state.connected
            self.state.connected = False
        if was:
            self.state.stats.disconnects += 1
            self._log("link", connected=False)
            self._emit("connection", False, False)

    def _wire(self, data: bytes, direction: str, t: float) -> None:
        try:
            self.wire_logger.write_wire(data, direction, t)
        except Exception as e:  # a full disk or a closed writer never stops the link
            log.debug("wire log: %s", e)

    def _connect(self) -> bool:
        try:
            t = self._factory()
        except Exception as e:  # OSError, serial.SerialException, ...
            self.state.stats.open_failures += 1
            self.last_open_error = str(e) or type(e).__name__
            self.open_fail_streak += 1
            if self.open_fail_streak == 1:
                log.warning("cannot open %s: %s; retrying (backoff %.1f-%.0f s)", self.state.port or "port",
                            self.last_open_error, self.cfg.reconnect_min_s, self.cfg.reconnect_max_s)
            else:
                log.debug("open failed: %s", e)
            return False
        if self.open_fail_streak:
            log.info("%s is back after %d failed opens", self.state.port or "port", self.open_fail_streak)
        self.open_fail_streak = 0
        self.last_open_error = None
        self.retry_in_s = None
        self._decoder = self.codec.new_decoder()
        self._broken.clear()
        with self._lock:
            self._transport = t
            self.state.connected = True
            self.state.connected_since = self.clock()
            if self._ever_connected:
                self.state.stats.reconnects += 1
            self._ever_connected = True
            self._hello_solicited_at = self.clock()
            # A (re)opened port may have reset the body (DTR): no motion goal
            # until it is heard on this connection.
            self._contact = False
            self._contact_needs_telemetry = self._ever_contact
            for g in self._goals.values():
                g.dirty = True
            # first contact: ask who is there (sent by the writer, liveness-gated)
            self._hello_on_connect = True
        log.info("connected to %s", self.state.port or "transport")
        self._log("link", connected=True, port=self.state.port)
        self._emit("connection", True, False)
        return True

    def _reader(self) -> None:
        backoff = self.cfg.reconnect_min_s
        while self._running.is_set():
            if self._transport is None or self._broken.is_set():
                if self._transport is not None:
                    self._close_transport()
                if not self._connect():
                    self.retry_in_s = backoff
                    self._sleep(backoff)
                    backoff = min(backoff * 2, self.cfg.reconnect_max_s)
                    continue
                backoff = self.cfg.reconnect_min_s
            t = self._transport
            try:
                data = t.read(self.cfg.read_size)
            except OSError as e:
                log.warning("read failed: %s", e)
                self._broken.set()
                continue
            if not data:
                continue
            now = self.clock()
            self.state.stats.bytes_rx += len(data)
            self.state.last_rx_at = now
            if self.wire_logger is not None and hasattr(self._decoder, "feed_wire"):
                items = []
                for item, wire in self._decoder.feed_wire(data):
                    if wire:
                        self._wire(wire, "rx", now)
                    items.append(item)
            else:
                items = self._decoder.feed(data)
            for item in items:
                try:
                    self._handle(item, now)
                except Exception:
                    log.exception("error handling %r", item)
            with self._changed:
                self._changed.notify_all()

    def _sleep(self, s: float) -> None:
        end = self.clock() + s
        while self._running.is_set() and self.clock() < end:
            time.sleep(min(0.02, s))

    def _handle(self, item: Frame | DecodeError, now: float) -> None:
        st = self.state.stats
        if isinstance(item, DecodeError):
            with self._lock:  # LinkStats.as_dict() copies the counters under it
                st.decode_errors[item.kind] += 1
            self._log("rx_error", error=item.kind, raw=bytes(item.raw).hex(), type=item.type,
                      seq=item.seq)
            return
        st.frames_rx += 1
        name = self.messages.name_of(item.type)
        try:
            msg = self.messages.unpack(item.type, item.payload)
        except ValueError as e:
            st.undecodable += 1
            msg = None
            log.debug("undecodable %s: %s", name, e)
        if self.logger is not None:
            self._log("rx", type=name, seq=item.seq, payload=item.payload.hex(),
                      fields=None if msg is None else _fields(msg))
        self._emit("frame", item, msg)
        if msg is None:
            # a query reply we cannot unpack still completes the request, raw
            self._match_reply(name, item)
            return
        handler = getattr(self, "_on_" + name.lower(), None)
        if handler is not None:
            handler(msg, now)
        self._match_reply(name, msg)

    def _match_reply(self, name: str, msg: Any) -> None:
        with self._lock:
            p = self._pending
            if p is not None and p.reply == name and not p.done.is_set():
                p.reply_msg = msg
                p.status = consts.OK
                p.done.set()

    def _on_ack(self, msg: Any, now: float) -> None:
        st = self.state.stats
        with self._lock:
            p = self._pending
            if p is not None and msg.seq == p.seq and msg.type == p.type and not p.done.is_set():
                p.acked = True
                if msg.status != consts.OK or p.reply is None:
                    p.status = msg.status
                    p.reply_msg = msg
                    p.done.set()
                return  # OK ack for a query: keep waiting for the data reply
            if self._last_done == (msg.seq, msg.type) or (p is not None and p.done.is_set()):
                st.dup_acks += 1
            else:
                st.unmatched_acks += 1

    def _on_telemetry(self, msg: Any, now: float) -> None:
        with self._locked_then_emit():
            backwards, prev_epoch = self.state.on_telemetry(msg, now)
            had_hello = self.state.hello is not None
            # Under the lock: no goal refresh is packed between the reboot or
            # epoch check and the contact that re-enables motion goals.
            if backwards and had_hello:
                self._reboot(None, "tick went backwards", msg.epoch)
            elif prev_epoch is not None and msg.epoch != prev_epoch:
                self.clear_motion_goals(f"epoch {prev_epoch}->{msg.epoch}", keep_epoch=msg.epoch)
            elif prev_epoch is None:
                self.clear_motion_goals("first epoch", keep_epoch=msg.epoch)
            self._set_contact()
        self._emit("telemetry", msg)

    def _set_contact(self) -> None:
        """The body was heard on this connection (lock held): motion goals may go out."""
        if not self._contact:
            self._contact = True
            self._contact_needs_telemetry = False
            self._ever_contact = True
            for g in self._goals.values():
                if _is_motion(g):
                    g.dirty = True

    def _on_event(self, msg: Any, now: float) -> None:
        code = msg.code
        ev = consts.EVENTS.get(code, str(code))
        with self._lock:
            self.state.events.append((now, msg))
        if code == consts.EV["IR_KEY"]:
            from kilo import ir

            log.info("event IR_KEY %s epoch=%s", ir.describe(msg.arg), msg.epoch)
        else:
            log.info("event %s arg=%s epoch=%s", ev, msg.arg, msg.epoch)
        self._log("event", name=ev, code=code, arg=msg.arg, tick=msg.tick, epoch=msg.epoch)
        with self._lock:
            prev_epoch = self.state.epoch
            self.state.epoch = msg.epoch
        if code in consts.CLEARING_EVENTS or prev_epoch != msg.epoch:
            # every EVENT carries the post-bump epoch: goals from older epochs go
            self.clear_motion_goals(ev if code in consts.CLEARING_EVENTS
                                    else f"epoch {prev_epoch}->{msg.epoch} ({ev})",
                                    keep_epoch=msg.epoch)
        if code == consts.EV["GOAL_REFUSED"]:
            arg = msg.arg & 0xFFFF
            gtype, reason = arg >> 8, arg & 0xFF
            gname = self.messages.name_of(gtype)
            rname = consts.REFUSE_REASON.get(reason, str(reason))
            with self._lock:
                self.state.stats.goals_refused[f"{gname}:{rname}"] += 1
            if reason == consts.STALE:
                with self._lock:
                    g = self._goals.get(gname)
                    if g is not None and (g.epoch is None or g.epoch != msg.epoch):
                        del self._goals[gname]
                    else:
                        g = None
                log.warning("goal %s refused as STALE (epoch now %s)%s", gname, msg.epoch,
                            "; dropped" if g else "")
            else:
                log.warning("goal %s refused: %s", gname, rname)
            self._log("goal", action="refused", name=gname, reason=rname, epoch=msg.epoch)
            self._emit("refused", gname, rname)
        self._emit("event", msg)

    def _on_hello(self, msg: Any, now: float) -> None:
        cause = consts.RESET_CAUSES.get(msg.reset_cause, msg.reset_cause)
        with self._locked_then_emit():
            p = self._pending
            solicited = (now - self._hello_solicited_at < self.cfg.hello_solicit_window_s
                         or (p is not None and p.name == "HELLO_REQ"))
            prev = self.state.hello
            first = prev is None
            changed = prev is not None and _hello_identity(prev) != _hello_identity(msg)
            self.state.on_hello(msg, now, self.messages.PROTO_MAJOR)
            # Clear before contact, both under the lock: no refresh of an old
            # goal can be packed between this HELLO and the clear.
            rebooted = not first and (not solicited or changed)
            if rebooted:
                self._reboot(msg, f"{'new' if solicited else 'unsolicited'} HELLO (reset cause {cause})", msg.epoch)
            else:
                self.clear_motion_goals("hello", keep_epoch=msg.epoch)
            if rebooted or not self._contact_needs_telemetry:
                self._set_contact()
        log.info("HELLO proto %s.%s build %08x reset %s epoch %s%s", msg.proto_major,
                 msg.proto_minor, msg.fw_build, cause, msg.epoch,
                 "" if solicited else " (unsolicited)")
        if self.state.proto_ok is False:
            log.error("protocol major %s != expected %s: refusing to drive; flash the body",
                      msg.proto_major, self.messages.PROTO_MAJOR)
        self._emit("hello", msg, solicited)

    def _reboot(self, hello: Any, why: str, epoch: int) -> None:
        """A body reboot. Motion goals are cleared every time, before
        anything else (a second reboot within 3 s, a crash loop, must not
        keep goals set after the first); only the count and the ``reboot``
        notice are debounced (the HELLO and the tick reset of one reboot)."""
        now = self.clock()
        with self._lock:
            self.state.reset_session()
            self.state.epoch = epoch
            self.clear_motion_goals("reboot")
            if now - self._last_reboot_at < 3.0:
                return
            self._last_reboot_at = now
            self.state.reboots += 1
        log.warning("body rebooted: %s", why)
        self._log("link", reboot=why)
        self._emit("reboot", hello, why)


_SAFE_POSTURES = frozenset({consts.POSTURES["rest"], consts.POSTURES["limp"]})


def _is_motion(g: Goal) -> bool:
    """A goal that moves the body: SET_POSTURE but REST/LIMP, a SET_BALANCE
    enable, SET_GAIT with run=1, any other epoch goal. Held back while out of
    contact (a gait stop, run=0, is not: stopping is always safe to send)."""
    if g.name == "SET_POSTURE":
        return g.fields.get("posture") not in _SAFE_POSTURES
    if g.name == "SET_BALANCE":
        return bool(g.fields.get("enable"))
    if g.name == "SET_GAIT":
        return bool(g.fields.get("run"))
    return g.name in consts.MOTION_GOALS


def _hello_identity(h: Any) -> tuple:
    """What every HELLO of one boot of the body repeats."""
    return (getattr(h, "reset_cause", None), getattr(h, "last_alive_tick", None), getattr(h, "fw_build", None))


def _fields(m: Any) -> Any:
    if hasattr(m, "__dataclass_fields__"):
        return fields_of(m)
    return repr(m)
