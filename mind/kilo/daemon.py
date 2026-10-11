"""`kilo run`: the daemon that owns the serial port (docs/architecture.md §5, Appendix G).

One process owns the port, the SerialLink and the BodyClient, and serves a
Unix socket (``kilo.remote.socket_path()``) speaking JSON lines (format in
kilo.remote). Every ``kilo`` command tries the socket first and falls back to
opening the port itself when no daemon listens.

Threads and the loop:

- The asyncio loop accepts clients, parses requests and fans out streams. It
  never blocks: every call that waits on the body (requests, ``wait_ready``,
  the port release) runs in a small thread pool, and writes to clients are
  buffered by asyncio; a client whose buffer passes ``STREAM_BUFFER_MAX``
  loses stream messages (counted, then one ``dropped`` notice), one past
  ``REPLY_BUFFER_MAX`` is disconnected.
- A pump task stamps the link's liveness every 0.2 s (§5: the writer thread's
  heartbeats, goal refresh and HELLO polls only go out while the loop proves
  alive), so a stalled daemon trips the body's watchdog. The same task checks
  hold leases, the idle timeline and the port-release lease.

Holds. Goals a client sets (``posture``, ``set_balance``) belong to that
client's session and are kept alive by the daemon while the session lives and
keeps its lease (any message within ``hold_lease_s``; RemoteBody pings while
it holds). When the session closes or its lease lapses, the hold ends like
``kilo posture --for`` ends without a daemon: the goal is dropped, the daemon
latches REST at once and LIMP after ``idle_limp_s`` (the watchdog's REST/LIMP
timeline, without the 0.5 s stop stage), and a balance enable is switched
off. With a behaviour runner active the daemon only drops the goal and leaves
the body to the runner.

Port release (Appendix F). ``release_port {lease_s}`` stops the link and
closes the port so ``kilo flash`` can run avrdude; ``reacquire_port`` (or the
lease running out, or the releasing client disconnecting) reopens it. While
released, ops that need the body fail with NotConnected.

Shutdown (SIGTERM/SIGINT or the ``shutdown`` op): stop accepting, stop the
behaviour runner, close clients, drop every goal and go silent (the body's
watchdog takes over: stop, REST, LIMP), close the port, remove the socket.

Behaviour runner (``--behaviour``): if ``kilo.behaviour.runner`` imports, the
daemon calls ``run_behaviour(client, state, stop_event, log_dir)`` in its own
thread (an ``async def`` is awaited on the loop instead): ``client`` is the
daemon's BodyClient, ``state`` its BodyState, ``stop_event`` a
threading.Event set at shutdown (return promptly once it is set), ``log_dir``
a pathlib.Path for the runner's own logs. The daemon keeps the link alive; the
runner only sets goals. An import failure or an exception is logged and the
daemon carries on without it.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import inspect
import logging
import os
import signal
import threading
import time
from pathlib import Path
from typing import Any, Callable

from kilo import __version__
from kilo.body.client import BodyClient
from kilo.link import consts
from kilo.link.serial_link import LinkConfig, LinkError, NotConnected, SerialLink
from kilo.link.transport import DEFAULT_BAUD, open_port
from kilo.remote import dumps, encode_error, jsonable, socket_path

log = logging.getLogger("kilo.daemon")

STREAM_BUFFER_MAX = 256 << 10   # drop stream messages for a client beyond this backlog
REPLY_BUFFER_MAX = 4 << 20      # disconnect a client that does not read its replies
LINE_LIMIT = 1 << 20
PUMP_PERIOD_S = 0.2
STREAM_KINDS = ("snapshot", "telemetry", "event", "hello", "reboot", "goals_cleared", "connection")
IDLE_POSTURES = ("rest", "limp", "none")


class Session:
    def __init__(self, sid: int, writer: asyncio.StreamWriter):
        self.id = sid
        self.writer = writer
        self.last_seen = time.monotonic()
        self.inflight = 0
        self.goals: set[str] = set()
        self.kinds: set[str] = set()
        self.rate_hz = 0.0
        self.stream_task: asyncio.Task | None = None
        self.last_telemetry_sent = -1e9
        self.dropped = 0
        self.closed = False

    def touch(self) -> None:
        self.last_seen = time.monotonic()


class Daemon:
    def __init__(self, port: str, *, baud: int = DEFAULT_BAUD, socket: str | os.PathLike | None = None,
                 logger: Any = None, transport_factory: Callable[[], Any] | None = None,
                 link_config: LinkConfig | None = None, idle_posture: str = "rest",
                 idle_limp_s: float = 10.0, hold_lease_s: float = 3.0, behaviour: bool = False,
                 log_dir: Path | None = None, release_lease_s: float = 120.0, workers: int = 8):
        if idle_posture not in IDLE_POSTURES:
            raise ValueError(f"idle posture must be one of {IDLE_POSTURES}")
        self.port = port
        self.baud = baud
        self.socket_path = socket_path(socket)
        self.logger = logger
        self.factory = transport_factory or (lambda: open_port(port, baud))
        self.link_config = link_config
        self.idle_posture = idle_posture
        self.idle_limp_s = idle_limp_s
        self.hold_lease_s = hold_lease_s
        self.behaviour = behaviour
        self.log_dir = Path(log_dir) if log_dir else Path.home() / "kilo-logs"
        self.release_lease_s = release_lease_s
        self.pool = concurrent.futures.ThreadPoolExecutor(workers, thread_name_prefix="kilo-daemon")
        self.link: SerialLink | None = None
        self.client: BodyClient | None = None
        self.sessions: dict[int, Session] = {}
        self._next_sid = 1
        self.goal_owner: dict[str, Session | None] = {}
        self.idle_since: float | None = None
        self.idle_stage: str | None = None  # "rest" | "limp" once the idle timeline ran
        self.released_by: Session | None = None
        self.release_until: float | None = None
        self.port_released = False
        self.started_wall = time.time()
        self.started = time.monotonic()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.stopping: asyncio.Event | None = None
        self.ready = threading.Event()  # set once the socket is listening (tests)
        self.behaviour_stop = threading.Event()
        self.behaviour_state = "off"
        self._behaviour_thread: threading.Thread | None = None
        self._behaviour_task: asyncio.Task | None = None
        self._server: asyncio.AbstractServer | None = None
        self._shutting_down = False

    # ---- lifecycle ----------------------------------------------------------------------
    def run(self, install_signals: bool = True) -> int:
        return asyncio.run(self.serve(install_signals))

    def request_stop(self) -> None:
        if self.loop is not None and self.stopping is not None:
            try:
                self.loop.call_soon_threadsafe(self.stopping.set)
            except RuntimeError:
                pass  # the loop already finished

    async def serve(self, install_signals: bool = True) -> int:
        self.loop = asyncio.get_running_loop()
        self.stopping = asyncio.Event()
        await self._prepare_socket()
        self.link = SerialLink(self.factory, logger=self.logger, port_name=self.port,
                               config=self.link_config, dispatch=self.loop.call_soon_threadsafe)
        self.client = BodyClient(self.link)
        for kind in ("telemetry", "event", "hello", "reboot", "goals_cleared", "connection"):
            self.link.subscribe(kind, self._make_fanout(kind))
        self.link.mark_alive()
        self.link.start()
        self._server = await asyncio.start_unix_server(self._handle_client, path=str(self.socket_path),
                                                       limit=LINE_LIMIT)
        try:
            os.chmod(self.socket_path, 0o660)
        except OSError:
            pass
        if install_signals and threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGTERM, signal.SIGINT):
                try:
                    self.loop.add_signal_handler(sig, self.stopping.set)
                except (NotImplementedError, RuntimeError):
                    pass
        pump = asyncio.create_task(self._pump())
        if self.behaviour:
            self._start_behaviour()
        log.info("kilo %s daemon: port %s, socket %s", __version__, self.port, self.socket_path)
        self.ready.set()
        try:
            await self.stopping.wait()
        finally:
            log.info("shutting down")
            pump.cancel()
            await self._shutdown()
        return 0

    async def _prepare_socket(self) -> None:
        p = self.socket_path
        p.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        if p.exists() or p.is_symlink():
            from kilo.remote import connect

            conn = await asyncio.get_running_loop().run_in_executor(None, connect, p)
            if conn is not None:
                pid = conn.info.get("pid")
                conn.close()
                raise SystemExit(f"kilo run: a daemon (pid {pid}) already listens on {p}")
            p.unlink()  # stale

    async def _shutdown(self) -> None:
        self._shutting_down = True
        if self._server is not None:
            self._server.close()
        await self._stop_behaviour()
        for s in list(self.sessions.values()):
            self._close_session(s)
        link = self.link
        if link is not None:
            # Drop every goal and send nothing more: the body's watchdog takes over.
            link.go_silent()
            await self.loop.run_in_executor(self.pool, link.stop)
        if self.logger is not None:
            await self.loop.run_in_executor(self.pool, self.logger.close)
        try:
            self.socket_path.unlink()
        except OSError:
            pass
        self.pool.shutdown(wait=False, cancel_futures=True)

    # ---- behaviour runner ----------------------------------------------------------------
    def _start_behaviour(self) -> None:
        try:
            from kilo.behaviour.runner import run_behaviour  # type: ignore[import-not-found]
        except Exception as e:  # ImportError or a broken module
            self.behaviour_state = f"unavailable: {e}"
            log.warning("behaviour runner not started: %s", e)
            return
        self.log_dir.mkdir(parents=True, exist_ok=True)
        args = (self.client, self.client.state, self.behaviour_stop, self.log_dir)
        self.behaviour_state = "running"
        if inspect.iscoroutinefunction(run_behaviour):
            async def runner() -> None:
                try:
                    await run_behaviour(*args)
                    self.behaviour_state = "finished"
                except asyncio.CancelledError:
                    raise
                except Exception as e:
                    self.behaviour_state = f"failed: {e}"
                    log.exception("behaviour runner failed")
            self._behaviour_task = asyncio.create_task(runner())
        else:
            def runner_thread() -> None:
                try:
                    run_behaviour(*args)
                    self.behaviour_state = "finished"
                except Exception as e:
                    self.behaviour_state = f"failed: {e}"
                    log.exception("behaviour runner failed")
            self._behaviour_thread = threading.Thread(target=runner_thread, name="kilo-behaviour", daemon=True)
            self._behaviour_thread.start()
        log.info("behaviour runner started")

    @property
    def behaviour_active(self) -> bool:
        return self.behaviour_state == "running"

    async def _stop_behaviour(self) -> None:
        self.behaviour_stop.set()
        if self._behaviour_task is not None:
            try:
                await asyncio.wait_for(self._behaviour_task, 5.0)
            except (asyncio.TimeoutError, asyncio.CancelledError, Exception):
                self._behaviour_task.cancel()
        if self._behaviour_thread is not None:
            await self.loop.run_in_executor(None, self._behaviour_thread.join, 5.0)
            if self._behaviour_thread.is_alive():
                log.warning("behaviour runner did not stop within 5 s")

    # ---- pump: liveness, leases, idle timeline ------------------------------------------
    async def _pump(self) -> None:
        while True:
            try:
                self.link.mark_alive()
                self._check_leases()
            except Exception:
                log.exception("pump")
            await asyncio.sleep(PUMP_PERIOD_S)

    def _check_leases(self) -> None:
        now = time.monotonic()
        for s in list(self.sessions.values()):
            if s.goals and s.inflight == 0 and now - s.last_seen > self.hold_lease_s:
                log.info("client %d: hold lease lapsed (%.1f s silent)", s.id, now - s.last_seen)
                self._end_hold(s)
        if self.port_released and self.release_until is not None and now > self.release_until:
            log.warning("port release lease expired: reacquiring %s", self.port)
            self.loop.create_task(self._reacquire())
        if (self.idle_stage == "rest" and self.idle_posture == "rest" and self.idle_since is not None
                and now - self.idle_since >= self.idle_limp_s and not self._held()):
            self.idle_stage = "limp"
            self._latch_idle("limp")

    def _held(self) -> bool:
        return any(o is not None for o in self.goal_owner.values())

    def _latch_idle(self, name: str) -> None:
        try:
            self.client.posture(name)
            self.goal_owner["SET_POSTURE"] = None
        except (LinkError, ValueError) as e:
            log.warning("idle %s: %s", name, e)

    def _end_hold(self, s: Session) -> None:
        names = [n for n in s.goals if self.goal_owner.get(n) is s]
        s.goals.clear()
        if not names:
            return
        for n in names:
            self.goal_owner.pop(n, None)
        log.info("client %d: hold ended (%s)", s.id, ", ".join(sorted(names)))
        link = self.link
        if self._shutting_down:
            return  # go_silent() drops everything next
        if self.behaviour_active or self.port_released:
            for n in names:
                link.clear_goal(n)
            return
        if "SET_BALANCE" in names:
            g = link.goals().get("SET_BALANCE")
            fields = dict(g.fields) if g is not None else {}
            if fields:
                fields["enable"] = 0
                try:
                    link.set_goal("SET_BALANCE", **fields)  # latched off, refreshed
                except ValueError:
                    link.clear_goal("SET_BALANCE")
        if "SET_POSTURE" in names:
            link.clear_goal("SET_POSTURE")
            if self.idle_posture != "none" and not self._held():
                self.idle_since = time.monotonic()
                self.idle_stage = self.idle_posture
                self._latch_idle(self.idle_posture)
        for n in names:
            if n not in ("SET_POSTURE", "SET_BALANCE"):
                link.clear_goal(n)

    def _own(self, s: Session, name: str) -> None:
        prev = self.goal_owner.get(name)
        if prev is not None and prev is not s:
            prev.goals.discard(name)
        self.goal_owner[name] = s
        s.goals.add(name)
        self.idle_since = None
        self.idle_stage = None

    # ---- streams -----------------------------------------------------------------------
    def _make_fanout(self, kind: str) -> Callable[..., None]:
        def fan(*args: Any) -> None:
            if not self.sessions:
                return
            if kind == "telemetry":
                data = args[0]
            elif kind == "hello":
                data = {"hello": args[0], "solicited": args[1]}
            elif kind == "reboot":
                data = {"hello": args[0], "why": args[1]}
            elif kind == "goals_cleared":
                data = {"reason": args[0], "names": args[1]}
            elif kind == "connection":
                data = {"connected": args[0], "alive": args[1]}
            else:
                data = args[0]
            now = time.monotonic()
            for s in list(self.sessions.values()):
                if kind not in s.kinds:
                    continue
                if kind == "telemetry" and s.rate_hz > 0 and now - s.last_telemetry_sent < 1.0 / s.rate_hz:
                    continue
                if kind == "telemetry":
                    s.last_telemetry_sent = now
                self._stream(s, kind, data)
        return fan

    def _stream(self, s: Session, kind: str, data: Any) -> None:
        if s.closed:
            return
        buf = s.writer.transport.get_write_buffer_size() if s.writer.transport else 0
        if buf > STREAM_BUFFER_MAX:
            s.dropped += 1
            return
        if s.dropped:
            s.writer.write(dumps({"stream": "dropped", "t": time.monotonic(), "data": {"count": s.dropped}}))
            s.dropped = 0
        s.writer.write(dumps({"stream": kind, "t": time.monotonic(), "data": data}))

    async def _snapshot_stream(self, s: Session) -> None:
        period = 1.0 / s.rate_hz
        while not s.closed:
            self._stream(s, "snapshot", self.client.state.snapshot())
            await asyncio.sleep(period)

    # ---- clients ------------------------------------------------------------------------
    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        s = Session(self._next_sid, writer)
        self._next_sid += 1
        self.sessions[s.id] = s
        log.debug("client %d connected", s.id)
        try:
            while not s.closed:
                try:
                    line = await reader.readline()
                except (ValueError, asyncio.LimitOverrunError, ConnectionError):
                    break
                if not line:
                    break
                s.touch()
                await self._dispatch(s, line)
        finally:
            self._close_session(s)
            log.debug("client %d disconnected", s.id)

    def _close_session(self, s: Session) -> None:
        if s.closed:
            return
        s.closed = True
        self.sessions.pop(s.id, None)
        if s.stream_task is not None:
            s.stream_task.cancel()
        self._end_hold(s)
        if self.released_by is s and self.port_released:
            log.warning("client %d released the port and went away: reacquiring", s.id)
            self.loop.create_task(self._reacquire())
        try:
            s.writer.close()
        except Exception:
            pass

    def _reply(self, s: Session, msg: dict[str, Any]) -> None:
        if s.closed:
            return
        t = s.writer.transport
        if t is not None and t.get_write_buffer_size() > REPLY_BUFFER_MAX:
            log.warning("client %d does not read its replies: disconnecting", s.id)
            self._close_session(s)
            return
        s.writer.write(dumps(msg))

    async def _dispatch(self, s: Session, line: bytes) -> None:
        import json

        try:
            req = json.loads(line)
            rid, op, args = req.get("id"), req["op"], req.get("args") or {}
            if not isinstance(args, dict):
                raise ValueError("args must be an object")
        except (ValueError, KeyError, AttributeError, TypeError) as e:
            self._reply(s, {"id": None, "ok": False, "error": {"type": "ValueError", "message": f"bad request: {e}",
                                                                "args": [f"bad request: {e}"]}})
            return
        handler = getattr(self, "op_" + str(op), None)
        if handler is None:
            self._reply(s, {"id": rid, "ok": False, "error": {"type": "ValueError", "message": f"unknown op {op!r}",
                                                                "args": [f"unknown op {op!r}"]}})
            return
        s.inflight += 1
        try:
            result = await handler(s, **args)
            self._reply(s, {"id": rid, "ok": True, "result": jsonable(result)})
        except TypeError as e:
            self._reply(s, {"id": rid, "ok": False, "error": encode_error(ValueError(f"{op}: {e}"))})
        except Exception as e:
            if not isinstance(e, (LinkError, ValueError, KeyError)):
                log.exception("op %s failed", op)
            self._reply(s, {"id": rid, "ok": False, "error": encode_error(e)})
        finally:
            s.inflight -= 1
            s.touch()

    async def _blocking(self, fn: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
        return await self.loop.run_in_executor(self.pool, lambda: fn(*args, **kwargs))

    def _need_port(self) -> None:
        if self.port_released:
            raise NotConnected(f"{self.port} is released (flashing); try again when it is reacquired")

    # ---- ops ---------------------------------------------------------------------------
    async def op_daemon(self, s: Session) -> dict[str, Any]:
        st = self.client.state
        return {"pid": os.getpid(), "version": __version__, "port": self.port, "socket": str(self.socket_path),
                "started": self.started_wall, "uptime_s": round(time.monotonic() - self.started, 1),
                "clients": len(self.sessions), "port_released": self.port_released,
                "behaviour": self.behaviour_state, "connected": st.connected, "alive": st.alive,
                "log_file": str(self.logger.path) if self.logger is not None else None,
                "held_goals": sorted(n for n, o in self.goal_owner.items() if o is not None),
                "idle": self.idle_stage}

    async def op_ping(self, s: Session) -> str:
        return "pong"

    async def op_alive(self, s: Session) -> bool:
        return True

    async def op_snapshot(self, s: Session) -> dict[str, Any]:
        return self.client.state.snapshot()

    async def op_wait_ready(self, s: Session, wait_s: float = 3.0, require_telemetry: bool = True) -> Any:
        self._need_port()
        return await self._blocking(self.client.wait_ready, wait_s, require_telemetry=require_telemetry)

    async def op_stats(self, s: Session) -> Any:
        self._need_port()
        return await self._blocking(self.client.stats)

    async def op_goals(self, s: Session) -> list[str]:
        latched = self.link.goals()
        return sorted(n for n in s.goals if n in latched and self.goal_owner.get(n) is s)

    async def op_posture(self, s: Session, name: str, transition_ticks: int = 0,
                         balance: bool | None = None) -> None:
        self._need_port()
        self.client.posture(name, transition_ticks=transition_ticks, balance=balance)
        self._own(s, "SET_POSTURE")

    async def op_set_balance(self, s: Session, enable: bool, **gains: Any) -> dict[str, int]:
        self._need_port()
        f = self.client.set_balance(bool(enable), **gains)
        self._own(s, "SET_BALANCE")
        return f

    async def op_release(self, s: Session) -> None:
        self._end_hold(s)

    async def op_cal_get(self, s: Session) -> list[int]:
        self._need_port()
        return await self._blocking(self.client.cal_get)

    async def op_cal_set(self, s: Session, joint: int, offset_a16: int) -> None:
        self._need_port()
        await self._blocking(self.client.cal_set, joint, offset_a16)

    async def op_cal_save(self, s: Session, wait_saved: float | None = 2.0) -> bool:
        self._need_port()
        return await self._blocking(self.client.cal_save, wait_saved)

    async def op_cal_revert(self, s: Session) -> None:
        self._need_port()
        await self._blocking(self.client.cal_revert)

    async def op_param_get(self, s: Session, id: Any) -> Any:
        self._need_port()
        return await self._blocking(self.client.param_get, id)

    async def op_param_set(self, s: Session, id: Any, value: int, persist: bool = False) -> None:
        self._need_port()
        await self._blocking(self.client.param_set, id, value, persist)

    async def op_param_list(self, s: Session) -> list[Any]:
        self._need_port()
        return await self._blocking(self.client.param_list)

    async def op_pca_read(self, s: Session) -> list[int]:
        self._need_port()
        return await self._blocking(self.client.pca_counts)

    async def op_eeprom_read(self, s: Session, addr: int, len: int) -> str:
        self._need_port()
        return (await self._blocking(self.client.eeprom_read, addr, len)).hex()

    async def op_request(self, s: Session, name: str, fields: dict[str, Any] | None = None,
                         timeout_s: float | None = None) -> Any:
        """The raw SerialLink.request(): a query's data reply, or a command's ACK."""
        self._need_port()
        if name in consts.GOAL_TYPES:
            raise ValueError(f"{name} is a goal: use posture/set_balance")
        return await self._blocking(self.link.request, name, fields or {}, timeout=timeout_s)

    async def op_call(self, s: Session, method: str, args: list[Any] | None = None,
                      kwargs: dict[str, Any] | None = None) -> Any:
        """Any public BodyClient method with JSON-able arguments (generic proxy)."""
        if method.startswith("_") or not callable(getattr(BodyClient, method, None)):
            raise ValueError(f"no BodyClient method {method!r}")
        self._need_port()
        return await self._blocking(getattr(self.client, method), *(args or []), **(kwargs or {}))

    async def op_subscribe(self, s: Session, rate_hz: float = 10.0,
                           kinds: list[str] | None = None) -> dict[str, Any]:
        kinds = list(kinds or ("snapshot", "event"))
        bad = [k for k in kinds if k not in STREAM_KINDS]
        if bad:
            raise ValueError(f"unknown stream kinds {bad} (known: {', '.join(STREAM_KINDS)})")
        rate = float(rate_hz)
        if not 0 < rate <= 50:
            raise ValueError("rate_hz must be in (0, 50]")
        s.kinds = set(kinds)
        s.rate_hz = rate
        if s.stream_task is not None:
            s.stream_task.cancel()
            s.stream_task = None
        if "snapshot" in s.kinds:
            s.stream_task = asyncio.create_task(self._snapshot_stream(s))
        return {"kinds": sorted(s.kinds), "rate_hz": rate}

    async def op_unsubscribe(self, s: Session) -> None:
        s.kinds = set()
        if s.stream_task is not None:
            s.stream_task.cancel()
            s.stream_task = None

    async def op_release_port(self, s: Session, lease_s: float | None = None) -> dict[str, Any]:
        """Stop the link and close the port (for avrdude) until reacquire_port,
        the lease runs out, or this client disconnects."""
        if self.port_released:
            raise LinkError(f"{self.port} is already released")
        lease = float(lease_s if lease_s is not None else self.release_lease_s)
        self.port_released = True
        self.released_by = s
        self.release_until = time.monotonic() + lease
        for n in list(self.goal_owner):
            self.goal_owner.pop(n, None)
        for sess in self.sessions.values():
            sess.goals.clear()
        for n in list(self.link.goals()):
            self.link.clear_goal(n)
        self.idle_since = self.idle_stage = None
        await self._blocking(self.link.stop)
        log.info("port %s released for %.0f s (client %d)", self.port, lease, s.id)
        return {"port": self.port, "lease_s": lease}

    async def op_reacquire_port(self, s: Session) -> dict[str, Any]:
        if not self.port_released:
            return {"port": self.port, "reacquired": False}
        await self._reacquire()
        return {"port": self.port, "reacquired": True}

    async def _reacquire(self) -> None:
        if not self.port_released:
            return
        self.port_released = False
        self.released_by = None
        self.release_until = None
        self.link.mark_alive()
        self.link.start()
        log.info("port %s reacquired", self.port)

    async def op_verify_hello(self, s: Session, since: float, wait_s: float = 8.0,
                              expected_build: int | None = None, any_cause: bool = False) -> dict[str, Any]:
        """``kilo flash`` verification on the daemon's link (``since`` is
        time.monotonic(), which is system-wide on Linux)."""
        from kilo.tools.flash import verify_hello

        self._need_port()
        r = await self._blocking(verify_hello, self.link, since, expected_build, timeout=wait_s,
                                 any_cause=any_cause)
        return {"ok": r.ok, "hello": r.hello}

    async def op_shutdown(self, s: Session) -> str:
        self.loop.call_later(0.05, self.stopping.set)
        return "stopping"


def run_daemon(args: Any) -> int:
    """``kilo run``: argparse namespace from kilo.cli."""
    from kilo.link.ports import detect_port

    choice = detect_port(args.port)
    for w in choice.warnings:
        log.warning("%s", w)
    if choice.port is None:
        log.error("no serial port found; set KILO_PORT (~/.config/kilo/env) or --port")
        return 1
    log_dir = Path(args.log_dir).expanduser() if getattr(args, "log_dir", None) else Path.home() / "kilo-logs"
    logger = None
    if args.log or args.log_file:
        from kilo.log import JsonlLogger

        logger = JsonlLogger(args.log_file or log_dir / time.strftime("kilo-%Y%m%d-%H%M%S.jsonl"))
        log.info("frame log %s", logger.path)
    d = Daemon(choice.port, baud=args.baud, socket=args.socket, logger=logger,
               idle_posture=args.idle_posture, idle_limp_s=args.idle_limp, behaviour=args.behaviour,
               log_dir=log_dir)
    log.info("port %s (%s)", choice.port, choice.reason)
    return d.run()
