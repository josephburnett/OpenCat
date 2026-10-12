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
  alive), so a stalled daemon trips the body's watchdog. With ``--behaviour``
  it stamps only while the runner has ticked within ``runner_stall_s`` (1 s;
  10 s of grace for its first tick), so a hung runner trips the watchdog too
  (logged once). Pool threads never stamp (``BodyClient.stamp_liveness`` is
  off). The same task checks hold leases, the idle timeline, the
  port-release lease and, hourly, the log directory's size (``prune_logs``:
  oldest kilo logs beyond 500 MB go).
- Ops run one at a time per client, in order, except ``trick_play``, which
  runs as a task of its session so the session still reads (pings, a
  ``trick_stop``, its own disconnect) while the trick plays.

Holds. Goals a client sets (``posture``, ``set_balance``, ``walk``, ``pose``) belong to that
client's session and are kept alive by the daemon while the session lives and
keeps its lease (any message within ``hold_lease_s``; RemoteBody pings while
it holds). When the session closes or its lease lapses, the hold ends like
``kilo posture --for`` ends without a daemon: the goal is dropped, the daemon
latches REST at once and LIMP after ``idle_limp_s`` (the watchdog's REST/LIMP
timeline, without the 0.5 s stop stage), and a balance enable is switched
off. With a behaviour runner active the daemon only drops the goal and leaves
the body to the runner. The idle REST is latched whenever a hold that owned
the posture (or a trick) ends and nobody else owns one, even if another
client still holds SET_BALANCE.

Tricks. ``trick_upload {name | trick}`` (a library name or ``Trick.to_dict()``),
``trick_play {name | trick, mirror, speed, repeats, posture, hold_s}`` and
``trick_stop {transition_ticks}``. ``trick_play`` uploads (cached), takes
the optional posture first (held by the session), sends TRICK_PLAY and
replies on EVENT TRICK_DONE (``{reason, trick_id, elapsed_s, expected_ticks,
epoch, stopped_by_us}``); ``hold_s`` stops it after that long. The session
owns the trick (``TRICK`` in ``held_goals``) and, like a posture, the body's
end posture: when the session closes (or releases, or its lease lapses) a
playing trick is stopped and the idle REST/LIMP timeline runs. A second
client cannot start a trick over a playing one (``trick_stop`` first).

Behaviour runner and CLI holds. While the runner is active, ``posture``,
``set_balance``, ``walk``, ``pose``, ``trick_upload`` and ``trick_play`` from clients are refused
("behaviour runner active ...") unless the request carries ``override``
(``kilo --override ...``): the runner is then paused (its behaviours halted,
nothing ticked) until that client's hold ends, and resumes re-asserting its
own goals. ``trick_stop``, ``stop_walk`` and ``release`` are always allowed. The generic
``call`` op refuses the goal and trick methods.

Port hot-plug. The daemon starts whether or not the port exists (robot
unplugged, adapter not enumerated yet) and the link's reader keeps trying to
open it, backing off from 2 s to 10 s (``DAEMON_LINK``); after a USB replug
the read error closes the port and the same retry loop reopens it. Without
``--port``/``$KILO_PORT`` the port is auto-detected again on every attempt.
``kilo daemon`` shows connected/alive, whether the device node exists, the
last open error and the reconnect count; the ``connection`` stream carries
``{connected, alive, port, error}`` on every change.

Percepts. The daemon feeds a ``kilo.perception.SharedPercepts`` on every
TELEMETRY: the ``percepts`` op returns its flags and recent events, the
``percepts`` stream kind pushes each batch of percept events, and the
behaviour runner gets the same object (``run_behaviour(..., percepts=)``
when its signature takes it), so its ``notify_self_motion()`` calls quiet
the IMU detectors for everyone.

Logs. ``--kilolog [FILE]`` hands a ``KilologWriter`` to the link (raw wire
frames both ways; its own thread writes them, flushed every 5 s);
``run_daemon`` also writes the daemon's own log to ``<log-dir>/daemon.log``
(1 MB x 3, rotating; through a QueueHandler, so a slow disk never stalls the
link's reader thread that logs), because the user journal is not always
readable or persistent on the Pi. ``--log`` (the JSON-lines frame log) is
off unless asked for; the log directory is pruned to 500 MB at start and
hourly (oldest kilo logs first, never an open one).

Port release (Appendix F). ``release_port {lease_s}`` stops the link and
closes the port so ``kilo flash`` can run avrdude; ``reacquire_port`` or the
lease running out reopens it. The lease is honoured even when the releasing
client goes away (a CLI killed mid-flash leaves avrdude running): the port
stays released until the lease ends. Every reopen first settles the port
(``settle_port``: DTR/HUPCL/baud, so avrdude's exit state does not reset the
body on the reopen). While released, ops that need the body fail with
NotConnected.

Shutdown (SIGTERM/SIGINT or the ``shutdown`` op): stop accepting, stop the
behaviour runner, close clients, drop every goal and go silent (the body's
watchdog takes over: stop, REST, LIMP), close the port, remove the socket.

Behaviour runner (``--behaviour``): if ``kilo.behaviour.runner`` imports, the
daemon calls ``run_behaviour(client, state, stop_event, log_dir)`` in its own
thread (an ``async def`` is awaited on the loop instead): ``client`` is the
daemon's BodyClient, ``state`` its BodyState, ``stop_event`` a
threading.Event set at shutdown (return promptly once it is set), ``log_dir``
a pathlib.Path for the runner's own logs; a ``paused`` keyword, if it takes
one, gets the ``--override`` Event. The runner stamps the link's liveness
each tick (the pump follows it, above). An import failure or an exception
is logged and the daemon carries on without it.

IR remote (``--remote``, off by default; the owner's request 2026-10-11).
The body relays the Petoi remote as EVENT IR_KEY; the daemon maps keys to
actions on its own BodyClient (``kilo.ir.ACTIONS``: postures, tricks, the
CH+ balance toggle, walking in a direction and the gait pattern keys;
held-key repeats, EQ, 9 and unknown keys are ignored). The actions run in
the thread pool, one at a time (``kilo.ir.RemoteControl``). A posture, trick
or walk set by the remote is held like the idle latch: nobody owns it and
the idle REST/LIMP timeline is stopped, so it stays until the next key or a
client goal. Keys are logged and ignored while the behaviour runner is
active, while the port is released, and while a client holds the posture, a
trick or the gait (the balance key: while a client holds SET_BALANCE).

Balance by default (``--balance [KP]``, off by default; the owner's request
2026-10-11: Kilo wakes up balancing). The daemon latches SET_BALANCE enable
with the default gains (Kp as given, default kilo.body.client.BALANCE_KP)
at start and after a port release, owned by nobody like the idle latch;
SET_BALANCE is not an epoch goal, so the link keeps refreshing it across a
body reboot. A client's balance hold that ends goes back to the default
(on) instead of off; the remote's CH+ key toggles it off until the next
CH+ or the next client hold that ends. ``daemon`` reports ``balance_default``
(the Kp, or None).

Gait (P2). ``walk {stride_mm, turn_mm, pattern, period_ms, duty, step_h_mm,
body_h_mm, bob, wag, balance, override}`` (BodyClient.walk; the session owns
SET_GAIT like a posture, and the posture goal is dropped: the gait replaces
it), ``stop_walk {wait_s}`` (any client, also under the behaviour runner;
with ``wait_s`` it replies once GAIT_STOPPED came: ``{stopped,
elapsed_s}``), ``gait`` (BodyClient.gait_state()), ``pose {pitch_deg,
roll_deg, dheight_mm, xshift_mm, override}`` and ``release_pose``. When a
session holding a walk ends, the daemon latches the stop (never just drops
a walk) and runs the idle timeline; a pose hold ending latches the neutral
pose. ``daemon`` reports ``gait`` (pattern, stride, turn, phase, mode).
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import inspect
import logging
import logging.handlers
import os
import signal
import threading
import time
from pathlib import Path
from typing import Any, Callable

from kilo import __version__
from kilo.body.client import TRICK_DONE_MARGIN_S, BodyClient, TrickError
from kilo.link import consts
from kilo.link.serial_link import LinkConfig, LinkError, NotConnected, SerialLink
from kilo.link.transport import DEFAULT_BAUD, open_port, settle_port
from kilo.log import LOG_DIR_MAX_BYTES, prune_logs
from kilo.perception.shared import SharedPercepts
from kilo.remote import dumps, encode_error, jsonable, listen_socket, socket_candidates

log = logging.getLogger("kilo.daemon")

STREAM_BUFFER_MAX = 256 << 10   # drop stream messages for a client beyond this backlog
REPLY_BUFFER_MAX = 4 << 20      # disconnect a client that does not read its replies
LINE_LIMIT = 1 << 20
PUMP_PERIOD_S = 0.2
STREAM_KINDS = ("snapshot", "telemetry", "event", "hello", "reboot", "goals_cleared", "connection", "percepts")
WIRE_FLUSH_S = 5.0
# The daemon's link: wait for a missing port patiently (2 s, backing off to 10 s).
DAEMON_LINK = dict(reconnect_min_s=2.0, reconnect_max_s=10.0)
DAEMON_LOG_BYTES = 1 << 20
DAEMON_LOG_BACKUPS = 3
IDLE_POSTURES = ("rest", "limp", "none")
RUNNER_STALL_S = 1.0          # the pump stops stamping when the runner has not ticked for this long
RUNNER_START_GRACE_S = 10.0   # ... or has not ticked at all this long after it started
PRUNE_EVERY_S = 3600.0
CONCURRENT_OPS = frozenset({"trick_play"})  # run as session tasks, not in line
TRICK_POSTURES = ("rest", "sit", "stand")
# BodyClient methods the generic `call` op refuses: goals and tricks have
# their own ops (session ownership, the runner check, the daemon loop).
CALL_REFUSED = frozenset({"posture", "limp", "rest", "stand", "sit", "balance", "set_balance", "release",
                          "upload_trick", "start_trick", "play", "stop_trick", "spin", "wait_ready",
                          "walk", "stop_walk", "wait_gait_stopped", "pose", "release_pose"})
# Holds that decide what the legs do: when the last one ends the idle timeline runs.
BODY_HOLDS = ("SET_POSTURE", "TRICK", "SET_GAIT")
BEHAVIOUR_ACTIVE = ("behaviour runner active; stop it with `kilo daemon stop` (or `sudo systemctl stop kilo`) "
                    "and run `kilo run` without --behaviour, or pass --override (`kilo --override ...`) to "
                    "pause it while this command holds the body")


class BehaviourActive(LinkError):
    pass


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
        self.tasks: set[asyncio.Task] = set()  # CONCURRENT_OPS in flight
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
                 log_dir: Path | None = None, release_lease_s: float = 120.0, workers: int = 8,
                 wire_logger: Any = None, auto_port: bool = False, log_cap_bytes: int = LOG_DIR_MAX_BYTES,
                 keep_logs: Any = (), remote: bool = False, balance_kp: float | None = None):
        if idle_posture not in IDLE_POSTURES:
            raise ValueError(f"idle posture must be one of {IDLE_POSTURES}")
        self.port = port
        self.auto_port = auto_port or port is None
        self.baud = baud
        self.socket_path, self.socket_why = listen_socket(socket)
        self._socket_explicit = bool(socket) or bool(os.environ.get("KILO_SOCKET"))
        self.logger = logger
        self.wire_logger = wire_logger
        self.factory = transport_factory or self._open_port
        self.link_config = link_config or LinkConfig(**DAEMON_LINK)
        self.percepts = SharedPercepts()
        self._loop_thread: int | None = None
        self._was_connected = False
        self._last_wire_flush = time.monotonic()
        self.idle_posture = idle_posture
        self.idle_limp_s = idle_limp_s
        self.hold_lease_s = hold_lease_s
        self.behaviour = behaviour
        self.remote = remote
        self._remote_lock = threading.Lock()  # one remote action at a time
        self.remote_last: str | None = None   # the last remote key acted on (op_daemon)
        self.balance_kp = balance_kp          # --balance: SET_BALANCE on by default with this Kp (None = off)
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
        self._reacquiring = False
        # behaviour runner liveness and pause (--override)
        self.runner_stall_s = RUNNER_STALL_S
        self._behaviour_started_at: float | None = None
        self._runner_stamping = True
        self.behaviour_paused = threading.Event()
        self.pausers: set[int] = set()
        # the trick being played through trick_play: (session, trick_id)
        self._trick_playing: tuple[Session, int] | None = None
        # log directory size cap
        self.log_cap_bytes = log_cap_bytes
        self.keep_logs = {Path(p) for p in keep_logs}
        self._last_prune = -1e9

    # ---- lifecycle ----------------------------------------------------------------------
    def run(self, install_signals: bool = True) -> int:
        return asyncio.run(self.serve(install_signals))

    def request_stop(self) -> None:
        if self.loop is not None and self.stopping is not None:
            try:
                self.loop.call_soon_threadsafe(self.stopping.set)
            except RuntimeError:
                pass  # the loop already finished

    def _open_port(self) -> Any:
        """The link's transport factory: (re)detect the port when it was not
        given, and fail fast (OSError) while the device node is missing."""
        port = self.port
        if self.auto_port:
            from kilo.link.ports import detect_port

            port = detect_port(None).port
            if port is None:
                raise OSError("no serial port found (plug the body in, or set KILO_PORT)")
        if not os.path.exists(port):
            raise FileNotFoundError(f"{port} does not exist (unplugged?)")
        t = open_port(port, self.baud)
        if port != self.port:
            log.info("port %s", port)
            self.port = port
            if self.link is not None:
                self.link.state.port = port
        return t

    async def serve(self, install_signals: bool = True) -> int:
        self.loop = asyncio.get_running_loop()
        self._loop_thread = threading.get_ident()
        self.stopping = asyncio.Event()
        await self._prepare_socket()
        self.link = SerialLink(self.factory, logger=self.logger, port_name=self.port,
                               config=self.link_config, dispatch=self.loop.call_soon_threadsafe,
                               wire_logger=self.wire_logger)
        self.client = BodyClient(self.link)
        self.client.stamp_liveness = False  # pool threads must not stamp (the pump and the runner do)
        for kind in ("telemetry", "event", "hello", "reboot", "goals_cleared", "connection"):
            self.link.subscribe(kind, self._make_fanout(kind))
        self.link.subscribe("telemetry", self._feed_percepts)
        self.link.subscribe("connection", self._log_connection)
        if self.remote:
            from kilo.ir import RemoteControl

            self._remote_control = RemoteControl(self.client, balance_kp=self.balance_kp)
            self.link.subscribe("event", self._on_ir_key)
        self.percepts.add_listener(self._percept_events)
        self.link.mark_alive("daemon")
        self.link.start()
        self._latch_balance_default()
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
        log.info("kilo %s daemon: port %s, socket %s (%s)", __version__, self.port or "auto-detect",
                 self.socket_path, self.socket_why)
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
        if not self._socket_explicit:
            # One daemon per body: refuse to start beside one the clients would find
            # first (the system service on /run/kilo, or a user-service daemon).
            from kilo.remote import connect

            for other, _why in socket_candidates():
                if other == p or not other.exists():
                    continue
                conn = await asyncio.get_running_loop().run_in_executor(None, connect, other)
                if conn is not None:
                    pid = conn.info.get("pid")
                    conn.close()
                    raise SystemExit(f"kilo run: a daemon (pid {pid}) already listens on {other}")
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
        if self.wire_logger is not None:
            await self.loop.run_in_executor(self.pool, self.wire_logger.close)
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
        kwargs: dict[str, Any] = {}
        try:
            params = inspect.signature(run_behaviour).parameters
            if "percepts" in params:
                kwargs["percepts"] = self.percepts
            if "paused" in params:
                kwargs["paused"] = self.behaviour_paused
        except (TypeError, ValueError):
            pass
        self.behaviour_state = "running"
        self._behaviour_started_at = time.monotonic()
        if inspect.iscoroutinefunction(run_behaviour):
            async def runner() -> None:
                try:
                    await run_behaviour(*args, **kwargs)
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
                    run_behaviour(*args, **kwargs)
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
                self._stamp_liveness()
                self._check_leases()
                now = time.monotonic()
                if self.wire_logger is not None and now - self._last_wire_flush >= WIRE_FLUSH_S:
                    self._last_wire_flush = now
                    self.wire_logger.flush()  # a request to its writer thread: does not block
                if now - self._last_prune >= PRUNE_EVERY_S:
                    self._last_prune = now
                    self.loop.run_in_executor(self.pool, self.prune_logs)
            except Exception:
                log.exception("pump")
            await asyncio.sleep(PUMP_PERIOD_S)

    def _stamp_liveness(self) -> None:
        """Stamp the link's liveness for the daemon loop; with a behaviour
        runner, only while the runner ticks too (it stamps as "mind" on
        every tick), so a hung runner trips the body's watchdog."""
        link = self.link
        if self.behaviour_active and self._behaviour_started_at is not None:
            last = link.alive_stamps.get("mind")
            if last is None:
                ok = time.monotonic() - self._behaviour_started_at < RUNNER_START_GRACE_S
                idle = time.monotonic() - self._behaviour_started_at
            else:
                idle = link.clock() - last
                ok = idle < self.runner_stall_s
            if not ok:
                if self._runner_stamping:
                    self._runner_stamping = False
                    log.warning("behaviour runner has not ticked for %.1f s: no longer stamping the link's "
                                "liveness (the body's watchdog takes over)", idle)
                return
            if not self._runner_stamping:
                self._runner_stamping = True
                log.warning("behaviour runner ticking again: stamping liveness")
        link.mark_alive("daemon")

    def prune_logs(self) -> list[Path]:
        """Keep the log directory under ``log_cap_bytes`` (pool thread)."""
        keep = set(self.keep_logs)
        for lg in (self.logger, self.wire_logger):
            if lg is not None and getattr(lg, "path", None) is not None:
                keep.add(Path(lg.path))
        try:
            gone = prune_logs(self.log_dir, self.log_cap_bytes, keep)
        except Exception:
            log.exception("pruning %s", self.log_dir)
            return []
        if gone:
            log.warning("log directory %s over %d MB: deleted %d oldest files (%s)", self.log_dir,
                        self.log_cap_bytes >> 20, len(gone), ", ".join(p.name for p in gone[:5])
                        + (" ..." if len(gone) > 5 else ""))
        return gone

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
                and now - self.idle_since >= self.idle_limp_s and not self._posture_held()):
            self.idle_stage = "limp"
            self._latch_idle("limp")

    def _held(self) -> bool:
        return any(o is not None for o in self.goal_owner.values())

    def _posture_held(self) -> bool:
        """Somebody owns what the body's legs do: SET_POSTURE, a trick or a gait."""
        return any(self.goal_owner.get(n) is not None for n in BODY_HOLDS)

    def _latch_idle(self, name: str) -> None:
        try:
            self.client.posture(name)
            self.goal_owner["SET_POSTURE"] = None
        except (LinkError, ValueError) as e:
            log.warning("idle %s: %s", name, e)

    def _latch_balance_default(self) -> None:
        """``--balance``: SET_BALANCE on with the default gains, owned by nobody."""
        if self.balance_kp is None:
            return
        try:
            f = self.client.set_balance(True, kp=self.balance_kp)
            self.goal_owner["SET_BALANCE"] = None
            log.info("balance on by default (Kp %.2f)", f["kp_q8"] / 256)
        except (LinkError, ValueError) as e:
            log.warning("balance default: %s", e)

    def _end_hold(self, s: Session) -> None:
        names = [n for n in s.goals if self.goal_owner.get(n) is s]
        s.goals.clear()
        self._resume_runner_for(s)
        if not names:
            return
        for n in names:
            self.goal_owner.pop(n, None)
        log.info("client %d: hold ended (%s)", s.id, ", ".join(sorted(names)))
        link = self.link
        if "TRICK" in names:
            names.remove("TRICK")
            playing = self._trick_playing
            if playing is not None and playing[0] is s and not self._shutting_down and not self.port_released:
                self.pool.submit(self._stop_trick_quietly)  # its TRICK_DONE ends the trick_play op
            posture_hold_ended = True
        else:
            posture_hold_ended = "SET_POSTURE" in names or "SET_GAIT" in names
        if self._shutting_down:
            return  # go_silent() drops everything next
        if "SET_GAIT" in names and not self.port_released:
            # A walk is never just dropped: latch the stop (the body ramps
            # down, hands over to STANCE, GAIT_STOPPED clears the goal).
            names.remove("SET_GAIT")
            g = link.goals().get("SET_GAIT")
            if g is not None and g.fields.get("run"):
                try:
                    self.client.stop_walk()
                except (LinkError, ValueError) as e:
                    log.warning("client %d: gait stop: %s", s.id, e)
        if "SET_BODY_POSE" in names and not self.port_released:
            names.remove("SET_BODY_POSE")
            self.client.release_pose()
        if self.behaviour_active or self.port_released:
            for n in names:
                link.clear_goal(n)
            return
        if "SET_BALANCE" in names:
            if self.balance_kp is not None:
                self._latch_balance_default()  # --balance: back to on with the default gains
            else:
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
        for n in names:
            if n not in ("SET_POSTURE", "SET_BALANCE"):
                link.clear_goal(n)
        # Whoever else still holds SET_BALANCE (or anything but the posture),
        # a posture nobody owns any more falls back to the idle timeline:
        # otherwise the body would keep the last posture forever (the daemon
        # feeds its watchdog).
        if posture_hold_ended and self.idle_posture != "none" and not self._posture_held():
            self.idle_since = time.monotonic()
            self.idle_stage = self.idle_posture
            self._latch_idle(self.idle_posture)

    def _stop_trick_quietly(self) -> None:
        try:
            self.client.stop_trick()
        except LinkError as e:
            log.warning("trick stop: %s", e)

    # ---- the behaviour runner vs CLI holds (--override) ----------------------------------
    def _check_runner(self, s: Session, override: bool) -> None:
        """Refuse a client goal while the behaviour runner drives the body,
        or (``override``) pause the runner until this client's hold ends."""
        if not self.behaviour_active:
            return
        if not override:
            raise BehaviourActive(BEHAVIOUR_ACTIVE)
        if s.id not in self.pausers:
            self.pausers.add(s.id)
            if not self.behaviour_paused.is_set():
                log.info("client %d: --override, behaviour runner paused", s.id)
            self.behaviour_paused.set()

    def _resume_runner_for(self, s: Session) -> None:
        if s.id in self.pausers:
            self.pausers.discard(s.id)
            if not self.pausers:
                self.behaviour_paused.clear()
                log.info("client %d: hold ended, behaviour runner resumed", s.id)

    def _own(self, s: Session, name: str) -> None:
        prev = self.goal_owner.get(name)
        if prev is not None and prev is not s:
            prev.goals.discard(name)
        self.goal_owner[name] = s
        s.goals.add(name)
        if name in BODY_HOLDS:  # a balance-, head- or pose-only hold leaves the idle timeline running
            self.idle_since = None
            self.idle_stage = None

    # ---- the IR remote (--remote) -----------------------------------------------------------
    def _on_ir_key(self, msg: Any) -> None:
        """Link event callback (on the loop): EVENT IR_KEY -> a remote action."""
        if msg.code != consts.EV["IR_KEY"]:
            return
        from kilo import ir

        text = ir.describe(msg.arg)
        action = ir.action_for(msg.arg)
        if action.kind == "ignore":
            log.debug("remote %s: ignored", text)
            return
        if self.behaviour_active:
            log.info("remote %s: ignored (behaviour runner active)", text)
            return
        if self.port_released or self._shutting_down:
            return
        owner = (self.goal_owner.get("SET_BALANCE") if action.kind == "balance_toggle"
                 else next((o for o in map(self.goal_owner.get, BODY_HOLDS) if o is not None), None))
        if owner is not None:
            log.info("remote %s: ignored (client %d holds the body)", text, owner.id)
            return
        if action.kind in ("posture", "trick", "walk"):
            # held like the idle latch: no owner, and no idle timeline over it
            self.goal_owner["SET_GAIT" if action.kind == "walk" else "SET_POSTURE"] = None
            self.idle_since = self.idle_stage = None
        self.remote_last = text
        self.pool.submit(self._remote_perform, action, text)

    def _remote_perform(self, action: Any, text: str) -> None:
        with self._remote_lock:
            try:
                did = self._remote_control.perform(action)
                log.info("remote %s: %s", text, did)
            except Exception as e:  # LinkError, TrickError, a missing trick file: log, keep serving
                log.warning("remote %s: %s failed: %s", text, action.kind, e)

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
                data = {"connected": args[0], "alive": args[1], "port": self.port,
                        "error": self.link.last_open_error if self.link is not None else None}
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

    # ---- percepts and connection notes (on the loop) ---------------------------------------
    def _feed_percepts(self, msg: Any) -> None:
        self.percepts.update(self.client.state)

    def _percept_events(self, events: list[Any], d: dict[str, Any]) -> None:
        """SharedPercepts listener: any thread (the loop, or the behaviour runner)."""
        data = {"events": [e.as_dict() for e in events],
                "flags": {k: bool(d.get(k)) for k in ("near", "approaching", "clear", "tilted", "picked_up",
                                                      "tap", "pet", "startle")}}
        if threading.get_ident() == self._loop_thread or self.loop is None:
            self._fan_percepts(data)
        else:
            try:
                self.loop.call_soon_threadsafe(self._fan_percepts, data)
            except RuntimeError:
                pass

    def _fan_percepts(self, data: dict[str, Any]) -> None:
        for s in list(self.sessions.values()):
            if "percepts" in s.kinds:
                self._stream(s, "percepts", data)

    def _log_connection(self, connected: bool, alive: bool) -> None:
        was, self._was_connected = self._was_connected, connected
        if was and not connected and not self._shutting_down and not self.port_released:
            log.warning("port %s closed; reopening (every %.0f-%.0f s until it is back)", self.port,
                        self.link.cfg.reconnect_min_s, self.link.cfg.reconnect_max_s)

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
        for t in list(s.tasks):
            t.cancel()
        if self.released_by is s and self.port_released:
            left = (self.release_until or time.monotonic()) - time.monotonic()
            log.warning("client %d released %s and went away: keeping it released for the rest of the "
                        "lease (%.0f s; avrdude may still be writing), or until `reacquire_port`",
                        s.id, self.port, max(0.0, left))
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
        if op in CONCURRENT_OPS:
            s.inflight += 1  # counted now: the lease must not lapse before the task starts
            t = asyncio.ensure_future(self._run_op(s, rid, op, handler, args, counted=True))
            s.tasks.add(t)
            t.add_done_callback(s.tasks.discard)
            return
        await self._run_op(s, rid, op, handler, args)

    async def _run_op(self, s: Session, rid: Any, op: str, handler: Callable[..., Any], args: dict[str, Any],
                      counted: bool = False) -> None:
        if not counted:
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
        link = self.link
        now = time.monotonic()
        return {"pid": os.getpid(), "version": __version__, "port": self.port, "socket": str(self.socket_path),
                "socket_why": self.socket_why, "auto_port": self.auto_port,
                "started": self.started_wall, "uptime_s": round(now - self.started, 1),
                "clients": len(self.sessions), "port_released": self.port_released,
                "behaviour": self.behaviour_state, "remote": ("off" if not self.remote else
                                                             f"on, last key {self.remote_last or '-'}"),
                "balance_default": self.balance_kp,
                "connected": st.connected, "alive": st.alive,
                "port_present": bool(self.port) and os.path.exists(self.port),
                "connected_for_s": round(now - st.connected_since, 1)
                if st.connected and st.connected_since is not None else None,
                "open_error": link.last_open_error, "open_fail_streak": link.open_fail_streak,
                "retry_in_s": link.retry_in_s if not st.connected else None,
                "open_failures": st.stats.open_failures, "reconnects": st.stats.reconnects,
                "disconnects": st.stats.disconnects,
                "log_file": str(self.logger.path) if self.logger is not None else None,
                "kilolog": str(self.wire_logger.path) if self.wire_logger is not None else None,
                "percepts": self.percepts.flags(),
                "held_goals": sorted(n for n, o in self.goal_owner.items() if o is not None),
                "idle": self.idle_stage, "gait": self.client.gait_state()}

    async def op_percepts(self, s: Session) -> dict[str, Any]:
        """The daemon's percepts: flags, distance/approach, recent events."""
        return self.percepts.state()

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
                         balance: bool | None = None, override: bool = False) -> None:
        self._need_port()
        self._check_runner(s, override)
        self.client.posture(name, transition_ticks=transition_ticks, balance=balance)
        self._own(s, "SET_POSTURE")

    async def op_set_balance(self, s: Session, enable: bool, override: bool = False, **gains: Any) -> dict[str, int]:
        self._need_port()
        self._check_runner(s, override)
        f = self.client.set_balance(bool(enable), **gains)
        self._own(s, "SET_BALANCE")
        return f

    async def op_release(self, s: Session) -> None:
        self._end_hold(s)

    # ---- gait and body pose (P2) ---------------------------------------------------------
    async def op_walk(self, s: Session, override: bool = False, **gait: Any) -> dict[str, int]:
        """BodyClient.walk(**gait), held by this session like a posture: when
        the session ends the daemon latches the stop and runs the idle
        timeline. Replaces a posture goal (the gait hands over to STANCE)."""
        self._need_port()
        self.client.gait_fields(**gait)  # range check before anything is sent or owned
        self._check_runner(s, override)
        if self.client._geometry_reboots != self.client.state.reboots:
            await self._blocking(self.client.load_geometry)
        f = self.client.walk(**gait)
        prev = self.goal_owner.pop("SET_POSTURE", None)  # walk() dropped the posture goal
        if prev is not None:
            prev.goals.discard("SET_POSTURE")
        self._own(s, "SET_GAIT")
        return f

    async def op_stop_walk(self, s: Session, wait_s: float | None = None) -> dict[str, Any]:
        """Latch the gait stop (any client, also while the behaviour runner is
        active); with ``wait_s``, reply once GAIT_STOPPED came (or the body
        shows no gait), at most ``wait_s`` later: ``{stopped, elapsed_s}``."""
        self._need_port()
        n = self.client.stop_walk()
        self._own(s, "SET_GAIT")
        if wait_s is None:
            return {"stopped": None, "elapsed_s": 0.0}
        return await self._wait_gait_stopped(n, float(wait_s))

    async def _wait_gait_stopped(self, n: int, wait_s: float) -> dict[str, Any]:
        clock = self.link.clock
        t0 = clock()
        quiet_since = None
        while True:
            if self.client.gait_stopped_since(n) is not None:
                return {"stopped": True, "elapsed_s": round(clock() - t0, 3)}
            now = clock()
            if self.client.walking():
                quiet_since = None
            elif quiet_since is None:
                quiet_since = now
            elif now - quiet_since >= 0.3:
                return {"stopped": True, "elapsed_s": round(now - t0, 3)}
            if now - t0 >= wait_s:
                return {"stopped": False, "elapsed_s": round(now - t0, 3)}
            await asyncio.sleep(0.02)

    async def op_gait(self, s: Session) -> dict[str, Any] | None:
        """BodyClient.gait_state(): the latched gait and the body's phase."""
        return self.client.gait_state()

    async def op_pose(self, s: Session, pitch_deg: float = 0, roll_deg: float = 0, dheight_mm: float = 0,
                      xshift_mm: float = 0, override: bool = False) -> dict[str, int]:
        self._need_port()
        self._check_runner(s, override)
        f = self.client.pose(pitch_deg, roll_deg, dheight_mm, xshift_mm)
        self._own(s, "SET_BODY_POSE")
        return f

    async def op_release_pose(self, s: Session) -> None:
        self._need_port()
        self.client.release_pose()
        if self.goal_owner.get("SET_BODY_POSE") is s:
            self.goal_owner.pop("SET_BODY_POSE", None)
            s.goals.discard("SET_BODY_POSE")

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
        if method in CALL_REFUSED or inspect.iscoroutinefunction(getattr(BodyClient, method)):
            raise ValueError(f"{method}: not through `call`; use the posture, set_balance, release and "
                             f"trick_upload/trick_play/trick_stop ops")
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
            for t in list(sess.tasks):
                t.cancel()  # a trick_play waiting on a link about to stop
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
        if not self.port_released or self._reacquiring:
            return
        self._reacquiring = True
        try:
            # avrdude leaves DTR low, HUPCL set and 115200: settle first so the
            # reopen does not reset the body (Appendix F step 5).
            await self._blocking(self._settle)
        finally:
            self._reacquiring = False
        self.port_released = False
        self.released_by = None
        self.release_until = None
        self.link.mark_alive("daemon")
        self.link.start()
        self._latch_balance_default()
        log.info("port %s reacquired", self.port)

    def _settle(self) -> None:
        if not self.port:
            return
        try:
            cycles = settle_port(self.port, self.baud) or []
        except Exception as e:  # settle_port records its own errors; anything else must not block the reopen
            log.warning("settling %s: %s", self.port, e)
            return
        errs = [c for c in cycles if getattr(c, "error", None)]
        if errs:
            log.warning("settling %s: %d of %d opens failed (%s)", self.port, len(errs), len(cycles), errs[0].error)

    # ---- tricks ------------------------------------------------------------------------
    @staticmethod
    def _load_trick(name: str | None, trick: dict[str, Any] | None, repeats: int | None = None) -> Any:
        from dataclasses import replace

        from kilo.tricks import library
        from kilo.tricks.model import Trick

        if trick is not None:
            t = Trick.from_dict(trick)
        elif name:
            t = library.load(name)
        else:
            raise ValueError("trick: give a library name or a trick")
        if repeats is not None:
            if not 0 <= int(repeats) <= 255:
                raise ValueError("repeats must be 0..255")
            t = replace(t, repeats=int(repeats))
        return t

    async def op_trick_upload(self, s: Session, name: str | None = None, trick: dict[str, Any] | None = None,
                              force: bool = False, override: bool = False) -> dict[str, Any]:
        """Upload a library trick (``name``) or ``trick`` (Trick.to_dict()); cached."""
        self._need_port()
        t = self._load_trick(name, trick)
        self._check_runner(s, override)
        tid, uploaded = await self._blocking(self.client.upload_trick, t, force=bool(force))
        return {"name": t.name, "trick_id": tid, "uploaded": uploaded, "crc16": t.crc16(), "frames": len(t.frames)}

    async def op_trick_play(self, s: Session, name: str | None = None, trick: dict[str, Any] | None = None,
                            mirror: bool = False, speed: float = 1.0, repeats: int | None = None,
                            posture: str | None = None, hold_s: float | None = None,
                            timeout_s: float | None = None, override: bool = False) -> dict[str, Any]:
        """Play a trick and reply on its TRICK_DONE (see the module docstring)."""
        from kilo.protocol import kilo_protocol as P

        self._need_port()
        t = self._load_trick(name, trick, repeats)
        BodyClient.speed_q6(speed)  # range check before anything moves
        if posture is not None and posture not in TRICK_POSTURES:
            raise ValueError(f"posture must be one of {', '.join(TRICK_POSTURES)}")
        self._check_runner(s, override)
        playing = self._trick_playing
        if playing is not None and playing[0] is not s:
            raise LinkError(f"client {playing[0].id} is playing a trick (`kilo trick stop` first)")
        if posture:
            self.client.posture(posture)
            self._own(s, "SET_POSTURE")
            await asyncio.sleep(1.0)  # the default posture move is 0.5 s
        elif self.client.state.mode == "LIMP":
            raise TrickError("the body is LIMP and a trick cannot start from LIMP; add --posture stand")
        self._own(s, "TRICK")
        ticks = BodyClient.expected_ticks(t, speed)
        if timeout_s is None and ticks is not None:
            timeout_s = (ticks + 1) * P.TICK_MS / 1000.0 + TRICK_DONE_MARGIN_S
        tid, n = await self._blocking(self.client.start_trick, t, bool(mirror), float(speed))
        self._trick_playing = (s, tid)
        clock = self.link.clock
        t0 = clock()
        stopped_by_us = False
        try:
            while True:
                m = self.client.trick_done_since(n, tid)
                if m is not None:
                    return {"reason": P.TrickDoneReason((m.arg & 0xFFFF) >> 8).name, "trick_id": tid,
                            "elapsed_s": clock() - t0, "expected_ticks": ticks, "epoch": m.epoch,
                            "stopped_by_us": stopped_by_us, "name": t.name}
                elapsed = clock() - t0
                if hold_s is not None and not stopped_by_us and elapsed >= hold_s:
                    stopped_by_us = True
                    await self._blocking(self._stop_trick_quietly)
                if timeout_s is not None and elapsed > timeout_s:
                    await self._blocking(self._stop_trick_quietly)
                    raise TrickError(f"no TRICK_DONE within {timeout_s:.2f} s")
                await asyncio.sleep(0.02)
        except asyncio.CancelledError:
            # the session went away (its hold end already sent TRICK_STOP) or
            # the port is being released
            if not s.closed and not self.port_released:
                self.pool.submit(self._stop_trick_quietly)
            raise
        finally:
            if self._trick_playing is not None and self._trick_playing[0] is s:
                self._trick_playing = None

    async def op_trick_stop(self, s: Session, transition_ticks: int = 0) -> None:
        """TRICK_STOP (any client; also while the behaviour runner is active)."""
        self._need_port()
        await self._blocking(self.client.stop_trick, int(transition_ticks))

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

    log_dir = Path(args.log_dir).expanduser() if getattr(args, "log_dir", None) else Path.home() / "kilo-logs"
    daemon_log = getattr(args, "daemon_log", None)
    if daemon_log != "none":
        add_file_log(Path(daemon_log).expanduser() if daemon_log else log_dir / "daemon.log")
    choice = detect_port(args.port)
    for w in choice.warnings:
        log.warning("%s", w)
    explicit = choice.reason in ("--port", "$KILO_PORT")
    if choice.port is None:
        log.warning("no serial port found yet: waiting for one to appear (set KILO_PORT in "
                    "~/.config/kilo/env or pass --port to pin it)")
    else:
        log.info("port %s (%s%s)", choice.port, choice.reason, "" if explicit else "; re-detected on reconnect")
        if not os.path.exists(choice.port):
            log.warning("%s does not exist yet (robot unplugged?): waiting for it", choice.port)
    logger = None
    if args.log or args.log_file:
        from kilo.log import JsonlLogger

        logger = JsonlLogger(args.log_file or log_dir / time.strftime("kilo-%Y%m%d-%H%M%S.jsonl"))
        log.info("frame log %s", logger.path)
    wire = None
    kl = getattr(args, "kilolog", None)
    if kl is not None:
        from kilo.logs.kilolog import KilologWriter

        wire = KilologWriter(Path(kl).expanduser() if kl else log_dir / time.strftime("kilo-%Y%m%d-%H%M%S.kilolog.gz"),
                             meta={"source": "kilo run", "port": choice.port})
        log.info("raw frame log %s", wire.path)
    keep = [log_dir / "daemon.log"] if daemon_log != "none" and not daemon_log else []
    d = Daemon(choice.port, baud=args.baud, socket=args.socket, logger=logger,
               idle_posture=args.idle_posture, idle_limp_s=args.idle_limp, behaviour=args.behaviour,
               remote=getattr(args, "remote", False), balance_kp=getattr(args, "balance", None),
               log_dir=log_dir, wire_logger=wire,
               auto_port=not explicit, keep_logs=keep)
    return d.run()


def add_file_log(path: Path) -> logging.Handler | None:
    """Also write the daemon's log to ``path`` (rotating, small): there is
    always something to read even when the user journal is not.

    The file is written by a QueueListener thread: the link's reader and
    writer threads log too, and must never wait for the disk. Returns the
    QueueHandler on the root logger (its ``listener`` is stopped at exit)."""
    import atexit
    import queue
    from logging.handlers import QueueListener, RotatingFileHandler

    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fh = RotatingFileHandler(path, maxBytes=DAEMON_LOG_BYTES, backupCount=DAEMON_LOG_BACKUPS)
    except OSError as e:
        log.warning("cannot write the daemon log %s: %s", path, e)
        return None
    fh.setFormatter(logging.Formatter("%(asctime)s %(name)s %(levelname)s %(message)s"))
    q: queue.Queue[Any] = queue.Queue(10000)
    h = _DroppingQueueHandler(q)
    h.setLevel(logging.INFO)
    h.listener = QueueListener(q, fh, respect_handler_level=False)  # type: ignore[attr-defined]
    h.listener.start()  # type: ignore[attr-defined]
    atexit.register(h.listener.stop)  # type: ignore[attr-defined]
    root = logging.getLogger()
    root.addHandler(h)
    if root.level > logging.INFO or root.level == logging.NOTSET:
        root.setLevel(logging.INFO)
    log.info("daemon log %s", path)
    return h


class _DroppingQueueHandler(logging.handlers.QueueHandler):
    """A QueueHandler that drops a record rather than block when its queue is full."""

    def enqueue(self, record: logging.LogRecord) -> None:
        try:
            self.queue.put_nowait(record)
        except Exception:  # queue.Full: the disk is that slow
            pass
