"""The Daemon class: construction, serve and shutdown, the pump.

``Daemon`` is assembled from one mixin per concern (module map and the
thread-affinity rules: kilo.daemon). This module keeps what ties them
together: the state every mixin shares (``__init__``), the loop's lifetime
(``serve``, ``_shutdown``), the transport factory and the pump.

Port hot-plug. The daemon starts whether or not the port exists (robot
unplugged, adapter not enumerated yet) and the link's reader keeps trying to
open it (``_open_port``, the link's transport factory), backing off from 2 s
to 10 s (``DAEMON_LINK``); after a USB replug the read error closes the port
and the same retry loop reopens it. Without ``--port``/``$KILO_PORT`` the
port is auto-detected again on every attempt. ``kilo daemon`` shows
connected/alive, whether the device node exists, the last open error and the
reconnect count; the ``connection`` stream carries ``{connected, alive,
port, error}`` on every change.

The pump (on the loop, every ``PUMP_PERIOD_S``) stamps the link's liveness
(kilo.link: the writer thread's heartbeats, goal refresh and HELLO polls
only go out while the loop proves alive, so a stalled daemon trips the
body's watchdog; with a behaviour runner only while the runner ticks:
kilo.daemon.behaviour), checks hold leases and the idle timeline
(kilo.daemon.sessions) and the port-release lease (kilo.daemon.port),
flushes the raw frame log every ``WIRE_FLUSH_S`` and, hourly, prunes the log
directory (kilo.daemon.logs).
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import logging
import os
import signal
import threading
import time
from pathlib import Path
from typing import Any, Callable

from kilo import __version__
from kilo.body.client import BodyClient
from kilo.daemon.behaviour import RUNNER_STALL_S, BehaviourMixin
from kilo.daemon.clients import LINE_LIMIT, ClientsMixin
from kilo.daemon.logs import LogsMixin
from kilo.daemon.ops import OpsMixin
from kilo.daemon.port import PortMixin
from kilo.daemon.remote_ir import RemoteIRMixin
from kilo.daemon.sessions import HoldsMixin, Session
from kilo.daemon.streams import StreamsMixin
from kilo.link.serial_link import LinkConfig, SerialLink
from kilo.link.transport import DEFAULT_BAUD, open_port
from kilo.log import LOG_DIR_MAX_BYTES
from kilo.perception.shared import SharedPercepts
from kilo.remote import listen_socket, socket_candidates

log = logging.getLogger("kilo.daemon")

PUMP_PERIOD_S = 0.2
WIRE_FLUSH_S = 5.0
# The daemon's link: wait for a missing port patiently (2 s, backing off to 10 s).
DAEMON_LINK = dict(reconnect_min_s=2.0, reconnect_max_s=10.0)
IDLE_POSTURES = ("rest", "limp", "none")
PRUNE_EVERY_S = 3600.0


class Daemon(OpsMixin, ClientsMixin, HoldsMixin, BehaviourMixin, RemoteIRMixin, StreamsMixin, PortMixin,
             LogsMixin):
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
        # ---- loop-owned state (kilo.daemon, thread-affinity rule 1) ----
        self.sessions: dict[int, Session] = {}
        self._next_sid = 1
        self.goal_owner: dict[str, Session | None] = {}
        self.idle_since: float | None = None
        self.idle_stage: str | None = None  # "rest" | "limp" once the idle timeline ran
        self.released_by: Session | None = None
        self.release_until: float | None = None
        self.port_released = False
        self._trick_playing: tuple[Session, int] | None = None  # the trick_play in progress: (session, trick_id)
        self.pausers: set[int] = set()  # sessions whose --override paused the runner
        # ----
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
        # log directory size cap
        self.log_cap_bytes = log_cap_bytes
        self.keep_logs = {Path(p) for p in keep_logs}
        self._last_prune = -1e9

    # ---- thread affinity (kilo.daemon) -------------------------------------------------------
    def _on_loop(self) -> bool:
        """True on the loop thread (or before the loop runs): rule 1's state may be touched."""
        return self._loop_thread is None or threading.get_ident() == self._loop_thread

    async def _blocking(self, fn: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
        """Run ``fn`` (anything that waits on the body) in the pool and await it (rule 2)."""
        return await self.loop.run_in_executor(self.pool, lambda: fn(*args, **kwargs))

    # ---- lifecycle ---------------------------------------------------------------------------
    def run(self, install_signals: bool = True) -> int:
        return asyncio.run(self.serve(install_signals))

    def request_stop(self) -> None:
        """Any thread."""
        if self.loop is not None and self.stopping is not None:
            try:
                self.loop.call_soon_threadsafe(self.stopping.set)
            except RuntimeError:
                pass  # the loop already finished

    def _open_port(self) -> Any:
        """The link's transport factory (the link's reader thread): (re)detect
        the port when it was not given, and fail fast (OSError) while the
        device node is missing."""
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
        self._subscribe_link()
        if self.remote:
            self._start_remote()
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

    # ---- pump: liveness, leases, idle timeline, log flush and cap -----------------------------
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
