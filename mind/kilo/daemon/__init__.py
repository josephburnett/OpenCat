"""`kilo run`: the daemon that owns the serial port (docs/architecture.md §5, Appendix G).

One process owns the port, the SerialLink and the BodyClient, and serves a
Unix socket (``kilo.remote.socket_path()``) speaking JSON lines (wire format
in kilo.remote). Every ``kilo`` command tries the socket first and falls back
to opening the port itself when no daemon listens.

Module map (``Daemon`` is one class assembled from one mixin per concern;
every mixin reads and writes the same instance attributes, set up in
``core.Daemon.__init__``):

- ``core``      Daemon: construction, ``serve``/shutdown, the port factory
                (hot-plug), the 5 Hz pump, the thread-affinity helpers
                (``_on_loop``, ``_blocking``).
- ``clients``   the socket protocol: accept, parse, dispatch an op, reply,
                close a session.
- ``ops``       every ``op_*`` method (what a client can ask), the ``call``
                allow-list.
- ``sessions``  Session, holds (``goal_owner``, ``_own``, ``_end_hold``),
                hold leases, the idle REST/LIMP latch, the balance default.
- ``behaviour`` the behaviour runner (``--behaviour``), its liveness stamp,
                and runner vs client holds (``--override``).
- ``remote_ir`` the IR remote glue (``--remote``).
- ``streams``   stream fanout to subscribed clients, percepts, connection notes.
- ``port``      port release/reacquire for ``kilo flash`` (Appendix F).
- ``logs``      the daemon's own log file and the log-directory cap.
- ``run``       ``run_daemon``: ``kilo run``'s argparse namespace -> Daemon.

Thread affinity (the rules, written once; every module follows them):

1. The asyncio loop thread owns the daemon's state: ``sessions``,
   ``goal_owner``, ``idle_since``/``idle_stage``, ``port_released``/
   ``released_by``/``release_until``, ``_trick_playing``, ``pausers``. Only
   code on the loop reads or writes them: the ``op_*`` coroutines, the
   client handlers, the pump (leases, idle timeline, runner stamp), the
   link's subscriber callbacks (the link hands them to the loop through
   ``dispatch=loop.call_soon_threadsafe``: stream fanout, percept feed,
   connection notes, ``_on_ir_key``) and the methods they call (``_own``,
   ``_end_hold``, ``_latch_idle``, ``_latch_balance_default``,
   ``_close_session``, ``_reply``, ``_stream``). ``_on_loop()`` asserts it
   where cheap.
2. The loop never blocks. Anything that waits on the body (a request
   through ``SerialLink.request``, which also waits for the link's one
   in-flight slot, ``wait_ready``, ``cal_save``'s EEPROM wait, ``link.stop``,
   the port settle, the flash verify, log pruning) runs in ``self.pool`` (8
   threads): from an op through ``await self._blocking(fn, ...)``, fire and
   forget through ``self.pool.submit`` (``_stop_trick_quietly``,
   ``_remote_perform``, ``prune_logs``). Pool code touches the body only,
   never the state in rule 1.
3. BodyClient goal setters (``posture``, ``set_balance``, ``walk``,
   ``stop_walk``, ``pose``, ``release_pose``) and ``link.set_goal``/
   ``clear_goal`` only latch a goal under the link's lock and queue it for
   the writer thread: they do not wait, so they run on the loop, in the
   same step that records the owner (no window where the goal is latched
   and unowned).
4. Liveness: only the pump (on the loop) and the behaviour runner stamp the
   link's liveness; ``client.stamp_liveness`` is off so pool threads that
   wait on the body never do (a hung loop must trip the body's watchdog).
5. Other threads: the link's reader/writer threads (kilo.link), the
   behaviour runner's thread (``--behaviour``; it drives the daemon's
   BodyClient directly and touches nothing in rule 1), the percept
   listener (``_percept_events``: any thread, hops to the loop),
   ``request_stop`` (any thread), the logging threads (``logs``).

Shutdown (SIGTERM/SIGINT or the ``shutdown`` op): stop accepting, stop the
behaviour runner, close clients, drop every goal and go silent (the body's
watchdog takes over: stop, REST, LIMP), close the port, remove the socket.
"""

from kilo.daemon.behaviour import BEHAVIOUR_ACTIVE, RUNNER_START_GRACE_S, RUNNER_STALL_S, BehaviourActive
from kilo.daemon.clients import CONCURRENT_OPS, LINE_LIMIT, REPLY_BUFFER_MAX
from kilo.daemon.core import DAEMON_LINK, IDLE_POSTURES, PRUNE_EVERY_S, PUMP_PERIOD_S, WIRE_FLUSH_S, Daemon
from kilo.daemon.logs import DAEMON_LOG_BACKUPS, DAEMON_LOG_BYTES, add_file_log
from kilo.daemon.ops import CALL_ALLOWED, CALL_REFUSED, REQUEST_TIMEOUT_MAX_S, TRICK_POSTURES
from kilo.daemon.run import run_daemon
from kilo.daemon.sessions import BODY_HOLDS, Session
from kilo.daemon.streams import STREAM_BUFFER_MAX, STREAM_KINDS

__all__ = [
    "BEHAVIOUR_ACTIVE", "BODY_HOLDS", "CALL_ALLOWED", "CALL_REFUSED", "CONCURRENT_OPS", "DAEMON_LINK",
    "DAEMON_LOG_BACKUPS", "DAEMON_LOG_BYTES", "IDLE_POSTURES", "LINE_LIMIT", "PRUNE_EVERY_S", "PUMP_PERIOD_S",
    "REPLY_BUFFER_MAX", "REQUEST_TIMEOUT_MAX_S", "RUNNER_STALL_S", "RUNNER_START_GRACE_S", "STREAM_BUFFER_MAX",
    "STREAM_KINDS", "TRICK_POSTURES", "WIRE_FLUSH_S", "BehaviourActive", "Daemon", "Session", "add_file_log",
    "run_daemon",
]
