"""The daemon's ops: everything a client can ask (``op_<name>(session, **args)``).

Info: ``daemon`` (pid, version, port and link state, clients, holds, idle
stage, behaviour, remote, balance default, percepts, walk), ``ping``,
``alive``, ``snapshot``, ``percepts`` (flags, distance/approach, recent
events), ``goals`` (the goals this session holds), ``walk_status``
(BodyClient.walk_status(): the walk, its stream and the model's error).

Body queries and commands (each a request on the link, run in the pool):
``wait_ready {wait_s, require_telemetry}``, ``stats``,
``cal_get|cal_set|cal_save|cal_revert``, ``param_get|param_set|param_list``,
``pca_read``, ``eeprom_read {addr, len}`` (hex string), ``request {name,
fields, timeout_s}`` (the raw ``SerialLink.request()``: a query's data reply
or a command's ACK; goals refused; ``timeout_s`` capped at
``REQUEST_TIMEOUT_MAX_S`` per try), ``call {method, args, kwargs}`` (only
the BodyClient methods in ``CALL_ALLOWED``; goals and tricks are refused
with a pointer to their ops, anything else as unknown), ``verify_hello``
(``kilo flash`` verification on the daemon's link).

Goals (held by the session; kilo.daemon.sessions): ``posture {name,
transition_ticks, balance}``, ``set_balance {enable, kp, kd_s,
deadband_deg, clamp_deg}``, ``release`` (end this session's hold), ``walk
{direction, pattern, speed_mm_s, balance}`` (BodyClient.walk: the daemon's
walker plans and streams; the session owns the hold ``WALK`` like a posture
and the posture goal is dropped: the walk replaces it; while walking it
changes the walk), ``stop_walk {wait_s}`` (any client, also under the
behaviour runner: the planned stop; with ``wait_s`` it replies once the walk
ended, at most ``wait_s`` later: ``{stopped, elapsed_s}``), ``pose
{pitch_deg, roll_deg, dheight_mm, xshift_mm}`` and ``release_pose``.
``posture``, ``set_balance``, ``walk``, ``pose``, ``trick_upload`` and
``trick_play`` take ``override`` (kilo.daemon.behaviour).

Tricks. ``trick_upload {name | trick, force}`` (a library name or
``Trick.to_dict()``; cached), ``trick_play {name | trick, mirror, speed,
repeats, posture, hold_s, timeout_s}`` and ``trick_stop
{transition_ticks}`` (any client, also under the behaviour runner).
``trick_play`` uploads (cached), takes the optional posture first (held by
the session), sends TRICK_PLAY and replies on EVENT TRICK_DONE (``{reason,
trick_id, elapsed_s, expected_ticks, epoch, stopped_by_us, name}``);
``hold_s`` stops it after that long. It runs as a task of the session
(kilo.daemon.clients ``CONCURRENT_OPS``). The session owns the trick
(``TRICK`` in ``held_goals``) and, like a posture, the body's end posture:
when the session closes (or releases, or its lease lapses) a playing trick
is stopped and the idle REST/LIMP timeline runs. A second client cannot
start a trick over a playing one (``trick_stop`` first).

Streams: ``subscribe {rate_hz, kinds}`` / ``unsubscribe``
(kilo.daemon.streams). Port: ``release_port {lease_s}`` /
``reacquire_port`` (kilo.daemon.port). ``shutdown``.

Requests. Every op that reaches the body as a request goes through
``SerialLink.request``, which allows one request in flight on the link
(``_req_lock``) for the whole daemon: client ops, the IR remote and the
behaviour runner queue for it. One request holds it for at most
``(retries + 1) x timeout``: 4 x 50 ms = 0.2 s when the body does not
answer, a few ms when it does. The waiters are served in the order the
lock hands it out (``threading.Lock`` is not FIFO): there is no ordering
between clients, only within one client (its ops run in order). An op that
makes several requests (``param_list``, a trick upload, ``eeprom_read``)
takes and releases the lock per request, so other clients' requests slot in
between. ``request`` is the only op whose per-try timeout a client chooses;
it is capped (``REQUEST_TIMEOUT_MAX_S``) so one client cannot hold the link
for long (test_req_lock.py measures this).
"""

from __future__ import annotations

import asyncio
import os
import time
from typing import Any

from kilo import __version__
from kilo.body.client import TRICK_DONE_MARGIN_S, BodyClient, TrickError
from kilo.link import consts
from kilo.link.serial_link import LinkError

TRICK_POSTURES = ("rest", "sit", "stand")
# BodyClient methods the generic `call` op serves: the ones RemoteBody (the
# CLI through the daemon) reaches through its __getattr__ proxy and that have
# no op of their own: `kilo look` / `kilo look --release` (SET_HEAD) and ping.
# Everything else is refused: a name not here gets "not served".
CALL_ALLOWED = frozenset({"look", "release_head", "ping"})
# BodyClient methods `call` refuses with a pointer to their ops: goals and
# tricks have their own ops (session ownership, the runner check, the loop).
CALL_REFUSED = frozenset({"posture", "limp", "rest", "stand", "sit", "balance", "set_balance", "release",
                          "upload_trick", "start_trick", "play", "stop_trick", "spin", "wait_ready",
                          "walk", "stop_walk", "walk_status", "walker", "pose", "release_pose"})
# The `request` op's per-try timeout cap: a request holds the link's one
# in-flight slot for up to (retries + 1) x timeout (4 x 0.5 s = 2 s).
REQUEST_TIMEOUT_MAX_S = 0.5


class OpsMixin:
    # ---- info ------------------------------------------------------------------------------
    async def op_daemon(self, s: Any) -> dict[str, Any]:
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
                "idle": self.idle_stage, "walk": self.client.walk_status()}

    async def op_percepts(self, s: Any) -> dict[str, Any]:
        """The daemon's percepts: flags, distance/approach, recent events."""
        return self.percepts.state()

    async def op_ping(self, s: Any) -> str:
        return "pong"

    async def op_alive(self, s: Any) -> bool:
        return True

    async def op_snapshot(self, s: Any) -> dict[str, Any]:
        return self.client.state.snapshot()

    async def op_wait_ready(self, s: Any, wait_s: float = 3.0, require_telemetry: bool = True) -> Any:
        self._need_port()
        return await self._blocking(self.client.wait_ready, wait_s, require_telemetry=require_telemetry)

    async def op_stats(self, s: Any) -> Any:
        self._need_port()
        return await self._blocking(self.client.stats)

    async def op_goals(self, s: Any) -> list[str]:
        latched = self.link.goals()
        return sorted(n for n in s.goals if n in latched and self.goal_owner.get(n) is s)

    # ---- goals (held by the session) -------------------------------------------------------
    async def op_posture(self, s: Any, name: str, transition_ticks: int = 0,
                         balance: bool | None = None, override: bool = False) -> None:
        self._need_port()
        self._check_runner(s, override)
        self.client.posture(name, transition_ticks=transition_ticks, balance=balance)
        self._own(s, "SET_POSTURE")

    async def op_set_balance(self, s: Any, enable: bool, override: bool = False, **gains: Any) -> dict[str, int]:
        self._need_port()
        self._check_runner(s, override)
        f = self.client.set_balance(bool(enable), **gains)
        self._own(s, "SET_BALANCE")
        return f

    async def op_release(self, s: Any) -> None:
        self._end_hold(s)

    async def op_walk(self, s: Any, direction: str = "forward", pattern: str = "trot",
                      speed_mm_s: float | None = None, balance: bool = True,
                      override: bool = False) -> dict[str, Any]:
        """BodyClient.walk (the daemon's walker), held by this session like a
        posture (``WALK``): when the session ends the daemon stops the walk the
        planned way, then runs the idle timeline. Replaces a posture goal."""
        from kilo.planner import patterns as pat

        self._need_port()
        pat.pattern(pattern), pat.direction(direction)  # ValueError before anything is sent or owned
        self._check_runner(s, override)
        st = await self._blocking(self.client.walk, direction, pattern, speed_mm_s, balance)
        prev = self.goal_owner.pop("SET_POSTURE", None)  # the walk dropped the posture goal
        if prev is not None:
            prev.goals.discard("SET_POSTURE")
        self._own(s, "WALK")
        return st

    async def op_stop_walk(self, s: Any, wait_s: float | None = None) -> dict[str, Any]:
        """The planned stop of the daemon's walk (any client, also while the
        behaviour runner is active); with ``wait_s``, reply once the walk
        ended, at most ``wait_s`` later: ``{stopped, elapsed_s}``."""
        self._need_port()
        t0 = time.monotonic()
        if wait_s is None:
            self.client.stop_walk()
            return {"stopped": None, "elapsed_s": 0.0}
        ok = await self._blocking(self.client.stop_walk, float(wait_s))
        return {"stopped": bool(ok), "elapsed_s": round(time.monotonic() - t0, 3)}

    async def op_walk_status(self, s: Any) -> dict[str, Any] | None:
        """BodyClient.walk_status(): the walk, its stream and the model's error."""
        return self.client.walk_status()

    async def op_pose(self, s: Any, pitch_deg: float = 0, roll_deg: float = 0, dheight_mm: float = 0,
                      xshift_mm: float = 0, override: bool = False) -> dict[str, int]:
        self._need_port()
        self._check_runner(s, override)
        f = self.client.pose(pitch_deg, roll_deg, dheight_mm, xshift_mm)
        self._own(s, "SET_BODY_POSE")
        return f

    async def op_release_pose(self, s: Any) -> None:
        self._need_port()
        self.client.release_pose()
        if self.goal_owner.get("SET_BODY_POSE") is s:
            self.goal_owner.pop("SET_BODY_POSE", None)
            s.goals.discard("SET_BODY_POSE")

    # ---- body queries and commands (requests, in the pool) ---------------------------------
    async def op_cal_get(self, s: Any) -> list[int]:
        self._need_port()
        return await self._blocking(self.client.cal_get)

    async def op_cal_set(self, s: Any, joint: int, offset_a16: int) -> None:
        self._need_port()
        await self._blocking(self.client.cal_set, joint, offset_a16)

    async def op_cal_save(self, s: Any, wait_saved: float | None = 2.0) -> bool:
        self._need_port()
        return await self._blocking(self.client.cal_save, wait_saved)

    async def op_cal_revert(self, s: Any) -> None:
        self._need_port()
        await self._blocking(self.client.cal_revert)

    async def op_param_get(self, s: Any, id: Any) -> Any:
        self._need_port()
        return await self._blocking(self.client.param_get, id)

    async def op_param_set(self, s: Any, id: Any, value: int, persist: bool = False) -> None:
        self._need_port()
        await self._blocking(self.client.param_set, id, value, persist)

    async def op_param_list(self, s: Any) -> list[Any]:
        self._need_port()
        return await self._blocking(self.client.param_list)

    async def op_pca_read(self, s: Any) -> list[int]:
        self._need_port()
        return await self._blocking(self.client.pca_counts)

    async def op_eeprom_read(self, s: Any, addr: int, len: int) -> str:
        self._need_port()
        return (await self._blocking(self.client.eeprom_read, addr, len)).hex()

    async def op_request(self, s: Any, name: str, fields: dict[str, Any] | None = None,
                         timeout_s: float | None = None) -> Any:
        """The raw SerialLink.request(): a query's data reply, or a command's ACK."""
        self._need_port()
        if name in consts.GOAL_TYPES:
            raise ValueError(f"{name} is a goal: use posture/set_balance")
        if timeout_s is not None:
            timeout_s = min(float(timeout_s), REQUEST_TIMEOUT_MAX_S)
        return await self._blocking(self.link.request, name, fields or {}, timeout=timeout_s)

    async def op_call(self, s: Any, method: str, args: list[Any] | None = None,
                      kwargs: dict[str, Any] | None = None) -> Any:
        """One of the BodyClient methods in ``CALL_ALLOWED`` (RemoteBody's proxy)."""
        if method in CALL_REFUSED:
            raise ValueError(f"{method}: not through `call`; use the posture, set_balance, release, walk, "
                             f"pose and trick_upload/trick_play/trick_stop ops")
        if method not in CALL_ALLOWED:
            raise ValueError(f"BodyClient.{method} is not served through `call` (served: "
                             f"{', '.join(sorted(CALL_ALLOWED))}; the other methods have their own ops)")
        self._need_port()
        return await self._blocking(getattr(self.client, method), *(args or []), **(kwargs or {}))

    async def op_verify_hello(self, s: Any, since: float, wait_s: float = 8.0,
                              expected_build: int | None = None, any_cause: bool = False) -> dict[str, Any]:
        """``kilo flash`` verification on the daemon's link (``since`` is
        time.monotonic(), which is system-wide on Linux)."""
        from kilo.tools.flash import verify_hello

        self._need_port()
        r = await self._blocking(verify_hello, self.link, since, expected_build, timeout=wait_s,
                                 any_cause=any_cause)
        return {"ok": r.ok, "hello": r.hello}

    # ---- streams ---------------------------------------------------------------------------
    async def op_subscribe(self, s: Any, rate_hz: float = 10.0,
                           kinds: list[str] | None = None) -> dict[str, Any]:
        from kilo.daemon.streams import STREAM_KINDS

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

    async def op_unsubscribe(self, s: Any) -> None:
        s.kinds = set()
        if s.stream_task is not None:
            s.stream_task.cancel()
            s.stream_task = None

    # ---- port release (kilo.daemon.port) -----------------------------------------------------
    async def op_release_port(self, s: Any, lease_s: float | None = None) -> dict[str, Any]:
        """Stop the link and close the port (for avrdude) until reacquire_port,
        the lease runs out, or this client disconnects."""
        return await self._release_port(s, lease_s)

    async def op_reacquire_port(self, s: Any) -> dict[str, Any]:
        if not self.port_released:
            return {"port": self.port, "reacquired": False}
        await self._reacquire()
        return {"port": self.port, "reacquired": True}

    # ---- tricks ------------------------------------------------------------------------------
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

    async def op_trick_upload(self, s: Any, name: str | None = None, trick: dict[str, Any] | None = None,
                              force: bool = False, override: bool = False) -> dict[str, Any]:
        """Upload a library trick (``name``) or ``trick`` (Trick.to_dict()); cached."""
        self._need_port()
        t = self._load_trick(name, trick)
        self._check_runner(s, override)
        tid, uploaded = await self._blocking(self.client.upload_trick, t, force=bool(force))
        return {"name": t.name, "trick_id": tid, "uploaded": uploaded, "crc16": t.crc16(), "frames": len(t.frames)}

    async def op_trick_play(self, s: Any, name: str | None = None, trick: dict[str, Any] | None = None,
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

    async def op_trick_stop(self, s: Any, transition_ticks: int = 0) -> None:
        """TRICK_STOP (any client; also while the behaviour runner is active)."""
        self._need_port()
        await self._blocking(self.client.stop_trick, int(transition_ticks))

    async def op_shutdown(self, s: Any) -> str:
        self.loop.call_later(0.05, self.stopping.set)
        return "stopping"
