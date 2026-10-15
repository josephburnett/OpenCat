"""Sessions and holds: who owns which goal, and what happens when a hold ends.

Holds. Goals a client sets (``posture``, ``set_balance``, ``walk``, ``pose``,
a trick) belong to that client's session (``goal_owner[name] is session``)
and are kept alive by the daemon while the session lives and keeps its lease
(any message within ``hold_lease_s``, 3 s by default; RemoteBody pings while
it holds). When the session closes, releases or its lease lapses
(``_check_leases``, from the pump), the hold ends like ``kilo posture --for``
ends without a daemon (``_end_hold``): the goal is dropped, the daemon
latches REST at once and LIMP after ``idle_limp_s`` (the watchdog's REST/LIMP
timeline, without the 0.5 s stop stage; ``kilo run --idle-posture
rest|limp|none``, ``--idle-limp S``), and a balance enable is switched off.
With a behaviour runner active the daemon only drops the goal and leaves the
body to the runner. The idle REST is latched whenever a hold that owned what
the legs do (``BODY_HOLDS``: the posture, a trick or a walk) ends and
nobody else owns one, even if another client still holds SET_BALANCE; a
balance-, head- or pose-only hold that ends leaves the posture timeline
alone. A walk is never just dropped: the daemon stops it the planned way
(each leg steps home, STANCE; kilo.planner.walker), and only then runs the
idle timeline (a REST latched earlier would take the legs mid-step); a pose
hold ending latches the neutral pose; a trick hold ending stops the
trick.

Latches owned by nobody (``goal_owner[name] is None``): the idle REST/LIMP,
the balance default, and whatever the IR remote set. They are held as long
as the daemon feeds the body's watchdog, until the next client goal.

Balance by default (``--balance [KP]``, off by default; the owner's request
2026-10-11: Kilo wakes up balancing). The daemon latches SET_BALANCE enable
with the default gains (Kp as given, default kilo.body.client.BALANCE_KP)
at start and after a port release, owned by nobody like the idle latch. A
client's balance hold that ends goes back to the default (on) instead of off;
the remote's CH+ key toggles it off until the next CH+ or the next client
hold that ends. ``daemon`` reports ``balance_default`` (the Kp, or None).
SET_BALANCE is not an epoch goal on the mind's side (the link keeps it
latched and refreshes it at 5 Hz across a reboot or an epoch bump), but the
body clears ``bal.enable`` on every epoch bump (body.cpp ``body_epoch_bump``,
P1 finding B1): after a fall, a watchdog stop or LOW battery balance is off
until the next refresh, at most ~200 ms later
(``test_balance_enable_restored_after_epoch_bump`` pins it).

Everything here runs on the loop thread (kilo.daemon, thread-affinity rule 1).
"""

from __future__ import annotations

import asyncio
import logging
import time

from kilo.link.serial_link import LinkError

log = logging.getLogger("kilo.daemon")

# Holds that decide what the legs do: when the last one ends the idle timeline runs.
BODY_HOLDS = ("SET_POSTURE", "TRICK", "WALK")


class Session:
    """One connected client."""

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


class HoldsMixin:
    """Ownership of goals by sessions, hold leases, the idle latch."""

    def _check_leases(self) -> None:
        """From the pump: lapsed hold leases, the port-release lease, the idle LIMP stage."""
        assert self._on_loop()
        now = time.monotonic()
        for s in list(self.sessions.values()):
            if s.goals and s.inflight == 0 and now - s.last_seen > self.hold_lease_s:
                log.info("client %d: hold lease lapsed (%.1f s silent)", s.id, now - s.last_seen)
                self._end_hold(s)
        self._check_release_lease(now)
        if (self.idle_stage == "rest" and self.idle_posture == "rest" and self.idle_since is not None
                and now - self.idle_since >= self.idle_limp_s and not self._posture_held()):
            self.idle_stage = "limp"
            self._latch_idle("limp")

    def _held(self) -> bool:
        return any(o is not None for o in self.goal_owner.values())

    def _posture_held(self) -> bool:
        """Somebody owns what the body's legs do: SET_POSTURE, a trick or a walk."""
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
        assert self._on_loop()
        try:
            f = self.client.set_balance(True, kp=self.balance_kp)
            self.goal_owner["SET_BALANCE"] = None
            log.info("balance on by default (Kp %.2f)", f["kp_q8"] / 256)
        except (LinkError, ValueError) as e:
            log.warning("balance default: %s", e)

    def _own(self, s: "Session", name: str) -> None:
        assert self._on_loop()
        prev = self.goal_owner.get(name)
        if prev is not None and prev is not s:
            prev.goals.discard(name)
        self.goal_owner[name] = s
        s.goals.add(name)
        if name in BODY_HOLDS:  # a balance-, head- or pose-only hold leaves the idle timeline running
            self.idle_since = None
            self.idle_stage = None

    def _end_hold(self, s: "Session") -> None:
        assert self._on_loop()
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
            posture_hold_ended = "SET_POSTURE" in names or "WALK" in names
        if self._shutting_down:
            return  # go_silent() drops everything next
        if "WALK" in names:
            names.remove("WALK")
            st = self.client.walk_status()
            if st is not None and st.get("active") and not self.port_released:
                # A walk is never just dropped: the planned stop first, the
                # idle timeline after it (_idle_after_walk).
                self.pool.submit(self._stop_walk_then_idle, s.id)
                posture_hold_ended = False
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

    def _stop_walk_then_idle(self, sid: int) -> None:
        """Pool thread: the planned stop of a walk whose hold ended, then the
        idle timeline on the loop."""
        from kilo.planner.walker import stop_timeout

        st = self.client.walk_status() or {}
        try:
            if not self.client.stop_walk(wait_s=stop_timeout(st.get("period_ms"))):
                log.warning("client %d: the walk did not stop in time", sid)
        except LinkError as e:
            log.warning("client %d: walk stop: %s", sid, e)
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self._idle_after_walk)

    def _idle_after_walk(self) -> None:
        """On the loop, after a walk whose hold ended has stopped: the idle
        timeline, unless somebody holds the legs again (or the runner, the
        port release or the shutdown has them)."""
        if (self._shutting_down or self.port_released or self.behaviour_active or self.idle_posture == "none"
                or self._posture_held()):
            return
        self.idle_since = time.monotonic()
        self.idle_stage = self.idle_posture
        self._latch_idle(self.idle_posture)

    def _stop_trick_quietly(self) -> None:
        """TRICK_STOP, errors logged (pool thread: a request)."""
        try:
            self.client.stop_trick()
        except LinkError as e:
            log.warning("trick stop: %s", e)
