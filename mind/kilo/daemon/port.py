"""Port release and reacquire for ``kilo flash`` (docs/architecture.md Appendix F).

``release_port {lease_s}`` drops every goal and hold, cancels trick plays,
stops the link and closes the port so ``kilo flash`` can run avrdude;
``reacquire_port`` or the lease running out (checked by the pump) reopens
it. The lease is honoured even when the releasing client goes away (a CLI
killed mid-flash leaves avrdude running): the port stays released until the
lease ends. Every reopen first settles the port (``settle_port``:
DTR/HUPCL/baud, so avrdude's exit state does not reset the body on the
reopen), then restarts the link and latches the balance default again
(``--balance``). While released, ops that need the body fail with
NotConnected (``_need_port``) and IR keys are ignored.

The state flips (``port_released``, the owners) happen on the loop; the
link stop and the settle wait, so they run in the pool (``_blocking``).
"""

from __future__ import annotations

import logging
import time
from typing import Any

from kilo.link.serial_link import LinkError, NotConnected
from kilo.link.transport import settle_port

log = logging.getLogger("kilo.daemon")


class PortMixin:
    def _need_port(self) -> None:
        if self.port_released:
            raise NotConnected(f"{self.port} is released (flashing); try again when it is reacquired")

    async def _release_port(self, s: Any, lease_s: float | None) -> dict[str, Any]:
        assert self._on_loop()
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
        await self._blocking(self._abort_walk)
        await self._blocking(self.link.stop)
        log.info("port %s released for %.0f s (client %d)", self.port, lease, s.id)
        return {"port": self.port, "lease_s": lease}

    def _abort_walk(self) -> None:
        """Pool thread: end a walk's stream at once (shutdown, port release)."""
        w = getattr(self.client, "_walker", None)
        if w is not None and w.active:
            w.abort()

    def _check_release_lease(self, now: float) -> None:
        """From the pump: reacquire once the release lease ran out."""
        if self.port_released and self.release_until is not None and now > self.release_until:
            log.warning("port release lease expired: reacquiring %s", self.port)
            self.loop.create_task(self._reacquire())

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
        """Pool thread: settle the port before the link reopens it."""
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
