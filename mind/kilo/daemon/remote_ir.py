"""The IR remote under the daemon (``kilo run --remote``, off by default; the owner's request 2026-10-11).

The body relays the Petoi remote as EVENT IR_KEY; the daemon maps keys to
actions on its own BodyClient (``kilo.ir.ACTIONS``: postures, tricks, the
CH+ balance toggle, walking in a direction, the gait pattern and speed keys;
held-key repeats, EQ, 9 and unknown keys are ignored). The key table and
what each action does are in kilo.ir and mind/README.md "IR remote".

The event arrives on the loop (``_on_ir_key``, a link subscriber); the
action itself runs in the thread pool, one at a time (``_remote_lock``,
``kilo.ir.RemoteControl``), because it may upload a trick or wait. A
posture, trick or walk set by the remote is held like the idle latch: nobody
owns it and the idle REST/LIMP timeline is stopped, so it stays until the
next key or a client goal. Keys are logged and ignored while the behaviour
runner is active, while the port is released, and while a client holds the
posture, a trick or a walk (the balance key: while a client holds
SET_BALANCE). ``daemon`` reports ``remote`` (on/off and the last key).
"""

from __future__ import annotations

import logging
from typing import Any

from kilo.daemon.sessions import BODY_HOLDS
from kilo.link import consts

log = logging.getLogger("kilo.daemon")


class RemoteIRMixin:
    def _start_remote(self) -> None:
        from kilo.ir import RemoteControl

        self._remote_control = RemoteControl(self.client, balance_kp=self.balance_kp)
        self.link.subscribe("event", self._on_ir_key)

    def _on_ir_key(self, msg: Any) -> None:
        """Link event callback (on the loop): EVENT IR_KEY -> a remote action."""
        if msg.code != consts.EV["IR_KEY"]:
            return
        assert self._on_loop()
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
            self.goal_owner["WALK" if action.kind == "walk" else "SET_POSTURE"] = None
            self.idle_since = self.idle_stage = None
        self.remote_last = text
        self.pool.submit(self._remote_perform, action, text)

    def _remote_perform(self, action: Any, text: str) -> None:
        """Pool thread: one remote action at a time."""
        with self._remote_lock:
            try:
                did = self._remote_control.perform(action)
                log.info("remote %s: %s", text, did)
            except Exception as e:  # LinkError, TrickError, a missing trick file: log, keep serving
                log.warning("remote %s: %s failed: %s", text, action.kind, e)
