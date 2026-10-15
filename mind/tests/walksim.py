"""Deterministic walking: the planner's Stream over the virtual body's FEET
player and physics, one 20 ms tick at a time, no threads and no wall time.

``WalkSim`` holds a VirtualBody (no transport) and a link stand-in whose
``send_stream`` hands the FEET frame to the body at once and whose
BodyState is fed with the body's TELEMETRY and EVENTs as the real link's
reader would (counts, history, epoch). Each ``step()``: the body ticks (its
telemetry arrives 1 ms later), then the stream wakes 5 ms after the tick,
unless the test stalls it."""

from __future__ import annotations

import math
from typing import Any

from kilo.body.state import BodyState
from kilo.planner.model import BodyModel
from kilo.planner.plan import Planner, PlannerConfig
from kilo.planner.walker import LOOKAHEAD, BodyClock, Stream
from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody

TICK = 0.02


class FakeLink:
    def __init__(self, sim: "WalkSim"):
        self.sim = sim
        self.state = BodyState(clock=lambda: sim.now)
        self.logger = None
        self.alive = True
        self.sent: list[Any] = []

    def send_stream(self, name: str, payload: bytes) -> bool:
        self.sent.append(P.Feet.unpack(payload))
        self.sim.vb.process(C.build_frame(P.MsgId.FEET, 0, payload), self.sim.now)
        return True

    def mind_alive(self) -> bool:
        return self.alive


class WalkSim:
    def __init__(self, pattern: str = "walk", direction: str = "forward", speed: float | None = None,
                 config: PlannerConfig | None = None, balance: bool = True, kp_q8: int = 253,
                 model: BodyModel | None = None, body_model: BodyModel | None = None, **vb_kw: Any):
        self.vb = VirtualBody(None, model=body_model, **vb_kw)
        self.vb.telem_div = 1
        self.vb.posture = "STANCE"
        if balance:
            self.vb.balance.update(enable=1, kp_q8=kp_q8)  # Kp 0.99: the daemon's default on Kilo
        self.now = 100.0
        self.link = FakeLink(self)
        self.handovers = 0
        self.telemetry: list[Any] = []
        self.events: list[tuple[str, int]] = []
        self.step()  # one telemetry: the body clock and the epoch
        bc = BodyClock()
        for rx, m in self.link.state.history:
            bc.add(m.tick, rx)
        start = int(math.floor(bc.now(self.now + 0.005))) + LOOKAHEAD
        self.planner = Planner(model or BodyModel(), pattern, direction, speed, tick=start, config=config)
        self.stream = Stream(self.link, self.planner, self.vb.epoch, bc, self._handover)

    def _handover(self) -> None:
        self.handovers += 1
        self.vb.process(C.build_frame(P.MsgId.SET_POSTURE, 1, P.SetPosture(epoch=self.vb.epoch,
                                                                             posture=P.Posture.STANCE).pack()),
                        self.now)

    def step(self, wake: bool = True) -> bool:
        """One body tick; then the stream's wake unless ``wake`` is False.
        Returns whether the stream goes on."""
        self.now += TICK
        self.vb._tick(self.now)
        st = self.link.state
        for typ, _, payload in self.vb.take_sent():
            m = P.decode_payload(typ, payload)
            if typ == P.MsgId.TELEMETRY:
                with st.lock:
                    st.on_telemetry(m, self.now + 0.001)
                self.telemetry.append(m)
            elif typ == P.MsgId.EVENT:
                with st.lock:
                    st.events.append((self.now + 0.001, m))
                    st.event_count += 1
                    st.epoch = m.epoch
                self.events.append((P.EventCode(m.code).name, m.arg))
        if not wake or not hasattr(self, "stream"):
            return True
        return self.stream.wake(self.now + 0.005)

    def run(self, ticks: int, stall: tuple[int, int] | None = None, each=None) -> int:
        """Up to ``ticks`` ticks (the stream asleep for ticks in ``stall``); returns the ticks run."""
        for i in range(ticks):
            on = self.step(wake=not (stall and stall[0] <= i < stall[1]))
            if each is not None:
                each(i)
            if not on:
                return i + 1
        return ticks

    # ---- what happened -------------------------------------------------------------------
    def gait(self) -> list[Any]:
        return [t for t in self.telemetry if t.mode == P.Mode.GAIT]

    def max_error(self, skip: int = 0) -> float:
        """The largest |measured - predicted| (degrees) in GAIT (the stream's
        prediction for the tick: the body reports a reference only with its
        balance on)."""
        pred = self.pred()
        return max((max(abs(t.pitch / 16.0 - pred[t.tick][0]), abs(t.roll / 16.0 - pred[t.tick][1]))
                    for t in self.gait()[skip:] if t.tick in pred), default=0.0)

    def pred(self) -> dict:
        """u16 tick -> the (pitch, roll) the stream sent for it."""
        return {f.tick: (f.pitch / 16.0, f.roll / 16.0) for f in self.link.sent}

    def fell(self) -> bool:
        return any(n == "FELL" for n, _ in self.events)
