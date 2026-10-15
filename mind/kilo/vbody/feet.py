"""The virtual body's FEET player: GAIT as the firmware plays the mind's
stream (protocol/spec.toml FEET, doc §3d, §3f), on kilo.vbody.physics.

- **Queue.** A FEET for a tick 1..``FEET_DEPTH`` after the last tick
  played is queued (a second one for the same tick replaces it); one for a
  tick already played, or further ahead, is dropped and counted (STATS
  feet_late). Until the first queued tick comes the legs stay as they were.
- **Play.** Each tick takes the FEET for that tick. The target is its feet
  plus the balance correction on the feet it marks as down; every joint
  moves at most ``slew_deg_per_tick`` toward it (the body's safety slew),
  so a late or missing tick never snaps.
- **Underrun.** No FEET for this tick: the last pose is held (EVENT
  FEET_UNDERRUN, arg = ticks since the last FEET arrived; STATS
  feet_underruns) for ``FEET_HOLD_TICKS``; then the feet that were in the
  air come down ``SETTLE_MM`` per tick at their x to the support feet's
  depth, and when all four are down the gait ends (``settled``). A FEET for
  the current tick during the hold or the settle resumes the play.
- **Balance** (doc §3f, as body/src/core/balance.h and the geometric loop
  of body.cpp): with SET_BALANCE enabled, per axis the ground slope
  s = measured - correction - reference, e = deadband(s), u = Kp e + Kd rate,
  the correction low-passes toward -u by 1/4 per tick, moves at most 1 deg
  per tick and is clamped; the reference is the FEET's pitch/roll. It is
  held (frozen) while any foot is up and for 2 ticks after all four are
  down (body.cpp: on three feet the corrections act through the feet a
  tipping body pivots on), and it acts through the support feet only, each
  leg's share ramping a quarter per tick as its support bit changes:
  z_i += share_i (end_i (Lb/2) sin(pitch corr) + side_i (W/2) sin(roll
  corr)) (end -1 front, +1 back; side -1 left, +1 right).
"""

from __future__ import annotations

import math
from typing import Any

from kilo.planner.model import FRONT, LEFT, LEGS, BodyModel
from kilo.protocol import kilo_protocol as P
from kilo.vbody.physics import Physics

FEET_DEPTH = P.FEET_DEPTH
FEET_HOLD_TICKS = P.FEET_HOLD_TICKS
BAL_HOLD_TICKS = 2
SETTLED_MM = 0.5
SETTLE_MM = 2.0  # a lifted foot's descent per tick when settling


class FeetPlayer:
    def __init__(self, tick: int, model: BodyModel, lag_ms: float | None = None, x0=None, z0=None):
        self.model = model
        self.queue: dict[int, Any] = {}     # unwrapped tick -> Feet
        self.last: Any = None               # the FEET played last
        self.last_tick = tick               # its tick
        self.dry = 0                        # ticks without a FEET
        self.state = "play"                 # play, hold, settle
        h = model.stance_height(P.GAIT_STANCE_H_256)
        self.x = list(x0) if x0 is not None else [0.0] * 4   # commanded feet (after the slew)
        self.z = list(z0) if z0 is not None else [h] * 4
        self.target = (list(self.x), list(self.z))
        self.physics = Physics(model, lag_ms=lag_ms, feet=(self.x, self.z))
        self.corr = [0.0, 0.0]              # pitch, roll correction (deg)
        self._lp = [0.0, 0.0]
        self._hold = 0
        self.support = 0xF
        self.ref = (0.0, 0.0)
        self.share = [1.0] * 4              # each leg's share of the correction, 0..1 in quarters
        self.rx_tick = tick                 # the body's tick when the last FEET arrived
        self._ground = 0.0

    # ---- queue ---------------------------------------------------------------------------
    def offer(self, msg: Any, now_tick: int) -> bool:
        """Queue a FEET (its u16 tick against the body's unwrapped tick). False: late or too far ahead."""
        d = (msg.tick - now_tick) & 0xFFFF
        if d >= 0x8000:
            d -= 0x10000
        if not 1 <= d <= FEET_DEPTH:
            return False
        self.queue[now_tick + d] = msg
        self.rx_tick = now_tick
        return True

    def queued(self, now_tick: int) -> int:
        return sum(1 for t in self.queue if t > now_tick)

    # ---- one tick ----------------------------------------------------------------------------
    def tick(self, now_tick: int, measured: tuple[float, float], rates: tuple[float, float], balance: dict,
             balance_on: bool) -> tuple[str | None, int]:
        """Play tick ``now_tick``. Returns (what happened, arg): ("underrun",
        ticks since the last FEET), ("settled", 0), or (None, 0)."""
        for t in [t for t in self.queue if t < now_tick]:
            del self.queue[t]
        msg = self.queue.pop(now_tick, None)
        happened: tuple[str | None, int] = (None, 0)
        if msg is not None:
            self.last, self.last_tick, self.dry, self.state = msg, now_tick, 0, "play"
            self.support = int(msg.support)
            self.ref = (msg.pitch / 16.0, msg.roll / 16.0)
            tx, tz = [float(v) for v in msg.x_mm], [float(v) for v in msg.z_mm]
            self.target = (tx, tz)
        elif self.last is None:
            pass  # GAIT entered, its first FEET not due yet: STANCE's feet
        else:
            self.dry += 1
            if self.state == "play":
                self.state = "hold"
                happened = ("underrun", now_tick - self.rx_tick)
            if self.state == "hold" and self.dry > FEET_HOLD_TICKS:
                self.state = "settle"
                down = [self.target[1][leg] for leg in LEGS if self.support >> leg & 1]
                self._ground = (sum(down) / len(down) if down
                                else self.model.stance_height(P.GAIT_STANCE_H_256))
                self.target = (list(self.x), list(self.target[1]))
            if self.state == "settle":  # lifted feet down SETTLE_MM a tick at their x
                tz = self.target[1]
                for leg in LEGS:
                    if not self.support >> leg & 1:
                        tz[leg] = min(self._ground, tz[leg] + SETTLE_MM)
                        if tz[leg] >= self._ground:
                            self.support |= 1 << leg
        self._balance(measured, rates, balance, balance_on)
        tx, tz = self.target
        tz = list(tz)
        cp, cr = math.radians(self.corr[0]), math.radians(self.corr[1])
        half_l, half_w = self.model.length_mm / 2, self.model.track_mm / 2
        for leg in LEGS:
            up = 0.25 if self.support >> leg & 1 else -0.25
            self.share[leg] = min(1.0, max(0.0, self.share[leg] + up))
            end = -1.0 if FRONT[leg] else 1.0
            side = -1.0 if LEFT[leg] else 1.0
            tz[leg] += self.share[leg] * (end * half_l * math.sin(cp) + side * half_w * math.sin(cr))
        self._slew(tx, tz)
        self.physics.command(self.x, self.z)
        if self.state == "settle" and self.support == 0xF and all(
                abs(self.x[i] - tx[i]) < SETTLED_MM and abs(self.z[i] - tz[i]) < SETTLED_MM for i in LEGS):
            happened = ("settled", 0)
        return happened

    def _slew(self, tx, tz) -> None:
        m = self.model
        lim = math.radians(m.slew_deg_per_tick)
        for leg in LEGS:
            a1, a2 = m.ik(leg, self.x[leg], self.z[leg])
            b1, b2 = m.ik(leg, tx[leg], tz[leg])
            n1 = a1 + max(-lim, min(lim, b1 - a1))
            n2 = a2 + max(-lim, min(lim, b2 - a2))
            if (n1, n2) == (b1, b2) and m.reachable(tx[leg], tz[leg]):
                self.x[leg], self.z[leg] = tx[leg], tz[leg]
            else:
                self.x[leg], self.z[leg] = m.fk(n1, n2)

    def _balance(self, measured, rates, bal: dict, on: bool) -> None:
        """balance.h's step on both axes (pitch, roll)."""
        if self.support & 0xF != 0xF:  # any foot up: held
            self._hold = BAL_HOLD_TICKS + 1
        if self._hold:
            self._hold -= 1
            return
        kp = bal["kp_q8"] / 256.0
        kd = bal["kd_ms"] / 1000.0
        db = float(bal["deadband_deg"])
        mx = float(bal["max_corr_deg"])
        for i in (0, 1):
            if on:
                # the geometric loop (body.cpp): the ground slope, the correction's own effect removed
                e = measured[i] - self.corr[i] - self.ref[i]
                e = 0.0 if abs(e) <= db else e - math.copysign(db, e)
                u = kp * e + kd * rates[i]
                target = max(-mx, min(mx, -u))
            else:
                target = 0.0
            self._lp[i] += (target - self._lp[i]) / 4.0
            self.corr[i] += max(-1.0, min(1.0, self._lp[i] - self.corr[i]))
            self.corr[i] = max(-mx, min(mx, self.corr[i]))
