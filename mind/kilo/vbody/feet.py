"""The virtual body's FEET player: GAIT as the firmware plays the mind's
stream (protocol/spec.toml FEET, doc §3d, §3f; body/src/core/feet.cpp
feet_step and body.cpp body_tick), on kilo.vbody.physics.

- **Queue.** A FEET for a tick 1..``FEET_DEPTH`` after the last tick
  played is queued (a second one for the same tick replaces it); one for a
  tick already played, or further ahead, is dropped and counted (STATS
  feet_late). Until the first queued tick comes (``wait``) the legs stay in
  STANCE.
- **Play.** Each tick takes the FEET for that tick. The target is its feet
  plus the balance correction on the feet it marks as down; every joint
  moves at most ``slew_deg_per_tick`` toward it (the body's safety slew),
  so a late or missing tick never snaps.
- **A lost frame** (no FEET for this tick, later ones queued): the pose
  played last is held, nothing else happens.
- **Underrun** (no FEET for this tick and none queued): EVENT
  FEET_UNDERRUN (arg = ticks since the last FEET arrived; STATS
  feet_underruns) and the pose is held; the hold counts only ticks with the
  queue empty, and after ``FEET_HOLD_TICKS`` of them the body settles: every
  foot whose support bit is clear goes down (or up) ``SETTLE_MM`` per tick
  at its x to the ground, its bit set when it is there. The ground under a
  lifted foot is the plane through the three support feet (feet.cpp
  ground_plane, in its integers, kept in 30..95 mm); with fewer down or the
  three on one line their mean depth; with none STANCE's height. The tick the last
  bit is set the stream ends (``settled``: EVENT GAIT_STOPPED arg 1). A FEET
  for a later tick during the hold or the settle is played on its tick and
  the stream goes on.
- **The end of a stream** (settled, or a SET_POSTURE STANCE, or an epoch
  bump): IDLE STANCE takes the legs over ``XFADE_TICKS`` ticks (the
  arbiter's crossfade, from the output at the hand-over, in joint space, on
  a cosine) to the STANCE feet (x 0, STANCE's height, no lean), the safety
  slew after it (``handover``). A reflex takes the legs at once instead
  (the sim drops the player).
- **Balance** (doc §3f, as body/src/core/balance.h and the geometric loop
  of body.cpp): with the balance on (SET_BALANCE enable or the posture
  flag), per axis the ground slope s = measured - correction - reference,
  e = deadband(s), u = Kp e + Kd rate, the correction low-passes toward -u
  by 1/4 per tick, moves at most 1 deg per tick and is clamped; the
  reference is the FEET's pitch/roll. It is held (frozen) while any foot is
  up and for 2 ticks after all four are down (body.cpp: on three feet the
  corrections act through the feet a tipping body pivots on); with the
  balance off it decays. While the stream plays it acts through the support
  feet only, each leg's share ramping a quarter per tick as its support bit
  changes: z_i += share_i (end_i (Lb/2) sin(pitch corr) + side_i (W/2)
  sin(roll corr)) (end -1 front, +1 back; side -1 left, +1 right); in
  STANCE (waiting, the crossfade) through all four.
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
SETTLE_MM = 2.0  # a lifted foot's descent per tick when settling (FEET_SETTLE_MM)

PLAYING = ("play", "hold", "settle")


def ease_cos(i: int, n: int) -> float:
    """fx_ease_cos: 0..1 over i = 0..n, zero slope at both ends."""
    if i >= n:
        return 1.0
    return (1.0 - math.cos(math.pi * i / n)) / 2.0


class FeetPlayer:
    def __init__(self, tick: int, model: BodyModel, lag_ms: float | None = None):
        self.model = model
        self.queue: dict[int, Any] = {}     # unwrapped tick -> Feet
        self.state = "wait"                 # wait, play, hold, settle; xfade once the stream has ended
        self.held = 0                       # hold: ticks held with the queue empty
        self.stance_h = model.stance_height(P.GAIT_STANCE_H_256)
        self.x = [0.0] * 4                  # commanded feet (after the slew): STANCE's
        self.z = [self.stance_h] * 4
        self.cur = (list(self.x), list(self.z))  # the pose played (FEET's feet; the settle moves its z)
        self.physics = Physics(model, lag_ms=lag_ms, feet=(self.x, self.z))
        self.corr = [0.0, 0.0]              # pitch, roll correction (deg)
        self._lp = [0.0, 0.0]
        self._hold = 0
        self.support = 0xF
        self.ref = (0.0, 0.0)
        self.share = [1.0] * 4              # each leg's share of the correction, 0..1 in quarters
        self.rx_tick = tick                 # the body's tick when the last FEET arrived
        self._xsrc: list | None = None      # xfade: the joint angles at the hand-over
        self._xi = 0
        self._xn = 0

    @property
    def playing(self) -> bool:
        """The stream holds the legs (feet_playing)."""
        return self.state in PLAYING

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

    # ---- the end of the stream -------------------------------------------------------------------
    def handover(self, ticks: int) -> None:
        """The stream is over: IDLE STANCE crossfades the legs from where they are over ``ticks``."""
        m = self.model
        self.state = "xfade"
        self.queue.clear()
        self._xsrc = [m.ik(leg, self.x[leg], self.z[leg]) for leg in LEGS]
        self._xi, self._xn = 0, max(1, int(ticks))
        self.support = 0xF
        self.ref = (0.0, 0.0)

    @property
    def xfade_done(self) -> bool:
        return self.state == "xfade" and self._xi >= self._xn

    # ---- one tick ----------------------------------------------------------------------------
    def tick(self, now_tick: int, measured: tuple[float, float], rates: tuple[float, float], balance: dict,
             balance_on: bool) -> tuple[str | None, int]:
        """Play tick ``now_tick`` (feet_step, then the balance and the legs).
        Returns (what happened, arg): ("underrun", ticks since the last
        FEET), ("settled", 0), or (None, 0)."""
        happened: tuple[str | None, int] = (None, 0)
        if self.state != "xfade":
            for t in [t for t in self.queue if t < now_tick]:
                del self.queue[t]
            msg = self.queue.pop(now_tick, None)
            if msg is not None:
                self.state = "play"
                self.support = int(msg.support)
                self.ref = (msg.pitch / 16.0, msg.roll / 16.0)
                self.cur = ([float(v) for v in msg.x_mm], [float(v) for v in msg.z_mm])
            elif self.state == "play" and not self.queue:  # ran dry: hold
                self.state = "hold"
                self.held = 0
                happened = ("underrun", now_tick - self.rx_tick)
            # (play with later ticks queued: a lost frame, the pose holds)
            if self.state == "hold" and not self.queue:
                self.held += 1
                if self.held > FEET_HOLD_TICKS:
                    self.state = "settle"
            if self.state == "settle" and self._settle():
                happened = ("settled", 0)
        if self.playing:  # each leg's share of the correction: a quarter per tick toward its support bit
            for leg in LEGS:
                up = 0.25 if self.support >> leg & 1 else -0.25
                self.share[leg] = min(1.0, max(0.0, self.share[leg] + up))
        self._balance(measured, rates, balance, balance_on)
        cp, cr = math.radians(self.corr[0]), math.radians(self.corr[1])
        half_l, half_w = self.model.length_mm / 2, self.model.track_mm / 2
        if self.playing:
            tx, tz = list(self.cur[0]), list(self.cur[1])
            share = self.share
        else:  # STANCE (waiting for the first tick, or the crossfade after the stream): all four correct
            tx, tz = [0.0] * 4, [self.stance_h] * 4
            share = (1.0, 1.0, 1.0, 1.0)
        for leg in LEGS:
            end = -1.0 if FRONT[leg] else 1.0
            side = -1.0 if LEFT[leg] else 1.0
            tz[leg] += share[leg] * (end * half_l * math.sin(cp) + side * half_w * math.sin(cr))
        if self.state == "xfade" and self._xi < self._xn:  # the arbiter's crossfade, joint space
            self._xi += 1
            e = ease_cos(self._xi, self._xn)
            m = self.model
            for leg in LEGS:
                a1, a2 = self._xsrc[leg]
                b1, b2 = m.ik(leg, tx[leg], tz[leg])
                tx[leg], tz[leg] = m.fk(a1 + (b1 - a1) * e, a2 + (b2 - a2) * e)
        self._slew(tx, tz)
        self.physics.command(self.x, self.z)
        return happened

    def _foot_xy2(self, leg: int) -> tuple[int, int]:
        """feet.cpp foot_xy2: the foot in the body's plane in doubled mm (X
        forward from the centre, Y to the right)."""
        lb, w = int(round(self.model.length_mm)), int(round(self.model.track_mm))
        return (lb if FRONT[leg] else -lb) + 2 * int(self.cur[0][leg]), (-w if LEFT[leg] else w)

    def _ground_plane(self, leg: int, abc: list) -> int:
        """feet.cpp ground_plane: the depth (mm, 30..95) of the plane through
        the three support feet ``abc`` at foot ``leg``, in the firmware's
        integers; -1 when the three are on one line."""
        zs = self.cur[1]
        X, Y = [0] * 4, [0] * 4
        for k, lg in enumerate((*abc, leg)):
            X[k], Y[k] = self._foot_xy2(lg)
        z = [int(zs[lg]) for lg in abc]
        for k in (1, 2, 3):
            X[k] -= X[0]
            Y[k] -= Y[0]
        d = X[1] * Y[2] - Y[1] * X[2]
        if not d:
            return -1
        num = (X[3] * Y[2] - Y[3] * X[2]) * (z[1] - z[0]) + (X[1] * Y[3] - Y[1] * X[3]) * (z[2] - z[0])
        if d < 0:
            d, num = -d, -num
        q = min(127, (abs(num) + d // 2) // d)
        q = z[0] + (-q if num < 0 else q)
        return 30 if q < 30 else 95 if q > 95 else q

    def _settle(self) -> bool:
        """feet.cpp settle(): every foot off the ground goes toward the ground
        at its x by SETTLE_MM a tick, its bit set when it is there. True when
        all four are down. The ground: the plane through the support feet
        when three are down, else (or a degenerate triangle) their mean
        depth, STANCE's default height when none are down."""
        zs = self.cur[1]
        down = [leg for leg in LEGS if self.support >> leg & 1]
        n = len(down)
        mean = (int(sum(int(zs[leg]) for leg in down)) + n // 2) // n if n else int(self.stance_h)
        done = True
        for leg in LEGS:
            if self.support >> leg & 1:
                continue
            p = self._ground_plane(leg, down) if n == 3 else -1
            g = mean if p < 0 else p
            z = zs[leg]
            if z + SETTLE_MM < g:
                zs[leg] = z + SETTLE_MM
                done = False
            elif z > g + SETTLE_MM:
                zs[leg] = z - SETTLE_MM
                done = False
            else:
                zs[leg] = float(g)
                self.support |= 1 << leg
        return done

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
        """body.cpp's geometric loop and balance.h's step on both axes (pitch, roll)."""
        if self.playing and self.support & 0xF != 0xF:  # a foot up: held
            self._hold = BAL_HOLD_TICKS + 1
        elif self._hold:
            self._hold -= 1
        if on and self._hold:
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
