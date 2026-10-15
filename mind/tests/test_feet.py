"""The virtual body's FEET contract (protocol 2.0, spec FEET; kilo.vbody.feet),
driven synchronously: HELLO 2.0, FEET never acked, the gating refusals, the
queue (tick matching, late and too-early drops, STATS feet_late), GAIT on
the first accepted FEET, the hold and the settle on an underrun, the ends
(SET_POSTURE, an epoch bump: GAIT_STOPPED 0; the settle: 1) and the new
TELEMETRY fields."""

from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody

EV = P.EventCode
R = P.GoalRefusedReason
M = P.MsgId
STANCE = [0, 0, 0, 0], [76, 76, 76, 76]


class Body:
    def __init__(self, **kw):
        self.vb = VirtualBody(None, **kw)
        self.vb.telem_div = 1
        self.vb.posture = "STANCE"
        self.now = 100.0
        self.seq = 0
        self.vb.take_sent()

    def send(self, name, **fields):
        cls = P.MESSAGES[M[name]]
        self.seq = (self.seq + 1) & 0xFF
        self.vb.process(C.build_frame(cls.ID, self.seq, cls(**fields).pack()), self.now)
        return [(t, P.decode_payload(t, p)) for t, _, p in self.vb.take_sent()]

    def feet(self, ahead=1, x=None, z=None, pitch=0, roll=0, support=0xF, epoch=None, tick=None):
        x = STANCE[0] if x is None else x
        z = STANCE[1] if z is None else z
        t = (self.vb.tick + ahead) & 0xFFFF if tick is None else tick
        return self.send("FEET", epoch=self.vb.epoch if epoch is None else epoch, tick=t, x_mm=x, z_mm=z,
                         pitch=pitch, roll=roll, support=support)

    def tick(self, n=1):
        out = []
        for _ in range(n):
            self.now += 0.02
            self.vb._tick(self.now)
            out += [(t, P.decode_payload(t, p)) for t, _, p in self.vb.take_sent()]
        return out


def events(out):
    return [(m.code, m.arg) for t, m in out if t == M.EVENT]


def telemetry(out):
    return [m for t, m in out if t == M.TELEMETRY]


def test_hello_is_protocol_2_and_feet_are_never_acked():
    vb = VirtualBody(None)
    vb._boot()
    hello = [P.decode_payload(t, p) for t, _, p in vb.take_sent() if t == M.HELLO][0]
    assert (hello.proto_major, hello.proto_minor) == (2, 0)
    b = Body()
    out = b.feet(3)
    assert out == []  # no ACK, no event
    assert b.vb.feet is not None and b.vb.mode == P.Mode.GAIT


def test_feet_gating():
    b = Body()
    assert events(b.feet(epoch=9)) == [(EV.GOAL_REFUSED, M.FEET << 8 | R.STALE)]
    assert b.vb.feet is None
    assert b.feet(epoch=9) == []  # rate-limited: one per type per second
    b.now += 1.1
    b.vb.batt_level = P.BatteryLevel.LOW
    assert events(b.feet()) == [(EV.GOAL_REFUSED, M.FEET << 8 | R.LOW_BATT)]
    u = Body(config_state=P.ConfigState.DEFAULTED)
    assert events(u.feet()) == [(EV.GOAL_REFUSED, M.FEET << 8 | R.UNCALIBRATED)]
    f = Body()
    f.vb.trigger_fall(1)
    f.vb.take_sent()
    assert events(f.feet()) == [(EV.GOAL_REFUSED, M.FEET << 8 | R.FALLEN)]
    r = Body()
    assert events(r.feet(pitch=481)) == [(EV.GOAL_REFUSED, M.FEET << 8 | R.BAD_ARG)]  # +-30 deg (feet_check)
    r.now += 1.1
    assert events(r.feet(roll=-481)) == [(EV.GOAL_REFUSED, M.FEET << 8 | R.BAD_ARG)]
    assert r.feet(pitch=480, roll=-480) == [] and r.vb.feet is not None
    p1 = Body(p2=False)
    assert events(p1.feet()) == [(EV.GOAL_REFUSED, M.FEET << 8 | R.UNKNOWN_TYPE)]


def test_queue_tick_matching_and_late_drops():
    b = Body()
    b.feet(1, x=[1, 1, 1, 1])
    b.feet(2, x=[2, 2, 2, 2])
    b.feet(2, x=[3, 3, 3, 3])  # the same tick again: replaces it
    b.feet(0)                  # the tick just played: late
    b.feet(P.FEET_DEPTH + 1)   # too far ahead
    assert b.vb.feet_late == 2 and b.vb.feet.queued(b.vb.tick) == 2
    t = telemetry(b.tick())[0]
    assert t.mode == P.Mode.GAIT and t.feet_queued == 1 and t.support == 0xF
    assert b.vb.feet.x == [1, 1, 1, 1]
    b.tick()
    assert b.vb.feet.x == [3, 3, 3, 3]
    stats = [m for t, m in b.send("STATS_REQ") if t == M.STATS][0]
    assert stats.feet_late == 2 and stats.feet_underruns == 0
    # the u16 tick wraps
    w = Body()
    w.vb.tick = 0xFFFE
    w.feet(tick=0x0001)
    assert w.vb.feet is not None and w.vb.feet_late == 0


def test_the_slew_limits_a_jump():
    b = Body()
    for i in range(1, 8):
        b.feet(i, x=[40, 40, 40, 40], z=[60, 60, 60, 60])
    b.tick()
    x = b.vb.feet.x[0]
    assert 0 < x < 40  # one tick moves each joint at most 10 degrees
    b.tick(6)
    assert b.vb.feet.x[0] == 40  # ...and it gets there


def test_underrun_holds_then_settles():
    b = Body()
    for i in range(1, 4):
        b.feet(i, z=[76, 76, 76, 60], support=0b0111)  # LB in the air
    out = b.tick(3)
    assert events(out) == []
    out = b.tick()  # dry: the hold starts
    assert events(out) == [(EV.FEET_UNDERRUN, 4)] and b.vb.feet.state == "hold"  # 4 ticks since the FEET arrived
    held = list(b.vb.feet.cur[1])
    out = b.tick(P.FEET_HOLD_TICKS - 1)  # FEET_HOLD_TICKS dry ticks in all: the last pose, held
    assert events(out) == [] and b.vb.feet.state == "hold" and b.vb.feet.cur[1] == held == [76, 76, 76, 60]
    b.tick()
    assert b.vb.feet.state == "settle" and b.vb.feet.cur[1] == [76, 76, 76, 62]  # then LB comes down, 2 mm a tick
    out = b.tick(6)
    assert events(out) == [] and b.vb.feet.cur[1][3] == 74
    out = b.tick()  # within 2 mm: on the ground, every bit set: the stream ends that tick
    assert events(out) == [(EV.GAIT_STOPPED, 1)]
    assert b.vb.feet is None and b.vb.posture == "STANCE" and b.vb.mode == P.Mode.IDLE
    assert b.vb.gait_stops == [1] and b.vb.feet_underruns == 1
    # a FEET during the hold resumes the play
    r = Body()
    r.feet(1)
    r.tick(2)
    assert r.vb.feet.state == "hold"
    r.feet(1)
    r.tick()
    assert r.vb.feet.state == "play" and r.vb.feet is not None


def test_a_posture_or_an_epoch_bump_ends_gait():
    b = Body()
    b.feet(1)
    b.tick()
    out = b.send("SET_POSTURE", epoch=b.vb.epoch, posture=P.Posture.SIT)
    assert events(out) == [(EV.GAIT_STOPPED, 0)] and b.vb.feet is None and b.vb.posture == "SIT"
    e = Body()
    e.feet(1)
    e.feet(2)
    e.tick()
    e.vb.trigger_fall(-1)
    out = [(t, P.decode_payload(t, p)) for t, _, p in e.vb.take_sent()]
    assert (EV.GAIT_STOPPED, 0) in events(out) and e.vb.feet is None
    stopped = [m for t, m in out if t == M.EVENT and m.code == EV.GAIT_STOPPED][0]
    assert stopped.epoch == e.vb.epoch  # the post-bump epoch


def test_telemetry_carries_reference_correction_and_support():
    b = Body()
    b.vb.balance.update(enable=1)
    for i in range(1, 6):
        b.feet(i, z=[80, 72, 72, 80], roll=-5 * 16, support=0b0111)
    t = telemetry(b.tick())[0]
    assert (t.ref_pitch, t.ref_roll, t.support) == (0, -80, 0b0111)
    assert t.flags & P.TelemetryFlags.GAIT and t.flags & P.TelemetryFlags.BALANCE_ACTIVE
    assert t.owners & 0xF == P.Owner.GAIT
    t = telemetry(b.tick(3))[-1]
    assert t.feet_queued == 1
    # a foot up: the corrections are held (frozen), flag BALANCE_HELD
    assert (t.corr_pitch, t.corr_roll) == (0, 0) and t.flags & P.TelemetryFlags.BALANCE_HELD
    # four feet down, the body off its reference: it corrects (after a 2-tick hold-over)
    for i in range(1, 8):
        b.feet(i, roll=-5 * 16)
    four = telemetry(b.tick(7))
    assert four[-1].corr_roll != 0 and not four[-1].flags & P.TelemetryFlags.BALANCE_HELD
    # the balance off: the reference reads 0
    b.vb.balance.update(enable=0)
    b.feet(1, roll=-5 * 16)
    assert telemetry(b.tick())[0].ref_roll == 0


def test_a_lost_frame_holds_the_pose_without_an_underrun():
    """No FEET for this tick but later ones queued (feet.cpp: a lost frame):
    the pose is held, no event, and the hold's count does not run."""
    b = Body()
    b.feet(1, x=[5, 5, 5, 5])
    b.feet(3, x=[9, 9, 9, 9])
    b.tick()
    out = b.tick()  # tick 2: nothing for it, tick 3 queued
    assert events(out) == [] and b.vb.feet.state == "play" and b.vb.feet.cur[0] == [5, 5, 5, 5]
    assert b.vb.feet_underruns == 0
    b.tick()
    assert b.vb.feet.cur[0] == [9, 9, 9, 9]
    # in the hold, a FEET queued further ahead stops the count until it plays
    h = Body()
    h.feet(1)
    h.tick(2)  # tick 2: dry, the hold starts (one held tick)
    assert h.vb.feet.state == "hold" and h.vb.feet.held == 1
    h.feet(P.FEET_DEPTH)
    out = h.tick(P.FEET_DEPTH - 1)  # the queue is not empty: held, never settling
    assert h.vb.feet.state == "hold" and h.vb.feet.held == 1 and events(out) == []
    h.tick()
    assert h.vb.feet.state == "play"


def test_the_settle_puts_the_foot_on_the_plane_of_the_support_feet():
    """A leaning body on three feet: the lifted foot goes to the plane
    through them (feet.cpp ground_plane), not their mean depth."""
    b = Body()
    for i in range(1, 3):
        b.feet(i, z=[70, 80, 84, 60], support=0b0111)  # LB up; LF 70, RF 80, RB 84: the plane gives 74 under LB
    b.tick(2 + 1 + P.FEET_HOLD_TICKS)
    assert b.vb.feet.state == "settle"
    out = b.tick(8)
    assert events(out) == [(EV.GAIT_STOPPED, 1)]
    assert b.vb.xfade is not None and b.vb.xfade.cur[1] == [70, 80, 84, 74]  # the mean would be 78
    # two support feet: their mean depth
    t = Body()
    t.feet(1, z=[70, 60, 82, 60], support=0b0101)
    t.tick(2 + P.FEET_HOLD_TICKS)
    assert events(t.tick(10)) == [(EV.GAIT_STOPPED, 1)]
    assert t.vb.xfade.cur[1] == [70, 76, 82, 76]


def test_the_end_of_the_stream_crossfades_to_stance():
    """GAIT_STOPPED, then IDLE STANCE takes the legs over XFADE_TICKS from
    where they are (feet x 0, STANCE's height, no lean), on the physics."""
    b = Body()
    for i in range(1, 6):
        b.feet(i, x=[20, 20, -20, -20], z=[70, 82, 82, 70])
    b.tick(5)
    out = b.send("SET_POSTURE", epoch=b.vb.epoch, posture=P.Posture.STANCE)
    assert events(out) == [(EV.GAIT_STOPPED, 0)] and b.vb.mode == P.Mode.IDLE
    g = b.vb.xfade
    assert g is not None and g.x == [20, 20, -20, -20]
    xs = []
    for _ in range(10):
        b.tick()
        xs.append(list(g.x))
    assert all(abs(a) <= 20 for x in xs for a in x) and abs(xs[4][0]) > 1  # gradually, not at once
    assert all(abs(a) < 0.5 for a in xs[-1]) and all(abs(z - 76) < 0.5 for z in g.z)
    assert b.vb.xfade is None  # done: STANCE as before


def test_feet_from_limp_or_rest_power_the_legs_in_stance():
    for posture in ("LIMP", "REST"):
        b = Body()
        b.vb.posture = posture
        b.feet(1)
        t = telemetry(b.tick())[0]
        assert b.vb.posture == "STANCE" and t.mode == P.Mode.GAIT and t.flags & P.TelemetryFlags.SERVOS_POWERED


def test_a_reflex_ends_the_stream_at_once():
    b = Body()
    for i in range(1, 6):
        b.feet(i)
    b.tick()
    b.vb.wd_stage = 2  # the watchdog's REST: a reflex holds the joints
    out = b.tick()
    assert events(out) == [(EV.GAIT_STOPPED, 0)] and b.vb.feet is None and b.vb.xfade is None
