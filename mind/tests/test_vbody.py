"""Link + client against the virtual body (real codec and messages over a
loopback), and the virtual body's P0 dispatch contract driven synchronously."""

import time

import pytest

from kilo.body.client import BodyClient
from kilo.link import consts
from kilo.link.serial_link import CommandError, LinkConfig, SerialLink
from kilo.link.transport import loopback_pair, open_port
from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody, open_pty

from conftest import FAST, keep_alive

EV = consts.EventCode
S = P.AckStatus
R = P.GoalRefusedReason


@pytest.fixture
def rig():
    made = []

    def make(**vb_kw):
        a, b = loopback_pair()
        vb = VirtualBody(b, **vb_kw).start()
        link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="loopback")
        link.mark_alive()
        link.start()
        client = BodyClient(link)
        made.append((link, vb))
        return client, vb

    yield make
    for link, vb in made:
        link.stop()
        vb.stop()


def _events(client, name):
    return [e for _, e in client.state.events if e.code == consts.EV[name]]


# ---- link + client against the running vbody ----------------------------------------

def test_ready_telemetry_and_posture(rig):
    client, vb = rig()
    hello = client.wait_ready(2)
    assert hello.reset_cause == P.ResetCause.COLD and client.state.reset_cause == "COLD"
    assert hello.proto_major == P.PROTO_MAJOR and hello.joint_count == P.JOINT_COUNT
    keep_alive(client.link, 0.4)
    n = len(client.state.history)
    assert 6 <= n <= 14, n  # 25 Hz
    client.stand()
    assert client.link.wait_until(lambda: vb.posture == "STAND", 1)
    client.sit()
    assert client.link.wait_until(lambda: vb.posture == "SIT", 1)
    assert client.link.wait_until(lambda: client.state.mode == "IDLE", 0.5)
    client.balance()
    assert client.link.wait_until(lambda: vb.posture == "STAND", 1)
    assert client.link.goals()["SET_POSTURE"].fields["flags"] == 1
    assert client.link.wait_until(lambda: client.state.snapshot()["balance"], 1)


def test_balance_goal_and_flag(rig):
    client, vb = rig()
    client.wait_ready(2)
    client.stand()
    assert client.link.wait_until(lambda: vb.posture == "STAND", 1)
    keep_alive(client.link, 0.2)
    assert not client.state.snapshot()["balance"]
    # SET_BALANCE enable turns it on without the posture flag; gains go live.
    f = client.set_balance(True, kp=0.75, kd_s=0.05, deadband_deg=3, clamp_deg=10)
    assert f == {"enable": 1, "kp_q8": 192, "kd_ms": 50, "deadband_deg": 3, "max_corr_deg": 10}
    assert client.link.wait_until(lambda: vb.balance == f, 1)
    assert client.link.wait_until(lambda: "balance_active" in client.state.flags, 1)
    # Not in REST (lying down), back on in SIT.
    client.rest()
    assert client.link.wait_until(lambda: "balance_active" not in client.state.flags, 1)
    client.sit()
    assert client.link.wait_until(lambda: "balance_active" in client.state.flags, 1)
    client.set_balance(False)
    assert client.link.wait_until(lambda: "balance_active" not in client.state.flags, 1)
    with pytest.raises(ValueError):
        client.set_balance(True, kp=1.0)
    with pytest.raises(ValueError):
        client.set_balance(True, clamp_deg=31)


def test_every_client_command(rig):
    client, vb = rig()
    client.wait_ready(2)
    assert client.ping(0xBEEF) < 0.5
    st = client.stats()
    assert st.dmp_rate_hz == 100 and client.state.last_stats is st
    assert client.cal_get() == [0] * 16
    client.cal_set(8, -40)
    assert client.cal_get()[8] == -40
    assert client.cal_save(wait_saved=1.0) is True
    client.cal_set(8, 16)
    client.cal_revert()
    assert client.cal_get()[8] == -40
    with pytest.raises(ValueError):
        client.cal_set(16, 0)
    pv = client.param_get("wd_stop_ms")
    assert (pv.id, pv.value, pv.min, pv.max, pv.default) == (0, 500, 100, 5000, 500)
    client.param_set("WD_STOP_MS", 600)
    assert client.param_get(0).value == 600
    with pytest.raises(ValueError):
        client.param_set("WD_STOP_MS", 99)  # client-side bounds
    n_saved = len(_events(client, "EEPROM_SAVED"))
    client.param_set("TELEM_DIV", 3, persist=True)
    assert client.link.wait_until(lambda: len(_events(client, "EEPROM_SAVED")) > n_saved, 1)
    assert [p.id for p in client.param_list()] == sorted(consts.PARAMS)
    with pytest.raises(CommandError) as ei:
        client.param_get(99)
    assert ei.value.status_name == "BAD_ARG"
    assert client.eeprom_read(0, 8) == b"\xff" * 8
    assert client.pca_counts() == [P.PCA_FULL_OFF_BIT] * 16
    client.imu_offsets_set([1, 2, 3, 4, 5, 6])
    assert vb.imu_offsets == [1, 2, 3, 4, 5, 6]
    client.set_stream(telem_div=5)
    assert vb.telem_div == 5
    reboots = []
    client.link.subscribe("reboot", lambda h, why: reboots.append(h))
    keep_alive(client.link, 0.4)  # past the HELLO solicitation window
    client.reboot()
    assert client.link.wait_until(lambda: bool(reboots), 1)
    assert client.state.reset_cause == "REQUESTED"


def test_ack_dedup_against_vbody(rig):
    client, vb = rig()
    client.wait_ready(2)
    link = client.link
    seq = 200
    for _ in range(2):  # a retry after a lost ACK: same seq and type
        link._txq.put((consts.TYPES["CAL_SAVE"], seq, b"", "CAL_SAVE"))
    keep_alive(link, 0.4)
    assert len(_events(client, "EEPROM_SAVED")) == 1  # executed once, acked twice
    assert link.state.stats.dup_acks + link.state.stats.unmatched_acks == 2


def test_fall_clears_goals_and_bumps_epoch(rig):
    client, vb = rig()
    client.wait_ready(2)
    client.stand()
    keep_alive(client.link, 0.1)
    e0 = client.state.epoch
    cleared = []
    client.link.subscribe("goals_cleared", lambda r, n: cleared.append(r))
    vb.trigger_fall(-1)
    assert client.link.wait_until(lambda: bool(cleared), 1)
    assert cleared[0] == "FELL"
    assert client.state.epoch == (e0 + 1) & 0xFF
    assert "SET_POSTURE" not in client.link.goals()
    assert client.link.wait_until(lambda: client.state.fallen, 0.5)
    # a goal from the old epoch is refused as STALE by the body and dropped by the link
    client.link.set_goal("SET_POSTURE", posture=3, transition_ticks=0, flags=0)
    client.link._goals["SET_POSTURE"].epoch = e0
    assert client.link.wait_until(lambda: "SET_POSTURE" not in client.link.goals(), 1)
    assert client.link.wait_until(lambda: _events(client, "RECOVERED"), 3)


def test_watchdog_when_mind_stalls(rig):
    client, vb = rig()
    client.wait_ready(2)
    client.stand()
    keep_alive(client.link, 0.2)
    assert vb.posture == "STAND"
    cleared = []
    client.link.subscribe("goals_cleared", lambda r, n: cleared.append(r))
    # stall: no mark_alive. Liveness window 0.3 s + body watchdog 0.5 s
    time.sleep(1.2)
    assert vb.wd_stage >= 1
    assert cleared == ["WATCHDOG"]
    assert client.state.mode == "LINK_LOST"
    # mind resumes: heartbeats flow, body reports LINK_RESTORED
    keep_alive(client.link, 0.3)
    assert _events(client, "LINK_RESTORED")
    assert "SET_POSTURE" not in client.link.goals()


def test_reboot_detected(rig):
    client, vb = rig()
    client.wait_ready(2)
    keep_alive(client.link, 0.4)
    reboots = []
    client.link.subscribe("reboot", lambda h, why: reboots.append(why))
    vb.reboot(P.ResetCause.WARM_UNEXPECTED)
    assert client.link.wait_until(lambda: bool(reboots), 1)
    assert client.state.reset_cause == "WARM_UNEXPECTED"


def test_pty_round_trip():
    transport, path, slave = open_pty()
    vb = VirtualBody(transport).start()
    link = SerialLink(lambda: open_port(path), config=LinkConfig(**FAST), port_name=path)
    link.mark_alive()
    link.start()
    try:
        client = BodyClient(link)
        client.wait_ready(2)
        assert client.cal_save(1.0)
        assert link.state.stats.decode_errors.total() == 0
    finally:
        link.stop()
        vb.stop()
        transport.close()
        import os
        os.close(slave)


# ---- the P0 dispatch contract, synchronously ---------------------------------------------

class Sync:
    """A VirtualBody without a transport or thread, driven by process()."""

    def __init__(self, **kw):
        self.vb = VirtualBody(None, **kw)
        self.vb.take_sent()
        self.now = 100.0
        self.seq = 0

    def send(self, name, seq=None, **fields):
        cls = P.MESSAGES[P.MsgId[name]]
        return self.raw(cls.ID, cls(**fields).pack() if fields or cls.MAX_LEN == 0 else cls().pack(), seq)

    def raw(self, type, payload, seq=None):
        seq = self.next_seq() if seq is None else seq
        self.vb.process(C.build_frame(type, seq, payload), self.now)
        return [(t, P.decode_payload(t, p)) for t, _, p in self.vb.take_sent()]

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        return self.seq


def acks(out):
    return [(m.type, m.status) for t, m in out if t == P.MsgId.ACK]


def events(out):
    return [(m.code, m.arg) for t, m in out if t == P.MsgId.EVENT]


def refused(type, reason):
    return (EV.GOAL_REFUSED, (type << 8) | reason)


def test_contract_a_watchdog_arms_and_link_restored():
    s = Sync()
    assert not s.vb.wd_armed
    s.raw(0x7D, b"")  # even an unknown type is a CRC-valid frame
    assert s.vb.wd_armed
    s.vb._tick(s.now + 0.6)
    assert s.vb.wd_stage == 1 and s.vb.epoch == 1
    assert (EV.WATCHDOG, 0) in events([(t, P.decode_payload(t, p)) for t, _, p in s.vb.take_sent()])
    s.now += 0.7
    assert events(s.send("HEARTBEAT")) == [(EV.LINK_RESTORED, 0)]
    assert s.vb.wd_stage == 0


def test_contract_b_unknown_type():
    s = Sync()
    assert acks(s.raw(0x7D, b"\x01")) == [(0x7D, S.UNKNOWN_TYPE)]


def test_contract_c_bad_length():
    s = Sync()
    assert acks(s.raw(P.MsgId.PING, b"\x01")) == [(P.MsgId.PING, S.BAD_LEN)]
    assert acks(s.raw(P.MsgId.CAL_SET, b"")) == [(P.MsgId.CAL_SET, S.BAD_LEN)]
    out = s.raw(P.MsgId.SET_POSTURE, b"\x00")
    assert acks(out) == [] and events(out) == [refused(P.MsgId.SET_POSTURE, R.BAD_LEN)]
    assert s.raw(P.MsgId.TELEMETRY, b"\x00") == []  # body->Pi type: ignored


def test_contract_d_goals():
    s = Sync()
    M = P.MsgId
    assert s.send("HEARTBEAT") == []
    assert s.send("SET_POSTURE", epoch=0, posture=P.Posture.STAND) == []
    assert s.vb.posture == "STAND"
    assert events(s.send("SET_POSTURE", epoch=0, posture=7)) == [refused(M.SET_POSTURE, R.BAD_ARG)]
    s.now += 1.1
    assert events(s.send("SET_POSTURE", epoch=0, posture=P.Posture.STANCE)) == [refused(M.SET_POSTURE, R.BAD_ARG)]
    s.now += 1.1
    assert events(s.send("SET_POSTURE", epoch=9, posture=P.Posture.SIT)) == [refused(M.SET_POSTURE, R.STALE)]
    # rate limit: one GOAL_REFUSED per type per second
    assert s.send("SET_POSTURE", epoch=9, posture=P.Posture.SIT) == []
    assert s.send("SET_POSTURE", epoch=9, posture=P.Posture.REST) == []  # REST is never stale
    assert s.vb.posture == "REST"
    for name, kw in (("SET_GAIT", {}), ("SET_BODY_POSE", {}), ("SET_EYES", {})):
        cls = P.MESSAGES[M[name]]
        assert events(s.raw(cls.ID, cls(**kw).pack())) == [refused(cls.ID, R.UNKNOWN_TYPE)], name
    assert s.send("BEEP", notes=[P.Note(note=12, dur_10ms=10)]) == []
    # SET_BALANCE (P1): accepted silently, BAD_ARG out of range, never stale.
    assert s.send("SET_BALANCE", enable=1, kp_q8=128, kd_ms=30, deadband_deg=2, max_corr_deg=15) == []
    assert s.vb.balance["enable"] == 1
    assert events(s.send("SET_BALANCE", enable=1, kp_q8=128, kd_ms=30, deadband_deg=2, max_corr_deg=31)) == [
        refused(M.SET_BALANCE, R.BAD_ARG)]
    assert s.vb.balance["max_corr_deg"] == 15
    s.now += 1.1
    s.vb.batt_level = P.BatteryLevel.LOW
    assert events(s.send("SET_POSTURE", epoch=0, posture=P.Posture.SIT)) == [refused(M.SET_POSTURE, R.LOW_BATT)]
    assert s.send("SET_POSTURE", epoch=0, posture=P.Posture.LIMP) == []
    u = Sync(config_state=P.ConfigState.DEFAULTED)
    assert events(u.send("SET_POSTURE", epoch=0, posture=P.Posture.STAND, flags=1)) == [
        refused(M.SET_POSTURE, R.UNCALIBRATED)]
    assert u.send("SET_POSTURE", epoch=0, posture=P.Posture.SIT) == []


def test_contract_d_head():
    """SET_HEAD (P1, docs §3c layer 1) in the body's order: BAD_ARG, release
    never stale, STALE, FALLEN, LOW_BATT; the hold lapses after HEAD_HOLD_MS
    and on an epoch bump; TELEMETRY HEAD_HELD and the PI_HOLD head owner."""
    s = Sync()
    M = P.MsgId
    s.send("SET_POSTURE", epoch=0, posture=P.Posture.STAND)
    hold = dict(mode=P.HeadMode.HOLD, pan=30 * 16, tilt=-10 * 16, tail=-32768, max_speed_dps=120)
    assert events(s.send("SET_HEAD", epoch=0, **{**hold, "mode": 2})) == [refused(M.SET_HEAD, R.BAD_ARG)]
    s.now += 1.1
    assert events(s.send("SET_HEAD", epoch=3, **hold)) == [refused(M.SET_HEAD, R.STALE)]
    assert s.vb.head_until is None
    assert s.send("SET_HEAD", epoch=0, **hold) == []
    assert s.vb.head_until is not None
    s.vb._send_telemetry(s.now)
    tel = [P.decode_payload(t, p) for t, _, p in s.vb.take_sent() if t == M.TELEMETRY]
    assert tel[-1].flags & P.TelemetryFlags.HEAD_HELD and tel[-1].owners >> 4 == P.Owner.PI_HOLD
    assert s.send("SET_HEAD", epoch=9, mode=P.HeadMode.RELEASE) == []  # never stale
    assert s.vb.head_until is None
    # Lapses without a refresh (the watchdog kept out of the way).
    s.vb.params[P.ParamId.WD_STOP_MS] = 5000
    s.vb.params[P.ParamId.WD_REST_MS] = 10000
    s.send("SET_HEAD", epoch=0, **hold)
    s.vb._tick(s.now + P.HEAD_HOLD_MS / 1000.0 - 0.05)
    assert s.vb.head_until is not None
    s.vb._tick(s.now + P.HEAD_HOLD_MS / 1000.0 + 0.01)
    assert s.vb.head_until is None
    # An epoch bump (a fall) releases it; while fallen SET_HEAD and postures
    # but LIMP are refused FALLEN, TRICK_PLAY REFUSED_FALLEN.
    s.now += 1.1
    s.send("SET_HEAD", epoch=0, **hold)
    s.vb.trigger_fall(1)
    s.vb.take_sent()
    assert s.vb.head_until is None and s.vb.epoch == 1
    assert events(s.send("SET_HEAD", epoch=1, **hold)) == [refused(M.SET_HEAD, R.FALLEN)]
    assert events(s.send("SET_POSTURE", epoch=1, posture=P.Posture.REST)) == [refused(M.SET_POSTURE, R.FALLEN)]
    assert s.send("SET_POSTURE", epoch=1, posture=P.Posture.LIMP) == []  # LIMP ends the reflex
    assert not s.vb.fallen and s.vb.posture == "LIMP"
    s.now += 1.1
    s.vb.batt_level = P.BatteryLevel.LOW
    assert events(s.send("SET_HEAD", epoch=1, **hold)) == [refused(M.SET_HEAD, R.LOW_BATT)]


def test_contract_e_queries_reply_only_never_deduped():
    s = Sync()
    for _ in range(2):
        out = s.send("PING", seq=5, token=0x1234)
        assert [t for t, _ in out] == [P.MsgId.PONG] and out[0][1].token == 0x1234
    (t, h), = s.send("HELLO_REQ")
    assert t == P.MsgId.HELLO and (h.proto_major, h.proto_minor) == (P.PROTO_MAJOR, P.PROTO_MINOR)
    assert (h.trick_capacity, h.joint_count, h.max_payload) == (P.TRICK_CAPACITY, P.JOINT_COUNT, P.MAX_PAYLOAD)
    assert [t for t, _ in s.send("STATS_REQ")] == [P.MsgId.STATS]
    assert [t for t, _ in s.send("PCA_READ")] == [P.MsgId.PCA_COUNTS]
    assert s.send("CAL_GET")[0][1].offset == [0] * 16
    pv = s.send("PARAM_GET", id=P.ParamId.WD_REST_MS)[0][1]
    assert (pv.value, pv.default) == (1500, 1500)
    assert acks(s.send("PARAM_GET", id=len(P.PARAMS))) == [(P.MsgId.PARAM_GET, S.BAD_ARG)]
    d = s.send("EEPROM_READ", addr=1000, len=24)[0][1]
    assert d.addr == 1000 and d.data == b"\xff" * 24
    for addr, n in ((1001, 24), (0, 0), (0, 57)):
        assert acks(s.send("EEPROM_READ", addr=addr, len=n)) == [(P.MsgId.EEPROM_READ, S.BAD_ARG)]


def test_contract_f_commands_and_dedup():
    s = Sync(save_busy_s=0.2)
    M = P.MsgId
    assert acks(s.send("SET_STREAM", telem_div=0, joints_div=0, imu_raw_div=0)) == [(M.SET_STREAM, S.OK)]
    assert acks(s.send("CAL_SET", joint=16, offset_a16=0)) == [(M.CAL_SET, S.BAD_ARG)]
    assert acks(s.send("CAL_SET", joint=8, offset_a16=401)) == [(M.CAL_SET, S.BAD_ARG)]
    assert acks(s.send("CAL_SET", seq=40, joint=8, offset_a16=-32)) == [(M.CAL_SET, S.OK)]
    # de-dup: same (seq, type) within 500 ms is re-acked, not executed again
    s.vb.cal[8] = 0
    assert acks(s.send("CAL_SET", seq=40, joint=8, offset_a16=-32)) == [(M.CAL_SET, S.OK)]
    assert s.vb.cal[8] == 0
    s.now += 0.6  # ...but not after the window
    assert acks(s.send("CAL_SET", seq=40, joint=8, offset_a16=-32)) == [(M.CAL_SET, S.OK)]
    assert s.vb.cal[8] == -32
    out = s.send("CAL_SAVE")
    assert acks(out) == [(M.CAL_SAVE, S.OK)] and events(out) == []
    assert acks(s.send("CAL_SAVE")) == [(M.CAL_SAVE, S.EEPROM_BUSY)]
    assert acks(s.send("CAL_SET", joint=8, offset_a16=0)) == [(M.CAL_SET, S.EEPROM_BUSY)]
    assert acks(s.send("CAL_SET", joint=99, offset_a16=0)) == [(M.CAL_SET, S.BAD_ARG)]  # BAD_ARG first
    assert acks(s.send("CAL_REVERT")) == [(M.CAL_REVERT, S.EEPROM_BUSY)]
    assert acks(s.send("IMU_OFFSETS_SET", save=2)) == [(M.IMU_OFFSETS_SET, S.BAD_ARG)]
    assert acks(s.send("IMU_OFFSETS_SET", save=0)) == [(M.IMU_OFFSETS_SET, S.EEPROM_BUSY)]
    assert acks(s.send("PARAM_SET", id=0, value=600, persist=1)) == [(M.PARAM_SET, S.EEPROM_BUSY)]
    assert acks(s.send("PARAM_SET", id=0, value=600, persist=0)) == [(M.PARAM_SET, S.OK)]
    s.now += 0.25
    out = s.send("CAL_REVERT")
    assert events(out) == [(EV.EEPROM_SAVED, 0)] and acks(out) == [(M.CAL_REVERT, S.OK)]
    assert acks(s.send("PARAM_SET", id=0, value=99, persist=0)) == [(M.PARAM_SET, S.BAD_ARG)]
    assert acks(s.send("PARAM_SET", id=99, value=0, persist=0)) == [(M.PARAM_SET, S.BAD_ARG)]
    assert acks(s.send("PARAM_SET", id=0, value=600, persist=2)) == [(M.PARAM_SET, S.BAD_ARG)]
    assert acks(s.send("PARAM_SET", id=0, value=700, persist=1)) == [(M.PARAM_SET, S.OK)]
    # P1 trick commands on an empty store (all-zero payloads); tests/test_trick_player.py has the rest
    expect = {"TRICK_BEGIN": S.BAD_ARG, "TRICK_COMMIT": S.NO_TRICK, "TRICK_PLAY": S.NO_TRICK, "TRICK_STOP": S.OK}
    for name, st in expect.items():
        assert acks(s.send(name, epoch=s.vb.epoch) if name == "TRICK_PLAY" else s.send(name)) == [(M[name], st)], name
    tf = P.TrickFrames(first=0, frames=[P.Keyframe(dur=1)])
    assert acks(s.raw(M.TRICK_FRAMES, tf.pack())) == [(M.TRICK_FRAMES, S.NO_TRICK)]
    assert acks(s.send("REBOOT", magic=1)) == [(M.REBOOT, S.BAD_ARG)]
    s.vb.epoch = 7
    out = s.send("REBOOT", magic=P.REBOOT_MAGIC)
    assert [t for t, _ in out] == [M.ACK, M.HELLO, M.EVENT]
    assert out[1][1].reset_cause == P.ResetCause.REQUESTED and out[1][1].epoch == 0
    assert events(out) == [(EV.BOOT, P.ResetCause.REQUESTED)]
    assert s.vb.posture == "LIMP" and not s.vb.wd_armed and s.vb.last_cmd is None


def test_contract_g_watchdog_stages_and_params():
    s = Sync()
    s.send("SET_POSTURE", epoch=0, posture=P.Posture.STAND)
    t0 = s.now
    s.vb._tick(t0 + 0.49)
    assert s.vb.wd_stage == 0
    s.vb._tick(t0 + 0.5)
    assert s.vb.wd_stage == 1 and s.vb.mode == P.Mode.LINK_LOST and s.vb.epoch == 1
    s.vb._tick(t0 + 1.5)
    assert s.vb.wd_stage == 2 and s.vb.posture == "REST"
    s.vb._tick(t0 + 11.5)
    assert s.vb.wd_stage == 3 and s.vb.posture == "LIMP"
    # already limp stays limp; WD_STOP_MS is live
    s2 = Sync()
    s2.send("PARAM_SET", id=P.ParamId.WD_STOP_MS, value=1000, persist=0)
    s2.vb._tick(s2.now + 0.6)
    assert s2.vb.wd_stage == 0
    s2.vb._tick(s2.now + 1.0)
    s2.vb._tick(s2.now + 1.5)
    assert s2.vb.wd_stage == 2 and s2.vb.posture == "LIMP"


def test_contract_h_boot_and_telemetry_divider():
    vb = VirtualBody(None, reset_cause=P.ResetCause.FLASHED)
    vb._boot()
    out = [(t, P.decode_payload(t, p)) for t, _, p in vb.take_sent()]
    assert [t for t, _ in out] == [P.MsgId.HELLO, P.MsgId.EVENT]
    assert out[0][1].reset_cause == P.ResetCause.FLASHED and events(out) == [(EV.BOOT, P.ResetCause.FLASHED)]
    for i in range(10):
        vb._tick(100.0 + i * 0.02)
    tel = [t for t, _, _ in vb.take_sent() if t == P.MsgId.TELEMETRY]
    assert len(tel) == 10 // P.PARAMS[P.ParamId.TELEM_DIV].default
