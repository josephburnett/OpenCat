"""The virtual body agrees with the real body logic, message by message (§6).

Every case is played twice from the same fresh state: through the body's
actual C dispatch (core/link.cpp and core/, built for the host by
body/tools/build_link_harness.sh) and through kilo.vbody. For each input frame
the replies are normalised (ACK seq/type/status, EVENT code/arg/epoch, the data
replies' meaningful fields) and compared as multisets: the C body queues
events until the tick's flush while the vbody writes them at once, so only
the order between an ACK and an event may differ.

Skipped when g++ is not available (or the body/ tree is absent).
"""

from __future__ import annotations

import json
import shutil
import subprocess
from collections import Counter
from pathlib import Path

import pytest

from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody

REPO = Path(__file__).resolve().parents[2]
BUILD_SCRIPT = REPO / "body" / "tools" / "build_link_harness.sh"
FRAMES = REPO / "protocol" / "vectors" / "frames.json"

pytestmark = pytest.mark.skipif(
    not BUILD_SCRIPT.exists() or shutil.which("g++") is None or shutil.which("gcc") is None,
    reason="needs body/ and a host C/C++ compiler")


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    out = tmp_path_factory.mktemp("harness") / "link_harness"
    subprocess.run(["bash", str(BUILD_SCRIPT), str(out)], check=True, capture_output=True, timeout=300)
    proc = subprocess.Popen([str(out)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    h = Harness(proc)
    h.p2 = h.implements_gait()
    yield h
    proc.stdin.close()
    proc.wait(5)


class Harness:
    def __init__(self, proc: subprocess.Popen):
        self.proc = proc
        self.p2 = True

    def implements_gait(self) -> bool:
        """Whether this firmware has the P2 goals: a body without them refuses
        SET_GAIT UNKNOWN_TYPE, and the vbody then plays the P1 body (p2=False)."""
        self.cmd("reset 0 0")
        out = self.cmd("rx " + wire("SET_GAIT", 1, epoch=0, run=0, pattern=1, period_ms=500, duty=140,
                                    stride_mm=0, turn_mm=0, step_h_mm=10, body_h_mm=80, flags=0).hex())
        unknown = (P.MsgId.SET_GAIT << 8) | P.GoalRefusedReason.UNKNOWN_TYPE
        return not any(t == P.MsgId.EVENT and P.decode_payload(t, p).code == P.EventCode.GOAL_REFUSED
                       and P.decode_payload(t, p).arg == unknown for t, p in out)

    def cmd(self, line: str) -> list[tuple[int, bytes]]:
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()
        out = []
        while True:
            ln = self.proc.stdout.readline().split()
            assert ln, "harness died"
            if ln[0] == "end":
                return out
            assert ln[0] == "tx", ln
            out.append((int(ln[1]), bytes.fromhex(ln[2]) if len(ln) > 2 else b""))


class VB:
    """A VirtualBody driven synchronously with a frozen clock."""

    def __init__(self, config_state: int, batt_level: int, p2: bool = True):
        self.now = 1000.0
        self.vb = VirtualBody(None, config_state=config_state, batt_level=batt_level,
                              clock=lambda: self.now, p2=p2)
        self.vb.take_sent()

    def rx(self, wire: bytes) -> list[tuple[int, bytes]]:
        self.vb.process(wire, self.now)
        return [(t, p) for t, _, p in self.vb.take_sent()]

    def save(self) -> list[tuple[int, bytes]]:
        self.now += 1.0  # the vbody's save completes after save_busy_s
        self.vb._housekeeping(self.now)
        return [(t, p) for t, _, p in self.vb.take_sent()]

    def advance(self, ticks: int) -> None:
        self.now += ticks * P.TICK_MS / 1000.0

    def tick(self, ticks: int) -> list[tuple[int, bytes]]:
        """Run ``ticks`` body ticks (the vbody's TELEMETRY is dropped: the C
        harness sends no streams)."""
        out = []
        for _ in range(ticks):
            # a hair over 20 ms, so float rounding never makes the vbody's
            # 0.5 s watchdog trip a tick later than the body's 25 ticks
            self.now += P.TICK_MS / 1000.0 + 1e-7
            self.vb._tick(self.now)
            out += [(t, p) for t, _, p in self.vb.take_sent() if t != P.MsgId.TELEMETRY]
        return out


def norm(frames: list[tuple[int, bytes]]) -> Counter:
    """What must agree, per reply type."""
    out = []
    for t, payload in frames:
        m = P.decode_payload(t, payload)
        name = P.MsgId(t).name
        if t == P.MsgId.ACK:
            out.append((name, m.seq, m.type, m.status))
        elif t == P.MsgId.EVENT:
            out.append((name, m.code, m.arg, m.epoch))
        elif t == P.MsgId.HELLO:
            out.append((name, m.proto_major, m.proto_minor, m.reset_cause, m.max_payload,
                        m.trick_capacity, m.joint_count, m.config_state, m.epoch))
        elif t == P.MsgId.PONG:
            out.append((name, m.token))
        elif t in (P.MsgId.CAL_VALUES, P.MsgId.PARAM_VALUE, P.MsgId.EEPROM_DATA, P.MsgId.PCA_COUNTS):
            out.append((name, json.dumps(P.to_dict(m), sort_keys=True)))
        else:  # STATS: counters differ by nature; the type must match
            out.append((name,))
    return Counter(out)


def wire(name: str, seq: int, **fields) -> bytes:
    cls = P.MESSAGES[P.MsgId[name]]
    return C.encode_message(P.from_dict(cls, fields), seq)


def raw(type_: int, seq: int, payload: bytes) -> bytes:
    return C.build_frame(type_, seq, payload)


def play(harness: Harness, steps: list, config_state: int = P.ConfigState.OK,
         batt_level: int = 0) -> list[tuple[int, bytes]]:
    """steps: wire bytes, or ("save",), ("advance", n) or ("tick", n).
    Returns everything the C body sent."""
    sent = []
    harness.cmd(f"reset {int(config_state)} {int(batt_level)}")
    vb = VB(config_state, batt_level, harness.p2)
    for i, step in enumerate(steps):
        if isinstance(step, tuple) and step[0] == "save":
            c, v = harness.cmd("save"), vb.save()
        elif isinstance(step, tuple) and step[0] == "advance":
            c, v = harness.cmd(f"advance {step[1]}"), (vb.advance(step[1]) or [])
        elif isinstance(step, tuple) and step[0] == "tick":
            c, v = harness.cmd(f"tick {step[1]}"), vb.tick(step[1])
        else:
            c, v = harness.cmd("rx " + step.hex()), vb.rx(step)
        assert norm(c) == norm(v), f"step {i} ({step!r}): body {sorted(norm(c))} != vbody {sorted(norm(v))}"
        sent += c
    return sent


def trick_dones(sent: list[tuple[int, bytes]]) -> list[tuple[int, int]]:
    """(reason, trick_id) of every EVENT TRICK_DONE."""
    out = []
    for t, p in sent:
        if t == P.MsgId.EVENT:
            m = P.decode_payload(t, p)
            if m.code == P.EventCode.TRICK_DONE:
                out.append(((m.arg & 0xFFFF) >> 8, m.arg & 0xFF))
    return out


def _pi_vectors() -> list[dict]:
    doc = json.loads(FRAMES.read_text())
    return [f for f in doc["frames"] if f["dir"] == "pi_to_body"]


@pytest.mark.parametrize("vec", _pi_vectors(), ids=lambda f: f["name"])
def test_every_p0_vector_agrees(harness, vec):
    play(harness, [bytes.fromhex(vec["wire"])])


def test_every_message_type_is_covered():
    types = {f["type"] for f in _pi_vectors()}
    assert types == {m.ID for m in P.MESSAGES.values() if m.DIR is P.Dir.PI_TO_BODY}


@pytest.mark.parametrize("vec", json.loads(FRAMES.read_text())["errors"], ids=lambda e: e["name"])
def test_error_vectors_agree(harness, vec):
    play(harness, [bytes.fromhex(vec["wire"])])


def test_dedup_and_busy(harness):
    play(harness, [
        wire("CAL_SAVE", 40), wire("CAL_SAVE", 40),       # executed once, re-acked OK
        wire("CAL_SAVE", 41), wire("CAL_SAVE", 41),       # EEPROM_BUSY, re-acked BUSY
        wire("CAL_SET", 42, joint=3, offset_a16=-40),     # BUSY while saving
        ("save",),                                        # EVENT EEPROM_SAVED
        wire("CAL_SET", 43, joint=3, offset_a16=-40),
        wire("CAL_GET", 44), wire("CAL_GET", 44),         # queries are answered again
        wire("CAL_REVERT", 45), wire("CAL_GET", 46),
        wire("SET_STREAM", 45, telem_div=2, joints_div=0, imu_raw_div=0),  # same seq, other type
        ("advance", 30), wire("SET_STREAM", 45, telem_div=2, joints_div=0, imu_raw_div=0),
    ])


def test_params(harness):
    p = P.ParamId
    play(harness, [
        wire("PARAM_GET", 1, id=p.WD_STOP_MS),
        wire("PARAM_SET", 2, id=p.WD_STOP_MS, value=600, persist=0),
        wire("PARAM_GET", 3, id=p.WD_STOP_MS),
        wire("PARAM_SET", 4, id=p.WD_STOP_MS, value=99, persist=0),          # below min
        wire("PARAM_SET", 5, id=len(P.PARAMS), value=1, persist=0),          # no such id
        wire("PARAM_GET", 6, id=len(P.PARAMS)),
        wire("PARAM_SET", 7, id=p.TELEM_DIV, value=5, persist=2),            # bad persist
        wire("PARAM_SET", 8, id=p.TELEM_DIV, value=5, persist=1),
        wire("PARAM_SET", 9, id=p.TELEM_DIV, value=6, persist=1),            # busy
        ("save",),
        wire("PARAM_GET", 10, id=p.TELEM_DIV),
    ])


def test_goals(harness):
    S = P.Posture
    play(harness, [
        wire("SET_POSTURE", 1, epoch=0, posture=S.STAND, transition_ticks=0, flags=1),
        wire("SET_POSTURE", 2, epoch=7, posture=S.SIT, transition_ticks=0, flags=0),     # STALE
        wire("SET_POSTURE", 3, epoch=7, posture=S.SIT, transition_ticks=0, flags=0),     # rate-limited
        wire("SET_POSTURE", 4, epoch=7, posture=S.REST, transition_ticks=0, flags=0),    # never stale
        wire("SET_POSTURE", 5, epoch=0, posture=S.STANCE, transition_ticks=0, flags=0),  # P1: BAD_ARG
        wire("SET_GAIT", 6, epoch=0, run=1, pattern=1, period_ms=500, duty=140, stride_mm=10,
             turn_mm=0, step_h_mm=10, body_h_mm=80, flags=0),                              # P1: UNKNOWN_TYPE
        wire("BEEP", 7, notes=[{"note": 12, "dur_10ms": 10}]),
        wire("HEARTBEAT", 8),
        raw(P.MsgId.SET_POSTURE, 9, b"\x00\x03\x00"),                                     # BAD_LEN
    ])


def test_balance_goal(harness):
    S = P.Posture
    play(harness, [
        wire("SET_BALANCE", 1, enable=1, kp_q8=200, kd_ms=40, deadband_deg=3, max_corr_deg=12),
        wire("SET_BALANCE", 2, enable=2, kp_q8=128, kd_ms=30, deadband_deg=2, max_corr_deg=15),   # BAD_ARG
        wire("SET_BALANCE", 3, enable=1, kp_q8=128, kd_ms=30, deadband_deg=2, max_corr_deg=31),   # rate-limited
        wire("SET_POSTURE", 5, epoch=0, posture=S.STAND, transition_ticks=0, flags=1),
        wire("SET_BALANCE", 6, enable=0, kp_q8=128, kd_ms=30, deadband_deg=2, max_corr_deg=15),
    ])
    play(harness, [
        wire("SET_BALANCE", 1, enable=0, kp_q8=128, kd_ms=30, deadband_deg=31, max_corr_deg=15),  # BAD_ARG
    ])
    play(harness, [
        raw(P.MsgId.SET_BALANCE, 1, b"\x01\x80\x1e\x02"),                                       # BAD_LEN
    ])


def test_uncalibrated_and_low_battery(harness):
    S = P.Posture
    play(harness, [
        wire("SET_POSTURE", 1, epoch=0, posture=S.STAND, transition_ticks=0, flags=0),   # UNCALIBRATED
        wire("SET_POSTURE", 2, epoch=0, posture=S.SIT, transition_ticks=0, flags=0),
        wire("HELLO_REQ", 3),
        wire("CAL_SAVE", 4), ("save",),
        wire("HELLO_REQ", 5),                                                             # config_state OK
        wire("SET_POSTURE", 6, epoch=0, posture=S.STAND, transition_ticks=0, flags=0),
    ], config_state=P.ConfigState.DEFAULTED)
    play(harness, [
        wire("SET_POSTURE", 1, epoch=0, posture=S.SIT, transition_ticks=0, flags=0),     # LOW_BATT
        wire("SET_POSTURE", 2, epoch=0, posture=S.REST, transition_ticks=0, flags=0),
    ], batt_level=P.BatteryLevel.LOW)


def test_misc_commands(harness):
    play(harness, [
        wire("PING", 1, token=0xBEEF), wire("PING", 1, token=0xBEEF),
        wire("EEPROM_READ", 2, addr=1000, len=24), wire("EEPROM_READ", 3, addr=1000, len=25),
        raw(P.MsgId.EEPROM_READ, 4, b"\x00\x00\x00"),                                    # len 0
        wire("PCA_READ", 5), wire("STATS_REQ", 6),
        wire("IMU_OFFSETS_SET", 7, accel_x=1, accel_y=2, accel_z=3, gyro_x=4, gyro_y=5, gyro_z=6, save=2),
        wire("IMU_OFFSETS_SET", 8, accel_x=1, accel_y=2, accel_z=3, gyro_x=4, gyro_y=5, gyro_z=6, save=1),
        ("save",),
        raw(0x7D, 9, b""), raw(0x90, 10, b"\x01"), raw(P.MsgId.PING, 11, b"\x01"),       # unknown, unknown, BAD_LEN
        wire("TRICK_STOP", 12, transition_ticks=5),
        wire("REBOOT", 13, magic=0x4C49),                                                 # BAD_ARG
        wire("REBOOT", 14, magic=P.REBOOT_MAGIC),                                         # ACK, HELLO, BOOT
        wire("HELLO_REQ", 15),
    ])


# ---- tricks (P1): upload rules, play gating, tick-exact TRICK_DONE ----------------
def _trick_msgs(trick, trick_id: int, seq: int) -> tuple[list[bytes], int]:
    out = []
    for m in trick.upload_messages(trick_id):
        out.append(C.encode_message(m, seq))
        seq += 1
    return out, seq


def _small_trick(repeats: int = 2):
    from kilo.tricks.model import Keyframe, Trick
    stand = {"lf_shoulder": 30, "rf_shoulder": 30, "rb_shoulder": -30, "lb_shoulder": -30,
             "lf_knee": 30, "rf_knee": 30, "rb_knee": -30, "lb_knee": -30, "head_pan": 0, "head_tilt": 0, "tail": 0}
    f = [Keyframe.from_joints({**stand, "head_pan": a}, dur=d, hold=h)
         for a, d, h in ((20, 3, 1), (-20, 2, 0), (10, 4, 2), (0, 1, 0))]
    return Trick(name="x", frames=tuple(f), loop_start=1, loop_end=2, repeats=repeats)


def _ticks_with_heartbeats(n: int, seq: int) -> tuple[list, int]:
    """n single ticks, a HEARTBEAT every 10 so the watchdog stays fed."""
    steps = []
    for i in range(n):
        if i % 10 == 9:
            steps.append(wire("HEARTBEAT", seq))
            seq += 1
        steps.append(("tick", 1))
    return steps, seq


@pytest.mark.parametrize("speed,mirror", [(64, 0), (96, 1), (16, 0)])
def test_trick_play_to_done(harness, speed, mirror):
    S = P.Posture
    t = _small_trick()
    up, seq = _trick_msgs(t, 5, 10)
    ticks, seq = _ticks_with_heartbeats(120 if speed >= 64 else 300, seq + 3)
    sent = play(harness, [
        wire("SET_POSTURE", 1, epoch=0, posture=S.STAND, transition_ticks=0, flags=0),
        ("tick", 5),
        *up,
        wire("TRICK_COMMIT", seq, trick_id=5, crc16=t.crc16()),            # the cache check: OK again
        wire("TRICK_PLAY", seq + 1, epoch=0, trick_id=5, speed_q6=speed, flags=mirror),
        wire("TRICK_PLAY", seq + 2, epoch=0, trick_id=5, speed_q6=speed, flags=mirror),  # BUSY
        *ticks,                                                             # TRICK_DONE(DONE) on the same tick
    ])
    assert trick_dones(sent) == [(P.TrickDoneReason.DONE, 5)]


def test_trick_upload_errors(harness):
    S = P.Posture
    t = _small_trick()
    up, seq = _trick_msgs(t, 6, 20)
    begin, frames, commit = up[0], up[1:-1], up[-1]
    sent = play(harness, [
        frames[0],                                                          # NO_TRICK: no header
        wire("TRICK_COMMIT", 1, trick_id=6, crc16=0),                       # NO_TRICK
        wire("TRICK_BEGIN", 2, trick_id=6, n_frames=P.TRICK_CAPACITY + 1, loop_start=0, loop_end=0, repeats=0,
             joint_mask=0xFF07, flags=0, end_posture=S.STAND, exp_roll=0, exp_pitch=0),   # BAD_ARG
        wire("TRICK_BEGIN", 3, trick_id=6, n_frames=2, loop_start=0, loop_end=0, repeats=0,
             joint_mask=0x0008, flags=0, end_posture=S.STAND, exp_roll=0, exp_pitch=0),   # BAD_ARG: joint 3
        wire("TRICK_BEGIN", 4, trick_id=6, n_frames=2, loop_start=0, loop_end=0, repeats=0,
             joint_mask=0x0007, flags=0, end_posture=S.LIMP, exp_roll=0, exp_pitch=0),    # BAD_ARG: end posture
        begin,
        wire("TRICK_FRAMES", 5, first=1, frames=[{"dur": 1, "hold": 0, "flags": 1, "ang": [0] * 11}]),  # out of order
        wire("TRICK_FRAMES", 6, first=0, frames=[{"dur": 0, "hold": 0, "flags": 1, "ang": [0] * 11}]),  # dur 0
        wire("TRICK_FRAMES", 7, first=0, frames=[{"dur": 1, "hold": 0, "flags": 3, "ang": [0] * 11}]),  # easing 3
        wire("TRICK_COMMIT", 8, trick_id=6, crc16=t.crc16()),               # BAD_ARG: frames missing
        *frames,
        wire("TRICK_COMMIT", 30, trick_id=6, crc16=t.crc16() ^ 1),          # CRC_MISMATCH
        wire("TRICK_PLAY", 31, epoch=0, trick_id=6, speed_q6=64, flags=0),  # NO_TRICK: not committed
        commit,
        wire("TRICK_PLAY", 32, epoch=0, trick_id=6, speed_q6=64, flags=0),  # BUSY: LIMP
        wire("SET_POSTURE", 33, epoch=0, posture=S.CALIB, transition_ticks=0, flags=0),
        wire("TRICK_PLAY", 34, epoch=0, trick_id=6, speed_q6=64, flags=0),  # BUSY: CALIB
        wire("SET_POSTURE", 35, epoch=0, posture=S.SIT, transition_ticks=0, flags=0),
        wire("TRICK_PLAY", 36, epoch=1, trick_id=6, speed_q6=64, flags=0),  # STALE
        wire("TRICK_PLAY", 37, epoch=0, trick_id=7, speed_q6=64, flags=0),  # NO_TRICK
        wire("TRICK_PLAY", 38, epoch=0, trick_id=6, speed_q6=15, flags=0),  # BAD_ARG
        wire("TRICK_PLAY", 39, epoch=0, trick_id=6, speed_q6=64, flags=2),  # BAD_ARG
        wire("TRICK_PLAY", 40, epoch=0, trick_id=6, speed_q6=64, flags=0),  # OK
        wire("TRICK_BEGIN", 41, trick_id=6, n_frames=2, loop_start=0, loop_end=0, repeats=0,
             joint_mask=0x0007, flags=0, end_posture=S.STAND, exp_roll=0, exp_pitch=0),   # BUSY
        ("tick", 3),
        wire("TRICK_STOP", 42, transition_ticks=5),                         # OK, TRICK_DONE(STOPPED)
        wire("TRICK_STOP", 43, transition_ticks=5),                         # OK, no event
        wire("TRICK_PLAY", 44, epoch=0, trick_id=6, speed_q6=64, flags=0),
        ("tick", 2),
        wire("SET_POSTURE", 45, epoch=0, posture=S.LIMP, transition_ticks=0, flags=0),  # TRICK_DONE(STOPPED)
        wire("TRICK_PLAY", 46, epoch=0, trick_id=6, speed_q6=64, flags=0),  # BUSY: LIMP
    ])
    assert trick_dones(sent) == [(P.TrickDoneReason.STOPPED, 6)] * 2
    # Uncalibrated and low battery.
    play(harness, [
        wire("SET_POSTURE", 1, epoch=0, posture=S.SIT, transition_ticks=0, flags=0),
        *_trick_msgs(t, 6, 2)[0],
        wire("TRICK_PLAY", 9, epoch=0, trick_id=6, speed_q6=64, flags=0),   # UNCALIBRATED
    ], config_state=P.ConfigState.DEFAULTED)
    play(harness, [
        wire("SET_POSTURE", 1, epoch=0, posture=S.REST, transition_ticks=0, flags=0),
        *_trick_msgs(t, 6, 2)[0],
        wire("TRICK_PLAY", 9, epoch=0, trick_id=6, speed_q6=64, flags=0),   # REFUSED_LOW_BATT
    ], batt_level=P.BatteryLevel.LOW)


def test_trick_preempted_by_watchdog(harness):
    S = P.Posture
    t = _small_trick(repeats=255)
    up, seq = _trick_msgs(t, 8, 2)
    sent = play(harness, [
        wire("SET_POSTURE", 1, epoch=0, posture=S.STAND, transition_ticks=0, flags=0),
        *up,
        wire("TRICK_PLAY", seq, epoch=0, trick_id=8, speed_q6=64, flags=0),
        *[("tick", 1) for _ in range(30)],      # silent: WATCHDOG + TRICK_DONE(PREEMPTED), epoch 1
        wire("TRICK_PLAY", seq + 1, epoch=0, trick_id=8, speed_q6=64, flags=0),  # LINK_RESTORED, STALE
    ])
    assert trick_dones(sent) == [(P.TrickDoneReason.PREEMPTED, 8)]


# ---- head (P1, docs §3c layer 1) ----------------------------------------------------------
def test_head_goal(harness):
    S = P.Posture
    hold = dict(mode=1, pan=30 * 16, tilt=-10 * 16, tail=-32768, max_speed_dps=120)
    release = dict(mode=0, pan=0, tilt=0, tail=0, max_speed_dps=0)
    ticks, seq = _ticks_with_heartbeats(130, 20)
    # (The harness never runs link_tick, so the 1/s GOAL_REFUSED window never
    # reopens: one refusal per SET_HEAD per play.)
    play(harness, [
        wire("SET_POSTURE", 1, epoch=0, posture=S.STAND, transition_ticks=0, flags=0),
        wire("SET_HEAD", 2, epoch=0, **hold),                              # held
        *ticks,                                                            # lapses after 2 s: no event
        wire("SET_HEAD", seq, epoch=9, **release),                         # release: never stale
        wire("SET_HEAD", seq + 1, epoch=0, **hold),
        *[("tick", 1) for _ in range(30)],                                 # silent: WATCHDOG releases it
        wire("SET_HEAD", seq + 2, epoch=0, **hold),                        # LINK_RESTORED, STALE
    ])
    play(harness, [wire("SET_HEAD", 1, epoch=0, **{**hold, "mode": 2})])  # BAD_ARG
    play(harness, [
        wire("SET_POSTURE", 1, epoch=0, posture=S.SIT, transition_ticks=0, flags=0),
        wire("SET_HEAD", 2, epoch=4, **hold),                              # STALE
        wire("SET_HEAD", 3, epoch=4, **release),
    ])
    play(harness, [
        wire("SET_HEAD", 1, epoch=0, **hold),                              # LOW_BATT
        wire("SET_HEAD", 2, epoch=0, **release),
    ], batt_level=P.BatteryLevel.LOW)


def test_gait_goal(harness):
    """P2 (§3d): refusals, the start, the stop's one-period ramp-down and
    GAIT_STOPPED on the same tick. Skipped against a firmware without the
    gait generator (until the P2 body is merged)."""
    if not harness.p2:
        pytest.skip("this firmware has no gait generator (SET_GAIT is UNKNOWN_TYPE)")
    S = P.Posture
    g = dict(pattern=1, period_ms=500, duty=140, stride_mm=20, turn_mm=0, step_h_mm=10, body_h_mm=80, flags=7)
    hb = lambda seq: wire("HEARTBEAT", seq)
    play(harness, [
        wire("SET_GAIT", 1, epoch=0, run=1, **{**g, "period_ms": 200}),   # BAD_ARG
        # The body's GOAL_REFUSED rate limit is one per goal type per second
        # of link_tick(), which the C harness never runs: the STALE check
        # therefore uses the other P2 goal type.
        wire("SET_BODY_POSE", 2, epoch=9, pitch=0, roll=0, dheight_mm=0, xshift_mm=0),  # STALE
        wire("SET_POSTURE", 3, epoch=0, posture=S.STANCE, transition_ticks=0, flags=0),
        ("tick", 20), hb(4),
        wire("SET_GAIT", 5, epoch=0, run=1, **g),
        ("tick", 20), hb(6), ("tick", 20), hb(7),
        wire("SET_GAIT", 8, epoch=0, run=0, **g),                          # ramp down over 25 ticks
        ("tick", 20), hb(9), ("tick", 20), hb(10),                         # GAIT_STOPPED in here
        wire("SET_GAIT", 11, epoch=9, run=0, **g),                         # a stop is never stale
    ])
