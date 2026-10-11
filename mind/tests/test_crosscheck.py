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
    yield Harness(proc)
    proc.stdin.close()
    proc.wait(5)


class Harness:
    def __init__(self, proc: subprocess.Popen):
        self.proc = proc

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

    def __init__(self, config_state: int, batt_level: int):
        self.now = 1000.0
        self.vb = VirtualBody(None, config_state=config_state, batt_level=batt_level,
                              clock=lambda: self.now)
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


def play(harness: Harness, steps: list, config_state: int = P.ConfigState.OK, batt_level: int = 0) -> None:
    """steps: wire bytes, or ("save",), or ("advance", n)."""
    harness.cmd(f"reset {int(config_state)} {int(batt_level)}")
    vb = VB(config_state, batt_level)
    for i, step in enumerate(steps):
        if isinstance(step, tuple) and step[0] == "save":
            c, v = harness.cmd("save"), vb.save()
        elif isinstance(step, tuple) and step[0] == "advance":
            c, v = harness.cmd(f"advance {step[1]}"), (vb.advance(step[1]) or [])
        else:
            c, v = harness.cmd("rx " + step.hex()), vb.rx(step)
        assert norm(c) == norm(v), f"step {i} ({step!r}): body {sorted(norm(c))} != vbody {sorted(norm(v))}"


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
        wire("SET_POSTURE", 5, epoch=0, posture=S.STANCE, transition_ticks=0, flags=0),  # BAD_ARG (P2)
        wire("SET_GAIT", 6, epoch=0, run=1, pattern=1, period_ms=500, duty=140, stride_mm=10,
             turn_mm=0, step_h_mm=10, body_h_mm=80, flags=0),                              # UNKNOWN_TYPE
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
