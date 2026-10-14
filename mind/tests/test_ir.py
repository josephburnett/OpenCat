"""The IR remote on the mind: the key table (against the legacy src/infrared.h),
the --remote key map, its RemoteControl actions on a mock client, the
renderings, and the daemon acting on EVENT IR_KEY from the virtual body."""

import re
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from kilo import ir
from kilo.body.timeline import Timeline
from kilo.cli import render_monitor  # noqa: F401  (import check: the monitor renders IR_KEY)
from kilo.cli import _event_text
from kilo.link import consts

from test_daemon import remote, start_daemon, vbody_pty, sockdir, wait_for  # noqa: F401  (fixtures)

LEGACY = Path(__file__).resolve().parents[2] / "src" / "infrared.h"


def test_key_table_matches_the_legacy_firmware():
    src = LEGACY.read_text()
    full = re.findall(r'case 0x([0-9A-F]{6}): /\*PTLF\(" ([^"]+)"\)', src)
    short = re.findall(r'case 0x([0-9A-F]{2}): /\*PTLF\(" ([^"]+)"\)', src)
    assert len(full) == len(short) == len(ir.KEYS) == 21
    for (code, label), (cmd, label2) in zip(full, short):
        v = int(code, 16)  # 0x00FFA25D: address 0x00, ~0xFF, command, ~command
        assert v >> 16 == 0xFF and (v >> 8) & 0xFF == int(cmd, 16) and ((v >> 8) ^ v) & 0xFF == 0xFF
        assert label == label2
        assert ir.BY_LABEL[label].command == int(cmd, 16)
    assert len(ir.BY_COMMAND) == 21 and ir.BY_COMMAND[0xA2].label == "CH-"


def test_describe():
    assert ir.describe(0x18) == "2 (0x18, sit)"
    assert ir.describe(0x118) == "2 (0x18, sit) repeat"
    assert ir.describe(0xA2) == "CH- (0xA2, rest)"
    assert ir.describe(0x45) == "? (0x45)"
    assert ir.split(0x1A2) == (0xA2, True)
    ev = consts.EV["IR_KEY"]
    assert ev == 18 and _event_text(ev, 0x118) == "IR_KEY 2 (0x18, sit) repeat"
    assert _event_text(consts.EV["FELL"], -1) == "FELL(-1)"


def test_timeline_renders_ir_key():
    lines = []
    tl = Timeline.__new__(Timeline)
    tl.say = lambda text, now=None: lines.append(text)
    tl._on_event(SimpleNamespace(code=consts.EV["IR_KEY"], arg=0x10, tick=5, epoch=2))
    assert lines == ["EVENT IR_KEY 4 (0x10, hi) tick=5 epoch=2"]


EXPECTED = {
    "CH-": ("posture", "rest"), ">>|": ("posture", "stand"), "-": ("posture", "limp"), "2": ("posture", "sit"),
    "3": ("trick", "stretch"), "4": ("trick", "hi"), "5": ("trick", "push_up"), "6": ("trick", "pee"),
    "7": ("trick", "wash_face"), "8": ("trick", "butt_up"), "CH+": ("balance_toggle", ""),
    "CH": ("walk", "forward"), "|<<": ("walk", "left"), ">||": ("walk", "right"), "+": ("walk", "backward"),
    "0": ("walk", "march"), "100+": ("gait_pattern", "crawl"), "200+": ("gait_pattern", "walk"),
    "1": ("gait_pattern", "trot"), "EQ": ("speed", "slower"), "9": ("speed", "faster"),
}


def test_key_map():
    assert set(ir.ACTIONS) == set(EXPECTED) == set(ir.BY_LABEL)
    for k in ir.KEYS:
        a = ir.action_for(k.command)
        assert (a.kind, a.name) == EXPECTED[k.label], k.label
        assert ir.action_for(k.command | ir.REPEAT).kind == "ignore"  # held keys do nothing
    assert ir.action_for(0x45).kind == "ignore"  # another remote
    from kilo.tricks import library

    for a in ir.ACTIONS.values():
        if a.kind == "trick":
            library.load(a.name)  # every mapped trick exists
        if a.kind == "posture":
            assert a.name in consts.POSTURES


class FakeClient:
    def __init__(self, mode="IDLE", balance=None):
        self.calls = []
        self.state = SimpleNamespace(mode=mode)
        goals = {} if balance is None else {"SET_BALANCE": SimpleNamespace(fields={"enable": balance})}
        self.goals = goals
        self.link = SimpleNamespace(goals=lambda: goals)

    def walk(self, **kw):
        self.calls.append(("walk", kw))
        self.goals["SET_GAIT"] = SimpleNamespace(fields={"run": 1})

    def stop_walk(self):
        self.calls.append(("stop_walk",))
        self.goals["SET_GAIT"] = SimpleNamespace(fields={"run": 0})

    def posture(self, name):
        self.calls.append(("posture", name))

    def stop_trick(self):
        self.calls.append(("stop_trick",))

    def start_trick(self, t):
        self.calls.append(("start_trick", t.name))

    def set_balance(self, on, **gains):
        self.calls.append(("set_balance", on))
        self.gains = gains


@pytest.mark.parametrize("mode,expected", [
    ("IDLE", [("posture", "sit")]),
    ("TRICK", [("stop_trick",), ("posture", "sit")]),
])
def test_remote_posture(mode, expected):
    c = FakeClient(mode)
    assert ir.RemoteControl(c).perform(ir.action_for(0x18)) == "posture sit"
    assert c.calls == expected


@pytest.mark.parametrize("mode,expected", [
    ("IDLE", [("start_trick", "hi")]),
    ("TRICK", [("stop_trick",), ("start_trick", "hi")]),
    ("LIMP", [("posture", "stand"), ("sleep", 1.0), ("start_trick", "hi")]),
])
def test_remote_trick(mode, expected):
    c = FakeClient(mode)
    rc = ir.RemoteControl(c, sleep=lambda s: c.calls.append(("sleep", s)))
    assert rc.perform(ir.action_for(0x10)) == "trick hi"
    assert c.calls == expected


def test_remote_walking():
    c = FakeClient()
    rc = ir.RemoteControl(c)
    key = lambda label: rc.perform(ir.action_for(ir.BY_LABEL[label].command))
    assert key("200+") == "pattern walk" and c.calls == []  # not walking: only stored
    assert key("CH") == "walk forward (walk)"
    assert c.calls[-1] == ("walk", {"pattern": "walk", "stride_mm": 30, "turn_mm": 0, "period_ms": 2000,
                                    "duty": 192})  # the default speed: the spec defaults
    assert key("1") == "walk forward (trot)"  # walking: re-sent with the new pattern
    assert c.calls[-1] == ("walk", {"pattern": "trot", "stride_mm": 25, "turn_mm": 0, "period_ms": 600,
                                    "duty": 192})
    key("+")
    assert c.calls[-1][1]["stride_mm"] == -25
    key("|<<")
    assert c.calls[-1][1] == {"pattern": "trot", "stride_mm": 0, "turn_mm": 20, "period_ms": 600, "duty": 192}
    key(">||")
    assert c.calls[-1][1]["turn_mm"] == -20
    key("0")
    assert c.calls[-1][1] == {"pattern": "trot", "stride_mm": 0, "turn_mm": 0, "period_ms": 600, "duty": 192}
    key("100+")
    assert c.calls[-1][1]["pattern"] == "crawl"
    n = len(c.calls)
    assert key("2") == "posture sit"  # a posture key stops the gait first
    assert c.calls[n:] == [("stop_walk",), ("posture", "sit")]
    assert key("1") == "pattern trot" and len(c.calls) == n + 2  # stopped: only stored
    key("CH")
    n = len(c.calls)
    key("4")  # a trick key stops it too
    assert c.calls[n:] == [("stop_walk",), ("start_trick", "hi")]


def test_remote_speed_keys():
    c = FakeClient()
    rc = ir.RemoteControl(c)
    key = lambda label: rc.perform(ir.action_for(ir.BY_LABEL[label].command))
    assert key("9") == "speed 1.4x (trot 78 mm/s)" and c.calls == []  # not walking: only stored
    assert key("CH") == "walk forward (trot)"
    assert c.calls[-1][1] == {"pattern": "trot", "stride_mm": 25, "turn_mm": 0, "period_ms": 471, "duty": 175}
    assert key("9") == "walk forward speed 2x (trot 90 mm/s)"  # walking: re-sent at the new speed
    assert c.calls[-1][1] == {"pattern": "trot", "stride_mm": 25, "turn_mm": 0, "period_ms": 427, "duty": 166}
    assert key("9") == "walk forward speed 2x (trot 90 mm/s)"  # the top step
    for _ in range(5):
        key("EQ")
    assert rc.speed_step == 0
    f = c.calls[-1][1]  # half the default speed: the same 150 ms swing, a longer stance
    assert (f["stride_mm"], f["period_ms"], f["duty"]) == (25, 1050, 219)
    assert key("200+") == "walk forward (walk)"  # the walk is at its longest stance: a shorter stride
    assert c.calls[-1][1] == {"pattern": "walk", "stride_mm": 15, "turn_mm": 0, "period_ms": 2000, "duty": 192}
    key("0")  # march: stride 0 at the speed's cadence
    assert c.calls[-1][1] == {"pattern": "walk", "stride_mm": 0, "turn_mm": 0, "period_ms": 2000, "duty": 192}
    n = len(c.calls)
    key("2")
    assert key("EQ") == "speed 0.5x (walk 10 mm/s)" and len(c.calls) == n + 2  # stopped: only stored


@pytest.mark.parametrize("balance,on", [(None, True), (0, True), (1, False)])
def test_remote_balance_toggle(balance, on):
    c = FakeClient(balance=balance)
    ir.RemoteControl(c).perform(ir.action_for(0xE2))
    assert c.calls == [("set_balance", on)] and c.gains == {"kp": 0.5}  # the firmware default Kp
    c = FakeClient(balance=balance)
    ir.RemoteControl(c, balance_kp=0.99).perform(ir.action_for(0xE2))  # kilo run --balance 0.99
    assert c.gains == {"kp": 0.99}


# ---- the daemon with --remote, keys injected by the virtual body -----------------------------

def test_daemon_remote_keys(start_daemon):  # noqa: F811
    d, vb, path, sock = start_daemon(remote=True)
    body = remote(sock)
    try:
        body.wait_ready(3)
        vb.ir_key(0x18)  # 2: sit
        assert wait_for(lambda: vb.posture == "SIT")
        time.sleep(1.5)  # past idle_limp_s (1 s): the remote's posture is held, no idle LIMP
        assert vb.posture == "SIT" and vb.wd_stage == 0
        assert body.conn.call("daemon")["remote"] == "on, last key 2 (0x18, sit)"
        vb.ir_key(0x02, repeat=True)  # a repeat does nothing
        vb.ir_key(0x30)  # 1: selects trot, nothing walks
        time.sleep(0.3)
        assert vb.posture == "SIT"
        vb.ir_key(0xE2)  # CH+: balance on, then off
        assert wait_for(lambda: vb.balance["enable"] == 1)
        vb.ir_key(0xE2)
        assert wait_for(lambda: vb.balance["enable"] == 0)
        vb.ir_key(0x10)  # 4: trick hi (uploaded, played)
        assert wait_for(lambda: vb.trick.running, 3)
        vb.ir_key(0xA2)  # CH-: rest stops the trick first
        assert wait_for(lambda: vb.posture == "REST" and not vb.trick.running, 3)
        # a client's hold wins over the remote
        body.posture("stand")
        assert wait_for(lambda: vb.posture == "STAND")
        vb.ir_key(0x18)
        body.spin(0.4)
        assert vb.posture == "STAND"
    finally:
        body.close()


def test_daemon_remote_toggles_the_default_balance(start_daemon):  # noqa: F811
    d, vb, path, sock = start_daemon(remote=True, balance_kp=0.99)
    body = remote(sock)
    try:
        body.wait_ready(3)
        assert wait_for(lambda: vb.balance["enable"] == 1 and vb.balance["kp_q8"] == 253)
        vb.ir_key(0xE2)  # CH+: off (stays off: the remote is not a client hold)
        assert wait_for(lambda: vb.balance["enable"] == 0)
        time.sleep(0.5)
        assert vb.balance["enable"] == 0
        vb.ir_key(0xE2)  # CH+: on again, with the daemon's default gains
        assert wait_for(lambda: vb.balance["enable"] == 1 and vb.balance["kp_q8"] == 253)
    finally:
        body.close()


def test_daemon_without_remote_ignores_keys(start_daemon):  # noqa: F811
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        assert vb.posture == "LIMP"
        vb.ir_key(0x18)
        time.sleep(0.4)
        assert vb.posture == "LIMP"
        assert body.conn.call("daemon")["remote"] == "off"
    finally:
        body.close()


def test_daemon_remote_walks(start_daemon):  # noqa: F811
    from kilo.protocol import kilo_protocol as P

    d, vb, path, sock = start_daemon(remote=True)
    body = remote(sock)
    try:
        body.wait_ready(3)
        vb.ir_key(0xB0)  # 200+: walk pattern
        vb.ir_key(0x62)  # CH: forward, from LIMP (the body moves to STANCE first)
        assert wait_for(lambda: vb.gait is not None and vb.gait.state == "run", 3)
        assert vb.gait.pattern == P.GaitPattern.WALK and vb.gait.msg.stride_mm == 30
        time.sleep(1.5)  # past idle_limp_s: no idle timeline over the remote's walk
        assert vb.gait is not None and vb.wd_stage == 0
        info = body.conn.call("daemon")
        assert info["gait"]["pattern"] == "walk" and info["gait"]["run"] == 1 and info["gait"]["mode"] == "GAIT"
        vb.ir_key(0x30)  # 1: trot, re-sent while walking (a stop then a start on the body)
        assert wait_for(lambda: vb.gait is not None and vb.gait.pattern == P.GaitPattern.TROT, 3)
        assert vb.gait_stops == 0
        vb.ir_key(0x30, repeat=True)  # a held key: nothing
        # a client's walk hold wins: keys are ignored
        body.walk(stride_mm=20)
        assert wait_for(lambda: vb.gait.msg.stride_mm == 20, 1)
        vb.ir_key(0x18)
        body.spin(0.4)
        assert vb.gait is not None and vb.gait.msg.stride_mm == 20
        body.release()  # the hold ends: the daemon latches the stop, then idle REST
        assert wait_for(lambda: vb.gait is None and vb.gait_stops == 1, 3)
        assert wait_for(lambda: vb.posture == "REST", 2)
        vb.ir_key(0x22)  # |<<: turn left in place
        assert wait_for(lambda: vb.gait is not None and vb.gait.msg.turn_mm == 20 and vb.gait.msg.stride_mm == 0, 3)
        vb.ir_key(0x02)  # >>|: stand stops the gait first
        assert wait_for(lambda: vb.gait is None and vb.posture == "STAND", 4)
        assert "SET_GAIT" not in d.link.goals()  # GAIT_STOPPED cleared the stop goal
    finally:
        body.close()
