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
    "CH": ("gait", ""), "|<<": ("gait", ""), ">||": ("gait", ""), "+": ("gait", ""), "0": ("gait", ""),
    "100+": ("gait", ""), "200+": ("gait", ""), "1": ("gait", ""), "EQ": ("ignore", ""), "9": ("ignore", ""),
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
        self.link = SimpleNamespace(goals=lambda: goals)

    def posture(self, name):
        self.calls.append(("posture", name))

    def stop_trick(self):
        self.calls.append(("stop_trick",))

    def start_trick(self, t):
        self.calls.append(("start_trick", t.name))

    def set_balance(self, on):
        self.calls.append(("set_balance", on))


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


@pytest.mark.parametrize("balance,on", [(None, True), (0, True), (1, False)])
def test_remote_balance_toggle(balance, on):
    c = FakeClient(balance=balance)
    ir.RemoteControl(c).perform(ir.action_for(0xE2))
    assert c.calls == [("set_balance", on)]


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
        vb.ir_key(0x30)  # 1: a gait key, P2
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
