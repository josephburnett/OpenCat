"""Round trips against the virtual body: link + client + vbody, length-prefixed
framing over a loopback, and once with the stub codec over a real pty."""

import time

import pytest

from kilo.body.client import BodyClient
from kilo.link import consts
from kilo.link.serial_link import CommandError, LinkConfig, SerialLink
from kilo.link.transport import loopback_pair, open_port
from kilo.vbody.sim import VirtualBody, open_pty

from conftest import FAST, LenPrefixCodec, PlumbingMessages, keep_alive


@pytest.fixture
def rig():
    made = []

    def make(**vb_kw):
        a, b = loopback_pair()
        vb = VirtualBody(b, LenPrefixCodec(), PlumbingMessages(), **vb_kw).start()
        link = SerialLink(lambda: a, codec=LenPrefixCodec(), messages=PlumbingMessages(),
                          config=LinkConfig(**FAST), port_name="loopback")
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


def test_ready_telemetry_and_posture(rig):
    client, vb = rig()
    hello = client.wait_ready(2)
    assert hello.reset_cause == 0 and client.state.reset_cause == "COLD"
    keep_alive(client.link, 0.4)
    n = len(client.state.history)
    assert 6 <= n <= 14, n  # 25 Hz
    client.stand()
    assert client.link.wait_until(lambda: vb.posture == "STAND", 1)
    client.sit()
    assert client.link.wait_until(lambda: vb.posture == "SIT", 1)
    assert client.state.mode == "IDLE"


def test_commands_ack_and_eeprom_saved(rig):
    client, vb = rig()
    client.wait_ready(2)
    assert client.cal_save(wait_saved=1.0) is True
    assert client.cal_get() == [0] * 16
    client.cal_set(3, -40)
    assert ("CAL_SET" in [r[0] for r in vb.received])
    assert client.ping() < 0.5
    with pytest.raises(CommandError):
        client.stats()  # vbody cannot build STATS without the protocol module


def test_ack_dedup_against_vbody(rig):
    client, vb = rig()
    client.wait_ready(2)
    # drop the body's first ACK by having it ignore nothing but stall: emulate a lost
    # ACK by sending the same (seq, type) twice by hand
    link = client.link
    seq = 200
    for _ in range(2):
        link._txq.put((consts.TYPES["CAL_SAVE"], seq, b"", "CAL_SAVE"))
    keep_alive(link, 0.4)
    assert len(_events(client, "EEPROM_SAVED")) == 1  # executed once, acked twice


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
    client.link._goals.clear()
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
    vb.reboot(3)
    assert client.link.wait_until(lambda: bool(reboots), 1)
    assert client.state.reset_cause == "WARM_UNEXPECTED"


def test_pty_round_trip_with_stub_codec():
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
