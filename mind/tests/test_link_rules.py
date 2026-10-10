"""Link rules settled in the P0 integration round: query replies, liveness
gating of every autonomous frame, epoch changes on any event, random seq."""

import time

import pytest

from kilo.link import consts
from kilo.link.codec import KiloCodec
from kilo.link.messages import KiloMessages
from kilo.link.serial_link import CommandError, LinkConfig, SerialLink
from kilo.link.transport import loopback_pair

from conftest import FAST, Peer, keep_alive


def test_query_error_ack_fails_without_retry(make_link):
    link, peer = make_link()

    def h(p, name, f):
        if name == "PARAM_GET":
            p.ack(f.seq, f.type, consts.BAD_ARG)
    peer.handler = h
    with pytest.raises(CommandError) as ei:
        link.request("PARAM_GET", {"id": 99})
    assert ei.value.status_name == "BAD_ARG"
    assert len(peer.of("PARAM_GET")) == 1


def test_connect_hello_req_is_liveness_gated():
    a, b = loopback_pair()
    peer = Peer(b)
    link = SerialLink(lambda: a, codec=KiloCodec(), messages=KiloMessages(), config=LinkConfig(**FAST))
    link.start()  # never marked alive
    try:
        assert link.wait_until(lambda: link.state.connected, 1, alive=False)
        time.sleep(0.3)
        assert peer.of("HELLO_REQ") == [] and peer.of("HEARTBEAT") == []
        link.mark_alive()
        assert peer.wait_for(lambda: peer.of("HELLO_REQ"), 0.5)
    finally:
        link.stop()
        peer.stop()


def test_any_event_with_new_epoch_clears_older_goals(make_link):
    link, peer = make_link()
    cleared = []
    link.subscribe("goals_cleared", lambda r, n: cleared.append((r, n)))
    peer.telemetry(epoch=4)
    link.wait_until(lambda: link.state.epoch == 4, 1)
    link.set_goal("SET_POSTURE", posture=3, transition_ticks=0, flags=0)
    peer.event("LINK_RESTORED", epoch=5)  # not a "clearing" event, but the epoch moved
    assert link.wait_until(lambda: bool(cleared), 1)
    assert cleared[0][1] == ["SET_POSTURE"] and link.state.epoch == 5
    # a goal set under the new epoch survives a same-epoch event
    link.set_goal("SET_POSTURE", posture=2, transition_ticks=0, flags=0)
    peer.event("EEPROM_SAVED", epoch=5)
    keep_alive(link, 0.1)
    assert "SET_POSTURE" in link.goals() and link.goals()["SET_POSTURE"].epoch == 5


def test_tx_seq_starts_random():
    starts = {SerialLink(lambda: None)._seq for _ in range(8)}
    assert len(starts) > 1


def test_goal_refused_reasons_include_new_codes():
    assert consts.REFUSE_REASON[4] == "UNKNOWN_TYPE"
    assert consts.REFUSE_REASON[11] == "UNCALIBRATED"
    assert consts.ACK_STATUS[11] == "UNCALIBRATED"
    assert consts.OWNERS[4] == "PI_HOLD"
