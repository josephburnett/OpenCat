"""Link rules settled in the P0 integration round: query replies, liveness
gating of every autonomous frame, epoch changes on any event, random seq."""

import pytest

from kilo.link import consts
from kilo.link.codec import KiloCodec
from kilo.link.messages import KiloMessages
from kilo.link.serial_link import CommandError, LinkConfig, SerialLink
from kilo.link.transport import loopback_pair
from kilo.stepclock import StepClock

from conftest import FAST, Peer, count


def test_query_error_ack_fails_without_retry(make_link):
    link, peer = make_link(stepped=True)  # no time passes: a retry could only come from the error ACK

    def h(p, name, f):
        if name == "PARAM_GET":
            p.ack(f.seq, f.type, consts.BAD_ARG)
    peer.handler = h
    with pytest.raises(CommandError) as ei:
        link.request("PARAM_GET", {"id": 99})
    assert ei.value.status_name == "BAD_ARG"
    assert len(peer.of("PARAM_GET")) == 1


def test_connect_hello_req_is_liveness_gated():
    clock = StepClock()
    a, b = loopback_pair()
    peer = Peer(b, clock=clock)
    link = SerialLink(lambda: a, codec=KiloCodec(), messages=KiloMessages(), config=LinkConfig(**FAST),
                      clock=clock)
    clock.attach(link.sync)
    clock.attach(peer.sync)
    link.start()  # never marked alive
    try:
        assert link.wait_until(lambda: link.state.connected, 1, alive=False)
        clock.run(1.0, 0.01)  # the HELLO_REQ poll (0.5 s) and heartbeat (0.1 s) periods pass, twice
        assert peer.frames == []
        t = clock()
        link.mark_alive()
        clock.step(0.001)
        assert [round(f[0] - t, 6) for f in peer.of("HELLO_REQ")] == [0.001]  # owed since the connect
        assert count(peer, "HEARTBEAT") == 0  # the HELLO_REQ broke the silence
    finally:
        link.stop()
        peer.stop()


def test_any_event_with_new_epoch_clears_older_goals(make_link):
    link, peer = make_link(stepped=True)
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
    assert link.wait_until(lambda: consts.EVENTS[link.state.events[-1][1].code] == "EEPROM_SAVED", 1)
    link.mark_alive()
    link.clock.step(0.1)
    assert "SET_POSTURE" in link.goals() and link.goals()["SET_POSTURE"].epoch == 5


def test_tx_seq_starts_random():
    starts = {SerialLink(lambda: None)._seq for _ in range(8)}
    assert len(starts) > 1


def test_goal_refused_reasons_include_new_codes():
    assert consts.REFUSE_REASON[4] == "UNKNOWN_TYPE"
    assert consts.REFUSE_REASON[11] == "UNCALIBRATED"
    assert consts.ACK_STATUS[11] == "UNCALIBRATED"
    assert consts.OWNERS[4] == "PI_HOLD"
