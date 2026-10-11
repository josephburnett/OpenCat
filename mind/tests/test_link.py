import asyncio
import threading
import time

import pytest

from kilo.link import consts
from kilo.link.serial_link import CommandError, LinkConfig, LinkTimeout, SerialLink
from kilo.link.transport import loopback_pair

from conftest import FAST, Peer, keep_alive
from kilo.link.codec import KiloCodec
from kilo.link.messages import KiloMessages

T = consts.TYPES


def test_connect_sends_hello_req(make_link):
    link, peer = make_link()
    assert peer.wait_for(lambda: peer.of("HELLO_REQ"))
    assert link.state.connected


def test_heartbeat_only_while_mind_alive(make_link):
    link, peer = make_link()
    keep_alive(link, 0.6)
    n = len(peer.of("HEARTBEAT"))
    assert n >= 4, n  # ~100 ms of silence each
    # mind stalls: within the liveness window (0.3 s here) heartbeats stop
    time.sleep(0.4)
    t0 = time.monotonic()
    time.sleep(0.5)
    assert peer.of("HEARTBEAT", since=t0) == []
    assert peer.of("HELLO_REQ", since=t0) == []  # no autonomous polling either
    # and resume
    t1 = time.monotonic()
    keep_alive(link, 0.35)
    assert len(peer.of("HEARTBEAT", since=t1)) >= 2


def test_goal_sent_immediately_and_refreshed_at_5hz(make_link):
    link, peer = make_link()
    peer.telemetry(epoch=5)
    assert link.wait_until(lambda: link.state.epoch == 5, 1)
    t0 = time.monotonic()
    link.set_goal("SET_POSTURE", posture=3, transition_ticks=0, flags=0)
    assert peer.wait_for(lambda: peer.of("SET_POSTURE"), 0.1)
    assert peer.of("SET_POSTURE")[0][0] - t0 < 0.05
    end = time.monotonic() + 1.0
    while time.monotonic() < end:  # a live body streams telemetry (a lost one gets no refresh)
        peer.telemetry(epoch=5)
        keep_alive(link, 0.04)
    sends = peer.of("SET_POSTURE")
    assert 5 <= len(sends) <= 8, len(sends)
    assert all(p[3][0] == 5 for p in sends)  # stamped with the epoch they were set under


def test_goal_refresh_stops_when_mind_stalls(make_link):
    link, peer = make_link()
    peer.telemetry(epoch=1)
    link.wait_until(lambda: link.state.epoch == 1, 1)
    link.set_goal("SET_EYES", led_mask=0x3F, r=1, g=2, b=3)
    keep_alive(link, 0.3)
    time.sleep(0.35)
    t0 = time.monotonic()
    time.sleep(0.5)
    assert peer.of("SET_EYES", since=t0) == []


def test_goal_waits_for_first_epoch(make_link):
    link, peer = make_link()
    link.set_goal("SET_POSTURE", posture=1, transition_ticks=0, flags=0)
    keep_alive(link, 0.25)
    assert peer.of("SET_POSTURE") == []
    peer.telemetry(epoch=9)
    assert peer.wait_for(lambda: peer.of("SET_POSTURE"), 0.5)
    assert peer.of("SET_POSTURE")[0][3][0] == 9
    assert "SET_POSTURE" in link.goals()


@pytest.mark.parametrize("ev", ["FELL", "WATCHDOG", "LOW_BATTERY"])
def test_clearing_events_drop_motion_goals(make_link, ev):
    link, peer = make_link()
    cleared = []
    events = []
    link.subscribe("goals_cleared", lambda reason, names: cleared.append((reason, names)))
    link.subscribe("event", lambda m: events.append(m))
    peer.telemetry(epoch=2)
    link.wait_until(lambda: link.state.epoch == 2, 1)
    link.set_goal("SET_POSTURE", posture=3, transition_ticks=0, flags=0)
    link.set_goal("SET_HEAD", mode=1, pan=0, tilt=0, tail=0, max_speed_dps=90)
    link.set_goal("SET_EYES", led_mask=0x3F, r=1, g=2, b=3)
    peer.event(ev, arg=1, epoch=3)
    assert link.wait_until(lambda: bool(cleared), 1)
    assert cleared[0][0] == ev
    assert sorted(cleared[0][1]) == ["SET_HEAD", "SET_POSTURE"]
    assert set(link.goals()) == {"SET_EYES"}  # not a motion goal
    assert link.state.epoch == 3
    # nothing from before the event is replayed
    t0 = time.monotonic()
    keep_alive(link, 0.5)
    assert peer.of("SET_POSTURE", since=t0) == []
    assert events and consts.EVENTS[events[-1].code] == ev


def test_telemetry_epoch_change_clears_old_goals(make_link):
    link, peer = make_link()
    peer.telemetry(epoch=4)
    link.wait_until(lambda: link.state.epoch == 4, 1)
    link.set_goal("SET_POSTURE", posture=3, transition_ticks=0, flags=0)
    peer.telemetry(tick=2, epoch=5)  # bump seen without the event (event lost)
    assert link.wait_until(lambda: "SET_POSTURE" not in link.goals(), 1)


def test_stale_refusal_drops_goal(make_link):
    link, peer = make_link()
    refused = []
    link.subscribe("refused", lambda g, r: refused.append((g, r)))
    peer.telemetry(epoch=4)
    link.wait_until(lambda: link.state.epoch == 4, 1)
    link.set_goal("SET_POSTURE", posture=3, transition_ticks=0, flags=0)
    peer.event("GOAL_REFUSED", arg=(T["SET_POSTURE"] << 8) | consts.STALE, epoch=5)
    assert link.wait_until(lambda: bool(refused), 1)
    assert refused == [("SET_POSTURE", "STALE")]
    assert "SET_POSTURE" not in link.goals()
    assert link.state.stats.goals_refused["SET_POSTURE:STALE"] == 1


def _acker(statuses=None, ignore=0, names=("CAL_SAVE",), delay=0.0, ack_all=False):
    """Handler: ignore the first `ignore` matching frames, then ACK."""
    count = {"n": 0}

    def h(peer, name, f):
        if name not in names:
            return
        count["n"] += 1
        if count["n"] <= ignore and not ack_all:
            return
        st = statuses[min(count["n"] - 1, len(statuses) - 1)] if statuses else 0

        def go():
            if delay:
                time.sleep(delay)
            peer.ack(f.seq, f.type, st)
        threading.Thread(target=go, daemon=True).start() if delay else go()
    return h


def test_request_ack_ok(make_link):
    link, peer = make_link()
    peer.handler = _acker()
    ack = link.request("CAL_SAVE")
    assert ack.status == 0
    assert link.state.stats.retries == 0


def test_request_retries_with_same_seq(make_link):
    link, peer = make_link()
    peer.handler = _acker(ignore=2)
    link.request("CAL_SAVE")
    sends = peer.of("CAL_SAVE")
    assert len(sends) == 3
    assert len({s[2] for s in sends}) == 1  # de-dup relies on the same seq
    assert link.state.stats.retries == 2


def test_request_timeout_after_three_retries(make_link):
    link, peer = make_link()
    t0 = time.monotonic()
    with pytest.raises(LinkTimeout):
        link.request("CAL_SAVE")
    dt = time.monotonic() - t0
    assert len(peer.of("CAL_SAVE")) == 4
    assert 0.18 <= dt < 0.5
    assert link.state.stats.timeouts == 1


def test_request_error_status(make_link):
    link, peer = make_link()
    peer.handler = _acker(statuses=[9])  # EEPROM_BUSY
    with pytest.raises(CommandError) as ei:
        link.request("CAL_SAVE")
    assert ei.value.status_name == "EEPROM_BUSY"


def test_late_ack_then_dedup_reack_is_counted(make_link):
    link, peer = make_link()
    # every copy is acked, 70 ms late: the first retry goes out, then both ACKs arrive
    peer.handler = _acker(delay=0.07, ack_all=True)
    link.request("CAL_SAVE")
    assert link.wait_until(lambda: link.state.stats.dup_acks >= 1, 0.5)
    assert link.state.stats.unmatched_acks == 0


def test_query_retry_uses_fresh_seq(make_link):
    link, peer = make_link()
    n = {"k": 0}

    def h(p, name, f):
        if name == "PING":
            n["k"] += 1
            if n["k"] >= 2:
                p.send("PONG", token=0x1234, tick=1)
    peer.handler = h
    msg = link.request("PING", {"token": 0x1234})
    assert msg.token == 0x1234
    seqs = [s[2] for s in peer.of("PING")]
    assert len(seqs) == 2 and seqs[0] != seqs[1]


def test_reconnect_after_failure():
    ends = []

    def factory():
        a, b = loopback_pair()
        ends.append((a, Peer(b)))
        return a

    link = SerialLink(factory, codec=KiloCodec(), messages=KiloMessages(),
                      config=LinkConfig(**FAST))
    link.mark_alive()
    link.start()
    try:
        assert link.wait_until(lambda: bool(ends), 1)
        assert ends[0][1].wait_for(lambda: ends[0][1].of("HELLO_REQ"))
        ends[0][1].telemetry(epoch=1)
        link.wait_until(lambda: link.state.epoch == 1, 1)
        link.set_goal("SET_EYES", led_mask=1, r=0, g=0, b=0)
        ends[0][0].fail()  # adapter unplugged
        assert link.wait_until(lambda: len(ends) >= 2 and link.state.connected, 2)
        peer2 = ends[-1][1]
        assert peer2.wait_for(lambda: peer2.of("HELLO_REQ"))
        assert peer2.wait_for(lambda: peer2.of("SET_EYES"))  # latched goals re-sent
        assert link.state.stats.reconnects == 1
        assert link.state.stats.disconnects == 1
    finally:
        link.stop()
        for _, p in ends:
            p.stop()


def test_open_failure_backoff():
    attempts = []
    a, b = loopback_pair()
    peer = Peer(b)

    def factory():
        attempts.append(time.monotonic())
        if len(attempts) < 3:
            raise OSError("no such device")
        return a

    link = SerialLink(factory, codec=KiloCodec(), messages=KiloMessages(),
                      config=LinkConfig(**FAST))
    link.start()
    try:
        assert link.wait_until(lambda: link.state.connected, 2)
        assert link.state.stats.open_failures == 2
        assert attempts[2] - attempts[1] > attempts[1] - attempts[0] - 0.005  # backoff grows
    finally:
        link.stop()
        peer.stop()


def test_reboot_detection_unsolicited_hello(make_link):
    link, peer = make_link()
    reboots = []
    link.subscribe("reboot", lambda h, why: reboots.append(why))
    peer.hello(epoch=0)
    assert link.wait_until(lambda: link.state.hello is not None, 1)
    peer.telemetry(epoch=0)
    link.wait_until(lambda: link.state.epoch == 0, 1)
    link.set_goal("SET_POSTURE", posture=3, transition_ticks=0, flags=0)
    keep_alive(link, 0.4)  # past the solicitation window
    peer.hello(epoch=0, reset_cause=3)
    assert link.wait_until(lambda: bool(reboots), 1)
    assert link.state.reboots == 1
    assert link.state.reset_cause == "WARM_UNEXPECTED"
    assert "SET_POSTURE" not in link.goals()


def test_reboot_detection_tick_backwards(make_link):
    link, peer = make_link()
    reboots = []
    link.subscribe("reboot", lambda h, why: reboots.append(why))
    peer.hello()
    link.wait_until(lambda: link.state.hello is not None, 1)
    for t in (30000, 60000, 65530, 3):  # forward steps across the u16 wrap
        peer.telemetry(tick=t)
    assert link.wait_until(lambda: link.state.tick == 65536 + 3, 1)
    assert reboots == []
    peer.telemetry(tick=40000)  # more than half a wrap backwards
    assert link.wait_until(lambda: bool(reboots), 1)
    assert link.state.reboots == 1


def test_body_lost_after_300ms_and_hello_polling(make_link):
    link, peer = make_link()
    conn = []
    link.subscribe("connection", lambda c, a: conn.append(a))
    peer.telemetry()
    assert link.wait_until(lambda: link.state.alive, 1)
    keep_alive(link, 0.4)
    assert not link.state.alive
    assert False in conn
    t0 = time.monotonic()
    keep_alive(link, 1.1)
    assert 2 <= len(peer.of("HELLO_REQ", since=t0)) <= 3  # 2 Hz


def test_protocol_mismatch_flag(make_link):
    link, peer = make_link()
    peer.hello(proto_major=7)
    assert link.wait_until(lambda: link.state.proto_ok is False, 1)


def test_asyncio_dispatch_and_liveness():
    a, b = loopback_pair()
    peer = Peer(b)
    link = SerialLink(lambda: a, codec=KiloCodec(), messages=KiloMessages(),
                      config=LinkConfig(**FAST))

    async def main():
        loop = asyncio.get_running_loop()
        got = []
        link.subscribe("telemetry", lambda m: got.append(threading.current_thread()))
        link.attach_asyncio(loop, period_s=0.05)
        link.start()
        await asyncio.sleep(0.1)
        peer.telemetry()
        for _ in range(50):
            if got:
                break
            await asyncio.sleep(0.01)
        await asyncio.sleep(0.3)
        return got

    try:
        got = asyncio.run(main())
        assert got and got[0] is threading.main_thread()
        assert peer.of("HEARTBEAT")  # liveness stamped by the loop task
    finally:
        link.stop()
        peer.stop()
