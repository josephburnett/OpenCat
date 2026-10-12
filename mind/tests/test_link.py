"""SerialLink against a scripted peer over a loopback.

Timing rules (heartbeat, liveness window, goal refresh, HELLO polling, ACK
timeout and retries, reconnect backoff) are tested on a StepClock
(``make_link(stepped=True)``): time moves only on ``clock.step(dt)``, which
returns once the link's writer has run at the new time and the peer has read
what it wrote. "Nothing was sent" is then exact, and periods are checked on
both sides of their boundary. Steps avoid landing exactly on a boundary: the
link compares float clock differences.
"""

import asyncio
import threading
import time

import pytest

from kilo.link import consts
from kilo.link.serial_link import CommandError, LinkConfig, LinkTimeout, SerialLink
from kilo.link.transport import loopback_pair
from kilo.stepclock import StepClock

from conftest import FAST, Background, Peer, count, stepped_alive
from kilo.link.codec import KiloCodec
from kilo.link.messages import KiloMessages

T = consts.TYPES
EPS = 1e-6


def test_connect_sends_hello_req(make_link):
    link, peer = make_link()
    assert peer.wait_for(lambda: peer.of("HELLO_REQ"))
    assert link.state.connected


def test_heartbeat_only_while_mind_alive(make_link):
    # HELLO polling off here (no telemetry: the body is "lost" and would be
    # polled); test_body_lost_after_300ms_and_hello_polling covers it.
    link, peer = make_link(stepped=True, hello_poll_s=1e6)
    clock = link.clock
    # HEARTBEAT after 100 ms of silence, not before
    for _ in range(5):
        n = count(peer, "HEARTBEAT")
        link.mark_alive()
        clock.step(0.099)
        assert count(peer, "HEARTBEAT") == n
        clock.step(0.002)
        assert count(peer, "HEARTBEAT") == n + 1
    # the mind stalls (last stamp at m): heartbeats go on inside the 0.3 s
    # liveness window and stop at its end
    m = clock()
    link.mark_alive()
    clock.step(0.101)
    clock.step(0.101)
    assert len(peer.of("HEARTBEAT", since=m + 0.1)) == 2  # at m+0.101 and m+0.202
    clock.step(0.101)  # m+0.303: silent for 0.101 s, but the mind is not alive
    clock.run(1.0, 0.01)
    assert peer.of("HEARTBEAT", since=m + 0.3) == []
    assert peer.frames[-1][0] < m + 0.3  # nothing autonomous at all
    # and resume at once
    t1 = clock()
    link.mark_alive()
    clock.step(0.001)
    assert len(peer.of("HEARTBEAT", since=t1)) == 1
    stepped_alive(link, 0.35, 0.03)
    assert len(peer.of("HEARTBEAT", since=t1)) == 4  # every 0.12 s with 0.03 s steps


def test_goal_sent_immediately_and_refreshed_at_5hz(make_link):
    link, peer = make_link(stepped=True)
    clock = link.clock
    peer.telemetry(epoch=5)
    assert link.wait_until(lambda: link.state.epoch == 5, 1)
    t0 = clock()
    link.set_goal("SET_POSTURE", posture=3, transition_ticks=0, flags=0)
    clock.settle()  # no time passes
    assert [f[0] for f in peer.of("SET_POSTURE")] == [t0]
    tick = [1]

    def telemetry():  # a live body streams telemetry (a lost one gets no refresh)
        tick[0] += 1
        peer.telemetry(tick=tick[0], epoch=5)
        assert link.wait_until(lambda: link.state.tick == tick[0], 1)
    dt = 0.007
    stepped_alive(link, 1.0, dt, telemetry)
    sends = peer.of("SET_POSTURE")
    gaps = [b[0] - a[0] for a, b in zip(sends[1:], sends[2:])]
    assert len(sends) >= 5 and gaps, sends
    assert all(0.2 - EPS <= g <= 0.2 + dt + EPS for g in gaps), gaps  # 5 Hz
    assert sends[1][0] - t0 <= 0.2 + dt + EPS
    assert all(p[3][0] == 5 for p in sends)  # stamped with the epoch they were set under


def test_goal_refresh_stops_when_mind_stalls(make_link):
    link, peer = make_link(stepped=True)
    clock = link.clock
    peer.telemetry(epoch=1)
    assert link.wait_until(lambda: link.state.epoch == 1, 1)
    link.set_goal("SET_EYES", led_mask=0x3F, r=1, g=2, b=3)
    stepped_alive(link, 0.3)
    m = clock() - 0.01  # the last liveness stamp, before the last step
    assert count(peer, "SET_EYES") >= 2
    clock.run(1.0, 0.01)
    assert peer.of("SET_EYES", since=m + 0.1)  # still refreshed inside the window
    assert peer.of("SET_EYES", since=m + 0.3) == []


def test_goal_waits_for_first_epoch(make_link):
    link, peer = make_link(stepped=True)
    link.set_goal("SET_POSTURE", posture=1, transition_ticks=0, flags=0)
    stepped_alive(link, 0.25)
    assert peer.of("SET_POSTURE") == []
    peer.telemetry(epoch=9)
    assert peer.wait_for(lambda: peer.of("SET_POSTURE"), 0.5)
    assert peer.of("SET_POSTURE")[0][3][0] == 9
    assert "SET_POSTURE" in link.goals()


@pytest.mark.parametrize("ev", ["FELL", "WATCHDOG", "LOW_BATTERY"])
def test_clearing_events_drop_motion_goals(make_link, ev):
    link, peer = make_link(stepped=True)
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
    link.clock.settle()  # a SET_POSTURE packed before the clear is out and counted
    n, n_eyes = count(peer, "SET_POSTURE"), count(peer, "SET_EYES")
    stepped_alive(link, 0.5)
    assert count(peer, "SET_POSTURE") == n
    assert count(peer, "SET_EYES") >= n_eyes + 2  # the refresh did run meanwhile
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


def _acker(statuses=None, ignore=0, names=("CAL_SAVE",)):
    """Handler: ignore the first `ignore` matching frames, then ACK."""
    n = {"n": 0}

    def h(peer, name, f):
        if name not in names:
            return
        n["n"] += 1
        if n["n"] <= ignore:
            return
        st = statuses[min(n["n"] - 1, len(statuses) - 1)] if statuses else 0
        peer.ack(f.seq, f.type, st)
    return h


def _ack_deadline(clock, t):
    """The one pending wait is the request's ACK timeout, ending at ``t``."""
    (d,) = clock.wait_for_waiters()
    assert abs(d - t) < EPS, (d, t)


# On a StepClock the ACK timeout only runs out when the test steps time, so
# "no retry" below holds however slow the machine is.

def test_request_ack_ok(make_link):
    link, peer = make_link(stepped=True)
    peer.handler = _acker()
    ack = link.request("CAL_SAVE")
    assert ack.status == 0
    assert link.state.stats.retries == 0
    assert count(peer, "CAL_SAVE") == 1


def test_request_retries_with_same_seq(make_link):
    link, peer = make_link(stepped=True)
    clock = link.clock
    peer.handler = _acker(ignore=2)
    t0 = clock()
    bg = Background(lambda: link.request("CAL_SAVE"), clock)
    for k in (1, 2):
        _ack_deadline(clock, t0 + 0.05 * k)
        clock.step(0.05)
    bg.result()
    assert abs(bg.ended_at - (t0 + 0.1)) < EPS
    sends = peer.of("CAL_SAVE")
    assert len(sends) == 3
    assert len({s[2] for s in sends}) == 1  # de-dup relies on the same seq
    assert link.state.stats.retries == 2


def test_request_timeout_after_three_retries(make_link):
    link, peer = make_link(stepped=True)
    clock = link.clock
    t0 = clock()
    bg = Background(lambda: link.request("CAL_SAVE"), clock)
    for k in range(4):  # the first try and 3 retries, 50 ms apart
        assert peer.wait_for(lambda: count(peer, "CAL_SAVE") == k + 1)
        _ack_deadline(clock, t0 + 0.05 * (k + 1))
        clock.step(0.049)
        assert count(peer, "CAL_SAVE") == k + 1  # not early
        clock.step(0.001)
    with pytest.raises(LinkTimeout):
        bg.result()
    assert abs(bg.ended_at - (t0 + 0.2)) < EPS
    assert [round(f[0] - t0, 6) for f in peer.of("CAL_SAVE")] == [0.0, 0.05, 0.1, 0.15]
    assert link.state.stats.timeouts == 1


def test_request_error_status(make_link):
    link, peer = make_link(stepped=True)
    peer.handler = _acker(statuses=[9])  # EEPROM_BUSY
    with pytest.raises(CommandError) as ei:
        link.request("CAL_SAVE")
    assert ei.value.status_name == "EEPROM_BUSY"
    assert count(peer, "CAL_SAVE") == 1


def test_late_ack_then_dedup_reack_is_counted(make_link):
    link, peer = make_link(stepped=True)
    clock = link.clock
    seen = []
    peer.handler = lambda p, name, f: seen.append(f) if name == "CAL_SAVE" else None
    t0 = clock()
    bg = Background(lambda: link.request("CAL_SAVE"), clock)
    _ack_deadline(clock, t0 + 0.05)
    clock.step(0.05)  # no ACK within 50 ms: the first retry goes out
    assert peer.wait_for(lambda: len(seen) == 2)
    assert seen[0].seq == seen[1].seq
    for f in seen:  # then both ACKs arrive (the body re-acks the duplicate)
        peer.ack(f.seq, f.type)
    bg.result()
    assert link.wait_until(lambda: link.state.stats.dup_acks == 1, 1)
    assert link.state.stats.unmatched_acks == 0
    assert link.state.stats.retries == 1


def test_query_retry_uses_fresh_seq(make_link):
    link, peer = make_link(stepped=True)
    clock = link.clock
    n = {"k": 0}

    def h(p, name, f):
        if name == "PING":
            n["k"] += 1
            if n["k"] >= 2:
                p.send("PONG", token=0x1234, tick=1)
    peer.handler = h
    t0 = clock()
    bg = Background(lambda: link.request("PING", {"token": 0x1234}), clock)
    _ack_deadline(clock, t0 + 0.05)
    clock.step(0.05)
    msg = bg.result()
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
    clock = StepClock()
    attempts = []
    a, b = loopback_pair()
    peer = Peer(b, clock=clock)

    def factory():
        attempts.append(clock())
        if len(attempts) < 3:
            raise OSError("no such device")
        return a

    link = SerialLink(factory, codec=KiloCodec(), messages=KiloMessages(),
                      config=LinkConfig(**FAST), clock=clock)
    t0 = clock()
    link.start()
    try:
        # reconnect_min_s 0.02, doubling: tries at t0, t0+0.02, t0+0.06
        (d,) = clock.wait_for_waiters()
        assert attempts == [t0] and abs(d - (t0 + 0.02)) < EPS
        clock.step(0.019)
        assert len(attempts) == 1
        clock.step(0.001)
        assert link.wait_until(lambda: len(attempts) == 2, 1)
        (d,) = clock.wait_for_waiters()
        assert abs(d - (t0 + 0.06)) < EPS  # the backoff grew to 0.04
        clock.step(0.039)
        assert len(attempts) == 2
        clock.step(0.001)
        assert link.wait_until(lambda: link.state.connected, 2)
        assert [round(t - t0, 6) for t in attempts] == [0.0, 0.02, 0.06]
        assert link.state.stats.open_failures == 2
    finally:
        link.stop()
        peer.stop()


def test_reboot_detection_unsolicited_hello(make_link):
    link, peer = make_link(stepped=True)
    reboots = []
    link.subscribe("reboot", lambda h, why: reboots.append(why))
    peer.hello(epoch=0)
    assert link.wait_until(lambda: link.state.hello is not None, 1)
    peer.telemetry(epoch=0)
    link.wait_until(lambda: link.state.epoch == 0, 1)
    link.set_goal("SET_POSTURE", posture=3, transition_ticks=0, flags=0)
    stepped_alive(link, 0.4)  # past the solicitation window (0.3 s here)
    peer.hello(epoch=0, reset_cause=3)
    assert link.wait_until(lambda: bool(reboots), 1)
    assert link.state.reboots == 1
    assert link.state.reset_cause == "WARM_UNEXPECTED"
    assert "SET_POSTURE" not in link.goals()


def test_hello_solicitation_window(make_link):
    """A HELLO that repeats the last one's identity is a reboot only when
    unsolicited: more than hello_solicit_window_s (0.3 s here) after the
    last HELLO_REQ (the connect's, at t0)."""
    link, peer = make_link(stepped=True)
    reboots = []
    link.subscribe("reboot", lambda h, why: reboots.append(why))
    peer.hello(epoch=0)
    assert link.wait_until(lambda: link.state.hello is not None, 1)
    stepped_alive(link, 0.29)
    n = link.state.stats.frames_rx
    peer.hello(epoch=0)  # t0+0.29: still an answer to the HELLO_REQ
    assert link.wait_until(lambda: link.state.stats.frames_rx > n, 1)
    assert reboots == []
    stepped_alive(link, 0.02)  # t0+0.31, before the next poll (t0+0.5)
    peer.hello(epoch=0)
    assert link.wait_until(lambda: bool(reboots), 1)
    assert "unsolicited" in reboots[0]


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
    link, peer = make_link(stepped=True)
    clock = link.clock
    conn = []
    link.subscribe("connection", lambda c, a: conn.append(a))
    t0 = clock()
    peer.telemetry()
    assert link.wait_until(lambda: link.state.alive, 1)
    stepped_alive(link, 0.29)
    assert link.state.alive and False not in conn
    stepped_alive(link, 0.02)  # t0+0.31: no telemetry for 300 ms
    assert not link.state.alive
    assert False in conn  # told by the writer's housekeeping, inside the step
    stepped_alive(link, 1.1)
    polls = [f[0] - t0 for f in peer.of("HELLO_REQ") if f[0] > t0]
    assert len(polls) == 2, polls  # 2 Hz: the connect's poll was at t0
    assert all(0.5 * k - EPS <= p <= 0.5 * k + 0.01 + EPS for k, p in zip((1, 2), polls)), polls
    # a stalled mind does not poll either
    m = clock() - 0.01
    clock.run(1.5, 0.01)
    assert peer.of("HELLO_REQ", since=m + 0.3) == []


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
