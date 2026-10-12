"""P1 trick player, mind side (docs §3e, §5 "Tricks"): the player timing model,
the BodyClient uploader/player against the virtual body, and the `kilo trick`
CLI end to end on a pty. The C player is tested in body/test/test_keyframe.cpp
and compared message by message with the vbody in test_crosscheck.py."""

from __future__ import annotations

import asyncio
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from kilo.body.client import BodyClient, TrickError, trick_id_for
from kilo.link import consts
from kilo.link.serial_link import CommandError, LinkConfig, SerialLink
from kilo.link.transport import loopback_pair
from kilo.protocol import kilo_protocol as P
from kilo.tricks import library
from kilo.tricks.model import Keyframe, Trick
from kilo.tricks.player import Player, phase_ticks, play_order, total_ticks
from kilo.vbody.sim import VirtualBody, open_pty

from conftest import FAST

MIND = Path(__file__).resolve().parents[1]
REPO = MIND.parent
STAND = {"lf_shoulder": 30, "rf_shoulder": 30, "rb_shoulder": -30, "lb_shoulder": -30, "lf_knee": 30,
         "rf_knee": 30, "rb_knee": -30, "lb_knee": -30, "head_pan": 0, "head_tilt": 0, "tail": 0}


def small(repeats=2, dur=3):
    f = [Keyframe.from_joints({**STAND, "head_pan": a}, dur=d, hold=h)
         for a, d, h in ((20, dur, 1), (-20, 2, 0), (10, 4, 2), (0, 1, 0))]
    return Trick(name="small", frames=tuple(f), loop_start=1, loop_end=2, repeats=repeats)


# ---- the timing model --------------------------------------------------------------
def test_generated_c_library_header_is_current():
    r = subprocess.run([sys.executable, str(REPO / "body" / "tools" / "gen_trick_library.py"), "--check"],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_phase_ticks_and_play_order():
    assert [phase_ticks(10, s) for s in (16, 32, 64, 96, 128, 255)] == [40, 20, 10, 7, 5, 3]
    assert phase_ticks(0, 64) == 0
    assert play_order(4, 1, 2, 0) == [0, 1, 2, 3] == play_order(4, 1, 2, 1)
    assert play_order(4, 1, 2, 3) == [0, 1, 2, 1, 2, 1, 2, 3]
    assert play_order(3, 0, 1, 2) == [0, 1, 0, 1, 2]


@pytest.mark.parametrize("name", library.names())
def test_player_matches_the_model_duration(name):
    t = library.load(name)
    durs, holds = [f.dur for f in t.frames], [f.hold for f in t.frames]
    want = total_ticks(durs, holds, t.loop_start, t.loop_end, t.repeats)
    assert want == t.duration_ticks
    for speed in (32, 64, 96):
        p = Player(durs, holds, t.loop_start, t.loop_end, t.repeats, speed)
        n = 0
        while not p.step():
            n += 1
            assert n < 10000
        assert n == total_ticks(durs, holds, t.loop_start, t.loop_end, t.repeats, speed)


def test_player_forever():
    p = Player([2, 3], [0, 1], 0, 1, 255)
    for _ in range(5000):
        assert not p.step()
    assert p.running and p.plays == 254
    assert total_ticks([2, 3], [0, 1], 0, 1, 255) is None


def test_trick_id_and_header_carry_expected_attitude():
    t = library.load("stretch")
    b = t.begin(9)
    assert (b.exp_roll, b.exp_pitch) == (0, 15) and len(b.pack()) == P.TrickBegin.MIN_LEN == 11
    back = Trick.from_wire(b, t.encode_frames(), name="stretch")
    assert back.expected_pitch == 15 and back.frames == t.frames
    ids = {trick_id_for(library.load(n)) for n in library.names()}
    assert all(1 <= i <= 254 for i in ids)
    assert trick_id_for(t) == trick_id_for(library.load("str"))


# ---- BodyClient against the running vbody ------------------------------------------
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
        client.wait_ready(3.0)
        return client, vb

    yield make
    for link, vb in made:
        link.stop()
        vb.stop()


def _received(vb, name):
    return [r for r in vb.received if r[0] == name]


@pytest.mark.slow
def test_upload_is_cached(rig):
    client, vb = rig()
    t = library.load("hi")
    tid, uploaded = client.upload_trick(t)
    assert uploaded and tid == trick_id_for(t)
    assert len(_received(vb, "TRICK_FRAMES")) == 2 and vb.trick.state == "ready"
    assert vb.trick.crc == t.crc16()
    # known to this client: nothing sent
    n = len(vb.received)
    assert client.upload_trick(t) == (tid, False) and len(vb.received) == n
    # a new client (another `kilo` process) asks the body with TRICK_COMMIT alone
    other = BodyClient(client.link)
    assert other.upload_trick(t) == (tid, False)
    assert [r[0] for r in vb.received[n:]] == ["TRICK_COMMIT"]
    # another trick replaces it: the body-side check sees the change
    other.upload_trick(library.load("sniff"))
    assert other.upload_trick(t)[1] is True
    # and a play from a stale local cache recovers from NO_TRICK by uploading again
    other.upload_trick(library.load("sniff"))
    client.posture("stand")
    client.spin(0.3)
    n = len(_received(vb, "TRICK_BEGIN"))
    assert asyncio.run(client.play(t)).reason == "DONE"
    assert len(_received(vb, "TRICK_BEGIN")) == n + 1


def test_play_resolves_on_trick_done(rig):
    client, vb = rig()
    client.posture("stand")
    client.spin(0.3)
    t = small()
    res = asyncio.run(client.play(t))
    assert res.reason == "DONE" and res.trick_id == trick_id_for(t)
    want = (total_ticks([3, 2, 4, 1], [1, 0, 2, 0], 1, 2, 2) + 1) * 0.02
    assert want * 0.8 < res.elapsed_s < want + 0.5, (res.elapsed_s, want)
    assert vb.posture == "STAND" and "SET_POSTURE" not in client.link.goals()
    # mirrored and faster: the flags and speed go in TRICK_PLAY
    res = asyncio.run(client.play(t, mirror=True, speed=2.0))
    play = P.TrickPlay.unpack(_received(vb, "TRICK_PLAY")[-1][2])
    assert res.reason == "DONE" and play.flags == 1 and play.speed_q6 == 128
    with pytest.raises(ValueError):
        asyncio.run(client.play(t, speed=5))


@pytest.mark.slow
def test_play_library_name_and_refusals(rig):
    client, vb = rig()
    # LIMP: the body refuses
    with pytest.raises(CommandError) as e:
        asyncio.run(client.play("sniff"))
    assert e.value.status == P.AckStatus.BUSY
    client.posture("sit")
    client.spin(0.3)
    res = asyncio.run(client.play("wsf", mirror=True))  # legacy token
    assert res.reason == "DONE" and vb.posture == "SIT"   # wash_face ends sitting


def test_stop_and_preempt(rig):
    client, vb = rig()
    client.posture("stand")
    client.spin(0.3)
    t = small(repeats=255)

    async def stop_soon():
        task = asyncio.ensure_future(client.play(t))
        await asyncio.sleep(0.3)
        assert vb.trick.running and client.state.mode == "TRICK"
        client.stop_trick()
        return await task

    res = asyncio.run(stop_soon())
    assert res.reason == "STOPPED"

    async def fall_soon():
        task = asyncio.ensure_future(client.play(t))
        await asyncio.sleep(0.3)
        vb.trigger_fall(1)
        return await task

    res = asyncio.run(fall_soon())
    assert res.reason == "FELL" and res.epoch == 1


def test_play_timeout_stops_the_trick(rig):
    client, vb = rig()
    client.posture("stand")
    client.spin(0.3)
    vb.ignore_types.add("TRICK_STOP")  # pretend nothing ends it... and drop the DONE by never finishing
    t = small(repeats=255)
    with pytest.raises(TrickError):
        asyncio.run(client.play(t, timeout=0.3))


# ---- the CLI on a pty ----------------------------------------------------------------
@pytest.fixture
def vbody_pty():
    transport, path, slave = open_pty()
    vb = VirtualBody(transport).start()
    yield vb, path
    vb.stop()
    transport.close()
    os.close(slave)


def _kilo(*args, port=None):
    pre = ["--port", port] if port else []
    return subprocess.Popen([sys.executable, "-m", "kilo.cli", *pre, *args], cwd=MIND,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


@pytest.mark.slow
def test_cli_list_and_show_need_no_body():
    p = _kilo("--port", "/dev/nonexistent-kilo", "trick", "list")
    out, err = p.communicate(timeout=15)
    assert p.returncode == 0, err
    assert all(n in out for n in library.names()) and "until stopped" not in out
    p = _kilo("trick", "show", "pu")
    out, err = p.communicate(timeout=15)
    assert p.returncode == 0, err
    assert "push_up" in out and "loop 1..2 x3" in out and out.count("cosine") == 6
    p = _kilo("trick", "show", "nope")
    assert p.wait(15) == 1


@pytest.mark.slow
def test_cli_upload_then_play(vbody_pty):
    vb, path = vbody_pty
    p = _kilo("trick", "upload", "check_around", port=path)
    out, err = p.communicate(timeout=15)
    assert p.returncode == 0, err
    assert "uploaded" in out and vb.trick.state == "ready"
    p = _kilo("trick", "play", "check_around", port=path)  # the body is LIMP
    out, err = p.communicate(timeout=15)
    assert p.returncode == 1 and "LIMP" in err
    p = _kilo("trick", "play", "check_around", "--posture", "stand", "--mirror", port=path)
    out, err = p.communicate(timeout=20)
    assert p.returncode == 0, err
    assert "cached" in out and "TRICK_DONE DONE after" in out, out


@pytest.mark.slow
def test_cli_play_forever_for_then_watch(vbody_pty):
    vb, path = vbody_pty
    vb.params[P.ParamId.WD_REST_MS] = 1000
    p = _kilo("trick", "play", "push_up", "--repeats", "255", "--for", "1.5", "--posture", "stand",
              "--then-watch", "2", port=path)
    seen_trick = False
    deadline = time.monotonic() + 25
    while p.poll() is None and time.monotonic() < deadline:
        seen_trick |= vb.trick.running
        time.sleep(0.02)
    out, err = p.communicate(timeout=5)
    assert p.returncode == 0, err
    assert seen_trick
    assert "TRICK_DONE STOPPED" in out and "silent: last frame" in out, out
    assert "WATCHDOG" in out, out
