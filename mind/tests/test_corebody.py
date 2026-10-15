"""The mind against the real body core (docs §6 "Virtual body").

kilo.vbody.core runs body/test/core_harness.cpp (the firmware's core/ and
link dispatch on host fakes) as a subprocess and keeps it in lockstep with a
SerialLink on a simulated clock: every step is one 20 ms body tick, and the
link's writer and reader each get their turn per tick. So these timelines are
the firmware's own, measured in body ticks, and the mind is the real
SerialLink + BodyClient. Where kilo.vbody.sim (the Python stand-in) behaves
differently, the C core is right; the differences found are listed in
CORE_VS_SIM below and pinned by the tests that found them.

Skipped when g++ is not available (or the body/ tree is absent).
"""

from __future__ import annotations

import shutil

import pytest

from kilo.body.client import BodyClient
from kilo.link import consts
from kilo.link.serial_link import CommandError, LinkConfig, SerialLink
from kilo.protocol import kilo_protocol as P
from kilo.tricks import library
from kilo.vbody.core import BUILD_SCRIPT, CORE_BUILD, TICK_S, CoreBody, build_harness

from conftest import FAST

pytestmark = pytest.mark.skipif(
    not BUILD_SCRIPT.exists() or shutil.which("g++") is None or shutil.which("gcc") is None,
    reason="needs body/ and a host C/C++ compiler")

EV = P.EventCode
R = P.GoalRefusedReason
LIMP, REST, SIT, STAND, STANCE = (int(P.Posture[n]) for n in ("LIMP", "REST", "SIT", "STAND", "STANCE"))
OFF = -32768
HOLD = 0xFE  # core/posture.h POSTURE_HOLD: IDLE holds the warm-boot snapshot
RH_WATCHDOG, RH_FALL = 3, 2  # core/body.h ReflexHolder
OWN_IDLE, OWN_GAIT, OWN_TRICK, OWN_REFLEX = 0, 1, 2, 3

# Where the Python vbody (kilo.vbody.sim) and the C core disagree (the core
# is right). Kept here so the next vbody pass has the list in one place.
CORE_VS_SIM = """
- fall: the core's timeline follows the IMU: 6 samples over 75 deg -> FELL,
  10 still samples -> rc (mode RECOVERING), rc's 3 frames (89 ticks), then
  25 upright samples -> RECOVERED (2.48 s from FELL when it lands upright),
  up to 3 tries, else RECOVERY_FAILED, REST, LIMP 2 s later. The sim's
  trigger_fall() is a fixed 1 s FALLEN + 1 s RECOVERING whatever the
  attitude, never re-tries or fails, and has no detector.
- REBOOT: the PCA9685 keeps driving through the AVR's reset, so the core
  boots warm: the warm-boot hold keeps the pose, then IDLE holds it (posture
  HOLD, mode IDLE, servos powered). The sim's reboot() is a cold power-on
  state (LIMP).
- watchdog: the core counts the tick a frame arrives in, so the stop is 24
  ticks after that tick (0.48-0.5 s after the frame), REST 74, LIMP 623
  (the REST tick already counts the LIMP timer down); the sim measures
  0.5 / 1.5 / 12.5 s from the frame's arrival. At most a tick apart.
- events: the core sends EVENTs in link_tick (the same or the next tick),
  the sim at once (test_crosscheck.py normalises that).
- SET_BALANCE enable cleared by an epoch bump: the sim kept it; aligned in
  this change (kilo.vbody.sim VirtualBody._epoch_bump).
"""


@pytest.fixture(scope="module")
def binary(tmp_path_factory):
    return build_harness(tmp_path_factory.mktemp("core") / "core_harness")


@pytest.fixture
def rig(binary):
    made = []

    def make(**kw) -> tuple[CoreBody, SerialLink, BodyClient]:
        core = CoreBody(binary, **kw)
        link = SerialLink(core.transport, config=LinkConfig(**FAST, housekeeping_s=0.001), clock=core.clock,
                          port_name="core")
        core.attach(link)
        link.mark_alive()
        link.start()
        made.append((core, link))
        client = BodyClient(link)
        assert core.run_until(lambda: link.state.alive and link._contact, 1.0) is not None
        return core, link, client

    yield make
    for core, link in made:
        link.stop()
        core.close()


def ticks(seconds: float) -> int:
    return round(seconds / TICK_S)


def events(link: SerialLink, name: str) -> list:
    return [e for _, e in link.state.events if e.code == EV[name]]


def refused(link: SerialLink, goal: str, reason: str) -> list:
    arg = (consts.TYPES[goal] << 8) | R[reason]
    return [e for e in events(link, "GOAL_REFUSED") if e.arg & 0xFFFF == arg]


def stand(core: CoreBody, client: BodyClient) -> None:
    client.stand()
    assert core.run_until(lambda: (d := core.dump())["posture"] == STAND and not d["idle_moving"], 2) is not None


# ---- HELLO, contact, telemetry -------------------------------------------------------

def test_hello_contact_and_telemetry(rig):
    core, link, client = rig()
    h = link.state.hello
    assert (h.reset_cause, h.fw_build, h.epoch, h.joint_count) == (P.ResetCause.COLD, CORE_BUILD, 0, P.JOINT_COUNT)
    assert h.caps & P.HelloCaps.DMP and h.config_state == P.ConfigState.OK
    assert [e.arg for e in events(link, "BOOT")] == [P.ResetCause.COLD]
    assert link.state.epoch == 0 and link.state.mode == "LIMP"
    # TELEMETRY at TELEM_DIV 2: 25 per second of body time.
    n = len(link.state.history)
    core.step(50)
    assert len(link.state.history) - n == 25
    assert "link_ok" in link.state.flags and "servos_powered" not in link.state.flags
    # HELLO_REQ is a query: answered at once, between ticks.
    assert client.hello().fw_build == CORE_BUILD
    # The core's STATS: 100 Hz DMP packets as the harness feeds them.
    assert client.stats().dmp_rate_hz == 100


def test_set_posture_stand(rig):
    core, link, client = rig()
    client.stand()
    core.step(1)
    d = core.dump()
    assert d["posture"] == STAND and d["idle_moving"] and d["mode"] == P.Mode.IDLE
    # POSTURE_TICKS (25) of cosine easing from limp, then settled.
    assert core.run_until(lambda: not core.dump()["idle_moving"], 1) == 24
    d = core.dump()
    assert OFF not in d["out"] and set(d["owner"]) == {OWN_IDLE}
    core.step(2)
    assert link.state.mode == "IDLE" and "servos_powered" in link.state.flags
    assert link.goals()["SET_POSTURE"].sends >= 1


def test_reboot_detected(rig):
    core, link, client = rig()
    stand(core, client)
    core.step(25)  # past the HELLO solicitation window
    reboots = []
    link.subscribe("reboot", lambda h, why: reboots.append((h, why)))
    client.reboot()
    assert core.run_until(lambda: bool(reboots), 1) is not None
    h = reboots[0][0]
    assert h.reset_cause == P.ResetCause.REQUESTED and h.epoch == 1  # boot epoch: last + 1
    assert "SET_POSTURE" not in link.goals()
    # The PCA kept driving: the warm-boot hold keeps the pose, and hands it
    # to IDLE once the Pi is heard (it is: the link's heartbeats).
    d = core.dump()
    assert OFF not in d["out"] and d["posture"] == HOLD  # (the sim's reboot goes LIMP)
    core.step(5)
    assert core.dump()["holder"] == 0 and core.dump()["mode"] == P.Mode.IDLE


# ---- the link watchdog ------------------------------------------------------------------

def test_watchdog_timeline_and_stale_goals(rig):
    core, link, client = rig()
    stand(core, client)
    e0 = link.state.epoch
    cleared = []
    link.subscribe("goals_cleared", lambda r, n: cleared.append(r))
    # The mind stalls: no liveness, so after its window the link sends nothing.
    assert core.run_until(lambda: core.dump()["wd_stage"] == 1, 2, alive=False) is not None
    t_last = link.last_tx[0]
    wd = core.events("WATCHDOG")
    assert len(wd) == 1
    # Stop: WD_STOP_TICKS (25) counted from the tick the last frame arrived
    # in (that tick counts as 1), so 24 ticks = 0.48 s after it.
    assert ticks(wd[0][0] - t_last) == 24
    assert wd[0][1].epoch == e0 + 1
    d = core.dump()
    assert d["wd_since"] == 25 and d["mode"] == P.Mode.LINK_LOST and d["posture"] == STAND
    assert cleared == ["WATCHDOG"] and "SET_POSTURE" not in link.goals()
    assert link.state.mode == "LINK_LOST" and link.state.epoch == e0 + 1
    # REST at 1.5 s (the watchdog reflex lies down over 1 s) ...
    assert core.run_until(lambda: core.dump()["wd_stage"] == 2, 2, alive=False) is not None
    d = core.dump()
    assert ticks(core.clock() - t_last) == 74
    assert (d["holder"], d["reflex_posture"], d["posture"]) == (RH_WATCHDOG, REST, REST)
    assert set(d["owner"]) == {OWN_REFLEX}
    # ... and LIMP WD_LIMP_S (10 s) after the end of that 1 s move; the REST
    # tick already counts the timer down once: 549 ticks after REST.
    assert core.run_until(lambda: core.dump()["wd_stage"] == 3, 12, alive=False) is not None
    d = core.dump()
    assert ticks(core.clock() - t_last) == 74 + 50 + 500 - 1
    assert d["out"] == [OFF] * 11 and d["reflex_posture"] == LIMP
    assert link.state.mode == "LINK_LOST"
    # The mind is back: its heartbeat gives LINK_RESTORED, the reflex lets go
    # and the body stays limp until a goal of the new epoch.
    assert core.run_until(lambda: bool(events(link, "LINK_RESTORED")), 1) is not None
    core.step(3)
    d = core.dump()
    assert (d["wd_stage"], d["holder"], d["mode"]) == (0, 0, P.Mode.LIMP)
    assert link.state.mode == "LIMP" and "link_ok" in link.state.flags
    # A goal stamped before the bump is refused STALE, and the link drops it.
    link.set_goal("SET_POSTURE", posture=STAND, transition_ticks=0, flags=0)
    link._goals["SET_POSTURE"].epoch = e0
    assert core.run_until(lambda: bool(refused(link, "SET_POSTURE", "STALE")), 1) is not None
    core.step(2)
    assert "SET_POSTURE" not in link.goals() and core.dump()["posture"] == LIMP
    with pytest.raises(CommandError) as ei:
        link.request("TRICK_PLAY", {"epoch": e0, "trick_id": 1, "speed_q6": 64, "flags": 0})
    assert ei.value.status_name == "STALE"
    # With the new epoch it goes through.
    stand(core, client)


# ---- tricks -----------------------------------------------------------------------------------

def test_trick_upload_play_and_preemption(rig):
    core, link, client = rig()
    stand(core, client)
    trick = library.load("hi")
    tid, uploaded = client.upload_trick(trick)
    assert uploaded and core.dump()["trick_store"] == 2  # KF_READY
    assert client.upload_trick(trick) == (tid, False)  # cached
    expected = client.expected_ticks(trick)
    tid, n = client.start_trick(trick)
    assert "SET_POSTURE" not in link.goals()  # the end posture replaces it
    core.step(1)
    d = core.dump()
    assert d["trick_run"] and d["mode"] == P.Mode.TRICK and OWN_TRICK in d["owner"]
    # TRICK_PLAY while one plays: BUSY.
    with pytest.raises(CommandError) as ei:
        client.start_trick(trick)
    assert ei.value.status_name == "BUSY"
    # TRICK_DONE(DONE) on the tick kilo.tricks.player predicts.
    k = core.run_until(lambda: client.trick_done_since(n, tid) is not None, 30)
    assert k is not None
    done = client.trick_done_since(n, tid)
    assert done.arg >> 8 == P.TrickDoneReason.DONE
    t_done = core.events("TRICK_DONE")[-1][0]
    t_play = [t for t, name, _ in core.sent if name == "ACK"][-2]  # the TRICK_PLAY's OK (before BUSY)
    # (expected_ticks counts to the last frame's end; TRICK_DONE goes out on
    # the tick after it, on the core as in the sim.)
    assert ticks(t_done - t_play) == expected + 1
    d = core.dump()
    assert not d["trick_run"] and d["posture"] == int(trick.end_posture)
    # Pre-emption: an epoch bump (here the watchdog of a stalled mind) ends a
    # playing trick with TRICK_DONE(PREEMPTED) carrying the new epoch.
    stand(core, client)
    e = link.state.epoch
    tid, n = client.start_trick(trick)
    core.step(5)
    assert core.dump()["trick_run"]
    assert core.run_until(lambda: client.trick_done_since(n, tid) is not None, 2, alive=False) is not None
    m = client.trick_done_since(n, tid)
    assert m.arg >> 8 == P.TrickDoneReason.PREEMPTED and m.epoch == e + 1
    # SET_POSTURE LIMP stops a trick at once: STOPPED.
    assert core.run_until(lambda: bool(events(link, "LINK_RESTORED")), 1) is not None
    stand(core, client)
    tid, n = client.start_trick(trick)
    core.step(5)
    client.limp()
    assert core.run_until(lambda: client.trick_done_since(n, tid) is not None, 1) is not None
    assert client.trick_done_since(n, tid).arg >> 8 == P.TrickDoneReason.STOPPED
    assert core.dump()["out"] == [OFF] * 11


# ---- the gait -------------------------------------------------------------------------------------

def test_feet_stream(rig):
    """Protocol 2.0 at the link: FEET a few ticks ahead puts the core in GAIT
    with the reference and the support feet in TELEMETRY; a stale epoch is
    refused; when the stream stops the core underruns, holds, settles and
    ends GAIT with GAIT_STOPPED arg 1 (kilo.vbody.feet is the Python stand-in)."""
    core, link, client = rig()
    client.posture("stance")
    assert core.run_until(lambda: (d := core.dump())["posture"] == STANCE and not d["idle_moving"], 2) is not None
    link.clear_goal("SET_POSTURE")  # a posture refresh would end GAIT
    epoch = link.state.epoch

    def send(ahead, epoch=epoch, roll=0, support=0xF):
        t = (link.state.telemetry.tick + ahead) & 0xFFFF
        return link.send_stream("FEET", P.Feet(epoch=epoch, tick=t, x_mm=[0] * 4, z_mm=[76] * 4, pitch=0,
                                               roll=roll, support=support).pack())

    assert send(3, epoch=(epoch + 5) & 0xFF)
    assert core.run_until(lambda: refused(link, "FEET", "STALE"), 1) is not None
    for _ in range(20):
        for ahead in range(2, 6):
            send(ahead, roll=-3 * 16)
        core.step(1)
    assert link.state.mode == "GAIT" and "gait" in link.state.flags and link.state.owners[0] == "GAIT"
    tel = link.state.telemetry
    assert tel.ref_roll == -3 * 16 and tel.support == 0xF and tel.feet_queued >= 1
    assert core.run_until(lambda: events(link, "FEET_UNDERRUN"), 1) is not None
    assert events(link, "FEET_UNDERRUN")[0].arg >= 1
    assert core.run_until(lambda: events(link, "GAIT_STOPPED"), 2) is not None
    assert events(link, "GAIT_STOPPED")[0].arg == 1
    core.step(5)
    assert link.state.mode == "IDLE"


# ---- the fall reflex ---------------------------------------------------------------------------

def test_fall_reflex_timeline(rig):
    core, link, client = rig()
    stand(core, client)
    e0 = link.state.epoch
    cleared = []
    link.subscribe("goals_cleared", lambda r, n: cleared.append(r))
    core.set_attitude(roll_deg=85)
    # 6 consecutive samples over 75 deg (120 ms); FELL +1 (roll), epoch bump.
    t = core.run_until(lambda: core.dump()["fall_state"] == 1, 1)
    assert t == 6
    core.step(1)  # the event goes out in the next tick's link_tick
    fell = events(link, "FELL")
    assert [e.arg for e in fell] == [1] and fell[0].epoch == e0 + 1
    assert cleared == ["FELL"] and link.state.fallen and "SET_POSTURE" not in link.goals()
    d = core.dump()
    assert (d["holder"], d["mode"], set(d["owner"])) == (RH_FALL, P.Mode.FALLEN, {OWN_REFLEX})
    # A posture goal of the new epoch is refused FALLEN while the reflex holds.
    client.stand()
    assert core.run_until(lambda: bool(refused(link, "SET_POSTURE", "FALLEN")), 1) is not None
    # Still for 10 samples (it already is): rc starts, mode RECOVERING.
    t = core.run_until(lambda: core.dump()["fall_state"] == 2, 1)
    assert t is not None and t <= 10
    core.step(1)
    assert link.state.mode == "RECOVERING"
    # rc rolls the body upright; once CHECK sees 25 level samples: RECOVERED.
    core.set_attitude()
    t = core.run_until(lambda: core.dump()["fall_state"] == 3, 5)
    assert t is not None
    t = core.run_until(lambda: core.dump()["fall_state"] == 0, 2)
    assert t == 25
    core.step(1)
    assert len(events(link, "RECOVERED")) == 1
    d = core.dump()
    assert (d["posture"], d["holder"], d["mode"]) == (STAND, 0, P.Mode.IDLE)
    # The latched stand (new epoch) is no longer refused: the body stands.
    core.step(15)
    assert link.state.mode == "IDLE" and not link.state.fallen


def test_fall_without_recovery_fails_then_limps(rig):
    core, link, client = rig()
    stand(core, client)
    core.set_attitude(roll_deg=85)
    # Never comes upright: 3 rc tries, then RECOVERY_FAILED, REST, LIMP 2 s later.
    assert core.run_until(lambda: bool(events(link, "RECOVERY_FAILED")), 15) is not None
    assert core.dump()["fall_state"] == 4 and core.dump()["fall_tries"] == 3
    assert core.run_until(lambda: core.dump()["out"] == [OFF] * 11, 3) is not None
    core.step(2)
    assert core.dump()["fall_state"] == 0 and link.state.mode == "LIMP"


# ---- balance ------------------------------------------------------------------------------------

def test_balance_enable_cleared_by_epoch_bump_and_restored_by_refresh(rig):
    core, link, client = rig()
    stand(core, client)
    client.set_balance(True)
    core.step(2)
    d = core.dump()
    assert d["balance_enable"] == 1 and d["balance_active"] == 1
    assert "balance_active" in link.state.flags
    # A fall bumps the epoch: the body clears SET_BALANCE's enable (P1 finding
    # B1). SET_BALANCE is not an epoch goal, so the link keeps it latched and
    # its 5 Hz refresh turns it back on within 200 ms.
    core.set_attitude(roll_deg=85)
    assert core.run_until(lambda: core.dump()["fall_state"] == 1, 1) == 6
    d = core.dump()
    assert d["balance_enable"] == 0 and d["balance_active"] == 0
    core.step(1)
    assert "SET_BALANCE" in link.goals() and "SET_POSTURE" not in link.goals()
    t = core.run_until(lambda: core.dump()["balance_enable"] == 1, 1)
    assert t is not None and t <= 11  # 0.2 s of refresh period, plus the tick it lands in
    # (The fall reflex holds the joints: balance is not active meanwhile.)
    assert core.dump()["balance_active"] == 0


# ---- real time on a pty (what `python -m kilo.vbody.core` serves) ----------------------

def test_realtime_on_a_pty(binary):
    import time

    from kilo.link.transport import open_port

    core = CoreBody(binary, clock=time.monotonic)
    path = core.open_pty()
    core.start_realtime()
    link = SerialLink(lambda: open_port(path), config=LinkConfig(**FAST), port_name=path)
    try:
        link.mark_alive()
        link.start()
        client = BodyClient(link)
        hello = client.wait_ready(3)
        assert hello.fw_build == CORE_BUILD
        client.stand()
        assert link.wait_until(lambda: link.state.mode == "IDLE" and "servos_powered" in link.state.flags, 2,
                               alive=True)
        assert core.dump()["posture"] == STAND
    finally:
        link.stop()
        core.close()
