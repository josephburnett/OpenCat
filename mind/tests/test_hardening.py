"""Regression tests for the second review of the mind (findings 1-17): tricks
through the daemon, goals into a rebooting body, flashing a busy port, a
failed verify, the runner's liveness and pause, the logs off the control
threads, a preempted trick, the port lease, idle latching, races."""

import asyncio
import fcntl
import functools
import logging
import os
import signal
import subprocess
import sys
import threading
import time
import types
from argparse import Namespace
from pathlib import Path

import pytest

from kilo import daemon as dmod
from kilo.behaviour.runner import Runner
from kilo.body.client import BodyClient
from kilo.body.state import BodyState
from kilo.link import consts
from kilo.link.codec import DecodeError, KiloCodec
from kilo.link.serial_link import LinkConfig, LinkError, NotConnected, SerialLink
from kilo.link.transport import loopback_pair
from kilo.logs.kilolog import KilologWriter, read_kilolog
from kilo.protocol import kilo_protocol as P
from kilo.remote import RemoteBody, connect, socket_candidates
from kilo.tricks import library
from kilo.vbody.sim import VirtualBody

from conftest import FAST, keep_alive
from test_behaviour import Clock, FakeBody, day_cfg
from test_daemon import MIND, _kilo, remote, sockdir, start_daemon, vbody_pty, wait_for  # noqa: F401 (fixtures)


@pytest.fixture
def rig():
    made = []

    def make(**vb_kw):
        a, b = loopback_pair()
        vb = VirtualBody(b, **vb_kw).start()
        link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="loopback")
        link.mark_alive()
        link.start()
        made.append((link, vb))
        return BodyClient(link), vb, a

    yield make
    for link, vb in made:
        link.stop()
        vb.stop()


def _keep(link, until, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        link.mark_alive()
        if until():
            return True
        time.sleep(0.02)
    return until()


# ---- 1, 17: tricks through the daemon ---------------------------------------------------

def test_trick_ops_through_the_daemon(start_daemon):
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        r = body.trick_upload(name="check_around")
        assert r["uploaded"] is True and r["name"] == "check_around" and vb.trick.state == "ready"
        assert body.trick_upload(name="check_around")["uploaded"] is False  # cached
        # the call that crashed: BodyClient.upload_trick(Trick) on a RemoteBody
        tid, uploaded = body.upload_trick(library.load("sniff"))
        assert uploaded is True and vb.trick.hdr.trick_id == tid
        res = body.trick_play(name="check_around", posture="stand")
        assert res["reason"] == "DONE" and res["name"] == "check_around" and vb.posture == "STAND"
        assert "TRICK" in body.conn.call("daemon")["held_goals"]
        # a looping trick, stopped by the trick_stop op from another client
        other = remote(sock)
        out = []
        t = threading.Thread(target=lambda: out.append(body.trick_play(name="sniff", repeats=255)))
        t.start()
        assert wait_for(lambda: vb.trick.running, 3)
        with pytest.raises(LinkError, match="playing a trick"):
            other.trick_play(name="sniff")
        other.stop_trick()
        t.join(3)
        assert out and out[0]["reason"] == "STOPPED" and not vb.trick.running
        other.close()
        # goals and tricks are not reachable through the generic `call` op
        with pytest.raises(ValueError):
            body.conn.call("call", method="play", args=["hi"])
        with pytest.raises(ValueError):
            body.conn.call("call", method="posture", args=["stand"])
    finally:
        body.close()
    assert wait_for(lambda: vb.posture == "REST", 2)  # the session's end posture hold ended: idle REST


def test_cli_trick_through_the_daemon(start_daemon):
    # idle REST lasts idle_limp_s before LIMP: long enough for the last CLI
    # process to exit on a loaded machine (with 1 s, -n 4 saw LIMP already)
    d, vb, path, sock = start_daemon(idle_limp_s=5.0)
    rc, out, err = _kilo("trick", "upload", "check_around")
    assert rc == 0 and "check_around: trick_id" in out and "uploaded" in out, err
    rc, out, err = _kilo("trick", "play", "check_around", "--posture", "stand")
    assert rc == 0 and "TRICK_DONE DONE" in out and "cached" in out, (out, err)
    rc, out, err = _kilo("trick", "play", "sniff", "--repeats", "255", "--for", "0.6", "--posture", "stand")
    assert rc == 0 and "TRICK_DONE STOPPED" in out, (out, err)
    rc, out, err = _kilo("trick", "stop")
    assert rc == 0 and "stopped" in out
    # --then-watch needs the port: the same polite refusal as posture --then-watch
    rc, out, err = _kilo("trick", "play", "check_around", "--then-watch", "1")
    assert rc == 1 and "holds " + path in err and "kilo daemon stop" in err
    assert wait_for(lambda: vb.posture == "REST", 2)


def test_looping_trick_stops_when_the_cli_dies(start_daemon):
    d, vb, path, sock = start_daemon()
    p = subprocess.Popen([sys.executable, "-m", "kilo.cli", "trick", "play", "sniff", "--repeats", "255",
                          "--posture", "stand"], cwd=MIND, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        assert wait_for(lambda: vb.trick.running, 15)
        time.sleep(0.5)
        assert vb.trick.running
        p.send_signal(signal.SIGKILL)  # no Ctrl-C handler, no trick_stop: the socket just closes
        p.wait(5)
        assert wait_for(lambda: not vb.trick.running, 2)
        assert wait_for(lambda: vb.posture == "REST", 2)  # then the idle timeline
        assert vb.wd_stage == 0
        assert d.goal_owner.get("TRICK") is None and d._trick_playing is None
    finally:
        if p.poll() is None:
            p.kill()


# ---- 2, 8: goals into a rebooting body -----------------------------------------------------

def test_reconnect_into_a_rebooting_body_does_not_stand_it_up():
    """The port reopens (USB replug / DTR) and the open resets the body, which
    buffers what it hears during its 0.6 s init: the latched STAND (epoch 0,
    as the rebooted body's) must not go out before the body is heard."""
    a, b = loopback_pair()
    vb = VirtualBody(b, boot_delay=0.6).start()
    opens = []

    def factory():
        if not opens:
            opens.append(a)
            return a
        a2, b2 = loopback_pair()
        vb.t = b2
        vb.reboot(P.ResetCause.WARM_UNEXPECTED)  # the open's DTR edge resets it
        opens.append(a2)
        return a2

    link = SerialLink(factory, config=LinkConfig(**FAST), port_name="loopback")
    client = BodyClient(link)
    link.mark_alive()
    link.start()
    try:
        client.wait_ready(2)
        client.stand()
        assert _keep(link, lambda: vb.posture == "STAND")
        assert vb.epoch == 0
        opens[0].fail()  # replug
        postures = []
        end = time.monotonic() + 2.5
        while time.monotonic() < end:
            link.mark_alive()
            postures.append(vb.posture)
            time.sleep(0.02)
        assert len(opens) == 2 and vb.booting_until is None
        i = postures.index("LIMP")  # the reboot
        assert "STAND" not in postures[i:], postures
        assert "SET_POSTURE" not in link.goals() and link.state.reboots == 1
        assert link.state.reset_cause == "WARM_UNEXPECTED"
        # contact is back: a new goal works
        client.stand()
        assert _keep(link, lambda: vb.posture == "STAND")
    finally:
        link.stop()
        vb.stop()


def test_lost_body_gets_no_motion_goal_until_it_is_heard(rig):
    """Same connection: the body reboots (1 s init). Once telemetry has been
    missing for 300 ms no SET_POSTURE refresh goes out; the boot HELLO is a
    reboot (new identity) and clears the goal before any refresh."""
    client, vb, a = rig()
    client.wait_ready(2)
    client.stand()
    assert _keep(client.link, lambda: vb.posture == "STAND")
    sent = []
    dec = KiloCodec().new_decoder()
    real_write = a.write

    def tap(data):
        for f in dec.feed(data):
            if not isinstance(f, DecodeError):
                sent.append((time.monotonic(), client.link.messages.name_of(f.type)))
        real_write(data)
    a.write = tap
    t0 = time.monotonic()
    vb.reboot(P.ResetCause.WARM_UNEXPECTED, boot_delay=1.0)
    keep_alive(client.link, 1.6)
    assert vb.booting_until is None
    late = [n for t, n in sent if t - t0 > 0.45 and n == "SET_POSTURE"]
    assert late == [], late
    assert any(n == "HELLO_REQ" for t, n in sent)  # still polling while lost
    assert "SET_POSTURE" not in client.link.goals() and client.state.reboots == 1
    with pytest.raises(NotConnected):  # TRICK_PLAY waits for contact too
        client.link._contact = False
        client.link.request("TRICK_PLAY", {"epoch": 0, "trick_id": 1, "speed_q6": 64, "flags": 0})


def test_second_reboot_within_3s_still_clears_goals(make_link):
    link, peer = make_link()
    peer.hello(epoch=0)
    peer.telemetry(tick=100, epoch=0)
    assert link.wait_until(lambda: link.state.hello is not None and link.state.tick, 1)
    keep_alive(link, 0.4)
    peer.hello(epoch=0, reset_cause=3)  # unsolicited: reboot 1
    assert link.wait_until(lambda: link.state.reboots == 1, 1)
    link.set_goal("SET_POSTURE", posture=consts.POSTURES["stand"], transition_ticks=0, flags=0)
    peer.telemetry(tick=5, epoch=0)
    keep_alive(link, 0.4)
    peer.hello(epoch=0, reset_cause=2)  # a crash loop: reboot 2 within 3 s
    assert link.wait_until(lambda: "SET_POSTURE" not in link.goals(), 1)
    assert link.state.reboots == 1  # the count is debounced, the clear is not


def test_solicited_hello_with_a_new_identity_is_a_reboot(make_link):
    link, peer = make_link()
    peer.hello(epoch=0)
    peer.telemetry(tick=100, epoch=0)
    assert link.wait_until(lambda: link.state.hello is not None, 1)
    link.set_goal("SET_POSTURE", posture=consts.POSTURES["stand"], transition_ticks=0, flags=0)
    keep_alive(link, 0.1)
    reboots = []
    link.subscribe("reboot", lambda h, why: reboots.append(why))
    # a HELLO inside the solicitation window, but from a new boot
    link._hello_solicited_at = link.clock()
    peer.hello(epoch=0, reset_cause=3)
    assert link.wait_until(lambda: bool(reboots), 1)
    assert "new HELLO" in reboots[0] and "SET_POSTURE" not in link.goals()


def test_runner_forgets_its_posture_cache_on_reboot_or_reconnect():
    clock = Clock(0.0)
    body = FakeBody(clock)
    body.snap.update(reboots=0, connected=True)
    paused = {"on": False}
    runner = Runner(body, day_cfg(), clock=clock, seed=1, log=False, paused=lambda: paused["on"])

    def steps(n=3):
        for _ in range(n):
            runner.step()
            clock.advance(0.1)

    def forgot(change):
        """Apply ``change`` to the snapshot; True if the next tick dropped the
        cached posture (it is gone, or the behaviour re-sent it)."""
        runner.ctx.data["posture"] = ("cached", None)
        k = len(body.of("posture"))
        body.snap.update(change)
        steps(1)
        return runner.ctx.data.get("posture") != ("cached", None) or len(body.of("posture")) > k

    steps()
    n = len(body.of("posture"))
    assert n >= 1
    steps()
    assert len(body.of("posture")) == n  # cached: sent on change only
    assert not forgot({})  # nothing changed: the cache stays
    assert forgot({"reboots": 1})  # same epoch (0 -> 0 across a reboot is the dangerous case)
    assert forgot({"connected": False})
    assert forgot({"connected": True})
    assert forgot({"epoch": 2})
    # pause (the daemon's --override): nothing ticked; resume halts and re-asserts
    m, t = len(body.calls), body.ticks
    paused["on"] = True
    steps()
    assert len(body.calls) == m and body.ticks > t
    paused["on"] = False
    steps()
    assert len(body.of("posture")) > n


# ---- 3, 4, 15: flash ---------------------------------------------------------------------

def _flash_args(**kw):
    a = dict(dry_run=False, port=None, legacy=False, no_build=True, hex=Path("/x.hex"), sketch=None, build_dir=None,
             reset="none", reset_ms=10.0, no_size_check=True, no_verify=False, verify_timeout=0.1, settle_cycles=0,
             baud=250000, verbose=0, force=False, direct=True, socket=None, cmd="flash")
    a.update(kw)
    return Namespace(**a)


def test_flash_refuses_a_busy_port_unless_forced(tmp_path, monkeypatch, capsys):
    import kilo.cli as cli
    import kilo.tools.flash as fl

    port = tmp_path / "ttyFAKE"
    port.write_bytes(b"")
    hexf = tmp_path / "b.hex"
    hexf.write_text(":00000001FF\n")
    ran = []
    monkeypatch.setattr(fl, "find_avrdude", lambda: ("avrdude", None))
    monkeypatch.setattr(fl, "flash", functools.partial(fl.flash, run=lambda cmd, check: ran.append(cmd),
                                                       settle=lambda p: []))
    holder = os.open(port, os.O_RDWR)
    fcntl.flock(holder, fcntl.LOCK_EX | fcntl.LOCK_NB)  # what a daemon's pyserial exclusive=True holds
    try:
        assert fl.port_busy(str(port)) is not None
        args = _flash_args(port=str(port), hex=hexf, no_verify=True)
        assert cli.cmd_flash(args) == 1
        err = capsys.readouterr().err
        assert "in use" in err and "--force" in err and ran == []
        args.force = True
        assert cli.cmd_flash(args) == 0
        assert len(ran) == 1 and "flashing anyway" in capsys.readouterr().err
    finally:
        os.close(holder)
    assert fl.port_busy(str(port)) is None and fl.port_busy(str(tmp_path / "missing")) is None


def test_sudo_finds_the_invoking_users_daemon():
    has = lambda d: True
    paths = [p for p, _ in socket_candidates(env={"SUDO_UID": "1000"}, euid=0, is_dir=has)]
    assert paths == [Path("/run/kilo/kilo.sock"), Path("/run/user/1000/kilo/kilo.sock")]
    assert [p for p, _ in socket_candidates(env={}, euid=0)] == [Path("/run/kilo/kilo.sock")]
    assert [p for p, _ in socket_candidates(env={"SUDO_UID": "0"}, euid=0)] == [Path("/run/kilo/kilo.sock")]


class _FakeConn:
    def __init__(self, port):
        self.info = {"port": port, "pid": 1}
        self.calls = []

    def call(self, op, timeout=None, **kw):
        self.calls.append(op)
        if op == "verify_hello":
            return {"ok": False, "hello": {"fw_build": 0x1234, "reset_cause": 3}}
        return {}

    def close(self):
        self.calls.append("close")


def test_flash_with_a_portless_daemon_and_a_failed_verify(tmp_path, monkeypatch, capsys):
    """15: the daemon has no port yet (conn.info["port"] is None); 4: a verify
    that fails twice exits 1 with the last HELLO."""
    import kilo.cli as cli
    import kilo.tools.flash as fl

    assert fl.is_usb_port(None) is True and fl.is_usb_port("/dev/ttyAMA0") is False
    port = tmp_path / "ttyUSB9"
    port.write_bytes(b"")
    hexf = tmp_path / "b.hex"
    hexf.write_text(":00000001FF\n")
    conn = _FakeConn(None)
    monkeypatch.setattr(cli, "_daemon_for", lambda args: conn)
    monkeypatch.setattr(fl, "find_avrdude", lambda: ("avrdude", None))
    monkeypatch.setattr(fl, "flash", functools.partial(fl.flash, run=lambda cmd, check: None, settle=lambda p: []))
    rc = cli.cmd_flash(_flash_args(port=str(port), hex=hexf, direct=False))
    cap = capsys.readouterr()
    assert rc == 1
    assert conn.calls.count("verify_hello") == 2, cap  # retried once
    assert conn.calls[:1] == ["release_port"] and "reacquire_port" in conn.calls
    assert "verify FAILED" in cap.err and "build 00001234" in cap.err
    assert "verifying once more" in cap.out


# ---- 5: a stalled behaviour runner trips the watchdog -------------------------------------

def test_hung_runner_stops_the_liveness_stamp(start_daemon, monkeypatch, caplog):
    hang = threading.Event()

    def run_behaviour(client, state, stop_event, log_dir):
        end = time.monotonic() + 1.0
        while time.monotonic() < end and not stop_event.is_set():
            client.link.mark_alive()  # the runner's tick (BodyAdapter.on_tick)
            time.sleep(0.1)
        hang.set()
        stop_event.wait(30)  # hung: no more ticks

    fake = types.ModuleType("kilo.behaviour.runner")
    fake.run_behaviour = run_behaviour
    monkeypatch.setitem(sys.modules, "kilo.behaviour.runner", fake)
    caplog.set_level(logging.WARNING, logger="kilo.daemon")
    d, vb, path, sock = start_daemon(behaviour=True, log_dir=Path(sockdir_tmp()))
    d.runner_stall_s = 0.5
    assert wait_for(lambda: d.client.state.alive, 3)
    time.sleep(0.5)
    assert vb.wd_stage == 0  # ticking: the daemon keeps the link alive
    assert hang.wait(3)
    assert wait_for(lambda: vb.wd_stage >= 1, 3)  # runner hung -> no stamps -> the body's watchdog
    assert any("has not ticked" in r.getMessage() for r in caplog.records)


def sockdir_tmp():
    import tempfile

    return tempfile.mkdtemp(prefix="kl")


# ---- 11: the runner vs CLI holds ---------------------------------------------------------

def test_cli_goals_are_refused_while_the_runner_drives_unless_override(start_daemon, monkeypatch):
    seen = {}

    def run_behaviour(client, state, stop_event, log_dir, paused=None):
        seen["paused"] = paused
        while not stop_event.is_set():
            client.link.mark_alive()
            if not paused.is_set():
                client.posture("sit")
            time.sleep(0.05)

    fake = types.ModuleType("kilo.behaviour.runner")
    fake.run_behaviour = run_behaviour
    monkeypatch.setitem(sys.modules, "kilo.behaviour.runner", fake)
    d, vb, path, sock = start_daemon(behaviour=True, log_dir=Path(sockdir_tmp()))
    assert wait_for(lambda: vb.posture == "SIT", 3)
    rc, out, err = _kilo("posture", "stand", "--for", "0.3")
    assert rc == 1 and "behaviour runner active" in err and "--override" in err
    body = remote(sock)
    with pytest.raises(LinkError, match="behaviour runner active"):
        body.set_balance(True)
    with pytest.raises(LinkError, match="behaviour runner active"):
        body.trick_play(name="check_around")
    body.close()
    over = RemoteBody(connect(sock), override=True)
    over.posture("stand")
    assert seen["paused"].is_set()
    assert wait_for(lambda: vb.posture == "STAND", 2)
    over.spin(0.5)
    assert vb.posture == "STAND"  # the paused runner does not fight the hold
    over.close()
    assert wait_for(lambda: not seen["paused"].is_set(), 2)
    assert wait_for(lambda: vb.posture == "SIT", 2)  # resumed: the runner's own goal again
    rc, out, err = _kilo("--override", "posture", "stand", "--for", "0.5")
    assert rc == 0, err


# ---- 6: logs off the control threads ------------------------------------------------------

class _SlowFile:
    def __init__(self):
        self.data = bytearray()
        self.closed = False

    def write(self, b):
        time.sleep(0.02)
        self.data += b

    def flush(self):
        time.sleep(0.05)

    def close(self):
        self.closed = True


def test_kilolog_slow_sink_does_not_delay_callers(tmp_path):
    w = KilologWriter(tmp_path / "x.kilolog")
    real = w._f
    slow = _SlowFile()
    w._f = slow  # before any record: the writer thread picks it up on its first get()
    w._thread.join(0.01)
    t0 = time.monotonic()
    for i in range(50):
        w.write_frame(0x80, i, b"\x01\x02", "rx")
        if i % 10 == 0:
            w.flush()
    took = time.monotonic() - t0
    assert took < 0.05, took  # 50 x 20 ms of disk happen elsewhere
    w.close()
    assert w.records == 50 and w.dropped == 0 and slow.closed
    real.close()
    # a real file round-trips; a full queue drops and counts instead of blocking
    w = KilologWriter(tmp_path / "y.kilolog", queue_size=4)
    w._f, keep = _SlowFile(), w._f
    t0 = time.monotonic()
    for i in range(40):
        w.write_frame(0x80, i, b"", "tx")
    assert time.monotonic() - t0 < 0.05 and w.dropped > 0
    w.close()
    keep.close()
    with KilologWriter(tmp_path / "z.kilolog") as w:
        for i in range(5):
            w.write_frame(0x80, i, b"\x07")
    assert len(list(read_kilolog(tmp_path / "z.kilolog")[1])) == 5


def test_daemon_file_log_does_not_block_the_logging_thread(tmp_path, monkeypatch):
    from logging.handlers import RotatingFileHandler

    def slow_emit(self, record):
        time.sleep(0.05)
    monkeypatch.setattr(RotatingFileHandler, "emit", slow_emit)
    h = dmod.add_file_log(tmp_path / "daemon.log")
    try:
        lg = logging.getLogger("kilo.link")
        t0 = time.monotonic()
        for i in range(20):
            lg.warning("reader thread line %d", i)
        assert time.monotonic() - t0 < 0.1
    finally:
        logging.getLogger().removeHandler(h)
        h.listener.stop()


# ---- 7, 13, 16: client ----------------------------------------------------------------------

def test_cancelled_play_stops_the_trick(rig):
    client, vb, _ = rig()
    client.wait_ready(2)
    client.stand()
    assert _keep(client.link, lambda: vb.posture == "STAND")
    t = library.load("sniff")
    from dataclasses import replace
    loop_forever = replace(t, repeats=255)

    async def go():
        task = asyncio.ensure_future(client.play(loop_forever))
        for _ in range(100):
            await asyncio.sleep(0.02)
            if vb.trick.running:
                break
        assert vb.trick.running
        task.cancel()  # Action.halt() when the runner preempts
        with pytest.raises(asyncio.CancelledError):
            await task
    asyncio.run(go())
    assert _keep(client.link, lambda: not vb.trick.running, 1)
    done = [e for _, e in client.state.events if e.code == consts.EV["TRICK_DONE"]]
    assert done and P.TrickDoneReason((done[-1].arg & 0xFFFF) >> 8) == P.TrickDoneReason.STOPPED


def test_waits_do_not_stamp_liveness(rig):
    client, vb, _ = rig()
    client.wait_ready(2)
    link = client.link
    before = link._mind_alive_at
    th = threading.Thread(target=lambda: link.wait_until(lambda: False, 0.3))
    th.start()
    th.join()
    assert link._mind_alive_at == before  # a pool thread waiting is not the mind's loop
    client.stamp_liveness = False  # the daemon's client
    client.cal_save(0.5)
    assert link._mind_alive_at == before
    client.stand()
    assert _keep(link, lambda: vb.posture == "STAND")
    from dataclasses import replace
    t = replace(library.load("sniff"), repeats=255)

    async def go():
        stamp = link._mind_alive_at
        task = asyncio.ensure_future(client.play(t))
        await asyncio.sleep(0.6)
        assert link._mind_alive_at == stamp  # a looping play() does not keep a dead loop alive
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    asyncio.run(go())


def test_concurrent_uploads_are_serialised(rig):
    client, vb, _ = rig()
    client.wait_ready(2)
    names = ["hi", "sniff", "check_around", "scratch"]
    errors, ids = [], {}

    def up(n):
        try:
            for _ in range(3):
                ids[n] = client.upload_trick(library.load(n), force=True)[0]
        except Exception as e:  # interleaved BEGIN/FRAMES/COMMIT would fail with BAD_ARG/NO_TRICK
            errors.append((n, e))
    ths = [threading.Thread(target=up, args=(n,)) for n in names]
    for th in ths:
        th.start()
    for th in ths:
        th.join(20)
    assert errors == []
    assert vb.trick.state == "ready" and vb.trick.hdr.trick_id in ids.values()
    assert client._trick_cached[0] == vb.trick.hdr.trick_id


# ---- 10: idle latching with split owners -----------------------------------------------------

def test_idle_rest_when_the_posture_owner_leaves_before_the_balance_owner(start_daemon):
    d, vb, path, sock = start_daemon()
    a, b = remote(sock), remote(sock)
    a.wait_ready(3)
    a.set_balance(True)
    b.posture("stand")
    assert wait_for(lambda: vb.posture == "STAND")
    b.close()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.6:  # a keeps its balance hold alive
        a.link.mark_alive()
        time.sleep(0.05)
    assert vb.posture == "REST"  # posture unowned: idle REST although a still holds SET_BALANCE
    a.close()
    assert wait_for(lambda: vb.balance["enable"] == 0, 1.0)
    assert wait_for(lambda: vb.posture == "LIMP", 2.5)
    assert vb.wd_stage == 0


# ---- 12: LinkStats race ------------------------------------------------------------------------

def test_link_stats_snapshot_under_concurrent_counter_growth():
    # Two writers add fresh Counter keys (each key a size change) while two readers copy the
    # stats. The growth is bounded (KEYS per writer) so each copy stays cheap, and the readers
    # copy for as long as the writers are adding keys, so the race window is the whole run.
    # (On CPython with the GIL dict(Counter) is a single C call, so the lock matters for
    # free-threaded builds and for any Python-level iteration added to as_dict() later.)
    KEYS = 4000
    st = BodyState()
    link = SerialLink(lambda: None, state=st)
    growing = threading.Event()
    growing.set()
    errors = []
    reads = [0, 0]
    start = threading.Barrier(4)

    def grow(w):
        start.wait()
        for i in range(KEYS):
            link._handle(DecodeError(f"w{w}k{i}"), 0.0)

    def read(r):
        start.wait()
        try:
            while growing.is_set() or reads[r] == 0:
                st.snapshot()
                reads[r] += 1
        except RuntimeError as e:
            errors.append(e)
    writers = [threading.Thread(target=grow, args=(w,)) for w in range(2)]
    readers = [threading.Thread(target=read, args=(r,)) for r in range(2)]
    for t in writers + readers:
        t.start()
    for t in writers:
        t.join(20)
    growing.clear()
    for t in readers:
        t.join(20)
    assert not any(t.is_alive() for t in writers + readers), "a thread did not finish"
    assert errors == []
    assert min(reads) >= 1 and sum(reads) > 20, reads  # the readers really overlapped the growth
    d = st.stats.as_dict()
    assert "lock" not in d and len(d["decode_errors"]) == 2 * KEYS


# ---- 14: log directory cap ------------------------------------------------------------------------

def test_prune_logs_deletes_the_oldest_beyond_the_cap(tmp_path):
    from kilo.log import prune_logs

    now = time.time()
    files = []
    for i, name in enumerate(["kilo-1.jsonl", "kilo-1.jsonl.1", "behaviour-1.jsonl", "kilo-2.kilolog.gz",
                              "notes.txt", "kilo-open.jsonl", "kilo-fresh.jsonl"]):
        p = tmp_path / name
        p.write_bytes(b"x" * 1000)
        age = 10000 - i * 100 if name != "kilo-fresh.jsonl" else 10
        os.utime(p, (now - age, now - age))
        files.append(p)
    gone = prune_logs(tmp_path, 3500, keep={tmp_path / "kilo-open.jsonl"}, now=now)
    # 7000 B: the four oldest kilo logs go (notes.txt is not ours; the open and the fresh one are kept)
    assert [p.name for p in gone] == ["kilo-1.jsonl", "kilo-1.jsonl.1", "behaviour-1.jsonl", "kilo-2.kilolog.gz"]
    assert sorted(p.name for p in tmp_path.iterdir()) == ["kilo-fresh.jsonl", "kilo-open.jsonl", "notes.txt"]
    assert prune_logs(tmp_path, 10 ** 9, now=now) == []


def test_daemon_prunes_its_log_dir_at_start(start_daemon, tmp_path):
    old = tmp_path / "kilo-old.jsonl"
    old.write_bytes(b"x" * 4096)
    os.utime(old, (time.time() - 7200,) * 2)
    d, vb, path, sock = start_daemon(log_dir=tmp_path, log_cap_bytes=1024)
    assert wait_for(lambda: not old.exists(), 3)
