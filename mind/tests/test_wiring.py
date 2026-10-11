"""The mind's pieces wired together: socket candidates, port hot-plug, the
behaviour runner under the daemon, raw .kilolog capture and `kilo log`,
`kilo monitor --percepts`, the daemon's own log file."""

import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from kilo.behaviour.adapter import BodyAdapter
from kilo.behaviour.body_events import BodyEvents
from kilo.behaviour.runner import PerceptFeed, Runner, behaviour_percepts
from kilo.cli_tools import daemon_link_lines, fix_kilolog_argv
from kilo.daemon import Daemon
from kilo.link import consts
from kilo.link.codec import DecodeError, Frame, KiloCodec
from kilo.link.serial_link import LinkConfig
from kilo.logs.kilolog import KilologWriter, read_kilolog
from kilo.perception.shared import SharedPercepts
from kilo.remote import RemoteBody, connect
from kilo.vbody.sim import VirtualBody, open_pty

from conftest import FAST
from test_daemon import MIND, _kilo, sockdir, start_daemon, vbody_pty, wait_for  # noqa: F401 (fixtures)


def _jsonl(path):
    return [json.loads(ln) for ln in Path(path).read_text().splitlines() if ln.strip()]


# ---- codec and link: wire bytes ----------------------------------------------------------

def test_feed_wire_pairs_each_frame_with_its_wire_bytes():
    from kilo.link.messages import KiloMessages

    c, m = KiloCodec(), KiloMessages()
    a, b = c.encode(m.type_of("HEARTBEAT"), 1, b""), c.encode(m.type_of("HELLO_REQ"), 2, b"")
    junk = b"\x05\x06\x07\x00"
    stream = b"\x00" + a + junk + b
    d = c.new_decoder()
    got = []
    for i in range(0, len(stream), 3):  # arbitrary chunking
        got += d.feed_wire(stream[i:i + 3])
    frames = [(r, w) for r, w in got if isinstance(r, Frame)]
    errors = [(r, w) for r, w in got if isinstance(r, DecodeError)]
    assert [w for _, w in frames] == [a, b]
    assert [r for r, _ in frames] == c.new_decoder().feed(a + b)
    assert len(errors) == 1 and errors[0][1] == junk


def test_link_writes_rx_and_tx_wire_frames(make_link, tmp_path):
    link, peer = make_link()
    w = KilologWriter(tmp_path / "l.kilolog")
    link.wire_logger = w
    for i in range(5):
        peer.telemetry(tick=i + 1)
        time.sleep(0.02)
    assert peer.wait_for(lambda: peer.of("HEARTBEAT"))
    link.stop()
    w.close()
    meta, recs = read_kilolog(w.path)
    recs = list(recs)
    assert all(r.error is None for r in recs)
    rx = [r for r in recs if r.direction == "rx"]
    assert len(rx) == 5 and {r.frame.type for r in rx} == {link.messages.type_of("TELEMETRY")}
    assert any(r.direction == "tx" for r in recs)


# ---- argv and status helpers ----------------------------------------------------------------

def test_bare_kilolog_does_not_swallow_the_command():
    cmds = {"status", "run"}
    assert fix_kilolog_argv(["--kilolog", "status"], cmds) == ["--kilolog=", "status"]
    assert fix_kilolog_argv(["--kilolog", "-v", "status"], cmds) == ["--kilolog=", "-v", "status"]
    assert fix_kilolog_argv(["--kilolog", "f.kilolog", "status"], cmds) == ["--kilolog", "f.kilolog", "status"]
    assert fix_kilolog_argv(["run", "--behaviour", "--kilolog"], cmds) == ["run", "--behaviour", "--kilolog="]


def test_kilo_daemon_lists_the_sockets_it_tried(sockdir, monkeypatch):
    sock = sockdir / "nobody.sock"
    env = dict(os.environ, KILO_SOCKET=str(sock))
    rc, out, err = _kilo("daemon", env=env)
    assert rc == 1 and f"no daemon listening on {sock}" in out
    assert f"tried {sock}: no socket file" in out


# ---- port hot-plug ---------------------------------------------------------------------------

def test_daemon_waits_for_the_port_and_survives_a_replug(sockdir, monkeypatch):
    """Start with no port (robot unplugged), plug a vbody in, pull it, plug another."""
    port = sockdir / "tty"  # a symlink to the vbody's pty, made later
    sock = sockdir / "hp.sock"
    monkeypatch.setenv("KILO_SOCKET", str(sock))
    d = Daemon(str(port), socket=sock, link_config=LinkConfig(**FAST), hold_lease_s=1.0, idle_limp_s=1.0)
    th = threading.Thread(target=d.run, kwargs={"install_signals": False}, daemon=True)
    th.start()
    assert d.ready.wait(5)
    vbs = []

    def plug():
        t, path, slave = open_pty()
        vb = VirtualBody(t).start()
        vbs.append((vb, t, slave))
        os.symlink(path, port)
        return vb

    def unplug():
        vb, t, slave = vbs.pop()
        vb.stop()
        t.close()
        os.close(slave)
        port.unlink()

    conn = connect(sock)
    got = []
    try:
        conn.subscribe(got.append, kinds=("connection",))
        info = conn.call("daemon")
        assert info["connected"] is False and info["port_present"] is False
        assert "does not exist" in info["open_error"] and info["open_failures"] >= 1
        assert any("MISSING" in ln for ln in daemon_link_lines(info))
        time.sleep(0.3)
        assert d.client.state.connected is False
        plug()
        assert wait_for(lambda: conn.call("daemon")["alive"], 5)
        body = RemoteBody(conn)
        body.wait_ready(3)
        body.posture("sit")
        assert wait_for(lambda: vbs[0][0].posture == "SIT", 3)
        body.release()
        unplug()  # USB pulled: the read fails, the daemon closes the port and keeps trying
        assert wait_for(lambda: not conn.call("daemon")["connected"], 5)
        info = conn.call("daemon")
        assert info["disconnects"] >= 1 and not info["port_present"]
        vb2 = plug()
        assert wait_for(lambda: conn.call("daemon")["alive"], 5)
        info = conn.call("daemon")
        assert info["reconnects"] >= 1 and info["open_error"] is None and info["port_present"]
        assert body.param_get("WD_LIMP_S").value == 10  # the new body answers
        body.posture("stand")
        assert wait_for(lambda: vb2.posture == "STAND", 3)
        states = [m["data"]["connected"] for m in got if m["stream"] == "connection"]
        assert True in states and False in states and states.index(False) > states.index(True)
        assert all(m["data"]["port"] == str(port) for m in got if m["stream"] == "connection")
    finally:
        conn.close()
        d.request_stop()
        th.join(5)
        while vbs:
            unplug()


# ---- behaviour through the daemon --------------------------------------------------------------

def test_behaviour_runner_under_the_daemon(start_daemon, monkeypatch, tmp_path):
    motions = []
    orig = SharedPercepts.notify_self_motion

    def spy(self, t=None, seconds=0.0):
        motions.append(seconds)
        return orig(self, t, seconds)
    monkeypatch.setattr(SharedPercepts, "notify_self_motion", spy)
    log_dir = tmp_path
    d, vb, path, sock = start_daemon(behaviour=True, log_dir=log_dir)
    assert wait_for(lambda: list(log_dir.glob("behaviour-*.jsonl")), 3)
    blog = next(log_dir.glob("behaviour-*.jsonl"))
    assert wait_for(lambda: vb.posture != "LIMP", 5), vb.posture  # the runner's goal reached the body
    assert motions and motions[0] > 0  # notify_self_motion before the posture move
    assert d.behaviour_state == "running"
    conn = connect(sock)
    try:
        assert conn.info["behaviour"] == "running"
        assert set(conn.call("percepts")["flags"]) >= {"near", "approaching", "tap", "pet", "startle"}
    finally:
        conn.close()
    assert wait_for(lambda: d.client.state.alive, 3)
    time.sleep(0.5)  # a few runner ticks see the live body (epoch 0) before it falls
    vb.trigger_fall(1)
    assert wait_for(lambda: any(r.get("kind") == "body_event" and r.get("name") == "FELL"
                                for r in _jsonl(blog)), 5)
    recs = _jsonl(blog)
    assert any(r.get("kind") == "body_event" and r.get("to") == 1 for r in recs)  # epoch 0 -> 1
    d.request_stop()
    assert wait_for(lambda: d.behaviour_state == "finished", 6)
    assert wait_for(lambda: _jsonl(blog)[-1].get("kind") == "stop", 3)
    assert _jsonl(blog)[0]["kind"] == "start"
    assert wait_for(lambda: vb.wd_stage >= 1, 2)  # the daemon went silent: the body's watchdog took over


def test_kilo_run_behaviour_kilolog_end_to_end(vbody_pty, sockdir, tmp_path):
    """`kilo run --behaviour --kilolog` against the vbody, `kilo monitor
    --percepts` through it, SIGTERM, then `kilo log summary|replay` on the file."""
    vb, path = vbody_pty
    sock = sockdir / "e2e.sock"
    log_dir = tmp_path
    env = dict(os.environ, KILO_SOCKET=str(sock))
    p = subprocess.Popen([sys.executable, "-m", "kilo.cli", "run", "--port", path, "--log-dir", str(log_dir),
                          "--behaviour", "--kilolog"], cwd=MIND, env=env, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, text=True)
    try:
        assert wait_for(lambda: sock.exists(), 10)
        assert wait_for(lambda: _kilo("daemon", env=env)[1].count("alive=True"), 10)
        rc, out, err = _kilo("daemon", env=env)
        assert rc == 0 and "behaviour   running" in out and "link        up" in out
        assert "raw log     " + str(log_dir) in out and ".kilolog.gz" in out
        mon = subprocess.Popen([sys.executable, "-m", "kilo.cli", "monitor", "--percepts", "--duration", "3",
                                "--rate", "4"], cwd=MIND, env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True)
        time.sleep(1.5)
        vb.trigger_fall(1)
        out, err = mon.communicate(timeout=20)
        assert mon.returncode == 0, err
        lines = [json.loads(ln) for ln in out.splitlines()]
        assert any(ln.get("percept") == "startle" for ln in lines), out
        assert any("startle" in (ln.get("percepts") or []) for ln in lines), out
        assert wait_for(lambda: vb.posture != "LIMP", 5)
        p.send_signal(signal.SIGTERM)
        p.wait(15)
        assert p.returncode == 0 and not sock.exists()
    finally:
        if p.poll() is None:
            p.kill()
    dlog = (log_dir / "daemon.log").read_text()
    assert f"socket {sock}" in dlog and "behaviour runner started" in dlog and "shutting down" in dlog
    blogs = list(log_dir.glob("behaviour-*.jsonl"))
    assert len(blogs) == 1 and _jsonl(blogs[0])[-1]["kind"] == "stop"
    kls = list(log_dir.glob("kilo-*.kilolog.gz"))
    assert len(kls) == 1
    rc, out, err = _kilo("log", "summary", str(kls[0]), env=env)
    assert rc == 0, err
    assert "(kilolog)" in out and "telemetry samples" in out and "SET_POSTURE" in out and "FELL" in out
    rc, out, err = _kilo("log", "replay", str(kls[0]), "--json", env=env)
    assert rc == 0, err
    names = [json.loads(ln)["name"] for ln in out.splitlines()]
    assert "startle" in names


def test_direct_kilolog_round_trip(vbody_pty, tmp_path):
    vb, path = vbody_pty
    f = tmp_path / "direct.kilolog"
    rc, out, err = _kilo("--port", path, "--kilolog", str(f), "posture", "sit", "--for", "0.5")
    assert rc == 0, err
    assert f"raw frame log {f}" in err
    meta, recs = read_kilolog(f)
    recs = list(recs)
    assert meta["port"] == path and meta["source"] == "kilo posture"
    assert {r.direction for r in recs} == {"rx", "tx"} and all(r.error is None for r in recs)
    rc, out, err = _kilo("log", "summary", str(f))
    assert rc == 0 and "SET_POSTURE" in out and "HEARTBEAT" in out, out
    rc, out, err = _kilo("log", "replay", str(f))
    assert rc == 0 and "percept events" in out
    rc, out, err = _kilo("log")
    assert rc == 2 and "summary" in out


def test_monitor_percepts_direct(vbody_pty):
    vb, path = vbody_pty
    mon = subprocess.Popen([sys.executable, "-m", "kilo.cli", "--port", path, "monitor", "--percepts",
                            "--duration", "2.5", "--rate", "4"], cwd=MIND, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, text=True)
    time.sleep(1.2)
    vb.trigger_fall(-1)
    out, err = mon.communicate(timeout=20)
    assert mon.returncode == 0, err
    lines = [json.loads(ln) for ln in out.splitlines()]
    assert any(ln.get("percept") == "startle" for ln in lines), out
    assert all("percepts" in ln for ln in lines if "percept" not in ln)


# ---- behaviour-side units -------------------------------------------------------------------

def test_body_events_from_snapshots():
    be = BodyEvents()
    ev = lambda tick, name, arg=0, epoch=0: (0.1, {"code": consts.EV[name], "arg": arg, "tick": tick,
                                                   "epoch": epoch})
    assert be.update({"epoch": 0, "connected": True, "alive": True, "events": [ev(1, "BOOT")]}) == []
    out = be.update({"epoch": 1, "connected": True, "alive": True,
                     "events": [ev(1, "BOOT"), ev(50, "FELL", 1, 1)]})
    assert out == [{"type": "epoch", "from": 0, "to": 1},
                   {"type": "event", "name": "FELL", "arg": 1, "epoch": 1, "tick": 50}]
    assert be.update({"epoch": 1, "connected": True, "alive": True,
                      "events": [ev(1, "BOOT"), ev(50, "FELL", 1, 1)]}) == []
    assert be.update({"epoch": 1, "connected": False, "alive": False, "events": []}) == [
        {"type": "connection", "connected": False, "alive": False}]


def test_percept_mapping_and_self_motion():
    p = behaviour_percepts({"pet": True, "near": True, "approaching": False, "tap": True, "startle": True,
                            "events": []})
    assert p["face"] is False and p["pet"] and p["near"] and not p["approaching"] and p["tap"]
    assert p["startle"] == 1.0 and "percept_events" not in p

    class Client:
        def __init__(self):
            self.calls = []

        def posture(self, name, balance=None):
            self.calls.append(name)

    sp = SharedPercepts()
    adapter = BodyAdapter(Client(), percepts=sp)
    adapter.posture("sit")
    assert sp.percepts.imu._quiet_from > 0  # IMU detectors stand down for the move
    assert adapter.client.calls == ["sit"]


def test_runner_keeps_a_fall_startle_over_a_quiet_percept_source():
    class Body:
        def __init__(self):
            self.snap = {"epoch": 0, "fallen": False, "events": []}

        def snapshot(self):
            return dict(self.snap)

        def posture(self, name, balance=None):
            pass

        def set_balance(self, enable):
            pass

    body = Body()
    r = Runner(body, None, percepts=lambda: {"startle": 0.0, "pet": False}, log=False, seed=1)
    r.step()
    assert "startle" not in r.ctx.percepts
    body.snap["fallen"] = True
    r.step()
    assert r.ctx.percepts["startle"] == 1.0
    feed = PerceptFeed(state=None)
    assert feed()["face"] is False  # nothing to read yet: defaults
