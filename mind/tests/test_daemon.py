"""The daemon (`kilo run`) over a real Unix socket, against the virtual body
on a pty: every op, hold semantics, streams, a slow client, port release,
shutdown, the behaviour hook, and the CLI through the daemon vs direct."""

import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

import pytest

from kilo import daemon as dmod
from kilo.body.client import BodyClient
from kilo.body.state import BodyState
from kilo.daemon import Daemon
from kilo.link import consts
from kilo.protocol import kilo_protocol as P
from kilo.link.serial_link import CommandError, LinkConfig, LinkError, NotConnected
from kilo.remote import DaemonError, RemoteBody, connect, socket_candidates, socket_path
from kilo.vbody.sim import VirtualBody, open_pty

from conftest import FAST

MIND = Path(__file__).resolve().parents[1]


def wait_for(pred, timeout=3.0, period=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(period)
    return pred()


@pytest.fixture
def vbody_pty():
    transport, path, slave = open_pty()
    vb = VirtualBody(transport).start()
    yield vb, path
    vb.stop()
    transport.close()
    os.close(slave)


@pytest.fixture
def sockdir():
    d = tempfile.mkdtemp(prefix="kd", dir="/tmp")  # short: AF_UNIX paths are limited to 108 bytes
    yield Path(d)
    shutil.rmtree(d, ignore_errors=True)


@pytest.fixture
def start_daemon(vbody_pty, sockdir, monkeypatch):
    made = []

    def start(**kw):
        vb, path = vbody_pty
        sock = sockdir / "k.sock"
        monkeypatch.setenv("KILO_SOCKET", str(sock))
        kw.setdefault("link_config", LinkConfig(**FAST))
        kw.setdefault("hold_lease_s", 1.0)
        kw.setdefault("idle_limp_s", 1.0)
        d = Daemon(path, socket=sock, **kw)
        t = threading.Thread(target=d.run, kwargs={"install_signals": False}, daemon=True)
        t.start()
        assert d.ready.wait(5)
        made.append((d, t))
        return d, vb, path, sock

    yield start
    for d, t in made:
        d.request_stop()
        t.join(5)


def remote(sock):
    conn = connect(sock)
    assert conn is not None
    return RemoteBody(conn)


# ---- socket path ---------------------------------------------------------------------

def test_socket_path_resolution():
    assert socket_path("/x/y.sock") == Path("/x/y.sock")
    assert socket_path(env={"KILO_SOCKET": "/s"}, euid=1000) == Path("/s")
    assert socket_path(env={"XDG_RUNTIME_DIR": "/run/user/1000"}, euid=0) == Path("/run/kilo/kilo.sock")
    assert socket_path(env={"XDG_RUNTIME_DIR": "/run/user/1000"}, euid=1000) == Path(
        "/run/user/1000/kilo/kilo.sock")
    no_run = lambda d: False
    assert socket_path(env={}, euid=1000, home="/home/joe", is_dir=no_run) == Path("/home/joe/.kilo/kilo.sock")
    # a non-login SSH shell: no $XDG_RUNTIME_DIR, but the systemd user runtime dir exists
    has_run = lambda d: d == "/run/user/1000"
    assert socket_path(env={}, euid=1000, home="/home/joe", is_dir=has_run) == Path(
        "/run/user/1000/kilo/kilo.sock")
    assert [p for p, _ in socket_candidates(env={}, euid=1000, home="/home/joe", is_dir=has_run)] == [
        Path("/run/user/1000/kilo/kilo.sock"), Path("/home/joe/.kilo/kilo.sock")]
    assert [p for p, _ in socket_candidates(env={"XDG_RUNTIME_DIR": "/run/user/1000"}, euid=1000,
                                            home="/home/joe", is_dir=has_run)] == [
        Path("/run/user/1000/kilo/kilo.sock"), Path("/home/joe/.kilo/kilo.sock")]  # deduplicated
    assert len(socket_candidates(env={"KILO_SOCKET": "/s"}, euid=1000)) == 1


def test_connect_tries_every_candidate(sockdir, monkeypatch, start_daemon):
    """No --socket and no $KILO_SOCKET: the client walks the candidates and
    finds a daemon on the second one; `kilo daemon` lists what it tried."""
    import kilo.remote as remote_mod

    d, vb, path, sock = start_daemon()
    first = sockdir / "first.sock"
    monkeypatch.setattr(remote_mod, "socket_candidates",
                        lambda explicit=None, **kw: [(Path(explicit), "--socket")] if explicit else
                        [(first, "first"), (sock, "second")])
    tried = []
    conn = connect(tried=tried)
    try:
        assert conn is not None and conn.path == sock
        assert tried == [(first, "no socket file"), (sock, "daemon answered")]
    finally:
        conn.close()
    monkeypatch.setattr(remote_mod, "socket_candidates",
                        lambda explicit=None, **kw: [(first, "first"), (sockdir / "x.sock", "second")])
    tried = []
    assert connect(tried=tried) is None and [p for p, _ in tried] == [first, sockdir / "x.sock"]


def test_connect_without_daemon(sockdir):
    assert connect(sockdir / "missing.sock") is None
    stale = sockdir / "stale.sock"
    s = socket.socket(socket.AF_UNIX)
    s.bind(str(stale))
    s.close()  # a socket file nobody listens on
    assert connect(stale) is None


# ---- ops -------------------------------------------------------------------------------

def test_ops_over_socket(start_daemon):
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        info = body.conn.info
        assert info["pid"] == os.getpid() and info["port"] == path and not info["port_released"]
        hello = body.wait_ready(3)
        assert hello.fw_build == 0x56424F44 and hello.reset_cause == 0
        snap = body.state.snapshot()
        assert snap["alive"] and snap["port"] == path and isinstance(snap["owners"], tuple)
        assert body.stats()["uptime_s"] >= 0
        assert body.param_get("WD_LIMP_S").value == 10
        body.param_set("WD_LIMP_S", 12)
        assert body.param_get(2).value == 12 and vb.params[2] == 12
        assert len(body.param_list()) == len(vb.params)
        body.cal_set(3, 16)
        assert body.cal_get()[3] == 16
        body.cal_revert()
        assert body.cal_get()[3] == 0
        assert body.cal_save(2.0) is True
        assert len(body.pca_counts()) == 16
        vb.eeprom[0:2] = b"\x12\x34"
        assert body.eeprom_read(0, 4) == b"\x12\x34\xff\xff"
        assert body.request("PING", {"token": 0x55}).token == 0x55
        assert body.hello().proto_major == hello.proto_major
        assert body.ping() > 0  # generic proxy (`call` op) for other BodyClient methods
        # errors come back as the same exception types
        with pytest.raises(CommandError) as e:
            body.param_get(200)
        assert e.value.status_name == "BAD_ARG"
        with pytest.raises(KeyError):
            body.param_get("NO_SUCH_PARAM")
        with pytest.raises(ValueError):
            body.param_set("WD_LIMP_S", 100000)
        with pytest.raises(ValueError):
            body.posture("jump")
        with pytest.raises(ValueError):
            body.conn.call("no_such_op")
        with pytest.raises(ValueError):
            body.conn.call("request", name="SET_POSTURE")  # goals go through posture/set_balance
        assert body.conn.call("ping") == "pong"
    finally:
        body.close()


def test_hold_ends_when_the_client_disconnects(start_daemon):
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    body.wait_ready(3)
    body.set_balance(True, kp=0.6)
    body.posture("stand")
    assert wait_for(lambda: vb.posture == "STAND")
    assert set(body.link.goals()) == {"SET_POSTURE", "SET_BALANCE"}
    assert d.op_daemon  # info shows the hold
    info = body.conn.call("daemon")
    assert info["held_goals"] == ["SET_BALANCE", "SET_POSTURE"]
    body.spin(1.5)  # alive pings keep the lease (1 s) going
    assert vb.posture == "STAND" and vb.wd_stage == 0
    body.close()
    # the hold ends: REST at once, balance off, then LIMP after idle_limp_s (1 s); never the watchdog
    assert wait_for(lambda: vb.posture == "REST", 1.0)
    assert wait_for(lambda: vb.balance["enable"] == 0, 1.0)
    assert wait_for(lambda: vb.posture == "LIMP", 2.5)
    assert vb.wd_stage == 0


def test_balance_on_by_default(start_daemon):
    """kilo run --balance [KP]: SET_BALANCE latched on at start, owned by nobody;
    a client's balance hold ending goes back to on; it survives a body reboot
    and a port release."""
    d, vb, path, sock = start_daemon(balance_kp=0.99)
    body = remote(sock)
    try:
        body.wait_ready(3)
        assert wait_for(lambda: vb.balance["enable"] == 1 and vb.balance["kp_q8"] == 253)
        info = body.conn.call("daemon")
        assert info["balance_default"] == 0.99 and info["held_goals"] == []
        # a client turns it off and holds that; when its hold ends, back to the default
        body.set_balance(False, kp=0.3)
        assert wait_for(lambda: vb.balance["enable"] == 0 and vb.balance["kp_q8"] == 77)
        assert body.conn.call("daemon")["held_goals"] == ["SET_BALANCE"]
        body.release()
        assert wait_for(lambda: vb.balance["enable"] == 1 and vb.balance["kp_q8"] == 253)
        assert body.conn.call("daemon")["held_goals"] == []
        # a body reboot: the goal is not an epoch goal, so the link keeps sending it
        vb.reboot()
        assert wait_for(lambda: vb.balance["enable"] == 0, 3)  # power-on state
        assert wait_for(lambda: vb.balance["enable"] == 1, 5)
        # a port release drops every goal; the reacquire latches the default again
        body.conn.call("release_port", lease_s=30)
        assert body.link.goals() == {}
        body.conn.call("reacquire_port")
        body.wait_ready(3)
        assert wait_for(lambda: "SET_BALANCE" in d.link.goals(), 3)  # the daemon's own latch, not a client's
        assert wait_for(lambda: vb.balance["enable"] == 1 and vb.balance["kp_q8"] == 253, 3)
    finally:
        body.close()


@pytest.mark.parametrize("held", [False, True], ids=["default", "client-hold"])
def test_balance_returns_after_an_epoch_bump(start_daemon, held):
    """Cross-layer (review §2.4): the body clears SET_BALANCE's enable on an
    epoch bump (body.cpp body_epoch_bump, P1 finding B1); SET_BALANCE carries
    no epoch, so the mind keeps it and its 5 Hz goal refresh turns it back on
    within one refresh period of the body hearing the Pi again. The
    bump here is the body's own watchdog stop: the vbody drops every frame from
    the Pi for longer than WD_STOP_MS, then hears them again."""
    d, vb, path, sock = start_daemon(balance_kp=None if held else 0.99)
    body = remote(sock)
    try:
        body.wait_ready(3)
        if held:
            body.set_balance(True, kp=0.99)
        assert wait_for(lambda: vb.balance["enable"] == 1 and vb.balance["kp_q8"] == 253)
        epoch = vb.epoch
        pi_types = {vb.msgs.name_of(t) for t in P.MESSAGES if t < 0x80}
        vb.ignore_types = pi_types  # the link goes silent for the body
        assert wait_for(lambda: vb.wd_stage == 1, 2)
        assert vb.epoch == (epoch + 1) & 0xFF
        assert vb.balance["enable"] == 0 and vb.balance["kp_q8"] == 253  # the goal cleared, gains kept
        vb.ignore_types = set()
        t0 = time.monotonic()
        assert wait_for(lambda: vb.balance["enable"] == 1, 1.0, period=0.005)
        took = time.monotonic() - t0
        assert took < 0.3, took  # within one 200 ms refresh period of the body hearing the Pi again
        assert vb.wd_stage == 0 and vb.balance["kp_q8"] == 253
    finally:
        body.close()


def test_balance_default_off_without_the_flag(start_daemon):
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        body.spin(0.5)
        assert vb.balance["enable"] == 0 and body.link.goals() == {}
        assert body.conn.call("daemon")["balance_default"] is None
    finally:
        body.close()


def test_hold_lease_lapses_without_pings(start_daemon):
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        body.posture("sit")
        assert wait_for(lambda: vb.posture == "SIT")
        time.sleep(1.6)  # lease 1 s, nothing sent
        assert vb.posture == "REST"
        assert body.link.goals() == {}
    finally:
        body.close()


def test_two_clients_last_writer_owns_the_goal(start_daemon):
    d, vb, path, sock = start_daemon()
    a, b = remote(sock), remote(sock)
    try:
        a.wait_ready(3)
        a.posture("sit")
        b.posture("stand")
        assert wait_for(lambda: vb.posture == "STAND")
        assert a.link.goals() == {} and set(b.link.goals()) == {"SET_POSTURE"}
        a.close()  # a no longer owns anything: b's hold survives
        b.spin(0.5)
        assert vb.posture == "STAND"
    finally:
        b.close()


def test_idle_posture_none_leaves_the_posture(start_daemon):
    d, vb, path, sock = start_daemon(idle_posture="none")
    body = remote(sock)
    body.wait_ready(3)
    body.posture("stand")
    assert wait_for(lambda: vb.posture == "STAND")
    body.close()
    time.sleep(0.8)
    assert vb.posture == "STAND" and vb.wd_stage == 0  # daemon still feeds the watchdog


# ---- streams -----------------------------------------------------------------------------

def test_subscription_streams_at_the_requested_rate(start_daemon):
    d, vb, path, sock = start_daemon()
    conn = connect(sock)
    got = []
    try:
        conn.subscribe(got.append, rate_hz=5, kinds=("snapshot", "telemetry", "event"))
        time.sleep(0.1)
        vb.trigger_fall(1)
        time.sleep(1.5)
        snaps = [m for m in got if m["stream"] == "snapshot"]
        tel = [m for m in got if m["stream"] == "telemetry"]
        ev = [m for m in got if m["stream"] == "event"]
        assert 6 <= len(snaps) <= 10, len(snaps)   # 5 Hz for 1.6 s
        assert 6 <= len(tel) <= 10, len(tel)       # 25 Hz telemetry decimated to 5 Hz
        assert any(e["data"]["code"] == consts.EV["FELL"] for e in ev), ev
        assert "pitch_deg" in snaps[-1]["data"] and "rate_pitch" in tel[-1]["data"]
        with pytest.raises(ValueError):
            conn.call("subscribe", rate_hz=500)
        with pytest.raises(ValueError):
            conn.call("subscribe", kinds=["nope"])
        conn.call("unsubscribe")
        n = len(got)
        time.sleep(0.5)
        assert len(got) <= n + 1
    finally:
        conn.close()


def test_a_slow_client_does_not_block_the_loop(start_daemon, monkeypatch):
    monkeypatch.setattr(dmod.streams, "STREAM_BUFFER_MAX", 4096)
    d, vb, path, sock = start_daemon()
    slow = socket.socket(socket.AF_UNIX)
    slow.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    slow.connect(str(sock))
    slow.sendall(b'{"id":1,"op":"subscribe","args":{"rate_hz":50,"kinds":["snapshot","telemetry"]}}\n')
    body = remote(sock)
    try:
        worst = 0.0
        end = time.monotonic() + 2.0
        while time.monotonic() < end:  # the slow client never reads
            t0 = time.monotonic()
            body.state.snapshot()
            worst = max(worst, time.monotonic() - t0)
            time.sleep(0.02)
        assert worst < 0.25, worst
        assert body.state.snapshot()["alive"] and vb.wd_stage == 0
        sess = [s for s in d.sessions.values() if s.kinds]
        assert sess and sess[0].dropped > 0
        # once it reads again it is told how much it lost
        slow.setblocking(False)
        data = b""
        deadline = time.monotonic() + 3
        while b'"stream":"dropped"' not in data and time.monotonic() < deadline:
            try:
                data += slow.recv(1 << 20)
            except BlockingIOError:
                time.sleep(0.02)
        assert b'"stream":"dropped"' in data
    finally:
        slow.close()
        body.close()


# ---- port release ---------------------------------------------------------------------------

def test_release_and_reacquire_port(start_daemon):
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.wait_ready(3)
        body.posture("stand")
        r = body.conn.call("release_port", lease_s=30)
        assert r == {"port": path, "lease_s": 30.0}
        assert body.conn.call("daemon")["port_released"] and not d.client.state.connected
        assert body.link.goals() == {}  # holds dropped while flashing
        with pytest.raises(NotConnected):
            body.param_get("WD_LIMP_S")
        with pytest.raises(NotConnected):
            body.posture("sit")
        with pytest.raises(Exception):
            body.conn.call("release_port")  # already released
        n_rx = len(vb.received)
        time.sleep(0.3)
        assert len(vb.received) == n_rx  # nothing written while released
        assert body.conn.call("reacquire_port") == {"port": path, "reacquired": True}
        assert body.wait_ready(3).proto_major >= 1
        assert body.param_get("WD_LIMP_S").value == 10
        assert body.conn.call("reacquire_port")["reacquired"] is False
    finally:
        body.close()


def test_release_lease_expires_and_disconnect_keeps_the_lease(start_daemon, monkeypatch):
    """Finding 9: a flashing CLI that dies mid-avrdude must not get the port
    reopened under avrdude: the lease is honoured after the disconnect, and
    every reopen settles the port first."""
    settled = []
    monkeypatch.setattr(dmod.port, "settle_port", lambda port, baud=None, **kw: settled.append(port) or [])
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    try:
        body.conn.call("release_port", lease_s=0.3)
        assert wait_for(lambda: not body.conn.call("daemon")["port_released"], 2)
        assert settled == [path]  # settled before the reopen
        assert body.wait_ready(3)
        body.conn.call("release_port", lease_s=1.5)
    finally:
        body.close()  # the releasing client went away: still released until the lease ends
    time.sleep(0.8)
    assert d.port_released and not d.client.state.connected
    n_rx = len(vb.received)
    assert wait_for(lambda: not d.port_released and d.client.state.connected, 3)
    assert settled == [path, path] and len(vb.received) >= n_rx
    # an explicit reacquire settles too
    body = remote(sock)
    try:
        body.conn.call("release_port", lease_s=60)
        assert body.conn.call("reacquire_port")["reacquired"] is True
        assert settled == [path] * 3 and body.wait_ready(3)
    finally:
        body.close()


# ---- shutdown and the behaviour hook --------------------------------------------------------

def test_shutdown_drops_goals_and_lets_the_watchdog_take_over(start_daemon):
    d, vb, path, sock = start_daemon()
    body = remote(sock)
    body.wait_ready(3)
    body.posture("stand")
    assert wait_for(lambda: vb.posture == "STAND")
    assert body.conn.call("shutdown") == "stopping"
    assert wait_for(lambda: not sock.exists(), 3)
    assert wait_for(lambda: vb.wd_stage >= 1, 1.5)  # silent: the body's watchdog stops it
    assert wait_for(lambda: vb.posture == "REST", 2.5)
    body.close()
    assert connect(sock) is None


def test_second_daemon_refuses_a_live_socket(start_daemon):
    d, vb, path, sock = start_daemon()
    d2 = Daemon(path, socket=sock)
    with pytest.raises(SystemExit, match="already listens"):
        d2.run(install_signals=False)


def test_behaviour_runner_hook(start_daemon, monkeypatch):
    calls = []

    def run_behaviour(client, state, stop_event, log_dir):
        calls.append((client, state, stop_event, log_dir))
        client.posture("sit")
        stop_event.wait(10)
        calls.append("stopped")

    fake = types.ModuleType("kilo.behaviour.runner")
    fake.run_behaviour = run_behaviour
    monkeypatch.setitem(sys.modules, "kilo.behaviour.runner", fake)
    d, vb, path, sock = start_daemon(behaviour=True, log_dir=Path(tempfile.mkdtemp(prefix="kl")))
    assert wait_for(lambda: calls, 2)
    client, state, ev, log_dir = calls[0]
    assert isinstance(client, BodyClient) and isinstance(state, BodyState)
    assert isinstance(ev, threading.Event) and isinstance(log_dir, Path) and log_dir.is_dir()
    assert wait_for(lambda: vb.posture == "SIT")
    assert remote(sock).conn.info["behaviour"] == "running"
    # with a runner a CLI goal is refused unless --override ...
    body = remote(sock)
    with pytest.raises(LinkError, match="behaviour runner active"):
        body.posture("stand")
    body.close()
    # ... and the end of an override hold leaves the body to the runner (no idle REST)
    body = RemoteBody(connect(sock), override=True)
    body.posture("stand")
    assert wait_for(lambda: vb.posture == "STAND")
    body.close()
    time.sleep(0.5)
    assert vb.posture == "STAND"
    d.request_stop()
    assert wait_for(lambda: "stopped" in calls, 6)


def test_behaviour_runner_missing_is_logged_not_fatal(start_daemon, monkeypatch):
    monkeypatch.setitem(sys.modules, "kilo.behaviour.runner", None)  # import fails
    d, vb, path, sock = start_daemon(behaviour=True)
    body = remote(sock)
    try:
        assert body.conn.info["behaviour"].startswith("unavailable")
        assert body.wait_ready(3)
    finally:
        body.close()


# ---- the CLI through the daemon vs direct ---------------------------------------------------

def _kilo(*args, env=None, timeout=30):
    p = subprocess.run([sys.executable, "-m", "kilo.cli", *args], cwd=MIND, capture_output=True, text=True,
                       timeout=timeout, env=env or os.environ.copy())
    return p.returncode, p.stdout, p.stderr


def _stable(out):
    """Drop the lines whose numbers legitimately differ between two runs."""
    return [ln for ln in out.splitlines()
            if not ln.startswith(("attitude", "link stats", "body stats", "            rx_crc", "distance"))]


def test_cli_through_the_daemon_matches_direct(start_daemon):
    cmds = [["status"], ["param", "get", "WD_LIMP_S"], ["param", "set", "WD_STOP_MS", "600"],
            ["param", "get", "200"], ["posture", "stand", "--for", "0.5"], ["cal", "show"],
            ["eeprom", "dump", "0", "16"]]
    # direct on one fresh virtual body, through the daemon on another (same cold start)
    transport, dpath, slave = open_pty()
    dvb = VirtualBody(transport).start()
    try:
        direct = {tuple(c): _kilo("--port", dpath, *c) for c in cmds}
    finally:
        dvb.stop()
        transport.close()
        os.close(slave)
    d, vb, path, sock = start_daemon()
    for c in cmds:
        rc, out, err = _kilo(*c)  # no --port: finds the daemon through $KILO_SOCKET
        drc, dout, derr = direct[tuple(c)]
        assert rc == drc, (c, err, derr)
        assert _stable(out.replace(path, "PTY")) == _stable(dout.replace(dpath, "PTY")), (c, out, dout)
        mine = lambda e: [ln for ln in e.splitlines() if ln.startswith("kilo ")]  # not link log warnings
        assert mine(err) == mine(derr), (c, err, derr)
    assert d.sessions == {} or all(not s.goals for s in d.sessions.values())
    # status --json through the daemon has the same keys and identity
    rc, out, _ = _kilo("status", "--json")
    snap = json.loads(out)
    assert rc == 0 and snap["port"] == path and snap["hello"]["fw_build"] == 0x56424F44
    assert snap["body_stats"]["dmp_rate_hz"] == 100
    # monitor --rate is honoured through the daemon too (stdout is a pipe: JSON lines)
    rc, out, err = _kilo("monitor", "--rate", "10", "--duration", "1")
    lines = out.splitlines()
    assert rc == 0 and 8 <= len(lines) <= 12, (len(lines), err)
    assert json.loads(lines[-1])["alive"] is True
    # --port of the daemon's port still goes through the daemon; --direct would not
    rc, out, err = _kilo("-v", "--port", path, "status")
    assert rc == 0 and "daemon pid" in err
    # passive modes need the port to themselves: polite refusal
    rc, out, err = _kilo("monitor", "--passive", "--duration", "1")
    assert rc == 1 and "kilo daemon stop" in err and "--port" in err
    rc, out, err = _kilo("posture", "stand", "--for", "0.5", "--then-watch", "1")
    assert rc == 1 and "holds " + path in err
    # daemon status / stop
    rc, out, err = _kilo("daemon", "status")
    assert rc == 0 and f"port        {path}" in out
    rc, out, err = _kilo("daemon", "stop")
    assert rc == 0 and "stopped daemon" in out
    assert not sock.exists()
    rc, out, err = _kilo("daemon")
    assert rc == 1 and "no daemon" in out


def test_cli_posture_hold_through_the_daemon(start_daemon):
    d, vb, path, sock = start_daemon(hold_lease_s=1.0)
    p = subprocess.Popen([sys.executable, "-m", "kilo.cli", "posture", "stand", "--for", "2.5"], cwd=MIND,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert wait_for(lambda: vb.posture == "STAND", 10)
    time.sleep(1.5)
    assert vb.posture == "STAND"  # held past the 1 s lease by the CLI's pings
    out, err = p.communicate(timeout=10)
    assert p.returncode == 0, err
    assert wait_for(lambda: vb.posture == "REST", 1.0)
    assert vb.wd_stage == 0


def test_cli_run_subcommand(vbody_pty, sockdir):
    vb, path = vbody_pty
    sock = sockdir / "run.sock"
    log_dir = sockdir / "logs"  # never the developer's ~/kilo-logs
    p = subprocess.Popen([sys.executable, "-m", "kilo.cli", "run", "--port", path, "--socket", str(sock),
                          "--log-dir", str(log_dir)],
                         cwd=MIND, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert wait_for(lambda: sock.exists(), 10)
        rc, out, err = _kilo("--socket", str(sock), "status")
        assert rc == 0 and "build 56424f44" in out
        p.terminate()  # SIGTERM: clean shutdown
        p.wait(10)
        assert p.returncode == 0 and not sock.exists()
        assert "shutting down" in p.stderr.read()
        assert "shutting down" in (log_dir / "daemon.log").read_text()
    finally:
        if p.poll() is None:
            p.kill()


def test_balance_enable_restored_after_epoch_bump(start_daemon):
    """Cross-layer (review §2.4): the body clears SET_BALANCE's enable on
    every epoch bump (body.cpp body_epoch_bump; the vbody does the same),
    while the mind keeps the goal latched (not an epoch goal) and refreshes
    it at 5 Hz: `kilo run --balance` is back on within 300 ms of a bump."""
    d, vb, path, sock = start_daemon(balance_kp=0.99)
    assert wait_for(lambda: vb.balance["enable"] == 1, 3)
    kp = vb.balance["kp_q8"]
    assert kp == round(0.99 * 256)
    epoch = vb.epoch
    with vb.lock:
        vb._epoch_bump()  # what a fall, a watchdog stop or LOW battery does
        assert vb.balance["enable"] == 0 and vb.balance["kp_q8"] == kp  # the gains stay
    t0 = time.monotonic()
    assert wait_for(lambda: vb.balance["enable"] == 1, 1.0, period=0.005)
    took = time.monotonic() - t0
    assert took < 0.3, f"balance back on after {took:.3f} s"
    assert vb.epoch == (epoch + 1) & 0xFF and vb.balance["kp_q8"] == kp
    assert d.goal_owner.get("SET_BALANCE", "unset") is None  # still the daemon's own latch
    assert "SET_BALANCE" in d.link.goals()
