"""The post-flash port settle (kilo.link.transport.settle_port), `kilo flash`
verification (timeout, the HELLO it saw, any cause over USB), `kilo flash`
through the daemon with a fake avrdude, and `kilo monitor --rate`.

The settle is checked against a model of the line as user space sees it
(tty core + cdc-acm + the NyBoard's AC-coupled DTR -> RESET), with each of the
mechanisms that can make a rising DTR edge after avrdude switched on:

- the kernel raises DTR on open unless the saved baud is B0;
- cdc-acm raises DTR when termios goes from B0 to a real baud;
- HUPCL drops DTR on the last close;
- (optional) the USB bridge glitches DTR on a line-coding (baud) change.

Whatever the mix, the settle must leave exactly one reset (its first open)
and the later production opens (SerialTransport) none.
"""

import os
import subprocess
import sys
import textwrap
import time
import types
from pathlib import Path

import pytest

from kilo.link import transport as tr
from kilo.tools import flash as fl

MIND = Path(__file__).resolve().parents[1]
HUPCL = 0o2000


class Line:
    def __init__(self, baud=115200, hupcl=True, dtr=False, glitch=False):
        self.baud, self.hupcl, self.dtr, self.glitch = baud, hupcl, dtr, glitch
        self.line_coding = baud or 9600
        self.resets = 0
        self.log = []
        self._edge_this_open = False

    def set_dtr(self, v):
        # A rising DTR edge pulses RESET through the capacitor. Edges inside one
        # open/configure land within milliseconds and give one boot.
        if v and not self.dtr and not self._edge_this_open:
            self.resets += 1
            self._edge_this_open = True
        self.dtr = v

    def kernel_open(self):
        self._edge_this_open = False
        if self.baud != 0:
            self.set_dtr(True)

    def kernel_close(self):
        if self.hupcl:
            self.set_dtr(False)

    def set_termios(self, baud, hupcl):
        old = self.baud
        self.hupcl = hupcl
        if baud == 0:
            self.set_dtr(False)
        elif old == 0:
            self.set_dtr(True)
        if baud and baud != self.line_coding:
            self.line_coding = baud
            if self.glitch and self.dtr:
                self.dtr = False
                self.set_dtr(True)
        self.baud = baud


def fake_modules(line):
    class FakeSerial:
        def __init__(self, port=None, baudrate=9600, **kw):
            self.port, self.baudrate = port, baudrate
            self._dtr = self._rts = True
            self.is_open = False
            if port is not None:
                self.open()

        @property
        def dtr(self):
            return self._dtr

        @dtr.setter
        def dtr(self, v):
            line.log.append(("dtr", v, self.is_open))
            self._dtr = v
            if self.is_open:
                line.set_dtr(v)

        @property
        def rts(self):
            return self._rts

        @rts.setter
        def rts(self, v):
            line.log.append(("rts", v, self.is_open))
            self._rts = v

        def open(self):  # pyserial: os.open, _reconfigure_port (keeps HUPCL), then DTR/RTS
            line.log.append(("open", self.baudrate))
            line.kernel_open()
            line.set_termios(self.baudrate, line.hupcl)
            line.set_dtr(self._dtr)
            self.is_open = True

        def fileno(self):
            return 99

        def close(self):
            line.log.append(("close",))
            if self.is_open:
                line.kernel_close()
            self.is_open = False

        def set_low_latency_mode(self, on):
            pass

        def reset_input_buffer(self):
            pass

    class FakeTermios:
        HUPCL = HUPCL
        TCSANOW = 0

        @staticmethod
        def tcgetattr(fd):
            return [0, 0, HUPCL if line.hupcl else 0, 0, line.baud, line.baud, []]

        @staticmethod
        def tcsetattr(fd, when, attrs):
            line.set_termios(attrs[4], bool(attrs[2] & HUPCL))

    return types.SimpleNamespace(Serial=FakeSerial), FakeTermios


def production_open(line, monkeypatch):
    """One SerialTransport open/close (what the verify and every later kilo command do)."""
    serial_mod, termios_mod = fake_modules(line)
    monkeypatch.setitem(sys.modules, "serial", serial_mod)
    orig = tr.clear_hupcl
    monkeypatch.setattr(tr, "clear_hupcl", lambda fd, t=None: orig(fd, termios_mod))
    tr.SerialTransport("/dev/ttyACM0").close()


def old_raw_clear(line, termios_mod):
    """The previous step 5: a raw open that only cleared HUPCL."""
    line.kernel_open()
    tr.clear_hupcl(3, termios_mod)
    line.kernel_close()


MODELS = {
    "after avrdude (115200, HUPCL, DTR low)": dict(baud=115200),
    "stty 0 (B0, HUPCL)": dict(baud=0),
    "line-coding glitch": dict(baud=115200, glitch=True),
    "B0 and glitch": dict(baud=0, glitch=True),
}


@pytest.mark.parametrize("model", list(MODELS))
def test_settle_leaves_exactly_one_reset(model, monkeypatch):
    line = Line(**MODELS[model])
    serial_mod, termios_mod = fake_modules(line)
    slept = []
    cycles = tr.settle_port("/dev/ttyACM0", 250000, cycles=3, serial_mod=serial_mod, termios_mod=termios_mod,
                            ioctl=None, sleep=slept.append)
    assert line.resets == 1, (model, line.resets)
    assert line.dtr and not line.hupcl and line.baud == 250000 and line.line_coding == 250000
    assert [c.cycle for c in cycles] == [1, 2, 3] and slept == [0.3, 0.3]
    assert cycles[0].hupcl_was_set and not cycles[1].hupcl_was_set and all(c.hupcl_clear for c in cycles)
    assert not any(c.error for c in cycles)
    # DTR and RTS are requested before open(), so open() asserts them
    first_open = line.log.index(("open", 250000))
    assert ("dtr", True, False) in line.log[:first_open] and ("rts", True, False) in line.log[:first_open]
    # the verify open and later commands: no further reset
    production_open(line, monkeypatch)
    production_open(line, monkeypatch)
    assert line.resets == 1


@pytest.mark.parametrize("model", list(MODELS))
def test_old_sequence_was_no_better(model, monkeypatch):
    """The previous raw HUPCL clear + verify open: never fewer resets, and two
    under the line-coding glitch (the measured pattern)."""
    line = Line(**MODELS[model])
    _, termios_mod = fake_modules(line)
    old_raw_clear(line, termios_mod)
    production_open(line, monkeypatch)
    production_open(line, monkeypatch)
    assert line.resets >= 1
    if model == "line-coding glitch":
        assert line.resets == 2


def test_settle_records_errors_and_skips_ptys():
    class Boom:
        def __init__(self, *a, **kw):
            self.port = None

        def open(self):
            raise OSError(16, "Device or resource busy")

        def close(self):
            pass

    cycles = tr.settle_port("/dev/ttyACM0", cycles=2, serial_mod=types.SimpleNamespace(Serial=Boom),
                            termios_mod=None, sleep=lambda s: None)
    assert len(cycles) == 2 and all("busy" in c.error for c in cycles)
    assert "busy" in tr.describe_settle(cycles)
    assert tr.settle_port("/dev/pts/4") == []


def test_flash_reports_the_settle(tmp_path):
    hexf = tmp_path / "b.hex"
    hexf.write_text(":00000001FF\n")
    said = []
    cyc = [tr.SettleCycle(1, True, True, True, True), tr.SettleCycle(2, False, True, True, True)]
    res = fl.flash(fl.FlashOptions(port="/dev/ttyACM0", hex_path=hexf, avrdude="avrdude", verbose=True),
                   run=lambda cmd, check: None, say=said.append, settle=lambda port: cyc)
    assert res.settle == cyc
    assert "settled /dev/ttyACM0: 2 open/close cycles at 250000, HUPCL was set on cycle 1" in said
    assert "  1: HUPCL was set; DTR 1 RTS 1" in said and "  2: HUPCL was clear; DTR 1 RTS 1" in said


# ---- verification -----------------------------------------------------------------------

def test_is_usb_port_any_cause():
    assert fl.is_usb_port("/dev/ttyACM0") and fl.is_usb_port("/dev/ttyUSB0")
    assert not fl.is_usb_port("/dev/ttyAMA0") and not fl.is_usb_port("/dev/serial0")


def test_flash_prints_the_hello_it_saw(tmp_path):
    hexf = tmp_path / "b.hex"
    hexf.write_text(":00000001FF\n")
    said = []
    seen = {"fw_build": 0x56424F44, "reset_cause": 3}
    timeouts = []

    def verify(since, build, timeout):
        timeouts.append(timeout)
        return fl.VerifyResult(False, seen)

    res = fl.flash(fl.FlashOptions(port="/dev/ttyAMA0", hex_path=hexf, avrdude="avrdude", reset="none"),
                   run=lambda cmd, check: None, say=said.append, settle=lambda p: [], verify=verify)
    assert timeouts == [8.0, 8.0] and res.verified is False and res.hello == seen  # verified twice
    assert said[-2].startswith("no HELLO with reset_cause=FLASHED within 8 s (last HELLO seen: build 56424f44 "
                               "reset ") and said[-2].endswith("; verifying once more")
    assert said[-1].startswith("verify FAILED: no HELLO with reset_cause=FLASHED within 2 x 8 s; last HELLO "
                               "seen: build 56424f44 reset ")
    said.clear()
    fl.flash(fl.FlashOptions(port="/dev/ttyACM0", hex_path=hexf, avrdude="avrdude", verify_timeout=2.5),
             run=lambda cmd, check: None, say=said.append, settle=lambda p: [],
             verify=lambda s, b, t: fl.VerifyResult(True, seen))
    assert said[-1].startswith("verified: HELLO any reset cause (build 56424f44 reset ")
    said.clear()
    fl.flash(fl.FlashOptions(port="/dev/ttyACM0", hex_path=hexf, avrdude="avrdude", verify_timeout=2.5),
             run=lambda cmd, check: None, say=said.append, settle=lambda p: [],
             verify=lambda s, b, t: fl.VerifyResult(False, None))
    assert said[-1] == "verify FAILED: no HELLO with any reset cause within 2 x 2.5 s; no HELLO seen at all"


@pytest.fixture
def vrig():
    from kilo.body.client import BodyClient
    from kilo.link.serial_link import LinkConfig, SerialLink
    from kilo.link.transport import loopback_pair
    from kilo.vbody.sim import VirtualBody

    from conftest import FAST

    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="loopback")
    link.mark_alive()
    link.start()
    yield BodyClient(link), vb
    link.stop()
    vb.stop()


def test_verify_hello_timeout_and_what_it_saw(vrig):
    client, vb = vrig
    client.wait_ready(2)
    since = time.monotonic() - 5
    r = fl.verify_hello(client.link, since, 0x56424F44, timeout=1, any_cause=True)
    assert r and r.hello["fw_build"] == 0x56424F44
    t0 = time.monotonic()
    r = fl.verify_hello(client.link, since, 0x12345678, timeout=1.0, any_cause=True)
    assert not r and 0.9 <= time.monotonic() - t0 < 2.0
    assert r.hello["fw_build"] == 0x56424F44  # reported anyway
    r = fl.verify_hello(client.link, since, None, timeout=0.5, any_cause=False)  # COLD, not FLASHED
    assert not r and r.hello["reset_cause"] == 0


def test_verify_hello_asks_when_the_boot_hello_was_missed(vrig):
    client, vb = vrig
    client.wait_ready(2)
    since = time.monotonic()  # the HELLO the link has is older than this
    r = fl.verify_hello(client.link, since, None, timeout=3, any_cause=True)
    assert r and client.state.hello_at >= since
    vb.ignore_types = {"HELLO_REQ"}
    since = time.monotonic()
    r = fl.verify_hello(client.link, since, None, timeout=1, any_cause=True)
    assert not r and r.hello is None


# ---- kilo flash end to end with a fake avrdude, direct and through the daemon ------------------

@pytest.fixture
def fake_avrdude(tmp_path):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    marker = tmp_path / "avrdude.log"
    script = bindir / "avrdude"
    script.write_text(textwrap.dedent(f"""\
        #!{sys.executable}
        import json, subprocess, sys
        out = subprocess.run([{sys.executable!r}, "-m", "kilo.cli", "daemon", "status", "--json"],
                             capture_output=True, text=True, cwd={str(MIND)!r})
        info = json.loads(out.stdout) if out.returncode == 0 else None
        with open({str(marker)!r}, "a") as f:
            f.write(json.dumps({{"argv": sys.argv[1:], "released": info and info["port_released"]}}) + "\\n")
        """))
    script.chmod(0o755)
    hexf = tmp_path / "body.hex"
    hexf.write_text(":00000001FF\n")
    env = dict(os.environ, PATH=f"{bindir}:{os.environ['PATH']}")
    return env, hexf, marker


def test_kilo_flash_direct_and_through_the_daemon(fake_avrdude, tmp_path):
    import json
    import shutil
    import tempfile
    import threading

    from kilo.daemon import Daemon
    from kilo.link.serial_link import LinkConfig
    from kilo.vbody.sim import VirtualBody, open_pty

    from conftest import FAST

    env, hexf, marker = fake_avrdude
    transport, path, slave = open_pty()
    vb = VirtualBody(transport).start()
    sd = Path(tempfile.mkdtemp(prefix="kf", dir="/tmp"))
    d = t = None
    try:
        p = subprocess.run([sys.executable, "-m", "kilo.cli", "--port", path, "flash", "--hex", str(hexf),
                            "--verify-timeout", "3"], cwd=MIND, env=env, capture_output=True, text=True,
                           timeout=60)
        assert p.returncode == 0, p.stderr
        assert "verified: HELLO any reset cause (build 56424f44 reset COLD)" in p.stdout, p.stdout
        rec = [json.loads(x) for x in marker.read_text().splitlines()]
        assert rec[-1]["released"] is None and rec[-1]["argv"][-1] == f"flash:w:{hexf}:i"

        sock = sd / "k.sock"
        env["KILO_SOCKET"] = str(sock)
        d = Daemon(path, socket=sock, link_config=LinkConfig(**FAST))
        t = threading.Thread(target=d.run, kwargs={"install_signals": False}, daemon=True)
        t.start()
        assert d.ready.wait(5)
        p = subprocess.run([sys.executable, "-m", "kilo.cli", "flash", "--hex", str(hexf), "--verify-timeout", "3"],
                           cwd=MIND, env=env, capture_output=True, text=True, timeout=60)
        assert p.returncode == 0, p.stderr
        out = p.stdout
        assert "daemon released" in out and "daemon reacquired" in out, out
        assert out.index("daemon released") < out.index("avrdude -c arduino") < out.index("daemon reacquired")
        assert "verified: HELLO any reset cause (build 56424f44 reset COLD)" in out, out
        rec = [json.loads(x) for x in marker.read_text().splitlines()]
        assert rec[-1]["released"] is True  # avrdude ran while the daemon had let go of the port
        assert not d.port_released and d.client.state.connected
    finally:
        if d is not None:
            d.request_stop()
            t.join(5)
        vb.stop()
        transport.close()
        os.close(slave)
        shutil.rmtree(sd, ignore_errors=True)


# ---- kilo monitor --rate ------------------------------------------------------------------------

def test_monitor_rate_is_honoured_when_piped():
    import json

    from kilo.cli import parse_args
    from kilo.vbody.sim import VirtualBody, open_pty

    assert parse_args(["monitor", "--json"]).json
    transport, path, slave = open_pty()
    vb = VirtualBody(transport).start()
    try:
        p = subprocess.run([sys.executable, "-m", "kilo.cli", "--port", path, "monitor", "--rate", "10",
                            "--duration", "1.5", "--json"], cwd=MIND, capture_output=True, text=True, timeout=30)
        assert p.returncode == 0, p.stderr
        lines = p.stdout.splitlines()
        assert 12 <= len(lines) <= 17, len(lines)  # 10 Hz, not 1 Hz
        recs = [json.loads(x) for x in lines]
        assert {"time", "tick", "mode", "pitch_deg", "alive"} <= set(recs[0])
        gaps = [b["time"] - a["time"] for a, b in zip(recs, recs[1:])]
        assert 0.07 <= sum(gaps) / len(gaps) <= 0.13
    finally:
        vb.stop()
        transport.close()
        os.close(slave)
