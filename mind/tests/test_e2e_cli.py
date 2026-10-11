"""End to end: the real `kilo` CLI (a subprocess) against the virtual body on a
pty, with the real COBS/CRC codec and generated messages on both ends."""

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody, open_pty

MIND = Path(__file__).resolve().parents[1]


@pytest.fixture
def vbody_pty():
    transport, path, slave = open_pty()
    vb = VirtualBody(transport).start()
    yield vb, path
    vb.stop()
    transport.close()
    os.close(slave)


def _kilo(path, *args, timeout=15):
    return subprocess.Popen([sys.executable, "-m", "kilo.cli", "--port", path, *args], cwd=MIND,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def test_kilo_status(vbody_pty):
    vb, path = vbody_pty
    p = _kilo(path, "status")
    out, err = p.communicate(timeout=15)
    assert p.returncode == 0, err
    assert f"proto {P.PROTO_MAJOR}.{P.PROTO_MINOR}" in out and "build 56424f44" in out, out
    assert "reset COLD" in out
    assert "body stats  uptime" in out
    assert ("HELLO_REQ" in [r[0] for r in vb.received]) and ("STATS_REQ" in [r[0] for r in vb.received])


def test_kilo_posture_stand(vbody_pty):
    vb, path = vbody_pty
    p = _kilo(path, "posture", "stand", "--for", "1", "--pca")
    seen_stand = False
    deadline = time.monotonic() + 15
    while p.poll() is None and time.monotonic() < deadline:
        seen_stand |= vb.posture == "STAND"
        time.sleep(0.02)
    out, err = p.communicate(timeout=5)
    assert p.returncode == 0, err
    assert "holding stand" in out
    assert out.count("channel") == 16 and "off" not in out  # read back while standing
    assert seen_stand
    # the CLI exited: heartbeats stopped, so the body's watchdog trips
    end = time.monotonic() + 2.5
    while time.monotonic() < end and vb.posture != "REST":
        time.sleep(0.05)
    assert vb.wd_stage >= 1 and vb.posture == "REST"


def test_kilo_balance_on(vbody_pty):
    vb, path = vbody_pty
    p = _kilo(path, "balance", "on", "--kp", "0.6", "--clamp", "10", "--for", "2.2")
    seen = False
    deadline = time.monotonic() + 15
    while p.poll() is None and time.monotonic() < deadline:
        seen |= vb.balance_active
        time.sleep(0.02)
    out, err = p.communicate(timeout=5)
    assert p.returncode == 0, err
    assert "balance on: stand, Kp 0.602" in out and "clamp 10°" in out
    assert "balance ACTIVE" in out, out  # the once-a-second line
    assert seen and vb.balance["kp_q8"] == 154 and vb.balance["max_corr_deg"] == 10


def test_kilo_param_list_and_cal_show(vbody_pty):
    vb, path = vbody_pty
    p = _kilo(path, "param", "list")
    out, err = p.communicate(timeout=15)
    assert p.returncode == 0, err
    assert "WD_STOP_MS" in out and "US_PERIOD_TICKS" in out
    p = _kilo(path, "cal", "show")
    out, err = p.communicate(timeout=15)
    assert p.returncode == 0, err
    assert out.count("joint") == 16


def test_kilo_eeprom_dump_and_pca(vbody_pty):
    vb, path = vbody_pty
    vb.eeprom[0:3] = b"\x0c\x0b\x04"
    p = _kilo(path, "eeprom", "dump", "0", "283")
    out, err = p.communicate(timeout=15)
    assert p.returncode == 0, err
    lines = out.splitlines()
    assert len(lines) == 18 and lines[0].startswith("0000: 0c 0b 04 ff") and lines[-1].startswith("0272: ")
    p = _kilo(path, "pca")
    out, err = p.communicate(timeout=15)
    assert p.returncode == 0, err
    assert out.count("off") == 16  # a cold vbody is limp
