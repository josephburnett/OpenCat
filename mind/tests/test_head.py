"""The head goal (SET_HEAD, docs §3c layer 1): BodyClient.look / release_head
against the virtual body, and `kilo look` end to end on a pty."""

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from kilo.body.client import HEAD_NOT_HELD, BodyClient
from kilo.link.serial_link import LinkConfig, SerialLink
from kilo.link.transport import loopback_pair
from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody, open_pty

from conftest import FAST, keep_alive

MIND = Path(__file__).resolve().parents[1]
HOLD_S = P.HEAD_HOLD_MS / 1000.0


@pytest.fixture
def rig():
    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="loopback")
    link.mark_alive()
    link.start()
    yield BodyClient(link), vb
    link.stop()
    vb.stop()


def _held(client):
    return "head_held" in client.state.flags and (client.state.owners or ("", ""))[1] == "PI_HOLD"


def test_look_is_a_refreshed_goal_and_release_head(rig):
    client, vb = rig
    client.wait_ready(2)
    client.stand()
    f = client.look(30, -10.5)
    assert f == {"mode": 1, "pan": 480, "tilt": -168, "tail": HEAD_NOT_HELD, "max_speed_dps": 120}
    assert client.link.goals()["SET_HEAD"].epoch == client.state.epoch
    assert client.link.wait_until(lambda: _held(client), 1)
    assert vb.head == (480, -168, HEAD_NOT_HELD)
    # Refreshed at 5 Hz: still held well past HEAD_HOLD_MS.
    keep_alive(client.link, HOLD_S + 0.5)
    assert vb.head_until is not None and _held(client)
    # release_head: the goal is gone and the body lets go at once.
    client.release_head()
    assert "SET_HEAD" not in client.link.goals()
    assert client.link.wait_until(lambda: vb.head_until is None, 0.5)
    assert client.link.wait_until(lambda: "head_held" not in client.state.flags, 1)
    assert client.look(0, 0, tail_deg=12, speed_dps=0)["tail"] == 192
    with pytest.raises(ValueError):
        client.look(181, 0)
    with pytest.raises(ValueError):
        client.look(0, 0, speed_dps=-1)


def test_look_dropped_by_an_epoch_bump(rig):
    client, vb = rig
    client.wait_ready(2)
    client.stand()
    client.look(20, 0)
    assert client.link.wait_until(lambda: vb.head_until is not None, 1)
    vb.trigger_fall(-1)
    assert client.link.wait_until(lambda: "SET_HEAD" not in client.link.goals(), 1)
    assert vb.head_until is None
    # After RECOVERED the body stands (doc §3g) and a new look works.
    assert client.link.wait_until(lambda: vb.posture == "STAND" and not vb.fallen, 3)
    client.look(-15, 5)
    assert client.link.wait_until(lambda: _held(client), 1)


@pytest.fixture
def vbody_pty():
    transport, path, slave = open_pty()
    vb = VirtualBody(transport).start()
    yield vb, path
    vb.stop()
    transport.close()
    os.close(slave)


def test_kilo_look(vbody_pty):
    vb, path = vbody_pty
    p = subprocess.Popen([sys.executable, "-m", "kilo.cli", "--direct", "--port", path, "look", "25", "-10",
                          "--tail", "5", "--for", "1", "--posture", "stand"],
                         cwd=MIND, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    seen = None
    deadline = time.monotonic() + 15
    while p.poll() is None and time.monotonic() < deadline:
        if vb.head_until is not None:
            seen = vb.head
        time.sleep(0.02)
    out, err = p.communicate(timeout=5)
    assert p.returncode == 0, err
    assert "looking at pan 25°, tilt -10°, tail 5° at 120°/s for 1 s" in out and "released" in out, out
    assert seen == (400, -160, 80) and vb.posture == "STAND"
    assert vb.head_until is None  # released by the CLI, not by the 2 s lapse
    assert ("SET_HEAD" in [r[0] for r in vb.received])
