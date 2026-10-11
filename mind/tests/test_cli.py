import pytest

from kilo.cli import build_parser, parse_args, render_monitor
from kilo.body.state import BodyState
from kilo.link.ports import detect_port


def test_parse_status_and_globals():
    a = parse_args(["--port", "/dev/ttyUSB0", "--log", "status", "--json"])
    assert a.cmd == "status" and a.json and a.port == "/dev/ttyUSB0" and a.log
    assert a.baud == 250000
    assert parse_args(["--log-file", "x.jsonl", "monitor"]).log_file == "x.jsonl"


def test_parse_posture():
    a = parse_args(["posture", "stand", "--transition", "50", "--for", "2"])
    assert (a.name, a.transition, a.hold) == ("stand", 50, 2.0)
    a = parse_args(["posture", "balance"])
    assert a.hold is None
    with pytest.raises(SystemExit):
        parse_args(["posture", "fly"])


def test_parse_balance():
    a = parse_args(["balance", "on", "--kp", "0.6", "--kd", "0.05", "--deadband", "3", "--clamp", "10",
                    "--for", "30"])
    assert (a.cmd, a.state, a.kp, a.kd, a.deadband, a.clamp, a.hold, a.posture) == (
        "balance", "on", 0.6, 0.05, 3.0, 10.0, 30.0, "stand")
    a = parse_args(["balance", "off", "--posture", "sit"])
    assert (a.state, a.kp, a.kd, a.deadband, a.clamp, a.posture) == ("off", 0.5, 0.03, 2, 15, "sit")
    with pytest.raises(SystemExit):
        parse_args(["balance", "maybe"])


def test_parse_cal():
    assert parse_args(["cal"]).cal_cmd == "show"
    a = parse_args(["cal", "set", "8", "-2.5"])
    assert (a.cal_cmd, a.joint, a.degrees) == ("set", 8, -2.5)
    assert parse_args(["cal", "save", "--no-wait"]).no_wait


def test_parse_param():
    a = parse_args(["param", "set", "WD_STOP_MS", "600", "--persist"])
    assert (a.param_cmd, a.id, a.value, a.persist) == ("set", "WD_STOP_MS", 600, True)
    with pytest.raises(SystemExit):
        parse_args(["param"])


def test_parse_flash_and_vbody():
    a = parse_args(["--port", "/dev/ttyAMA0", "flash", "--legacy", "--reset", "gpio", "--dry-run"])
    assert a.legacy and a.reset == "gpio" and a.dry_run and a.reset_ms == 100
    a = parse_args(["vbody", "--fall-after", "3", "--no-keys"])
    assert a.fall_after == 3 and a.no_keys


def test_requires_command():
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def _fs(paths, links=None):
    links = links or {}
    return dict(exists=lambda p: p in paths or p in links,
                realpath=lambda p: links.get(p, p),
                globber=lambda pat: [p for p in paths if p.startswith(pat.rstrip("*"))])


def test_port_rules():
    fs = _fs({"/dev/ttyAMA0", "/dev/ttyUSB0", "/dev/ttyACM0"}, {"/dev/serial0": "/dev/ttyAMA0"})
    assert detect_port("/dev/x", env={}, on_pi=True, **fs).port == "/dev/x"
    assert detect_port(None, env={"KILO_PORT": "/dev/y"}, on_pi=True, **fs).port == "/dev/y"
    c = detect_port(None, env={}, on_pi=True, **fs)
    # A plugged-in USB adapter wins over the header UART (whose node always
    # exists on a Pi), with a warning naming the ignored UART.
    assert c.port == "/dev/ttyUSB0" and c.reason == "USB serial adapter"
    assert len(c.warnings) == 1 and "/dev/ttyAMA0" in c.warnings[0] and "KILO_PORT" in c.warnings[0]
    c = detect_port(None, env={}, on_pi=True,
                    **_fs({"/dev/ttyS0", "/dev/ttyACM0"}, {"/dev/serial0": "/dev/ttyS0"}))
    assert c.port == "/dev/ttyACM0" and "/dev/ttyS0" in c.warnings[0]
    c = detect_port(None, env={"KILO_PORT": "/dev/ttyAMA0"}, on_pi=True, **fs)
    assert c.port == "/dev/ttyAMA0" and not c.warnings
    c = detect_port(None, env={}, on_pi=True, **_fs({"/dev/ttyAMA0"}))
    assert c.port == "/dev/ttyAMA0" and not c.warnings
    c = detect_port(None, env={}, on_pi=True, **_fs({"/dev/ttyS0"}, {"/dev/serial0": "/dev/ttyS0"}))
    assert c.port == "/dev/ttyS0" and c.reason == "Raspberry Pi header UART"
    c = detect_port(None, env={}, on_pi=False, **fs)
    assert c.port == "/dev/ttyUSB0" and not c.warnings
    assert detect_port(None, env={}, on_pi=False, **_fs({"/dev/ttyACM0"})).port == "/dev/ttyACM0"
    assert detect_port(None, env={}, on_pi=False, **_fs(set())).port is None


def test_render_monitor_empty_state():
    text = render_monitor(BodyState().snapshot())
    assert "LOST" in text and "battery" in text and "balance  off" in text
