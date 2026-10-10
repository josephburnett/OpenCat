from pathlib import Path

import pytest

from kilo.tools import flash as fl
from kilo.tools.gpio import GpiodBackend, PinctrlBackend, SysfsBackend, pick_backend


class FakeLink:
    def __init__(self):
        self.running = True
        self.calls = []

    def stop(self):
        self.running = False
        self.calls.append("stop")

    def start(self):
        self.running = True
        self.calls.append("start")


class FakeReset:
    name = "fake"

    def __init__(self, calls):
        self.calls = calls

    def pulse(self, ms):
        self.calls.append(("pulse", ms))


def test_commands():
    assert fl.compile_cmd(Path("/r/body"), Path("/o"), "abc12345") == [
        "arduino-cli", "compile", "--fqbn", "arduino:avr:uno", "--output-dir", "/o",
        "--build-property", "compiler.cpp.extra_flags=-DKILO_BUILD=0xabc12345", "/r/body"]
    assert fl.avrdude_cmd("/dev/ttyAMA0", Path("x.hex")) == [
        "avrdude", "-c", "arduino", "-p", "atmega328p", "-P", "/dev/ttyAMA0", "-b", "115200",
        "-D", "-U", "flash:w:x.hex:i"]


def test_reset_method():
    assert fl.reset_method("/dev/ttyAMA0") == "gpio"
    assert fl.reset_method("/dev/serial0") == "gpio"
    assert fl.reset_method("/dev/ttyUSB0") == "dtr"
    assert fl.reset_method("/dev/ttyACM0", "none") == "none"


def test_flash_sequence_gpio_stops_and_restarts_link(tmp_path):
    hexf = tmp_path / "b.hex"
    hexf.write_text(":00000001FF\n")
    calls = []
    link = FakeLink()

    def run(cmd, check):
        calls.append(("run", link.running, cmd[0]))

    res = fl.flash(fl.FlashOptions(port="/dev/ttyAMA0", hex_path=hexf, avrdude="avrdude"),
                   link=link, run=run, reset_backend=FakeReset(calls),
                   verify=lambda since, build: True, say=lambda s: None)
    assert link.calls == ["stop", "start"]
    assert calls == [("pulse", 100.0), ("run", False, "avrdude")]  # link stopped while flashing
    assert res.reset == "gpio" and res.verified is True and res.attempts == 1


def test_flash_retries_once_and_restarts_link_on_failure(tmp_path):
    import subprocess

    hexf = tmp_path / "b.hex"
    hexf.write_text(":00000001FF\n")
    link = FakeLink()
    n = []

    def run(cmd, check):
        n.append(cmd)
        raise subprocess.CalledProcessError(1, cmd)

    with pytest.raises(fl.FlashError):
        fl.flash(fl.FlashOptions(port="/dev/ttyUSB0", hex_path=hexf, avrdude="avrdude"),
                 link=link, run=run, say=lambda s: None)
    assert len(n) == 2 and link.calls == ["stop", "start"]


def test_flash_legacy_and_build_dry_run():
    res = fl.flash(fl.FlashOptions(port="/dev/ttyUSB0", legacy=True, dry_run=True), say=lambda s: None)
    assert res.hex_path == fl.LEGACY_HEX
    assert res.commands[-1][-1] == f"flash:w:{fl.LEGACY_HEX}:i"
    assert not any(c[0] == "arduino-cli" for c in res.commands)
    res = fl.flash(fl.FlashOptions(port="/dev/ttyUSB0", dry_run=True, size_check=False), say=lambda s: None)
    assert res.commands[0][:2] == ["arduino-cli", "compile"]
    assert res.commands[1][0] == "avrdude"


def test_firmware_dir_exists():
    assert (fl.FIRMWARE_DIR / "README.md").exists()


def test_gpio_backends_sequences(tmp_path):
    cmds = []
    PinctrlBackend(run=cmds.append, sleep=lambda s: None).pulse(100)
    assert cmds == [["pinctrl", "set", "4", "op", "dl"], ["pinctrl", "set", "4", "ip", "pn"]]
    cmds.clear()
    GpiodBackend(run=cmds.append, version=2).pulse(100)
    assert cmds[0] == ["gpioset", "-c", "gpiochip0", "--drive=open-drain", "--toggle=100ms,0", "4=0"]
    assert cmds[1][0] == "gpioget"
    cmds.clear()
    GpiodBackend(run=cmds.append, version=1).pulse(50)
    assert cmds[0] == ["gpioset", "--mode=time", "--usec=50000", "gpiochip0", "4=0"]
    # sysfs with a 6.6-style base of 512
    root = tmp_path / "gpio"
    chip = root / "gpiochip512"
    chip.mkdir(parents=True)
    (chip / "label").write_text("pinctrl-bcm2835\n")
    (chip / "base").write_text("512\n")
    (root / "export").write_text("")
    (root / "unexport").write_text("")
    g = root / "gpio516"
    g.mkdir()
    (g / "direction").write_text("in")
    writes = []
    b = SysfsBackend(root=str(root), sleep=lambda s: writes.append((g / "direction").read_text()))
    assert b.gpio_number() == 516
    b.pulse(100)
    assert writes == ["low"] and (g / "direction").read_text() == "in"


def test_pick_backend_order(tmp_path):
    assert pick_backend(which=lambda c: "/usr/bin/" + c).name == "pinctrl"
    assert pick_backend(which=lambda c: None if c == "pinctrl" else "/x").name == "gpiod"
    (tmp_path / "export").write_text("")
    assert pick_backend(which=lambda c: None, sysfs_root=str(tmp_path)).name == "sysfs"
    with pytest.raises(RuntimeError):
        pick_backend(which=lambda c: None, sysfs_root=str(tmp_path / "nope"))
