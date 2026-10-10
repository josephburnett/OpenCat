"""Reset pulse on GPIO4 as an emulated open-drain output (Appendix F).

Drive the line low for ``ms`` milliseconds, then return it to an input with no
pull. It is never driven high, so it cannot fight the NyBoard's reset circuit,
and it works whether GPIO4 is AC-coupled to RESET (falling edge) or wired
directly (low level).

libgpiod's Python bindings are often not installed, so three command-line
backends are tried in order; the first one present is used:

1. ``pinctrl`` (raspi-utils, standard on Raspberry Pi OS 12/13):
   ``pinctrl set 4 op dl`` ... ``pinctrl set 4 ip pn``.
2. libgpiod tools ``gpioset``/``gpioget`` (apt ``gpiod``), v1 or v2 syntax.
   v2 requests the line open-drain, so the kernel itself releases it on "1".
3. sysfs ``/sys/class/gpio`` (deprecated, still in Raspberry Pi kernels): the
   GPIO number is offset by the bcm2835 chip's base (512 on 6.6+ kernels).

Each backend takes an injectable ``run`` so the sequences can be tested.
"""

from __future__ import annotations

import glob
import os
import re
import shutil
import subprocess
import time
from pathlib import Path
from typing import Callable, Sequence

Runner = Callable[[Sequence[str]], "subprocess.CompletedProcess[str]"]


def _run(cmd: Sequence[str]) -> "subprocess.CompletedProcess[str]":
    return subprocess.run(list(cmd), check=True, capture_output=True, text=True, timeout=5)


class ResetBackend:
    name = "?"

    def pulse(self, ms: float) -> None:
        raise NotImplementedError


class PinctrlBackend(ResetBackend):
    name = "pinctrl"

    def __init__(self, pin: int = 4, run: Runner = _run, sleep=time.sleep):
        self.pin, self.run, self.sleep = pin, run, sleep

    def pulse(self, ms: float) -> None:
        self.run(["pinctrl", "set", str(self.pin), "op", "dl"])
        try:
            self.sleep(ms / 1000)
        finally:
            self.run(["pinctrl", "set", str(self.pin), "ip", "pn"])


class GpiodBackend(ResetBackend):
    name = "gpiod"

    def __init__(self, pin: int = 4, chip: str = "gpiochip0", run: Runner = _run,
                 version: int | None = None, sleep=time.sleep):
        self.pin, self.chip, self.run, self.sleep = pin, chip, run, sleep
        self.version = version

    def _detect_version(self) -> int:
        try:
            out = self.run(["gpioset", "--version"]).stdout
        except (subprocess.CalledProcessError, OSError):
            return 1
        m = re.search(r"v(\d+)\.", out)
        return int(m.group(1)) if m else 1

    def pulse(self, ms: float) -> None:
        v = self.version or self._detect_version()
        ms_i = max(1, int(round(ms)))
        if v >= 2:
            # open-drain: "0" pulls low, "1" releases (kernel emulates on bcm2835);
            # toggle after ms to 1, then a zero period = exit.
            self.run(["gpioset", "-c", self.chip, "--drive=open-drain",
                      f"--toggle={ms_i}ms,0", f"{self.pin}=0"])
            self.run(["gpioget", "-c", self.chip, "--bias=disable", str(self.pin)])
        else:
            self.run(["gpioset", "--mode=time", f"--usec={ms_i * 1000}", self.chip, f"{self.pin}=0"])
            self.run(["gpioget", self.chip, str(self.pin)])  # re-request as input


class SysfsBackend(ResetBackend):
    name = "sysfs"

    def __init__(self, pin: int = 4, root: str = "/sys/class/gpio", sleep=time.sleep):
        self.pin, self.root, self.sleep = pin, Path(root), sleep

    def gpio_number(self) -> int:
        for chip in sorted(glob.glob(str(self.root / "gpiochip*"))):
            try:
                label = Path(chip, "label").read_text().strip()
                if "bcm2835" in label or "bcm2711" in label:
                    return int(Path(chip, "base").read_text()) + self.pin
            except OSError:
                continue
        return self.pin

    def pulse(self, ms: float) -> None:
        n = self.gpio_number()
        g = self.root / f"gpio{n}"
        exported = False
        if not g.exists():
            (self.root / "export").write_text(str(n))
            exported = True
            for _ in range(50):  # udev may take a moment to fix permissions
                if os.access(g / "direction", os.W_OK):
                    break
                self.sleep(0.01)
        try:
            (g / "direction").write_text("low")  # output, driven low, atomically
            self.sleep(ms / 1000)
        finally:
            (g / "direction").write_text("in")
            if exported:
                (self.root / "unexport").write_text(str(n))


def pick_backend(pin: int = 4, which=shutil.which, sysfs_root: str = "/sys/class/gpio") -> ResetBackend:
    if which("pinctrl"):
        return PinctrlBackend(pin)
    if which("gpioset") and which("gpioget"):
        return GpiodBackend(pin)
    if os.path.exists(os.path.join(sysfs_root, "export")):
        return SysfsBackend(pin, sysfs_root)
    raise RuntimeError("no GPIO backend: install raspi-utils (pinctrl) or gpiod")
