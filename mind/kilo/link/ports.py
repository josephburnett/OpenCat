"""Serial port auto-detection.

Rules (README "Port rules"):

1. ``--port`` (or ``$KILO_PORT``) wins.
2. On a Raspberry Pi, the header UART: ``/dev/serial0`` resolved, else
   ``/dev/ttyAMA0``. On the Pi 2 B (no Bluetooth) the PL011 is ttyAMA0 on
   GPIO14/15. On Pi 3/4/Zero 2 W ``serial0`` resolves to the mini-UART
   ``ttyS0``, which is why it is consulted first.
3. Else the first ``/dev/ttyUSB*`` then ``/dev/ttyACM*`` (the Petoi CH340/CH343
   adapter).

We glob the device nodes instead of using ``serial.tools.list_ports``:
pyserial classifies ``ttyS*``/``ttyAMA*`` as "platform" ports and leaves them
out on recent kernels (pyserial issue #489; serialMaster/ardSerial.py works
around the same thing). On a Pi with a USB adapter also present, a warning is
returned: both drive the 328P's single RX line.
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass, field
from typing import Callable


@dataclass
class PortChoice:
    port: str | None
    reason: str
    warnings: list[str] = field(default_factory=list)


def is_raspberry_pi(read_model: Callable[[], str] | None = None) -> bool:
    if read_model is None:
        def read_model() -> str:
            try:
                with open("/proc/device-tree/model", "rb") as f:
                    return f.read().decode(errors="replace")
            except OSError:
                return ""
    return "Raspberry Pi" in read_model()


def detect_port(
    explicit: str | None = None,
    *,
    env: dict[str, str] | None = None,
    on_pi: bool | None = None,
    exists: Callable[[str], bool] = os.path.exists,
    realpath: Callable[[str], str] = os.path.realpath,
    globber: Callable[[str], list[str]] = glob.glob,
) -> PortChoice:
    env = os.environ if env is None else env
    if explicit:
        return PortChoice(explicit, "--port")
    if env.get("KILO_PORT"):
        return PortChoice(env["KILO_PORT"], "$KILO_PORT")
    if on_pi is None:
        on_pi = is_raspberry_pi()
    usb = sorted(globber("/dev/ttyUSB*")) + sorted(globber("/dev/ttyACM*"))
    if on_pi:
        hdr = None
        if exists("/dev/serial0"):
            hdr = realpath("/dev/serial0")
        elif exists("/dev/ttyAMA0"):
            hdr = "/dev/ttyAMA0"
        if hdr:
            warn = []
            if usb:
                warn.append(
                    f"USB serial adapter(s) present ({', '.join(usb)}): unplug the CH343 "
                    "adapter while the Pi drives the UART; both drive the same RX line")
            return PortChoice(hdr, "Raspberry Pi header UART", warn)
    if usb:
        return PortChoice(usb[0], "USB serial adapter")
    return PortChoice(None, "no serial port found")
