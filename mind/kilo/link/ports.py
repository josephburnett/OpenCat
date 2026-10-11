"""Serial port auto-detection.

Rules (README "Port rules"):

1. ``--port`` (or ``$KILO_PORT``) wins.
2. A plugged-in USB serial adapter: the first ``/dev/ttyUSB*`` then
   ``/dev/ttyACM*`` (the Petoi CH340/CH343 adapter). Plugging one in is an
   explicit act; the header UART's device node, by contrast, exists on every
   Pi whether or not anything is wired to it. On a Pi this choice carries a
   warning that the header UART was ignored (select it with ``KILO_PORT`` or
   ``--port``).
3. Else, on a Raspberry Pi, the header UART: ``/dev/serial0`` resolved, else
   ``/dev/ttyAMA0``. On the Pi 2 B (no Bluetooth) the PL011 is ttyAMA0 on
   GPIO14/15. On Pi 3/4/Zero 2 W ``serial0`` resolves to the mini-UART
   ``ttyS0``, which is why it is consulted first.

We glob the device nodes instead of using ``serial.tools.list_ports``:
pyserial classifies ``ttyS*``/``ttyAMA*`` as "platform" ports and leaves them
out on recent kernels (pyserial issue #489; serialMaster/ardSerial.py works
around the same thing).
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
    hdr = None
    if on_pi:
        if exists("/dev/serial0"):
            hdr = realpath("/dev/serial0")
        elif exists("/dev/ttyAMA0"):
            hdr = "/dev/ttyAMA0"
    if usb:
        warn = []
        if hdr:
            warn.append(
                f"using USB serial adapter {usb[0]}; the Pi header UART {hdr} was ignored "
                f"(select it with KILO_PORT={hdr} or --port {hdr}, and unplug the adapter: "
                "both drive the 328P's single RX line)")
        return PortChoice(usb[0], "USB serial adapter", warn)
    if hdr:
        return PortChoice(hdr, "Raspberry Pi header UART")
    return PortChoice(None, "no serial port found")
