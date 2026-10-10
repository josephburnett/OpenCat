# Mounting a Raspberry Pi on Kilo (Nybble, NyBoard V1_2)

Research notes gathered 2026-10-10 for using a Raspberry Pi as a higher-level
"mind" that talks to the NyBoard firmware over serial. Claims are sourced where
Petoi documents them; unconfirmed items are flagged at the end.

## Checklist

| Item | Finding |
|---|---|
| Mount | Solder a 2x5 socket onto the NyBoard; the Pi plugs onto it directly (no cable). Nybble-specific printable parts: `Nybble_y1Pi.stl` (replaces y1) and `Nybble_xPlateForPi.stl` / Pi Stand, in PetoiCamp/NonCodeFiles `stl/NybbleWood`. Early NyBoards have tall jumper pins that must be bent to clear the Pi. |
| Model | Petoi's documented pick: Pi 3A+ ("best fit for NyBoard's dimension"). The mount geometry was designed around the 3B/3A+ footprint. Pi Zero / Zero 2 W fit is not documented. |
| Power | NyBoard V1_1/V1_2 has a TPS565201 buck converter giving 5 V 3 A (5 A peak, thermal cutoff) for the Pi, fed from the battery. The Pi rail is live only with the battery connected, not from the USB uploader. |
| Wiring | The socket mates Pi header pins 1-10: 1=3V3, 2=5V, 3=SDA, 4=5V, 5=SCL, 6=GND, 7=GPIO4 (lands on the NyBoard reset/DTR line), 8=GPIO14 TXD -> NyBoard RX, 9=GND, 10=GPIO15 RXD -> NyBoard TX. UART at 3.3 V. **Derived from docs, no official diagram found: verify with a meter before soldering.** |
| I2C switch | The "Arduino / Raspberry Pi" switch on the board selects the I2C master of the MPU6050, PCA9685 and EEPROM. Leave it on **Arduino**; on "Pi" the firmware loses its peripherals. |
| Serial | 115200 baud (`BAUD_RATE` in src/OpenCat.h). The ATmega328P has one UART, shared with the 6-pin socket (USB uploader, Bluetooth dongle, ESP8266). **Unplug the USB adapter / dongle while the Pi drives serial.** |
| Pi config | `raspi-config` -> Interface Options -> Serial Port: login shell **No**, hardware **Yes**. **Disable the 1-wire interface**, otherwise GPIO4 keeps resetting the NyBoard. On Ubuntu: `enable_uart=1`, remove `console=serial0,115200` from cmdline, disable `serial-getty@ttyS0`. |
| Port name | Petoi's `serialMaster/ardSerial.py` globs `/dev/ttyS*` and strips `ttyAMA0` (workaround for a pyserial platform-port issue on Bookworm / Ubuntu 22.04). On Pi 3/4/Zero 2 W, Bluetooth owns the PL011 (`ttyAMA0`) and GPIO14/15 get the mini-UART `ttyS0` (= `/dev/serial0`); if 115200 is flaky, add `dtoverlay=disable-bt` or `core_freq_min=500`. On Pi models without onboard BT (Pi 2 B, Pi 1, original Zero) the PL011 is on GPIO14/15 as `ttyAMA0`, so `ardSerial.py`'s port filter needs adjusting. |
| USB alternative | Skip the header entirely: plug the Petoi CH340/CH343 USB adapter into a Pi USB port and use `/dev/ttyUSB0`. `ardSerial.py` auto-detects it. Avoids the mini-UART, GPIO4 and I2C-switch issues at the cost of a dangling dongle. Good first step for developing the mind off-robot. |

## Software

- `serialMaster/` in this repo is Petoi's Pi-side Python library (`ardSerial.py`, `SerialCommunication.py`, `example.py`, `allSkills.py`, `demos/`). Needs `pyserial` from pip (not apt's python-serial). `./ardSerial.py kcrF` is equivalent to typing `kcrF` in the Arduino serial monitor.
- Same token protocol as USB serial: `k`+skill, `m`/`M` and `i`/`I` joint moves, `L` listed angles, `b`/`B` buzzer, `c` calibrate, `d` rest, `G`/`g` gyro, `?` query. Full list in `src/OpenCat.h`; docs at https://docs.petoi.com/apis/serial-protocol. Uppercase tokens take binary arguments (one byte per angle).
- ROS / ROS 2 wrapper: https://github.com/PetoiCamp/ros_opencat (Docker; "recommended to use with Raspberry Pi").
- No Petoi Raspberry Pi OS image exists; use stock Raspberry Pi OS or Ubuntu.
- The IR receiver is on PD4 and does not conflict with Pi serial, but IR and Pi commands fight for the single command slot in the firmware. The MU camera uses Grove G1 (I2C) on the head; it shares the I2C bus the switch arbitrates.

## Power budget

- Battery: ZCF503060 Li-ion, 1000 mAh, 7.4 Wh (2S, ~7.4 V nominal), charged at 5 V 1 A. Standby "several hours", running "less than 0.5 hours". https://docs.petoi.com/technical-support/battery
- Servos run straight off the 2S pack; the Pi's 5 V comes off the same pack through the buck converter, so Pi and servo current share one cell stack. The pack's protection trips on peaks (roughly >2.5 A, approximate figure).
- Petoi's warning for Nybble: "Reduced motion capability may happen when connected to Pi! A stronger battery is needed... Nybble will be less capable for intense movements, such as trot." They suggest a high-drain 7.4 V LiPo or 2S-18650 pack.
- Typical Pi draw at 5 V: Zero 2 W ~0.4 W idle / 1.4 W load; Pi 2 B ~1.1 W idle / ~2.1 W load; 3B+ ~1.9 W / 5.2 W; 4B ~2.9 W / 6-8 W. A Pi 4 is not realistic on the stock battery.

## Recommendation

- Final build: **Pi Zero 2 W** (lowest draw that still has the 40-pin header and a quad-core A53). Needs a soldered header; same `ttyS0`/disable-bt dance as the 3A+. Petoi's own validated choice is the 3A+.
- Development: any Pi on the desk, powered separately, through the USB adapter to `/dev/ttyUSB0`. The protocol is identical, so the mind layer can be written before touching the header.
- Petoi's framing fits the goal: "With Nybble's tiny body, it's better to serve as a platform for initiating the communication framework and behavior tree rather than a racing beast."

## Pi 2 Model B v1.1 (on hand)

- Same 40-pin header as the 3A+/3B (pins 1-10 identical), same 85 x 56 mm "B" footprint and mounting holes as the 3B that the Nybble mount references, so it should fit the y1Pi / Pi Stand parts; it overhangs more than a 3A+ because of the USB/Ethernet block.
- No onboard Wi-Fi or Bluetooth: the PL011 is on GPIO14/15 as `/dev/ttyAMA0` with no mini-UART clock issue, which is better for serial reliability, but `ardSerial.py`'s port filter must be changed to accept it. Networking needs a USB Wi-Fi dongle (adds ~0.5-1 W) or Ethernet.
- Draw ~1.1-2.1 W: between a Zero 2 W and a 3B. Workable with the stock battery for development and gentle motion; expect reduced trot headroom, as Petoi warns for any Pi.
- Quad-core Cortex-A7 at 900 MHz, 1 GB RAM: ample for a Python behaviour tree sending tokens; not enough for on-board vision models. Raspberry Pi OS (32-bit) still supports it.
- Verdict: fine as the first mind, especially off-robot over USB; upgrade to a Zero 2 W later if weight, power or Wi-Fi become the limit.

## Alternatives

- **BiBoard (ESP32)**: Petoi's full NyBoard replacement with Wi-Fi/BT, documented for Bittle X / Nybble Q, not for the original wooden Nybble. Treat as unsupported DIY on Kilo.
- **ESP32 / Pi Pico W on the same header**: electrically plausible (3.3 V UART, 5 V 3 A available), much kinder to the battery, nothing documented by Petoi; the token sender would be a port of `ardSerial.py`.
- **HeyPCB/whisker**: community KiCad NyBoard replacement for Nybble (ATmega328P, USB-C, 3 A buck). Third-party.

## Unconfirmed / flagged

- Exact 2x5 socket pinout (derived, no official diagram).
- Pi Zero / Zero 2 W and Pi 2 B fit on the printed Nybble Pi mount.
- Whether Petoi sells a Nybble Pi mount (the STLs are free; none listed in the 3D-print parts store).
- The ~2.5 A battery protection trip figure.
- BiBoard on the original Nybble; ESP32/Pico on the header.
- The manual's old forum link about Pi 3B placement predates the forum move and may be dead.

## Sources

1. https://docs.petoi.com/apis/raspberry-pi-serial-port-as-an-interfac (the essential page: socket, 3A+, 3.3 V, unplug USB, 1-wire/GPIO4, raspi-config)
2. https://docs.petoi.com/nyboard/nyboard-v1_1-and-nyboard-v1_2 (I2C switch, TPS565201 5 V 3 A)
3. https://nybble.petoi.com/chapter3 (y1Pi / Pi Stand, jumper-pin warning)
4. https://github.com/PetoiCamp/NonCodeFiles/tree/master/stl/NybbleWood (Nybble_y1Pi.stl, Nybble_xPlateForPi.stl)
5. https://docs.petoi.com/apis/serial-protocol
6. https://docs.petoi.com/communication-modules/introduction-for-nyboard (6-pin socket, 115200)
7. https://docs.petoi.com/technical-support/battery
8. https://docs.petoi.com/faq-frequently-asked-questions
9. https://wiki.gentoo.org/wiki/Raspberry_Pi_Serial_Ports (ttyS0 / ttyAMA0 / BT overlays)
10. https://github.com/PetoiCamp/ros_opencat
11. https://pidramble.com/wiki/benchmarks/power-consumption (Pi power figures)
12. Local: `serialMaster/ardSerial.py` (port detection), `src/OpenCat.h` (tokens, baud)
