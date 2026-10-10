# Kilo mind

The Python side of Kilo (docs/architecture.md §5): the serial link to the body
firmware, the body-state model, the `kilo` CLI, the flash tool and a virtual
body for testing without hardware. Python 3.11+, standard library plus
pyserial at runtime; pytest for the tests.

**P0 status.** The protocol module from `protocol/` (generated message
dataclasses and the COBS/CRC codec) is not integrated yet. Until it is:

- `kilo/link/codec_stub.py` frames with a minimal transcription of §4, and
  `kilo/link/messages_stub.py` knows only HELLO, TELEMETRY, ACK, EVENT and the
  empty-payload messages;
- commands that need other layouts (`kilo posture`, `kilo cal show/set`,
  `kilo param`, STATS in `kilo status`) report "not available yet:
  TODO(protocol)" and exit with status 3. `grep -rn "TODO(protocol)" kilo`
  lists every integration point.

## Install on the Pi

pyserial comes from apt (`python3-serial`, 3.5). Recommended: a venv that can
see the system site packages, so pyserial is not installed twice and the
system Python stays untouched:

```sh
sudo apt install python3-serial python3-venv
python3 -m venv --system-site-packages ~/kilo-venv
~/kilo-venv/bin/pip install -e ~/OpenCat/mind          # add '.[test]' for pytest
echo 'export PATH="$HOME/kilo-venv/bin:$PATH"' >> ~/.bashrc
```

The alternative, `pip install --break-system-packages -e .` into the system
Python, works but mixes pip-managed files into `/usr/local`; prefer the venv.
`pip install -e` makes the source tree live, so `git pull` is enough to update.

Serial access needs the `dialout` group (`sudo usermod -aG dialout $USER`), and
the Pi configuration in docs/architecture.md Appendix F (serial console off,
UART on, 1-Wire off, UART clock 48 MHz, GPIO4 left as an input).

## Port rules

1. `--port P` (or `$KILO_PORT`) always wins.
2. On a Raspberry Pi, the header UART: `/dev/serial0` resolved, else
   `/dev/ttyAMA0`. On the Pi 2 B that is the PL011 `ttyAMA0` on GPIO14/15.
3. Otherwise the first `/dev/ttyUSB*`, then `/dev/ttyACM*` (the Petoi CH340 /
   CH343 USB adapter, e.g. in WSL2 through usbipd).

Device nodes are globbed rather than listed with `serial.tools.list_ports`,
which hides `ttyAMA*`/`ttyS*` as "platform" ports on recent kernels (the same
problem Petoi's `serialMaster/ardSerial.py` works around).

> **Unplug the CH343 USB adapter whenever the Pi drives the UART**, and vice
> versa. Both drive the 328P's single RX line; with both attached, frames from
> each side corrupt the other's. On a Pi with an adapter plugged in, `kilo`
> prints a warning and still picks the header UART.

The link runs at 250000 baud 8N1. Flashing uses 115200 (optiboot).

## CLI

```sh
kilo status                  # HELLO, mode, attitude, battery, link stats (--json)
kilo monitor                 # live telemetry table; Ctrl-C to quit
kilo posture stand           # holds the posture until Ctrl-C (or --for SECONDS)
kilo cal [show] | cal set JOINT DEG | cal save | cal revert
kilo param get ID | param set ID VALUE [--persist]
kilo flash                   # build body/, size harness, reset, avrdude, verify HELLO
kilo flash --no-build        # flash the last build
kilo flash --legacy          # flash mind/firmware/opencat-legacy.hex
kilo vbody                   # virtual body on a pty; prints the pty path
```

Global options: `--port`, `--baud`, `--log` / `--log-file FILE` (JSON-lines
log of every frame and event, written by a background thread),
`--timeout`, `-v`/`-vv`.

There is no daemon yet, so each command opens the port itself. Goals are only
held while the mind is alive: when `kilo posture` exits, heartbeats stop and
the body's watchdog stops motion at 0.5 s and moves to REST at 1.5 s. That is
by design (§4).

### Flashing

`kilo flash` follows Appendix F:

1. `arduino-cli compile --fqbn arduino:avr:uno --output-dir ~/.cache/kilo-build body/`
   with the git hash as `-DKILO_BUILD`, then `tools/size.sh` if present.
2. Stops the serial link (in-process; a second kilo process cannot hold the port
   because it is opened exclusively).
3. Resets the board: on a USB adapter avrdude toggles DTR itself; on the Pi
   header (`--reset auto` picks it for ttyAMA*/ttyS*/serial0) GPIO4 is driven
   low for 100 ms (`--reset-ms`) and released back to an input (emulated
   open-drain, never driven high). The pulse uses the first available of:
   `pinctrl` (raspi-utils, standard on Raspberry Pi OS), the libgpiod tools
   `gpioset`/`gpioget` (apt `gpiod`, v1 or v2), or `/sys/class/gpio`. The
   libgpiod Python bindings are not required.
4. `avrdude -c arduino -p atmega328p -P PORT -b 115200 -D -U flash:w:HEX:i`,
   retried once with a fresh reset. avrdude is taken from PATH or from
   arduino-cli's `~/.arduino15` tools.
5. Restarts the link and waits up to 3 s for HELLO with reset cause FLASHED.

`--dry-run` prints the commands without running them.

### Virtual body

```sh
kilo vbody                    # prints e.g. /dev/pts/5; keys: f/F fall, r reboot, q quit
kilo --port /dev/pts/5 monitor
kilo vbody --fall-after 5 --no-keys
```

The virtual body is a Python simulation (HELLO, 25 Hz telemetry with a
swaying pitch/roll, ACKs with de-dup, the two-stage watchdog, falls with epoch
bumps). It is a P0 stand-in for §6's `body/test/vbody`, which will run the
real firmware core on stub HALs.

## Tests

```sh
pip install -e '.[test]'      # or: sudo apt install python3-pytest
python3 -m pytest             # from mind/
```

The link tests run over an in-memory loopback with a trivial length-prefixed
codec (they test the link's plumbing, not the framing): heartbeat gating,
goal refresh and epoch clearing, ACK/retry/de-dup, reconnect, reboot
detection. `tests/test_vbody.py` runs link + client against the virtual body,
once over a real pty with the stub codec. The suite takes about 15 s.

## Layout

```
kilo/link/codec.py          Codec / Decoder interface (Frame, DecodeError)
kilo/link/codec_stub.py     STUB §4 framing until protocol/ lands
kilo/link/messages.py       Messages adapter interface (pack/unpack by name)
kilo/link/messages_stub.py  STUB layouts: HELLO, TELEMETRY, ACK, EVENT
kilo/link/consts.py         type codes, statuses, events, modes (from the doc)
kilo/link/transport.py      pyserial, raw-fd (pty) and loopback transports
kilo/link/ports.py          port auto-detection
kilo/link/serial_link.py    SerialLink: reader/writer threads, goals, commands
kilo/body/state.py          BodyState
kilo/body/client.py         BodyClient: postures, calibration, params, stats
kilo/log.py                 JSON-lines logger thread
kilo/cli.py                 the `kilo` command
kilo/tools/flash.py         build + flash; kilo/tools/gpio.py GPIO4 reset
kilo/vbody/                 virtual body
firmware/                   stored hex images (legacy firmware)
```
