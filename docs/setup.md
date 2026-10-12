# Setting up a dev host and the Pi

Everything needed to rebuild the two machines Kilo's software runs on: a
Debian dev host (tests, builds) and the Raspberry Pi 2 B that is Kilo's mind.
The checkouts are still called `~/OpenCat` on both machines: the systemd unit
and the venv path expect that name (`mind/systemd/kilo.service`), so keep it
until that is changed in one go.

## Dev host (Debian 13)

| Tool | Purpose |
|---|---|
| `build-essential`, `g++` (gcc 14, with libasan) | body host tests, built with `-fsanitize=address,undefined` |
| `python3`, `python3-venv` (3.11+) | the mind and its tests |
| `git`, `jq` | |
| `arduino-cli` 1.5.x with core `arduino:avr` 1.8.8 | build the body and run `size_check.sh` (optional: the Pi can build and flash) |

```sh
sudo apt install build-essential g++ git jq python3 python3-venv
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
~/.local/bin/arduino-cli core update-index && ~/.local/bin/arduino-cli core install arduino:avr
git clone git@github.com:josephburnett/kilo.git ~/OpenCat && cd ~/OpenCat
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -e mind pytest            # add numpy for the one skipped test
export PATH="$HOME/OpenCat/.venv/bin:$HOME/.local/bin:$PATH"
body/tools/host_tests.sh && body/tools/size_check.sh && protocol/test.sh && .venv/bin/python -m pytest mind/tests -q
```

The four suites are the check before every commit. `size_check.sh` needs
arduino-cli; it also runs `body/tools/stack_check.sh` (the static worst-case
stack estimate, a second build with `-fstack-usage`, a few seconds; it fails
below 128 B of margin; `STACK_CHECK=0` skips it); `protocol/test.sh` needs the venv on `PATH` (it runs pytest).
`.venv/` is gitignored.

## The Pi

Raspberry Pi 2 Model B, Raspbian 13 (trixie) 32-bit, user `joe`, hostname
`kilo`, USB Wi-Fi on the home LAN (`ssh -i ~/.ssh/kilo.local joe@192.168.88.32`;
the key's public half is in the Pi's `authorized_keys`). Tailscale SSH is
installed too but asks for a browser re-check now and then, which hangs
scripts; prefer the LAN.

The Pi is in its own enclosure with its own supply for now, joined to the
NyBoard by a USB cable and a CH343 serial adapter (`/dev/ttyACM0`). Mounting
it on Kilo (header UART, `/dev/ttyAMA0`) is the goal; `docs/raspberry-pi.md`
and `docs/architecture.md` Appendix F cover that path.

Done by hand on 2026-10-10, in this order:

```sh
sudo apt install python3-serial python3-venv                                  # pyserial 3.5 from apt
sudo usermod -aG dialout joe                                                  # serial access; log in again
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
~/.local/bin/arduino-cli core update-index && ~/.local/bin/arduino-cli core install arduino:avr
git clone https://github.com/josephburnett/kilo.git ~/OpenCat && cd ~/OpenCat
python3 -m venv --system-site-packages .venv && .venv/bin/pip install -e mind pytest
echo 'export PATH="$HOME/OpenCat/.venv/bin:$HOME/.local/bin:$PATH" KILO_PORT=/dev/ttyACM0' >> ~/.bashrc
mind/systemd/install.sh --start      # user unit, ~/.config/kilo/env, lingering: the daemon starts at boot
```

Then `~/.config/kilo/env` was set to `KILO_RUN_ARGS=--remote --balance 0.99`
(2026-10-11): Kilo wakes up with the IR remote live and balance on.
`mind/README.md` "Daemon and systemd" explains the flags.

How reproducible this is: `install.sh` scripts the service part (unit, env
file, lingering, enable) and is idempotent. The rest is the eight lines
above, by hand. Not scripted, and not needed yet: the `raspi-config` UART
settings for the header path (serial console off, UART on, 1-Wire off,
`gpio=4=ip,np`), `python3-matplotlib`, `python3-opencv`. Also on the image:
numpy, PyYAML, picamera2. A fresh card would take the eight lines, the env
edit, and copying the ssh key.

Gotchas:
- A non-login ssh shell does not read `~/.bashrc`, so `kilo flash` cannot find
  arduino-cli: `export PATH=$HOME/.local/bin:$PATH` first.
- `sudo` wants a password, so apt installs and reboots need someone at a
  keyboard (or an ssh session with a terminal).
- While the daemon runs, every `kilo` command goes through its socket; stop
  it (`systemctl --user stop kilo`) for passive watching (`kilo monitor
  --passive`) or anything that opens the port directly.

Power. The root filesystem is ext4 (journaled) with `fsck.repair=yes` on the
kernel line and a persistent systemd journal. A sudden power cut is usually
harmless (the journal replays; this Pi has already recovered from one), but
repeated cuts wear the SD card and can corrupt it beneath the filesystem.
Before the Pi rides on Kilo's battery: consider the read-only overlay root
(`raspi-config`), and a clean-shutdown path (a remote key or a low-battery
rule that runs `shutdown` before the switch is flipped). The body does not
depend on any of this: when the Pi goes quiet it stops within 0.5 s, lies
down at 1.5 s and goes limp 10 s later.

## Day-to-day loop

1. Edit and run the four suites on the dev host. Commit on `main`, push.
2. On the Pi:
   ```sh
   cd ~/OpenCat && git pull && .venv/bin/pip install -e mind   # pull alone is enough unless pyproject.toml changed
   systemctl --user restart kilo                                 # if daemon code changed
   ```
3. Firmware: `export PATH=$HOME/.local/bin:$PATH; kilo -v flash` on the Pi
   (builds there in about 25 s, asks the daemon to release the port, flashes,
   verifies the HELLO). Check with the owner before anything that moves Kilo.
