# Setting up a dev host and the Pi

Everything needed to rebuild the two machines Kilo's software runs on: a
Debian dev host (tests, builds) and the Raspberry Pi 2 B that is Kilo's mind.
The checkouts are still called `~/OpenCat` on both machines: the systemd units
and the venv path expect that name (`mind/systemd/`), so keep it
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
`kilo`, USB Wi-Fi on the home LAN. Its LAN address comes from DHCP and has
changed (192.168.88.32, then .27 on 2026-10-12); the stable address is the
Tailscale one, `ssh -i ~/.ssh/kilo.local joe@100.96.106.5` (the key's public
half is in the Pi's `authorized_keys`; the Tailscale host key differs from the
LAN one, so accept it once). Tailscale SSH is
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
mind/systemd/install.sh --start      # (2026-10-10: the user unit; superseded by the system unit below)
```

Then `~/.config/kilo/env` was set to `KILO_RUN_ARGS=--remote --balance 0.99`
(2026-10-11): Kilo wakes up with the IR remote live and balance on.
`mind/README.md` "Daemon and systemd" explains the flags.

How reproducible this is: `install.sh` scripts the service part (unit, env
file, enable) and is idempotent; on a fresh card use the system service and
the boot trim below instead of the user unit. The rest is the eight lines
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
  it (`sudo systemctl stop kilo`) for passive watching (`kilo monitor
  --passive`) or anything that opens the port directly.

Power. The root filesystem is ext4 (journaled) with `fsck.repair=yes` on the
kernel line. The systemd journal is *not* persistent (`Storage=auto` and no
`/var/log/journal`), so each boot's log is gone at the next one; the daemon's
own `~/kilo-logs/daemon.log` is the only log that survives. A sudden power
cut is usually harmless (the filesystem journal replays; this Pi has recovered
from several), but a file being rewritten at that moment can come back empty,
and repeated cuts wear the SD card. That happened on 2026-10-12: after two
unclean shutdowns the saved Wi-Fi profile was a 0-byte file, the dongle never
connected, and the Pi was reachable only by Ethernet cable. Repair, as root
on the Pi: `sudo mind/systemd/fix-wifi.sh` rebuilds the profile from the
settings the Pi Imager left in `/boot/firmware/network-config` (the saved
profile now lives in `/etc/NetworkManager/system-connections/`).
Before the Pi rides on Kilo's battery: consider the read-only overlay root
(`raspi-config`), and a clean-shutdown path (a remote key or a low-battery
rule that runs `shutdown` before the switch is flipped). The body does not
depend on any of this: when the Pi goes quiet it stops within 0.5 s, lies
down at 1.5 s and goes limp 10 s later.

### The daemon as a system service, and boot time

Why: measured on 2026-10-11, the daemon (`kilo run`, the program that owns
the serial port to the body and that every `kilo` command talks to) started
67 s after power. It was a systemd *user* service, and systemd starts a
user's services only after the network is up (NetworkManager, the program
that runs the Wi-Fi, took 34 s). It is now a *system* service that runs as
joe and does not wait for the network at all. Everything below is run on
the Pi, in an ssh shell, from `~/OpenCat`; `sudo` asks for joe's password.

1. Install the system service (it also switches the old user service and
   "lingering", the setting that started it at boot, off; settings in
   `~/.config/kilo/env` are kept):
   ```sh
   cd ~/OpenCat && git pull
   sudo mind/systemd/install.sh --system --start
   systemctl status kilo        # "active (running)"
   kilo daemon                  # socket /run/kilo/kilo.sock, port, link state
   ```
   Run without `sudo`, `install.sh --system` prints the exact command to run
   instead. It is safe to run again.
2. Trim the boot. First look at what would change, then change it:
   ```sh
   mind/systemd/trim-boot.sh --dry-run
   sudo mind/systemd/trim-boot.sh
   ```
   It switches off cloud-init (the Pi Imager's first-boot setup, finished
   long ago; only if a non-empty Wi-Fi profile is saved on disk, which a
   dry run can only see with sudo), makes the systemd journal persistent
   (Raspberry Pi OS keeps it in RAM by default, so a power cut erases the
   evidence), masks
   `NetworkManager-wait-online` and `systemd-networkd-wait-online` (services
   whose only job is to make other things wait for the network; masked, so
   nothing can switch them back on), and removes `splash` (the graphical
   boot screen) from the kernel command line `/boot/firmware/cmdline.txt`,
   keeping a copy as `cmdline.txt.kilo-backup`. It does not touch
   NetworkManager, Wi-Fi, ssh or Tailscale, and leaves
   `rpi-resize-swap-file.service` alone (not known for certain to be safe to
   switch off; `systemctl cat rpi-resize-swap-file.service` shows what it
   runs). It prints each change and is safe to run twice.
3. Reboot (`sudo reboot`), wait until ssh answers again, then:
   ```sh
   systemd-analyze                              # total boot time
   systemd-analyze critical-chain kilo.service  # what kilo.service waited for
   journalctl -b -u kilo -o short-monotonic | head -3   # first line: seconds since power-on
   ```
   Expected: the daemon is up about 15-20 s after power (kernel 8 s, then
   basic system start, then the 2-3 s Python start of `kilo run`), even
   while the Wi-Fi is still connecting. Not yet measured on the Pi.

The fallback, if the system service ever misbehaves: `sudo systemctl
disable --now kilo` and `mind/systemd/install.sh --start` (the user service,
which starts about a minute after power).

**With no network.** The Wi-Fi dongle may be pulled out: the Pi boots and
Kilo runs without it, since nothing on the way to `kilo run` waits for the
network. NetworkManager stays enabled, so plugging the dongle back in brings
the Wi-Fi back by itself (expect a few seconds to a minute). While there is
no network there is no ssh: the only controls are the IR remote (needs
`--remote` in `KILO_RUN_ARGS`) and the behaviour runner (`--behaviour`), and
the logs in `~/kilo-logs` (`daemon.log`, plus `.kilolog.gz` with
`--kilolog`) are the only record of what happened until the dongle is back. (Inferred,
not checked on the Pi: the Pi has no battery clock, so with no network its
time is not set and the log timestamps carry on from the last time it was
saved at shutdown.)

**Why NetworkManager takes 34 s** (to look into later; commands only):
```sh
journalctl -b -u NetworkManager -o short-monotonic | head -60   # where the gaps are
systemd-analyze blame | head -20
systemd-analyze critical-chain NetworkManager.service
systemctl status systemd-udev-settle.service                    # active = something waits for udev to settle
journalctl -b -u systemd-udevd -o short-monotonic | tail -20
dmesg -T | grep -iE 'usb|wlan|80211|firmware|rtl|mt7|brcm'      # the Wi-Fi dongle's driver and firmware load
dmesg | grep -iE 'firmware' ; lsusb                             # which dongle, and missing firmware files
nmcli -f GENERAL,WIFI-PROPERTIES device show wlan0 | head -20
systemctl list-dependencies --reverse network-online.target     # what still asks for the network at boot
```

## Day-to-day loop

1. Edit and run the four suites on the dev host. Commit on `main`, push.
2. On the Pi:
   ```sh
   cd ~/OpenCat && git pull && .venv/bin/pip install -e mind   # pull alone is enough unless pyproject.toml changed
   sudo systemctl restart kilo                                   # if daemon code changed
   ```
3. Firmware: `export PATH=$HOME/.local/bin:$PATH; kilo -v flash` on the Pi
   (builds there in about 25 s, asks the daemon to release the port, flashes,
   verifies the HELLO). Check with the owner before anything that moves Kilo.
