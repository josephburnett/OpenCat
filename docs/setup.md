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
kernel line. The systemd journal is persistent since `trim-boot.sh` (below);
before that each boot's log was gone at the next one. A sudden power
cut is usually harmless (the filesystem journal replays; this Pi has recovered
from several), but a file being rewritten at that moment can come back empty,
and repeated cuts wear the SD card. That happened on 2026-10-12: after two
unclean shutdowns the saved Wi-Fi profile was a 0-byte file, the dongle never
connected, and the Pi was reachable only by Ethernet cable. Repair, as root
on the Pi: `sudo mind/systemd/fix-wifi.sh` rebuilds the profile from the
settings the Pi Imager left in `/boot/firmware/network-config` (the saved
profile now lives in `/etc/NetworkManager/system-connections/`).
Safe mode (next section) makes the system read-only against this. Still to do
before the Pi rides on Kilo's battery: a clean-shutdown path (a remote key or
a low-battery rule that runs `shutdown` before the switch is flipped). The body does not
depend on any of this: when the Pi goes quiet it stops within 0.5 s, lies
down at 1.5 s and goes limp 10 s later.

### Power: safe mode

In safe mode the system files are read-only: the Pi still writes them, but
into RAM, and those writes are gone at the next boot (the "overlay root").
What must survive lives on a third partition of the card, `/data`, written
normally: the home directory (repo, venv, `kilo-logs`, `~/.config/kilo/env`,
arduino-cli and its cores, ssh keys), Tailscale's identity
(`/var/lib/tailscale`), the system journal (`/var/log/journal`) and the clock
file (`/var/lib/systemd/timesync`; the Pi has no battery clock). They are
"bind mounts": the same folders, shown at their usual paths.

Setting it up, once:

1. On the Pi: `sudo poweroff`; when the green light stops, take the card out.
2. On a Linux machine with a card reader (rtb has none; the reader used with
   Raspberry Pi Imager will do), from a checkout of this repo:
   ```sh
   lsblk                                                    # the card: about 60G, partitions bootfs and rootfs
   sudo mind/systemd/data-partition.sh /dev/sdX --dry-run   # shows the card and the plan, changes nothing
   sudo mind/systemd/data-partition.sh /dev/sdX --yes
   ```
   `/dev/sdX` is the whole card (`sdb`, not `sdb1`). It refuses if anything on
   the card is mounted (desktops mount it on insert: `sudo umount /dev/sdX1
   /dev/sdX2`), if it is this machine's own disk, or if it does not look like a
   Raspberry Pi OS card. It shrinks the system partition to 16 GiB (5.7 GiB
   used), makes `/data` (ext4, label `kilo-data`) from the rest, copies the
   folders above into it, and adds `/data` and the bind mounts to the card's
   `/etc/fstab`. Safe to run again. The original folders stay on the system
   partition, hidden under the mounts: if `/data` ever fails to mount, the Pi
   still boots and ssh still works (with those old copies), but `kilo.service`
   does not start (`systemctl status kilo` says why).
3. Card back in the Pi, power on, then on the Pi:
   ```sh
   cd ~/OpenCat && git pull
   sudo mind/systemd/install.sh --system       # the unit now waits for /home/joe (RequiresMountsFor)
   mind/systemd/safe-mode.sh status            # /data and the four mounts: "mounted, in fstab"
   sudo mind/systemd/safe-mode.sh on --dry-run
   sudo mind/systemd/safe-mode.sh on && sudo reboot
   mind/systemd/safe-mode.sh status            # "overlay root now: ON", /boot/firmware read-only
   ```
   `on` refuses unless `/data` and every bind mount are in place. It uses
   raspi-config's own switches (`raspi-config nonint enable_bootro` and
   `enable_overlayfs`; the first time this installs the `overlayroot`
   package, so it needs the network), then writes
   `overlayroot=tmpfs:recurse=0` on the kernel line instead of raspi-config's
   `overlayroot=tmpfs`: without `recurse=0`, `/data` would get a RAM layer too
   and nothing would survive. `off` is the reverse. Both act at the next reboot.

**Updating in safe mode: no switching needed.** `mind/systemd/update.sh`
(`git pull`, `.venv/bin/pip install -e mind`, `sudo -n systemctl restart kilo`),
`kilo flash` and edits to `~/.config/kilo/env` write only under `/home/joe` (on `/data`),
`/run` and `/tmp`, so the day-to-day loop below works as it is, and its
results survive reboots.

**Needs safe mode off** (`sudo mind/systemd/safe-mode.sh off && sudo reboot`,
make the change, then `on` and reboot again): `apt`, `install.sh --system`
(the unit file in `/etc`), `trim-boot.sh`, `fix-wifi.sh` or any Wi-Fi
change, `raspi-config`, `usermod`, `passwd`. Done in safe mode they appear to
work and are gone after the next reboot. The Wi-Fi profile
(`/etc/NetworkManager/system-connections`) stays on the read-only system on
purpose: nothing in daily work changes it, and a power cut can no longer
empty it.

Done on the Pi 2026-10-13 (card step on rtb with a USB reader; `safe-mode.sh
on` then one reboot): root 16 GiB, `/data` 43 GiB; boot to the daemon 22 s,
whole boot 40 s; `/data` and the four bind mounts in place; `update.sh` and
`sudo -n systemctl restart kilo.service` work as before (the rule needs the
`.service` suffix). Checked at the same time:

- Swap is off in safe mode: `rpi-swap` (zram with `/var/swap` as backing
  file on the read-only root) stays inactive, `/proc/swaps` is empty. The
  daemon runs in well under 100 MB; the Pi 2 has 920 MB.
- The clock service orders after its bind mount
  (`After=var-lib-systemd-timesync.mount`, `RequiresMountsFor=/var/lib/systemd/timesync`)
  and writes the clock file on `/data`. The clock still starts at the last
  saved time and jumps once NTP answers, so the first minute of each boot's
  journal carries the old time.
- The RAM overlay held 86 MB after the first boot, all `/var/cache/apt`:
  `apt-daily` refreshed package lists into RAM. Candidate for the next
  safe-mode-off window: `systemctl disable --now apt-daily.timer
  apt-daily-upgrade.timer` (nothing is upgraded unattended in safe mode
  anyway; it would vanish at reboot).

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
   Measured on 2026-10-12 after `install.sh --system` and `trim-boot.sh`:
   kernel 7.8 s, `kilo.service` started at 22 s, first daemon log line at
   26.6 s after power (the Python start of `kilo run` takes ~4.6 s on the
   Pi 2 B), Wi-Fi connected on its own; whole boot 38.6 s. Before: 74 s.

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
2. On the Pi, one command, no password (the system-service install adds a
   sudoers rule that lets joe restart, start and stop `kilo.service` only):
   ```sh
   ~/OpenCat/mind/systemd/update.sh            # git pull, pip if needed, restart the daemon if needed
   ~/OpenCat/mind/systemd/update.sh --flash    # also rebuild and flash the firmware when body/ changed
   ```
   By hand, that is `git pull`, `.venv/bin/pip install -e mind` (only when
   `mind/pyproject.toml` changed) and `sudo -n systemctl restart kilo`.
3. Firmware: `export PATH=$HOME/.local/bin:$PATH; kilo -v flash` on the Pi
   (builds there in about 25 s, asks the daemon to release the port, flashes,
   verifies the HELLO). Check with the owner before anything that moves Kilo.
