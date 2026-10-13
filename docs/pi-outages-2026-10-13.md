# Pi outages while Kilo is on the USB cable (evidence, 2026-10-12/13)

The Pi (own enclosure, own power supply) talks to Kilo over a USB cable to the
CH343 serial adapter (`/dev/ttyACM0`). Three times the Pi went down or would not
boot; each time Kilo was on that cable. This is what the Pi's own journal
(persistent since the boot trim) shows. Boot numbers are `journalctl --list-boots`
as of 2026-10-13 11:45 PDT.

| boot | when (PDT)              | what the journal shows                                                                                   |
|------|-------------------------|----------------------------------------------------------------------------------------------------------|
| -5   | 10-12 21:04 → hung ~20:27 next day | last line of the boot: `usb 1-1.5: USB disconnect, device number 5` (1-1.5 = the CH343, Kilo). Then nothing: the Pi was dead until power-cycled. |
| -4   | 10-13 06:51 → shutdown 11:17 | 153 s: daemon connects to Kilo. 156.5 s: Kilo's adapter disconnects by itself. 158.2 s: kernel `hwmon hwmon1: Undervoltage detected!`. 162.3 s: `Voltage normalised`. Adapter re-enumerates. |
| -3   | 11:12 (clock stale)      | 20.9 s: `usb 1-1.3: reset high-speed USB device number 4` (1-1.3 = the Wi-Fi dongle). Boot ends (power loss / reset). |
| -2   | 11:12                    | boot ends at 22.4 s, right after the journal flush.                                                      |
| -1   | 11:12                    | 21.9 s: `usb 1-1.3: USB disconnect, device number 4` (Wi-Fi dongle). Boot ends.                           |
| 0    | 11:42 (real time)        | the owner unplugged Kilo's USB cable; the Pi booted cleanly within a minute, Wi-Fi up at 36 s.           |

Second event, same day, with safe mode on (overlay root; boot numbers as of 12:45 PDT):

| boot | when (PDT)              | what the journal shows                                                                                   |
|------|-------------------------|----------------------------------------------------------------------------------------------------------|
| -4   | 12:03 → 12:31:45         | safe-mode boot, running 28 min, Wi-Fi and Tailscale up, Kilo unplugged. The owner plugs Kilo in: the journal just stops. No USB enumeration line, no under-voltage line: the Pi reset before anything could be written. |
| -3, -2, -1 | right after        | three boots with Kilo attached (dongle at 1-1.2, CH343 at 1-1.4), each dead at 21 s, before the mind service (22 s) touches the port; boot -3 shows the dongle being reset on the hub 4 s after enumeration. |
| 0    | 12:40:55                 | Kilo unplugged, power cycle: clean boot. Re-seating the Wi-Fi dongle while the Pi was in the failed-boot state did nothing, as expected. |

So plugging Kilo into a running Pi resets it outright (the clock file on /data
puts the Pi alive until 12:31), and the Pi cannot boot while Kilo is attached.
Nothing in software touches the port before the daemon: ModemManager is not
installed, no udev rule matches the adapter. The boots die before the daemon
runs, so this is not the DTR reset of the NyBoard either; it is the two 5 V
rails meeting through the cable.

Boots -3 to -1 all died at 21–22 s, the moment the mind service starts (23 s in
boot 0) and opens the serial port. The card changes of the same morning (root
shrunk to 16 GiB, data partition, bind mounts) are present in boot 0, which works.

What this is evidence of: an electrical interaction over the USB cable between
Kilo and the Pi. USB devices on the Pi's hub reset or drop, the Pi's 5 V rail
sags below 4.63 V (`Undervoltage detected!`), and the SoC resets. It is not
evidence of which side causes it. Candidates, in the order to check:

1. The adapter's 5 V wire. If the CH343 adapter passes USB VBUS to the NyBoard's
   5 V rail (or the NyBoard's 5 V to the Pi's VBUS), the two supplies are tied
   together through a thin cable. Servo load on Kilo then sags the Pi, and a Pi
   boot (USB power-up) hits Kilo's rail. Check: does the adapter's header to the
   NyBoard carry VCC/5V, and is the adapter powered from Kilo or from the Pi?
2. The Pi's supply. Pi 2 B + Ethernet + RT5370 dongle + CH343 can exceed a weak
   USB supply; the under-voltage line says the rail did sag at least once.
3. Not the SD card or software: boot 0 runs the same card and code as the three
   failed boots; the only difference is the cable.

How to test without guessing: `vcgencmd get_throttled` after a session with
Kilo attached (bit 0 / bit 16 = under-voltage now / since boot); a USB cable
with the VBUS wire cut, or a data-only adapter, between Pi and Kilo; power Kilo
on before the Pi and watch whether the Pi still boots.

Third event, 13:00 PDT, the decisive one. Logger on the Pi (`~/kilo-logs/plugin-watch.log`,
`plugin-kernel.log`: flags, core voltage, USB count every 0.3 s, synced to /data) and a
ping from rtb once a second. The owner plugged Kilo in **with Kilo's power switch
off**:

- 13:00:25.1 the CH343 enumerates (1-1.4); 13:00:25.3 `Undervoltage detected!`,
  core voltage drops to 1.2000 V (throttled); 13:00:31.6 `Voltage normalised`;
  `get_throttled` stays 0x50000 (under-voltage and throttling have occurred).
  The Pi survived this time; no ping lost.
- `kilo status` 35 s later: firmware build 74347362 running, reset
  WARM_UNEXPECTED, uptime 35 s, IMU at 100 Hz, **battery 93 mV, flag
  no_battery, mode LIMP**. Kilo's switch was off, so the NyBoard (MCU, IMU,
  PCA9685) was running on the Pi's USB 5 V through the adapter's 5 V pin.

That is the mechanism. The adapter ties the Pi's USB 5 V to the NyBoard's 5 V
rail. With Kilo off, the Pi powers Kilo's board and its supply sags below
4.63 V (a modest load; the Pi's supply or micro-USB cable is marginal). With
Kilo on, the two rails are tied and Kilo's servo load pulls the Pi down (last
night's hang). At boot, USB power-up at 15 s plus the board's inrush resets the
Pi at 21 s. Same wiring every time; whether the Pi survives is a matter of
how deep the dip goes.

Ways out, owner's choice (all hardware; the card is safe now, so each
plug-in costs only a reboot):

1. Break the 5 V line between the adapter and the NyBoard (pull or tape the
   5V/VCC pin of the 6-pin header; keep GND, TX, RX, DTR). Each side on its
   own supply; flashing needs Kilo switched on. Cleanest.
2. A powered USB hub between the Pi and the cable: the hub's supply feeds the
   adapter, the Pi's rail is no longer tied to Kilo.
3. A USB "data-only" blocker on the Pi side: works only if the adapter is
   powered from Kilo's board; if the adapter then fails to enumerate, it was
   powered from USB and (1) or (2) is needed.

Independently: the Pi's own supply path is marginal (6 s under 4.63 V from a
board-sized load). A short, thick micro-USB cable or a 5.1 V supply would give
headroom whatever is done about the adapter.
