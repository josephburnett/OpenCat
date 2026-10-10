# P0 first hardware test

The first run of the new body firmware on Kilo, from the laptop (WSL2) over
the Petoi CH343 USB adapter. It checks that the firmware boots without
moving anything it should not, follows the watchdog timeline, talks the
protocol, and that the legacy firmware can be put back. Allow about an hour.
The checks map to H1, H2, H5 and H15 in docs/architecture.md Appendix H.

**Before you start:**
- The Pi is **not** connected to the NyBoard's UART. The adapter and the Pi
  header share the 328P's only UART.
- The battery is charged. USB alone cannot power the servos; with no battery
  the body reports `NO_BATTERY` and the battery reflexes are off.
- Kilo is on a stand, with all four legs and the head free to move, for every
  step from 3 on.

## 0. Setup (laptop, once)

On Windows, attach the adapter to WSL (the busid may change after a replug;
`usbipd.exe list` shows it):

```sh
"/mnt/c/Program Files/usbipd-win/usbipd.exe" attach --wsl --busid 1-2
ls -l /dev/ttyACM0
```

Opening a USB serial port asserts DTR, and on the NyBoard DTR resets the
328P. Stop every later `kilo` command from resetting Kilo by leaving DTR
asserted between opens. This opens the port once, which resets Kilo once:

```sh
stty -F /dev/ttyACM0 -hupcl
```

avrdude still resets the board for flashing, because it toggles DTR itself.
Repeat `stty` after every replug or re-attach.

Install the mind in a venv (pyserial comes from apt) and point it at the
adapter:

```sh
sudo apt install python3-serial python3-venv
python3 -m venv --system-site-packages ~/kilo-venv
~/kilo-venv/bin/pip install -e ~/OpenCat/mind
export PATH="$HOME/kilo-venv/bin:$PATH" KILO_PORT=/dev/ttyACM0
```

Build the legacy image for the rollback in step 9. `mind/firmware/` does not
ship one yet. On `mega`, `OpenCat.ino` and `src/` are the legacy firmware:

```sh
cd ~/OpenCat
arduino-cli compile --fqbn arduino:avr:uno --output-dir /tmp/legacy-build .
cp /tmp/legacy-build/OpenCat.ino.hex mind/firmware/opencat-legacy.hex
```

## 1. Dump the legacy EEPROM (0..282)

The new firmware never writes addresses 0..703. The legacy layout is only
read, once, to import the calibration. Still, keep a copy from before the
first flash, and commit it as the P0 task asks (§7).

This tiny sketch prints the bytes in the same format as `kilo eeprom dump`, so
the two can be diffed later:

```sh
mkdir -p /tmp/eedump && cat > /tmp/eedump/eedump.ino <<'EOF'
#include <EEPROM.h>
void setup() {
  Serial.begin(115200);
  delay(500);
  char line[8];
  for (int a = 0; a < 283; a++) {
    if (a % 16 == 0) { if (a) Serial.println(); sprintf(line, "%04d:", a); Serial.print(line); }
    sprintf(line, " %02x", EEPROM.read(a));
    Serial.print(line);
  }
  Serial.println();
  Serial.println("END");
}
void loop() {}
EOF
arduino-cli compile --fqbn arduino:avr:uno /tmp/eedump
arduino-cli upload --fqbn arduino:avr:uno -p /dev/ttyACM0 /tmp/eedump
python3 - <<'EOF' | tee /tmp/kilo-eeprom-legacy.txt
import serial, time
s = serial.Serial("/dev/ttyACM0", 115200, timeout=5)
s.dtr = False; time.sleep(0.2); s.reset_input_buffer(); s.dtr = True   # DTR pulse: reset, so it prints again
out = b""
while b"END" not in out:
    chunk = s.read(256)
    if not chunk:
        break
    out += chunk
print(out.decode(errors="replace").split("END")[0].strip())
EOF
```

Expect 18 lines, `0000:` to `0272:`. Bytes 0..15 are the PWM pin map `0c 0b
04 03 0d 0a 05 02 0e 09 06 01 0f 08 07 00`. Commit the dump:

```sh
{ echo '# Kilo legacy EEPROM 0..282'; echo; echo 'Dumped before the first flash of the P0 body firmware.'; echo;
  echo '```'; cat /tmp/kilo-eeprom-legacy.txt; echo '```'; } > ~/OpenCat/docs/kilo-eeprom-legacy.md
```

The sketch replaced the legacy firmware. Put it back for step 3:

```sh
kilo flash --legacy
```

## 2. Host checks (laptop)

Run these before the first flash:

```sh
cd ~/OpenCat
body/tools/host_tests.sh                       # host unit tests: ALL PASSED
protocol/test.sh                               # needs pytest: PYTHON=~/kilo-venv/bin/python
(cd mind && python3 -m pytest -q)              # incl. the C-vs-vbody cross-check
body/tools/size_check.sh                       # see the note on FLASH_MAX below
```

**Flash ceiling.** The P0 build is about 18.2 KB, over the 16 KB P0 ceiling
(docs/architecture.md §3h, §8 question 9). Until the owner decides, flash with
an explicit temporary ceiling. The forbidden-symbol scan still runs:
`FLASH_MAX=18432 kilo flash`.

## 3. Legacy pose, then flash (H1, H2)

With the legacy firmware running and Kilo on the stand, make it stand up:

```sh
python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200); time.sleep(3); s.write(b'kup\n'); time.sleep(2)"
```

Kilo stands. The PCA9685 keeps driving that pose through the 328P resets that
follow. Now build and flash the new firmware:

```sh
cd ~/OpenCat
FLASH_MAX=18432 kilo flash -v
```

The output shows `arduino-cli compile`, the size harness, `reset: DTR
(avrdude)`, avrdude, then `verified: HELLO ... fw_build=<first 8 hex digits
of git rev-parse HEAD>`. Over USB the verify accepts any reset cause:
reopening the port resets Kilo once more.

**Expected, watching Kilo:**

1. While avrdude runs (≈5 s), the pose holds. The servos stay powered by the
   PCA9685.
2. On boot the LED is on for ≈0.3 s (DMP bring-up), and the pose is still held
   with **no twitch**. This is the warm boot: the counts read back from the
   PCA9685 become the held pose.
3. Within about a second, a **smooth** move to REST, lying down over ≈1 s.
   Nobody is talking to Kilo, and the body never stands without the Pi.
4. **≈10 s later, LIMP**: all servos go slack.
5. **No reset loop**: no repeated LED flashes and no repeated movements.

If the pose jumps on boot, stop and power off. It means the calibration
import failed (HELLO `config_state` 2 = DEFAULTED, see step 4) or the warm
read-back is wrong. Note what the HELLO says.

## 4. Status and telemetry

```sh
kilo status
```

Expect:
- `firmware proto 1.1  build <hash>`;
- `reset` = FLASHED or WARM_UNEXPECTED (the port reopen, above);
- `config_state 1` (MIGRATED) on the first boot, `0` (OK) afterwards. `2`
  means the legacy calibration did not pass the import checks: STAND is then
  refused as UNCALIBRATED. Send the HELLO and `kilo eeprom dump 0 283`;
- `mode LIMP`, a sane pitch/roll (± a few degrees when level), `batt_mv` near
  the meter reading;
- body stats with `overruns 0`, `max_tick_us` well under 20000, and
  `stack_free_min` > 150.

The EEPROM must be untouched:

```sh
kilo eeprom dump 0 283 > /tmp/after.txt
diff /tmp/kilo-eeprom-legacy.txt /tmp/after.txt && echo "legacy EEPROM intact"
```

Live view (Ctrl-C to quit). Tilt the stand gently: pitch and roll follow.
Nose up is +pitch, left side up is +roll.

```sh
kilo monitor
```

## 5. Postures (H2 counts, REST → STAND → SIT → REST)

Each `posture` command holds the posture until it exits. Then the watchdog
stops at 0.5 s, lies down at 1.5 s, and limps 10 s later. That is the H5
timeline, so watch it every time.

```sh
kilo posture rest --for 3      # from LIMP: straight to REST (slew-limited), then the watchdog
kilo posture stand --for 5 --pca   # REST -> STAND over 0.5 s; prints the PCA counts while standing
kilo posture sit --for 5
kilo posture balance --for 3   # = STAND with the balance flag (balance itself is P1)
kilo posture rest --for 3
```

- The moves are smooth, with no jumps at the start or the end.
- The `--pca` read-back during STAND gives the H2 numbers (only one `kilo`
  process can hold the port, so it is read inside the posture command): channels 0-2 and 8-15 within
  ±1 count of what the legacy `kup` drove. Compare them with a legacy
  read-back if you have one; otherwise record them.
- After each command exits: still for 0.5 s, REST by ≈2.5 s, LIMP ≈10 s after
  that.

## 6. Calibration and params

```sh
kilo cal show                  # 16 offsets = the legacy CALIB values (H2), in degrees
kilo param list                # 13 params, all at their defaults
kilo param set WD_LIMP_S 5     # live, not persisted
kilo param get WD_LIMP_S
```

Do **not** `kilo cal set` or `kilo cal save` yet. The imported calibration is
the reference for H2.

## 7. Power cycle

Switch the battery off, wait 5 s, then switch it on with the adapter still
attached. Run `stty -F /dev/ttyACM0 -hupcl` again if the adapter
re-enumerated.

- Kilo **stays LIMP**: a cold boot has no pose to hold.
- `kilo status` shows `reset COLD` and `config_state 0` (the imported slot is
  now in EEPROM, at 704..959).

## 8. Watchdog with a held posture (H5, standing)

```sh
kilo posture stand --for 2
```

Then Kilo stops (0.5 s), lies down (REST by ≈1.5 s plus 1 s of movement),
and limps 10 s later. To repeat it while monitoring, run `kilo monitor` in a
second terminal: the mode goes IDLE → LINK_LOST → LIMP, and the epoch
increments at the watchdog trip.

## 9. Roll back (H15)

```sh
kilo flash --legacy
python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200); time.sleep(3); s.write(b'kup\n'); time.sleep(2)"
```

The legacy firmware boots ("Nybble ... Ready!" at 115200) and `kup` stands
with the old calibration. Its EEPROM was never written.

## What to report

- The `kilo flash -v` output and the first `kilo status`, including the HELLO
  line and `config_state`.
- Anything that moved when it should not, with the step number.
- The `--pca` counts while standing (H2) and the diff result from step 4.
- The watchdog timings you saw in steps 5 and 8.
