# P3 hardware test: results

Procedure: `docs/p3-hardware-test.md`. Kilo = Petoi Nybble, NyBoard V1_2,
Raspberry Pi 2 B. Owner: Joseph Burnett. Remote keys by grid: columns A B C
left to right, rows 1-7 bottom to top.

## 0. Flash, 2026-10-14 20:04 PDT

Firmware build 56768228 (main 5676822), proto 1.5, flashed with Kilo on his
own battery (8.09 V, 85 %); the Pi reported no under-voltage during the
plug-in or the flash. `update.sh --flash` had skipped the flash ("firmware
unchanged") because the pull that brought the firmware had happened in an
earlier run: fixed in 04cdbc6 (`--flash` always flashes).

## 1-2. In-air checks: not possible

There is no stand. Held in the hand the body cannot lean (the lean is the
legs pushing the body over), so sections 1 and 2 of the procedure give no
information. Not repeated. The sign of the lean rests on the P1 balance
test (the legs correct a measured roll the right way on Kilo) and the host
test that the sway uses the same convention.

## 3. Floor, 2026-10-14 20:08-20:29 PDT

Wood floor. Telemetry `kilo monitor --json --rate 25`
(`~/kilo-logs/p3-20261014-201204.jsonl`, `p3-20261014-202022.jsonl` on the
Pi; the first went silent at a daemon restart, so the 20:19 run has no
trace). Balance on by the daemon's default (Kp 0.99) unless stated.

| run | command | balance | sway | roll trace | result |
|---|---|---|---|---|---|
| trot forward, back, turn right, march (remote, ~20:07, on the desk) | A3 then B7 / B5 / C6 / A4 | on | n/a (trot has none) | not recorded | all held, no fall |
| walk forward (remote, 20:10, desk) | C4 then B7 | on | on | not recorded | FELL (roll -) after 3.3 s |
| walk forward (remote, 20:15:40, floor) | B6, B7 | on | on | level 1.2 s, then peaks +6.3, +6.8, +14.6° at 0.35 s spacing, snap to -107° | FELL (roll -) after 2.7 s |
| walk forward, slowest (20:19) | `kilo walk forward --pattern walk --speed 5 --for 6` | on | on | no trace (see above); CLI: roll -1.3° at 1.1 s, 68° at 2.1 s | FELL (roll +) after ~2 s |
| walk forward, no sway (20:20:38) | `... --pattern walk --no-sway --for 6` | on | off | started at -5°, +3.9° at 0.96 s, +9.4° at 1.32 s, snap to -105° | FELL (roll -) after 1.9 s |
| walk forward, "balance off" hold (20:27, owner) | `kilo balance off --for 12 &` then the walk | **on** (the walk ended the hold and the default re-latched; flag on in the trace) | off | level 1.1 s, +7.5° at 1.36 s, -25° by 1.6 s, held there 0.5 s, over at 2.3 s | FELL (roll -) |
| walk forward, balance off (20:28:44) | `... --pattern walk --no-sway --no-balance --for 6` | **off** | off | -14.7 .. +17.8° (sd 6.3), pitch -3.8 .. 9.7 | **stayed up 6 s**, "wobbling a lot" |

Pattern walk at the defaults: period 2000 ms, duty 192/256, swing 500 ms,
stride 30 mm at 20 mm/s (8 mm at 5 mm/s), step 15 mm, body 76 mm. The
slowest speed does not change the lift cadence: with four legs at duty 0.75
the next leg lifts as the last one lands, so the roll reference flips every
500 ms at any speed.

### What the traces say

- Every fall with the balance on has the same shape: level for about a
  second, then a side-to-side swing at roughly 3 per second that grows each
  swing (6, 7, 15° or 4, 9°) and ends in a snap of 30° in 0.2 s. The sway
  on or off changes nothing about it.
- With the balance off the same gait rocks 15° each way and does not fall:
  each lifted leg lets the body tip toward it until the next foot catches
  it.
- So on three feet the balance loop turns a wobble into a fall. On four
  feet the body cannot tip and the loop only trims; on three it can tip
  about the far diagonal in under 0.1 s, the loop's correction arrives
  ~0.25 s later, after the body has turned round, and adds to the swing.
  The trot survives because its two-foot phase is 150 ms and all four feet
  are down in between.
- The sway as built (a roll reference handed to that same loop, merged
  405d139) cannot help: it asks the unstable loop to do more.

Sections 4 and 5 of docs/walking-and-balance.md record the direction
agreed on this evidence.

## 4. Speed keys

Not exercised (C5 slower / C1 faster were not pressed; log shows no
`speed` actions).

## 5. Tuning

None. No spec value changed.

## 6. Can the Pi keep up with a 50 Hz plan stream (2026-10-14 20:57-21:07 PDT)

Kilo unplugged. A 50 Hz Python loop on the Pi 2 B doing four legs of
IK-style float math per tick, with a thread writing a 200-byte line to the
SD card 25 times a second (fsync every 10 s), a 20 MB scp to the Pi over the
Wi-Fi dongle every 30 s, and the daemon running (`~/kilo-logs/tick_load.py`):

| | |
|---|---|
| ticks | 30000 over 600 s |
| lateness, median / 99 % / 99.9 % / max | 0.01 / 0.01 / 3.07 / 9.59 ms |
| ticks later than 5 ms / 20 ms / 100 ms | 4 / 0 / 0 |
| work per tick, median / max | 0.18 / 3.45 ms |
| garbage collections | 5 |

The worst tick was a tenth of the 100 ms lookahead the FEET stream gives
the body (link.feet_lookahead_ticks). Not covered: real serial traffic on
the same USB controller as the dongle (Kilo was unplugged), and a longer
soak.
