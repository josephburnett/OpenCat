# Timing: Kilo vs the legacy OpenCat firmware

**Summary**
1. The old firmware has no clock for walking: it writes one servo per pass of its main loop, 12 writes per gait frame, so its speed depends on how long a pass takes. Nobody has timed it on Kilo.
2. Worked out from its code, an old walk cycle takes about 0.32 s (somewhere in 0.21-0.47 s), trot 0.24 s, crawl 0.31 s. That is 4-7 times faster than Kilo's walking today (2.0 / 1.0 / 2.0 s) and faster than the 1.0 / 0.5 / 1.5 s you called "way too fast".
3. The old steps are also 2-2.5 times longer (walk 74 mm, trot 86 mm; Kilo 30 / 35 mm), and its trot keeps feet on the ground 70% of the time (Kilo's 55%). A guess, not tested: the wobbling comes from that shorter ground time, not from the speed.
4. Kilo can't copy the old timing with the old step sizes. Its 50-per-second update would need 2-4 times the joint speed its safety limit allows. The old firmware asked the servos for more than they can do. The proposal (on a branch, not to flash before a measurement) uses the old timing with small steps.
5. Tricks already run at about the old speed (0.7-1.0 of the old time on the best estimate). Posture changes take 0.5 s on Kilo and 0.2-0.4 s on the old firmware.

Labels used below: **[measured]** = seen on the robot; **[code]** = worked out
from the source in this repo; **[assumed]** = a typical figure for the chip or
part, not checked on Kilo. Times in ms unless marked s.

## 1. How the legacy firmware times a gait [code]

- A gait is a table of frames (`src/InstinctNybble.h`; `wkF` 44 frames, `trF`
  33, `crF` 42, `bk` 32, `vtF` 14). Each frame holds 8 leg angles.
- `loop()` (`OpenCat.ino`) runs `readEnvironment()` (IMU), `readSignal()` (IR,
  serial, ultrasonic), then `reaction()`, which calls `skill.perform()` once.
- `perform()` (`src/skill.h`) writes **one joint per call**: joints 0-3 (head,
  tail, one unused), then 8-15 (legs). So **one frame = 12 passes of
  `loop()`**. There is no `delay()` in the gait path (the `delayMicroseconds(850)`
  is compiled only without the gyro). The header's "speed" field does not
  exist for gaits; a gait's speed is its frame count times the pass time.
- The IMU is read on every pass of every 3rd frame (`frame % imuSkip`,
  `IMU_SKIP 3`), and those frames also run the balance maths (`adjust()`) for
  every joint. The IR key "CH+" (`g`, "turn off gyro feedback to boost speed")
  skips all of that.
- I2C runs at 500 kHz (`Wire.setClock(500000L)`). One servo write
  (`setPWM`) is 6 bytes ≈ 0.11 ms on the wire.
- The IR library (`src/ir/IRremote.cpp`) samples the receiver in a 50 µs timer
  interrupt with `digitalRead()`, which takes roughly 10-20% of the CPU
  [assumed share].

Cost per pass, from the code with typical ATmega328P float and I2C costs
[assumed figures: float add ≈ 7 µs, multiply ≈ 9 µs, divide ≈ 30 µs,
sin/cos ≈ 0.1 ms, atan2 ≈ 0.17 ms]:

| Part | fast | central | slow |
|---|---|---|---|
| One leg write (`calibratedPWM` + `setPWM`) | 0.20 | 0.30 | 0.42 |
| One head write (adds a `sin()`) | 0.33 | 0.44 | 0.58 |
| Extra per pass in an IMU frame (FIFO count read, `adjust()`, deviation) | 0.25 | 0.33 | 0.43 |
| One DMP packet (28 B read + quaternion to angles; 100 Hz packets) | 1.4 | 1.6 | 1.9 |
| Frame without IMU (8 leg + 4 head writes, + IR interrupt) | 3.2 | 4.8 | 6.8 |
| Frame with IMU (adds 12 × extra and 1-2 packets) | 8.1 | 12.2 | 18.2 |
| **Average frame (1 in 3 with IMU)** | **4.8** | **7.3** | **10.6** |

**Ultrasonic caveat [code].** Three seconds after the last command (an IR key
or serial line), `readSignal()` calls `readRGBultrasonic()` on every pass.
When nothing is within 120 cm, or something is 50-90 cm away, it pings on every
pass, and `pulseIn()` waits for the echo: about 7 ms or more per pass at 120 cm
and beyond. That would make each frame ≥ 85 ms and a walk cycle ≥ 3.7 s.
With a hand 10-50 cm away it stops walking and does its follow posture
instead. So a legacy walk started from the remote was probably fast for 3 s
and then much slower, if the module answered as the code expects. Not seen
on Kilo; this is the biggest unknown in the old timing.

## 2. Gaits: legacy vs Kilo

Cycle = one full period of all four legs.

| Gait | Legacy frames | Legacy cycle, gyro on: fast / central / slow [code] | Legacy, gyro off, central [code] | Kilo before 2026-10-12 | Kilo now (main) | Proposal (branch) |
|---|---|---|---|---|---|---|
| Walk (`wkF`) | 44 | 214 / **322** / 470 | 211 | 1000 | 2000 | **320** |
| Trot (`trF`, the remote's default in both) | 33 | 160 / **240** / 350 | 158 | 500 | 1000 | **300** (240 is below SET_GAIT's 300 minimum) |
| Crawl (`crF`) | 42 | 203 / **305** / 445 | 202 | 1500 | 2000 | **305** |
| Backward (`bk`) | 32 | 156 / **235** / 343 | 154 | the selected pattern with a negative stride (same period) | same | same |
| Step in place (`vtF`) | 14 | 69 / 104 / 152 | 67 | `march`: the pattern's period | same | same |

Geometry, read off the legacy frames with Kilo's leg model (§3d forward
kinematics, L1 45 mm and L2 50 mm [measured], the IK sign conventions of
`body/src/core/ik.h`) [code]:

| Gait | Stride, legacy / Kilo now | Swing clearance, legacy / Kilo now | Duty (feet down), legacy / Kilo now | Body height, legacy / Kilo now |
|---|---|---|---|---|
| Walk | 74 mm / 30 | ≈ 10 mm / 15 | 0.70-0.75 / 0.75 | 77 mm (0.81) / 76 (0.80) |
| Trot | 86 mm / 35 | ≈ 8 mm / 12 | 0.70 / 0.55 | 73 mm (0.77) / 76 (0.80) |
| Crawl | 61 mm / 25 | ≈ 13 mm / 18 | 0.57-0.60 / 0.85 | front 53, back 69, mean 61 (0.64), ≈ 7° nose down / 66.5 (0.70), level |
| Backward | 63 mm / the pattern's | ≈ 5.5 mm | 0.72-0.75 | 73 mm |

- Duty is the share of frames in which the foot moves backward relative
  to the body (on the ground). Clearance is how far a swinging foot rises
  above the other foot at the same end.
- The legacy stance is not flat: the foot drifts 6-8 mm in height while on the
  ground, so the body bobs.
- Swing order: the legacy walk has the same order as Kilo's WALK (LB, LF, RB,
  RF), but uneven (LF and RB about 0.1 cycle apart, the other gaps about
  0.4). Kilo spaces them a quarter cycle apart. The legacy trot pairs the
  diagonals half a cycle apart, as Kilo does.
- Speed on paper (stride ÷ cycle): legacy walk ≈ 230 mm/s, trot ≈ 360 mm/s;
  Kilo before today 30 and 70 mm/s, now 15 and 35 mm/s. Kilo's formula held on
  the floor: 25 mm at 1200 ms covered "15 cm exactly" in 8 s [measured, P2].
- The legacy frames ask for up to 13-16° per frame, about 2° per ms
  [code]. A small hobby servo manages about 0.6° per ms [assumed]. So the old
  legs could not keep up with their own tables: the real stride was
  shorter and lagged. The timing still held, because frames advance whether
  or not the servo got there.

## 3. Why the proposal keeps the steps small

Kilo's gait runs at 50 Hz (a 20 ms tick; not 10 ms) and its safety limit
(`SLEW_MAX_A16`) caps every joint at 10° per tick. Highest joint step per
tick for each set (float model of §3d, checked against a host build of
`body/test/test_gait.cpp`) [code]:

| Set | Cycle | Duty | Stride | Step | Highest joint step per tick |
|---|---|---|---|---|---|
| Walk now (main) | 2000 | 0.75 | 30 | 15 | 3.7° |
| Trot now (main) | 1000 | 0.55 | 35 | 12 | 3.5° |
| Walk at legacy cadence and legacy stride | 320 | 0.70 | 74 | 10 | 25° (clipped) |
| Trot at legacy cadence and legacy stride | 240 | 0.70 | 86 | 8 | 37° (clipped; 240 is also out of range) |
| **Proposal walk** | 320 | 0.75 | 15 | 8 | 8.5° |
| **Proposal trot** | 300 | 0.70 | 12 | 8 | 9.7° |
| **Proposal crawl** | 305 | 0.59 | 20 | 12 | 9.3° |

At the old cadence a swing lasts only 4-6 Kilo ticks, so only small steps fit
under the limit. The proposal matches the old **timing**, duty and body
height, not the old step size. On paper (stride ÷ cycle) it moves at 47 /
40 / 66 mm/s (walk / trot / crawl): the walk faster than at the first floor
drive (30 mm/s), the trot slower (70 mm/s), and every leg steps 1.7-3 times
as often as it did then.

Proposed `protocol/spec.toml` `[gait_patterns]` values:

| Pattern | period_ms | duty /256 | stride_mm | step_h_mm | body_h_256 |
|---|---|---|---|---|---|
| WALK | 320 | 192 (0.75; legacy front 0.75, back 0.70) | 15 | 8 | 207 (0.81) |
| TROT | 300 (legacy 240, below the range) | 179 (0.70) | 12 | 8 | 197 (0.77) |
| CRAWL | 305 | 151 (0.59) | 20 | 12 | 164 (0.64) |

Offsets are unchanged. The legacy crawl's nose-down pitch is not a pattern
parameter (it would be `SET_BODY_POSE`). At duty 0.59 the CRAWL has two feet up
for part of the cycle, as the legacy crawl does.

Tests that pin the old defaults and will fail with this proposal (not changed
on the branch):
- `body/test/test_gait.cpp`: `STEP[3]` in `gp()` and `STRIDE[3]` in
  `test_defaults_and_cost` are hard-coded; with them set to the proposal it
  passes. Still failing then: CRAWL "one foot in the air at a time" (line 177,
  by the legacy duty), the pattern-change and after-trick slew checks (they
  use a 30 mm stride, too much at 300 ms), and the watchdog timing (line 685,
  it assumes the crawl is the slowest at 1.2 s).
- `mind/tests/test_gait.py`: `test_gait_fields_defaults_per_pattern`,
  `test_walk_args`, `test_client_walk_stop_and_geometry`,
  `test_kilo_walk_direct` (the defaults, the stop time and the printed
  period).

## 4. Tricks [code]

A legacy behaviour frame is `transform()` at `speed/4` degrees per step
(each step writes all 16 joints with a cosine ease, `steps + 1` passes), then
`delay(|delay| × 50 ms)`. One step costs 16 × (cos + float maths + one servo
write) ≈ **5.6 / 7.4 / 10.2 ms** (fast / central / slow; no IMU reads inside).
The importer (`mind/kilo/tricks/legacy.py`) uses `LEGACY_STEP_MS = 5.5` (an
estimate; the planned calibration by filming has not been done) and maps the
delay exactly (50 ms = 2.5 ticks).

| Trick (remote key) | Legacy total, fast / central / slow (s) | Kilo (s) | Kilo ÷ legacy central |
|---|---|---|---|
| hi (4) | 1.7 / 2.2 / 3.0 | 2.2 | 1.0 |
| push_up `pu` (5) | 3.6 / 4.8 / 6.5 | 3.4 | 0.71 |
| pee (6) | 2.1 / 2.6 / 3.4 | 2.2 | 0.82 |
| wash_face `wsf` (7) | 3.3 / 4.0 / 5.1 | 3.7 | 0.92 |
| check_around `ck` | 1.5 / 2.0 / 2.7 | 1.4 | 0.72 |
| sniff `snf` | 1.5 / 1.9 / 2.7 | 1.5 | 0.76 |
| scratch `scrh` | 1.1 / 1.5 / 2.0 | 1.4 | 0.99 |
| stretch `str` (3), a posture | 0.30 / 0.39 / 0.54 to get there, then held until the next command | 0.50 there, then straight back to STAND | 1.3 for the move |
| butt_up (8), a posture | 0.26 / 0.34 / 0.47 | 0.50 | 1.5 |

Both start from `up`. The legacy `balance` is a posture equal to `up` (no
move). Where the differences come from:
- `LEGACY_STEP_MS` 5.5 ms vs the central 7.4: Kilo's moves are up to 1.35×
  faster.
- Legacy runs `steps + 1` passes (`s = 0..steps`); the importer counts
  `steps`, so each frame is 6-10 ms short.
- Each Kilo move is rounded up to whole 20 ms ticks, up to 20 ms per frame.
- Loops: the importer sizes a loop's first frame from the frame before it
  in the file, but on a repeat the legacy firmware moves there from the
  loop's last frame. In `hi`, frame 2 is 300 ms every time on Kilo. On the
  legacy repeats it is about 90 ms (22° instead of 105°), so `hi` comes out
  equal only because the two errors cancel.
- `stretch` still ends at once: the single-frame posture hold noted in
  `docs/p1-hardware-test-results.md` is not in the importer yet.

## 5. Postures and the head

| Move | Legacy [code] | Kilo [spec] |
|---|---|---|
| Posture change (`sit`, `rest`, `up`, ...) | 2° per step, all 16 joints per step: ≈ 3.7 ms per degree of the largest joint change (2.8-5.1). From `up`: sit 0.22 / 0.29 / 0.40 s, rest 0.23 / 0.30 / 0.42 s | `SET_POSTURE` default 25 ticks = 500 ms whatever the distance, cosine eased |
| Into a gait | 2° per step on the 8 leg joints to the first frame, then frames | STANCE move, then the stride ramps in over one period |
| Head and tail while walking | pan ±10° once a cycle (+25° to the side on L/R gaits), tilt ±10° 1.5 times a cycle, tail ±10° twice a cycle, all at the frame rate | tilt bob ±4° twice a cycle, tail wag ±15° once a cycle, no pan |
| Head command | serial `i` (not on the remote): 4° per step ≈ 390-710°/s | `SET_HEAD` 120°/s (`[head] speed_dps`) |

## 6. What would settle it

All the legacy numbers here come from the code, not from timing the robot.
One video would replace them: flash the legacy firmware (`kilo flash
--legacy`), put Kilo on its stand, send `kwkF` (and `ktrF`), film the side,
and count leg cycles in the first 3 s and again after 10 s (the ultrasonic
caveat). Then `kilo -v flash` to go back. Filming `khi` the same way would
calibrate `LEGACY_STEP_MS` for the tricks.
