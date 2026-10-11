# Kilo architecture: body firmware and mind software

This is the design for rebuilding the firmware and control software of Kilo,
a Petoi Nybble on a NyBoard V1_2 with a Raspberry Pi 2 Model B on its back.

**Status:** revised draft for implementation, 2026-10-10.

**Ground truth:** `OpenCat.ino` and `src/` on branch `mega` at 684d0eb, plus
`docs/raspberry-pi.md`. Numbers marked **(est.)** are estimates. All other
numbers were read from the code or measured on that commit. Compiling it with
`arduino-cli -b arduino:avr:uno` gives 32016 B of flash and 1620 B of globals.

**Units:**

| Name | Meaning | Type |
|---|---|---|
| `a16` | angle in 1/16 degree | `int16_t`, ±2047.9° |
| `q2` | length in 1/4 mm | `int16_t` |
| `Qn` | fixed point with n fractional bits | stated at each use |
| tick | one 20 ms motor-loop period (50 Hz) | |
| count | one PCA9685 step, 1/4096 of the PWM period (≈1 µs) | |

All multi-byte protocol fields are little-endian. Appendices B to H hold the
derivations, the archaeology of the legacy firmware, the full byte tables,
operations notes and test tables. The main body links to them.

---

## Decisions at a glance

| Topic | Decision | § |
|---|---|---|
| Split | BODY = new NyBoard firmware (real-time, reflexes); MIND = Python on the Pi (goals, personality). The Pi is always onboard. | 2 |
| Tick | 50 Hz, polled against a fixed `micros()` deadline. Nothing blocks. | 3a |
| Libraries | Custom UART, polled I2C, PCA9685 and DMP drivers. No HardwareSerial, Wire, I2Cdev, Print or `pulseIn`. Integer maths; float is a sanctioned fallback for IK only. | 3 |
| Ultrasonic | Echo timed by a pin-change ISR on D6. 80 ms cadence, 25 ms cap. | 3a |
| Reset cause | `.noinit` RAM magic plus a tick-alive canary, because optiboot clears `MCUSR`. | 3a |
| Calibration | Imported from the legacy EEPROM's per-robot values. New A/B 128 B slots at 704..959. The legacy layout is never written. | 3b |
| Arbitration | Owners REFLEX > TRICK > GAIT > IDLE per joint, a 200 ms crossfade, then head and balance layers, then limits and slew. | 3c |
| Stale goals | A goal epoch byte. Reflex takeovers and watchdog trips bump it, and goals carrying an old epoch are refused. | 3c, 4 |
| Gait | Parametric phase oscillator, per-leg planar 2-link IK, fixed point. | 3d |
| Tricks | 24 × 14 B keyframes in RAM (32 until the P1 RAM shrink, 3h), int8 degrees, cosine, linear or step easing, uploaded by the Pi (CRC-checked, cached by `(trick_id, crc)`), played with speed and mirror by the body; the TRICK owner hands back with a 200 ms crossfade into the trick's end posture. Implemented in P1. | 3e |
| Balance | `corr = −(Kp·e + Kd·rate)` at 50 Hz. Geometric (through IK) for gait and stance, legacy joint gains for joint-space owners (P1: implemented for IDLE SIT and STAND). | 3f |
| Attitude | Integer pitch/roll from the DMP quaternion, keeping the legacy pitch reflection. The DMP is dropped in P5. | 3f |
| Fall | 75° on 6 consecutive samples (120 ms) trips it: epoch bump, REFLEX holds, `FELL`; once still, the legacy `rc` (PROGMEM, mirrored by roll sign), up to 3 tries, then STAND and `RECOVERED`, or REST, `RECOVERY_FAILED` and LIMP. Implemented in P1. | 3g |
| Head | `SET_HEAD` holds pan/tilt/tail slew-limited over IDLE/GAIT, without balance corrections; released by mode 0, 2 s without a refresh or an epoch bump. Implemented in P1. | 3c |
| Watchdog | Armed by the first valid frame from the Pi. Two stages: stop motion at 0.5 s, REST at 1.5 s, LIMP 10 s later. | 3g |
| Protocol | COBS + CRC-16/CCITT-FALSE at 250000 baud. Goals unacked and refreshed at 5 Hz; queries answered by their data reply; other commands acked and de-duplicated. | 4 |
| IR remote | **Relayed** (owner's request, 2026-10-11; was dropped). Received on the body: NEC decoded on D4 from a pin-change interrupt shared with the ultrasonic, no library (≈450 B flash, 17 B RAM); relayed as `EVENT IR_KEY`; mapped to actions on the mind (`kilo run --remote`, `kilo.ir`). | 3a, 4, 5 |
| Buzzer | **Kept, Pi-driven.** A `BEEP` note queue plus built-in reflex chirps. ISR-toggled on D5. | 3a, 4 |
| First tricks | Basics: sit, rest, stand, stretch, balance. Social: hi, wash face, check around, sniff, scratch, pee, push-up, butt-up. Athletic tricks (backflip, roll, jump) are not ported. Fall recovery stays a body reflex. | 5 |
| Behaviour | An in-house behaviour tree under a utility selector over drives, ticked at 10 Hz. | 5 |
| Testing | Host g++ tests with sanitizers, the same tests under simavr, a virtual body on a pty, a size/symbol harness, and a HIL checklist. | 6 |

---

## 1. Goals and non-goals

**Goals**

1. A companion with personality. Kilo explores, reacts to people and
   obstacles, rests when tired or low on battery, and expresses mood with eyes,
   head, tail, tricks and sounds. The personality lives on the Pi.
2. A platform for learning control. The body has a fixed-rate loop, an explicit
   owner model, a parametric gait with real IK, and a tunable balance
   controller. Every parameter can be changed live, and the state can be logged
   at 50 Hz.
3. Safety stays on the body. Falls, low battery, joint limits and a lost Pi are
   handled within one tick, with no Pi involvement.
4. Verifiable by construction. Firmware logic is plain C++ that runs on a
   desktop. There is one protocol specification with cross-language test
   vectors. Size and symbols are checked on every build.

**Non-goals**

- Compatibility with Petoi's token protocol, the mobile app, `serialMaster`, or
  the setup sketch.
- Skill storage in EEPROM. Tricks live on the Pi and are sent into RAM on
  demand.
- Behaviour on the body. No random mind, task queue, ultrasonic reactions or
  eye effects.
- Legacy gait tables on the body. The gait generator replaces them.
- Vision on the NyBoard, or anything beyond classical computer vision on the
  Pi 2.
- 12-DoF support. Joints 3 to 7 keep their indices but are never driven.

---

## 2. System overview

```
             +-------------------------- Raspberry Pi 2 B ("MIND", Python) --------------------------+
             |  perception (camera, later mic)   behaviour tree + drives   trick library + authoring |
             |  telemetry log      CLI / daemon socket      flash tool (arduino-cli + avrdude)         |
             |                         kilo.link.SerialLink  (/dev/ttyAMA0, 250000 8N1)              |
             +---------------------------------------|---------------------------------------------+
                  goals (+epoch): posture, gait, head, | body pose, eyes, beeps; trick upload/play; params
                  telemetry 25-50 Hz, events, acks     |                           GPIO4 -> RESET (flashing only)
             +---------------------------------------|---------------------------------------------+
             |                        NyBoard V1_2, ATmega328P @ 16 MHz ("BODY", C++)                 |
             |  UART/COBS/CRC  ->  goal registers  ->  50 Hz tick:                                   |
             |   IMU (DMP) -> reflexes -> owners {REFLEX > TRICK > GAIT > IDLE} -> crossfade          |
             |   -> head layer -> balance layer -> limits + slew -> angle->counts -> PCA9685 (I2C)    |
             |  side work: ultrasonic (pin-change ISR), battery ADC, eyes (WS2812), buzzer, EEPROM     |
             +-----------------------------------------------------------------------------------------+
                 I2C 400 kHz: PCA9685 (0x40), MPU6050 (0x68), [I2C EEPROM 0x54, unused]
```

**What runs where**

| Function | Where | Why |
|---|---|---|
| Servo PWM | PCA9685 | Autonomous. It holds the last pose while the AVR resets. |
| 50 Hz loop, interpolation, arbitration, gait + IK, balance | Body | Needs deterministic timing and a few ms from IMU to servo. Linux on a Pi 2 gives neither. |
| Reflexes (fall, battery, limits, link watchdog) | Body | Must work while the Pi hangs, reboots or is absent. |
| Ultrasonic ranging | Body | Needs µs echo timing. Only the range goes to the Pi. |
| Eyes, buzzer | Body executes, Pi decides | Simple actuators. The body adds its own reflex chirps. |
| Tricks, perception, personality, logging, flashing, calibration UI | Mind | Needs storage, Python and CPU. |

**Latency**

| Path | Latency |
|---|---|
| IMU sample → PCA9685 write | ≈4 ms within the tick, plus ≤10 ms DMP sample age **(est.)** |
| Pi goal → applied | ≤2.6 ms on the wire + ≤20 ms tick phase |
| Body state → Python callback | ≤20 ms + 1.3 ms wire + 1-10 ms Linux tty **(est.)** |
| Camera percept → head motion | 150-350 ms **(est.)**. Servo mechanics (≈100-150 ms per 60°) usually dominate. |

Rule: anything that must react within about 100 ms runs on the body.

---

## 3. Body firmware design

The sketch is `body/body.ino`. Logic lives in `body/src/core/` (plain C++) and
hardware access in `body/src/hal/`. The full tree is in Appendix G. Four rules
apply to `core/`:

1. **Fixed-width types only.** On AVR `int` is 16 bits; on the host it is 32.
   A grep-lint rejects `int`, `unsigned` and `long`, and products are widened
   explicitly with `(int32_t)a * b`.
2. **No Arduino headers, floats, heap or virtual functions.** Time and sensor
   values come in as arguments.
3. **C++11** (`gnu++11` under avr-gcc 7.3).
4. **Every core module has host tests** (§6).

**Why no standard libraries.** Flash is not the constraint: the whole design is
≈15 KB of 32 KB. The reasons are these:

- HardwareSerial's 64 B RX buffer is smaller than one maximum 66 B frame.
- Wire's 32 B transaction limit blocks the single 66 B PCA9685 burst.
- Together they cost ≈330 B of RAM (Serial 157 B, Wire buffers ≈165 B), which
  is 16% of the 2 KB.
- Wire's `endTransmission` and `pulseIn` block with no bound we control.

The replacements are small and bounded: a UART driver with ISR rings, a polled
TWI master (2 ms timeout, bus recovery), a PCA9685 burst writer, and a DMP
driver. The DMP upload needs only single-register writes, 16 B block writes and
16 B read-backs; uploading the 3062 B image with verification is the I2C
driver's acceptance test.

Integer maths is the default. Float is a sanctioned fallback for IK only: about
1.6 KB of flash and 2-3 ms per tick **(est.)**, if the fixed-point IK proves
too costly to get right.

### 3a. Timing: the 50 Hz tick

The loop polls a fixed deadline instead of using a timer interrupt. Almost all
tick work is I2C, which cannot run in an ISR, so a timer ISR would only set a
flag.

```
loop():
  uart_poll()                       // decode complete frames from the RX ring, apply to goal registers
  if ((int32_t)(micros() - next_tick) >= 0):
      lag = micros() - next_tick
      next_tick += 20000            // fixed grid, no drift
      if lag > 40000: next_tick = micros() + 20000; stats.overruns++   // resync, never "catch up"
      run_tick()
  else:
      background()                  // EEPROM writer, eye update, event/TX queues, IMU re-init chunk (LIMP only)
```

| # | Tick step | Time **(est.)** |
|---|---|---|
| 1 | IMU: read FIFO count, read `floor(count/28)·28` bytes, keep the last packet | ≈1.5 ms at 100 Hz DMP rate (Appendix B) |
| 2 | Quaternion → pitch/roll/rates (integer) | 0.15 ms |
| 3 | `uart_poll()` | <0.1 ms |
| 4 | Reflexes, owner selection, epoch bookkeeping | 0.05 ms |
| 5 | Owner outputs (gait + IK for 4 legs / trick / posture) | 0.8 ms gait, 0.1 ms otherwise |
| 6 | Crossfade, head and balance layers, limits, slew, angle → counts | 0.3 ms |
| 7 | `uart_poll()` | <0.1 ms |
| 8 | PCA9685 burst write of all 16 channels | 1.6 ms |
| 9 | Ultrasonic trigger if due (20 µs), battery ADC if due (104 µs) | 0.13 ms |
| 10 | Telemetry into the TX ring, `wdt_reset()`, tick-alive canary, timing stats | 0.15 ms |
| | **Total** | **≈4.5-5 ms of 20 ms** |

The measured value is reported as `tick_load` in TELEMETRY and `max_tick_us`
in STATS. Acceptance: under 12 ms over a 10-minute trot.

**Non-blocking rules**

- No `delay()` and no `pulseIn()`. Busy-waits are limited to 50 µs, except
  inside I2C transactions, which are bounded at 2 ms.
- No section runs longer than ≈2 ms without calling `uart_poll()`. At 250000
  baud a byte arrives every 40 µs, and the 192 B RX ring gives 7.7 ms of slack.
- EEPROM writes go through a background writer: one byte when `EEPE` is clear,
  ≈3.4 ms per byte.
- Eye updates disable interrupts for 180 µs, which can drop 2-3 UART bytes.
  They happen only on change, only in `background()`, and never while an
  ultrasonic echo is in flight. A damaged frame fails its CRC and is recovered
  by goal refresh or command retry (§4).
- The buzzer is a Timer2 compare ISR toggling D5 in software at 2× the note
  frequency: ≈1% CPU at 2 kHz and ≈2-3 µs of jitter on ultrasonic timestamps
  and UART RX. D5, D6 (ultrasonic) and D7 (WS2812) share PORTD, so PORTD access
  must be atomic: the ISR toggles with `PIND = _BV(5)`, D6/D7 change only via
  single-instruction `sbi`/`cbi` or inside the interrupts-off WS2812 routine,
  and read-modify-write of PORTD with interrupts enabled is forbidden.
- The IMU is never re-initialised inside a tick (see the boot sequence below).

**Ultrasonic.** Every 4th tick the body pulses D6 high for 20 µs and switches it
to input with `PCINT22` enabled. The ISR timestamps both echo edges with
`micros()`. No falling edge within 25 ms means "no echo" (`dist_mm = 0`).
Appendix B has the step list and the error budget.

**IR remote** (added 2026-10-11 at the owner's request: "receive on the body
and just relay to the mind"). The NyBoard's on-board IR receiver is on D4 =
PD4 = `PCINT20`, in the same pin-change group as the ultrasonic's D6, so the
one `PCINT2_vect` (`hal/pcint2.cpp`) serves both: it reads `PIND` once, hands
the level of D6 to the ultrasonic's state machine (which only acts on the
edge it waits for) and detects D4's falling edges against the previous
`PIND`. `PCINT20` stays enabled from boot; the ultrasonic sets and clears only
its own `PCMSK2` bit and no longer clears `PCIFR` (that could drop an IR
edge). D4 is a plain input (the receiver drives the line). The receiver's
output idles high and goes low during a mark, so the NEC decoder
(`core/ir_nec`, host-tested) works on the intervals between falling edges:
leader 13.5 ms, bit 0 1.125 ms, bit 1 2.25 ms, repeat 11.25 ms, each with a
tolerant window; an out-of-range interval or a 20 ms gap resets it. It runs
inside the ISR (a few compares and a shift per edge, ≈20 µs), checks
address/~address and command/~command, and leaves the key for
`background()`, which pushes `EVENT IR_KEY` (arg = command, `| 0x100` for a
held-key repeat, sent only within 150 ms of a valid frame or repeat). The
legacy IRremote library (Timer2 at 20 kHz, ≈2 KB) is not used.
`-DKILO_IR=0` drops it; HELLO caps bit 1 says it is built in.

**Boot sequence**

1. Read the `.noinit` reset record (below) and initialise the UART.
2. Initialise I2C and probe the PCA9685. Already configured (awake, expected
   prescaler) = warm start: read back the 16 `LEDn_OFF` registers to seed the
   pose and keep outputting it. Otherwise = cold start: configure it all
   full-off (LIMP), because the servo pose is unknown.
3. Run `dmpInitialize` **before the hardware watchdog is enabled**. It takes
   ≈300 ms (192 chunks of 16 B, each with read-back verify), with up to 3
   attempts. If all fail, set `IMU_FAULT` and continue without balance or fall
   detection.
4. Load the config slot (3b), send `HELLO` and `EVENT BOOT`, then
   `wdt_enable(WDTO_250MS)`.
5. Enter the loop. A warm start holds the read-back pose until the IMU is ready
   or 1 s has passed, then moves to REST under the watchdog policy (3g). A cold
   start stays LIMP until the Pi sends a goal.

At runtime the IMU is re-initialised only while LIMP. It runs as a state
machine in `background()` that uploads one 16 B chunk per pass (≈192 passes,
about 4 s) and kicks the watchdog in between.

**Reset cause.** The stock Uno optiboot clears `MCUSR` before the application
runs, so hardware reset flags cannot be read.¹ Instead, a `.noinit` record (not
zeroed by the C runtime) holds `magic` u16, `build` u32 (hash of the running
firmware), `reason` u8 (set to `REQUESTED` just before a `REBOOT`) and
`alive_tick` u16 (the tick-alive canary, written every tick). At boot:

| Condition | Reported `reset_cause` |
|---|---|
| `magic` invalid | `COLD`: power-on, or a brown-out that lost RAM |
| `magic` ok, `build` ≠ this build | `FLASHED` |
| `reason == REQUESTED` | `REQUESTED` (a `REBOOT` message) |
| otherwise | `WARM_UNEXPECTED`: watchdog hang, reset button, a GPIO4 pulse not used for flashing, or a RAM-preserving brown-out. `HELLO` carries `last_alive_tick`, and the Pi knows whether it pulsed GPIO4. |

The record is then rewritten for the current boot.

¹ Optiboot v8 passes `MCUSR` to the application in `r2`, but installing it
needs an ISP programmer. That is not planned.

### 3b. Joint model

| Idx | Joint | PCA ch | dir | midShift ° | limits ° |
|---|---|---|---|---|---|
| 0 | head pan | 12 | +1 | 0 | -120..120 |
| 1 | head tilt | 11 | −1 | 15 | -80..45 |
| 2 | tail | 4 | +1 | 0 | -120..120 |
| 3-7 | unused (12-DoF slots) | 3,13,10,5,2 | | | never driven |
| 8 | LF shoulder | 14 | +1 | 10 | -100..80 |
| 9 | RF shoulder | 9 | −1 | 10 | -100..80 |
| 10 | RB shoulder | 6 | −1 | −10 | -80..100 |
| 11 | LB shoulder | 1 | +1 | −10 | -80..100 |
| 12 | LF knee | 15 | −1 | −30 | -60..120 |
| 13 | RF knee | 8 | +1 | −30 | -60..120 |
| 14 | RB knee | 7 | +1 | 30 | -120..60 |
| 15 | LB knee | 0 | −1 | 30 | -120..60 |

The values come from `src/OpenCat.h`, except the tail `dir`: current
`src/OpenCat.h` says −1 because upstream commit b152455 (N_240907, Bittle
robot arm) flipped the shared `rotationDirection[2]`; Nybble firmware up to
N_240702, and Kilo's own EEPROM, say +1, and the legacy firmware drives with
the EEPROM value. The leg order LF, RF, RB, LB is not
written down in the code; it is inferred from it (Appendix C).

- The canonical wire order for the 11 driven joints is
  `[0,1,2,8,9,10,11,12,13,14,15]`.
- Channel, `dir`, `midShift` and limits are PROGMEM constants (robot model).
  Per-robot data lives in EEPROM.
- Unused channels are written full-off. The used channels span 0..15, so a
  single 16-channel burst is cheapest.

**Angle → PWM (per-robot legacy transfer function).** Each robot's legacy
EEPROM holds the values the setup sketch computed for it:
`CALIBRATED_ZERO_POSITIONS` (CZP) @96, `ANGLE2PULSE_FACTOR` (A2P) @160,
`B_OFFSET` @274, `PCA9685_FREQ` @275, `PWM_PIN` @0, `CALIB` @16, `MID_SHIFT` @32
and `ROTATION_DIRECTION` @48 (types in Appendix C4). The new firmware imports
these values instead of hard-coding them.

The legacy path computes `counts = A2P/1000 · (CZP + dir·angle) + B_OFFSET·10`,
clipped to 500..2500. The new firmware evaluates the same function in integers:

```
servo_a16 = zero_a16[j] + dir[j]·angle_a16                 // zero_a16 = CZP·16, imported per joint
counts    = (a2p[j]·(int32_t)servo_a16 + 8000) / 16000 + b_offset_counts
counts    = clamp(counts, 500, 2500)
```

The PCA prescaler is set exactly as legacy `setPWMFreq(240)` sets it.
`PCA9685_FREQ` is stored for future µs conversions; it does not affect this
path. Calibration is exposed on the protocol as the legacy-style offset
`calib = dir·(zero − 90 − dir·midShift)` in `a16`, and `CAL_SET` rewrites
`zero_a16`.

**Migration** runs when neither config slot is valid. It checks that
`PWM_PIN`, `MID_SHIFT` and `ROTATION_DIRECTION` equal the PROGMEM constants,
that `calib_j = dir·(CZP_j − 90 − dir·midShift_j)` matches `CALIB_j` within 1°,
and that A2P ∈ 9000..13000 and `B_OFFSET·10` ∈ 400..800. If all pass, it writes
slot A and emits `CONFIG_MIGRATED`. Otherwise it uses model defaults (A2P 11000,
B 600, CZP = 90 + dir·midShift), emits `CONFIG_DEFAULTED(reason)`, and refuses
`STAND` (`GOAL_REFUSED` reason `UNCALIBRATED`) until the Pi has run calibration
and a `CAL_SAVE` has completed. Appendix C explains where 600 and 11 come from, and why
neutral is ≈1590 counts rather than 1500.

**P0 task:** dump EEPROM 0..282 from Kilo once with `EEPROM_READ` before the
first flash of new firmware, and commit it as `docs/kilo-eeprom-legacy.md`
(hex + decoded).

**EEPROM layout (new).** The legacy layout uses 0..282, 320..700 and
≈966..1023, and the new firmware never writes those. Flashing the legacy
firmware back therefore finds its EEPROM intact. The new firmware uses two
128 B slots: A at 704..831 and B at 832..959.

| Slot offset | Size | Field |
|---|---|---|
| 0 | 2 | magic `'K','L'` |
| 2 | 1 | layout version |
| 3 | 1 | sequence number (serial-number arithmetic) |
| 4 | 32 | `zero_a16[16]` |
| 36 | 32 | `a2p[16]` |
| 68 | 2 | `b_offset_counts` |
| 70 | 2 | `pca9685_freq_khz` |
| 72 | 12 | IMU offsets (accel xyz, gyro xyz, MPU offset-register values) |
| 84 | 2 | battery scale, mV per 64 ADC counts |
| 86 | 40 | 20 persisted params (int16, 0x7FFF = default). Layout version 2 (proto 1.4, P2); version 1 had 16 params and 8 reserved bytes, is still accepted (params 16..19 read as default) and is rewritten as version 2 on the next save. |
| 126 | 2 | CRC-16/CCITT-FALSE over 0..125 |

**Writing a slot (`CAL_SAVE`, `PARAM_SET persist`):**

1. Copy the RAM image, with sequence + 1, into the **older** slot.
2. Write only the bytes that differ from what is already in that slot.
3. Write the CRC last.

A torn write leaves that slot invalid, and boot falls back to the other valid
slot, never to stale legacy data. If both slots are invalid, migration runs.

### 3c. Arbitration

| Priority | Owner | Mask | Active when |
|---|---|---|---|
| 3 | REFLEX | all driven | Fall response and recovery, low-battery REST, watchdog stop and REST, warm-boot hold |
| 2 | TRICK | from `TRICK_BEGIN` (default all 11) | A trick is playing |
| 1 | GAIT | legs 8-15 | The gait is running or stopping |
| 0 | IDLE | all driven | Always. Holds the current posture: LIMP, REST, SIT, STAND, STANCE (IK), CALIB or ZERO. |

Per tick and per joint, the highest-priority active owner whose mask contains
the joint supplies the base angle. When the owner of a joint changes, the
crossfade starts from a snapshot of that joint's previous output:

```
if owner(j) changed: src[j] = last_out[j]; n[j] = xfade(prev, new); i[j] = 0
if i[j] < n[j]:     base[j] = src[j] + (base[j] − src[j]) · ease_cos(++i[j] / n[j])
```

The snapshot is the last **output** (after balance, limits and slew; as
implemented in P1, `core/arbiter.cpp`), so a hand-over never jumps even when
the previous owner asked for an angle beyond a limit or carried a balance
correction. The same snapshot is the TRICK owner's start pose (frame 0 moves
from it).

Crossfade lengths:
- 0 ticks into TRICK or REFLEX, because their first frame interpolates from the
  current pose;
- 10 ticks (`XFADE_TICKS`) out of REFLEX, and out of TRICK when it ends or
  is preempted; `TRICK_STOP transition_ticks` when the Pi stops it;
- the posture's transition time between IDLE postures (default 25 ticks).

Layers on top of the base pose:

1. **Head layer** (joints 0-2, only when the head owner is IDLE or GAIT):
   - If the Pi holds a target (`SET_HEAD mode=1`), the head follows it,
     slew-limited.
   - Otherwise the head uses the posture's head plus the gait's head bob and
     tail wag.
   - The hold is released by `mode=0`, by `HEAD_HOLD_MS` (2 s) without a
     refresh, or by any epoch bump.
   - As implemented in P1 (`core/arbiter.cpp`, `body_goal_head`): a held
     joint steps from its last output toward the target by at most
     `max_speed_dps` (as a16 per tick, rounded up; 0 or > 2500 °/s = only
     the safety slew), and the safety stage then clamps the target to the
     joint limits. A target of −32768 leaves that joint to its owner (the
     mind's `look()` sends it for the tail unless asked). Held joints take
     **no balance correction**: the Pi is pointing them (a camera), and a
     correction would fight it; unheld head joints keep theirs. A held joint
     is recorded as owner `PI_HOLD` (TELEMETRY head nibble 4, flags bit 5
     `HEAD_HELD`), so taking and releasing the hold are hand-overs: the
     release crossfades back to the owner's angle over `XFADE_TICKS`.
     Precedence: a trick that owns a head joint wins over the hold (and the
     hold resumes from the trick's output when it ends, if still refreshed);
     a trick that does not own the head leaves it to the hold; REFLEX wins
     over everything and its epoch bump drops the hold. `SET_HEAD`: BAD_ARG
     for mode > 1; mode 0 is never stale; mode 1 is refused STALE, FALLEN,
     LOW_BATT in that order. The hold lasts `HEAD_HOLD_MS / 20` = 100 ticks
     from the last refresh (the 5 Hz goal refresh keeps it).
2. **Balance layer** (3f, `core/balance.*`):
   - For GAIT and STANCE, corrections enter the IK as body pitch/roll (P2).
   - For joint-space owners (built-in postures, tricks with `BALANCE`), they
     pass through the per-joint gain table and are added to the joint's base
     angle after the crossfade, before the safety stage.
   - Requested by `SET_POSTURE` flags bit 0 (cleared by an epoch bump, so
     the mind re-asserts it) or by `SET_BALANCE enable=1`.
   - As implemented in P1 (`body_tick`): runs only for the IDLE owner in SIT
     or STAND, once the posture move has settled. Off for REFLEX (fall,
     battery, watchdog, warm-boot hold), LIMP, CALIB, ZERO and REST (lying
     on the belly there is nothing to balance) and during an IDLE posture
     move: there the corrections are zeroed at once, which never jumps
     because the reflex or the new mover starts from the corrected output.
     A request turned off or `IMU_FAULT` lets them decay through the
     low-pass and the 1°/tick slew instead.
   - As implemented for tricks (P1): while a trick plays, the controller
     runs only during frames with the balance bit, against the trick's
     expected attitude from `TRICK_BEGIN` (`exp_roll`, `exp_pitch`; roll
     negated when mirrored), whatever the IDLE request; between balanced
     frames the corrections decay through the low-pass and slew. They are
     zeroed when a trick starts and when it ends: the trick's first frame
     and the crossfade out both start from the corrected output. The
     corrections are added to TRICK-owned joints and to the IDLE-owned rest.
   - TELEMETRY flags bit 0 (`BALANCE_ACTIVE`) reports that it ran this tick.
3. **Safety stage** (always applied):
   - Clamp to joint limits.
   - Slew-limit to `SLEW_MAX` (10°/tick ≈ 500°/s, about G41 no-load speed
     **(est.)**).
   - Convert angle → counts.

**Goals and the goal epoch.** The Pi never sends joint angles except as trick
keyframes. Goals are latched registers that the Pi overwrites:
- `SET_POSTURE`, `SET_GAIT`, `SET_HEAD`, `SET_BODY_POSE`;
- `SET_BALANCE`, `SET_EYES`, `BEEP`;
- `TRICK_PLAY`.

Without an epoch, a goal re-sent at 5 Hz from before a fall would restart a
gait after recovery. So:

- The body keeps `epoch` (u8, wraps), increments it on every REFLEX takeover
  (fall, low battery) and watchdog trip, and reports it in every TELEMETRY and
  EVENT. Each bump also clears the latched motion goals (gait stops, head
  released, pose zeroed, trick stopped).
- `SET_POSTURE`, `SET_GAIT`, `SET_HEAD`, `SET_BODY_POSE` and `TRICK_PLAY` carry
  `epoch` as their first byte. A mismatch is refused with
  `EVENT GOAL_REFUSED(type, STALE)` (`ACK STALE` for `TRICK_PLAY`), except
  `SET_POSTURE` LIMP or REST, which is never stale.
- An EVENT carries the epoch after any bump it reports.
- `BALANCE` is not a posture: it is `SET_POSTURE STAND` with flags bit 0.
- Each goal carries the epoch that was current on the mind when the goal was
  set; it is never re-stamped. Any epoch change the mind sees, in TELEMETRY,
  in any EVENT or in HELLO, clears its motion goals from older epochs; it
  then adopts the new epoch and decides afresh (§5).
- **Boot epoch: a reboot never restarts at the previous epoch.** A body
  that reboots (brown-out, watchdog reset, the DTR edge of a port reopen)
  would otherwise accept the goals the mind refreshes during the 0.3-2 s
  init (the UART buffers them) as if nothing happened. The reset record in
  `.noinit` RAM (`core/reset.h`, which survives every reset but not power
  loss) carries the epoch: the tick writes `b.epoch` into it next to the
  alive canary, and a boot starts at that value + 1, skipping 0
  (`reset_boot_epoch`: 255 wraps to 1). So the epoch counter keeps running
  across warm resets (requested, flashed, watchdog, reset pin): consecutive
  boots never repeat an epoch, and every goal stamped before the reset,
  including after a last-moment bump, is STALE. A cold boot has no valid
  record; its byte is whatever power-up RAM holds, + 1 (random-ish). HELLO,
  EVENT BOOT and the first TELEMETRY report the boot epoch, and the mind
  adopts it (§5). The mind's side (§5 "Contact": no motion goal after a
  reconnect or once telemetry has been missing for 300 ms, until the body is
  heard again; a HELLO with a new identity is a reboot) remains as the
  second layer. The virtual body does the same (`boot_epoch` in
  `kilo.vbody.sim`; its first power-on is epoch 0, as the link harness).

### 3d. Gait generator

**Parameters** (`SET_GAIT`; slew-limited per tick). Defaults are **(est.)**,
to be tuned.

| Param | Unit | Range | Walk / trot / crawl |
|---|---|---|---|
| `pattern` | enum | WALK, TROT, CRAWL | |
| `period_ms` | ms | 300..2000 | 800 / 500 / 1200 |
| `duty` | /256 | 128..230 | 192 / 140 / 218 |
| `stride_mm` | mm, signed | −60..60 | 30 / 35 / 25 |
| `turn_mm` | mm, signed, + = left | −40..40 | 0 |
| `step_h_mm` | mm | 0..40 | 15 / 12 / 18 |
| `body_h_mm` | mm | IK workspace | 0.80 / 0.80 / 0.70 × (L1+L2) |
| `flags` | bits | head bob, tail wag, balance | all on |

`SET_BODY_POSE` (pitch, roll, height delta, x-shift) applies on top. It also
drives IDLE STANCE.

**Phase.**
- The global phase `φ` is a `uint16_t` (65536 per cycle). Each tick adds
  `65536·20/period_ms`, so changing the period never makes the phase jump.
- Leg phase `φ_i = φ + offset_i` and wraps naturally.
- Leg `i` starts its swing when `φ_i` crosses `duty`, so a larger offset swings
  earlier.

| Pattern | LF | RF | RB | LB | Swing order |
|---|---|---|---|---|---|
| WALK | 0.50 | 0.00 | 0.25 | 0.75 | LB → LF → RB → RF (lateral sequence) |
| CRAWL | as WALK | | | | higher duty, lower body |
| TROT | 0.00 | 0.50 | 0.00 | 0.50 | diagonal pairs |

The host test asserts the resulting swing order and the pairings, not the
table values.

**Foot trajectory.** The leg frame has its origin at the shoulder axis, x
forward and z down. With `p = φ_i/65536`, `β = duty`, neutral foot `x0_i`,
height `H_i` and stride `S_i`:

- **Stance** (`p < β`): `s = p/β`, `x = x0 + S_i(½ − s)`, `z = H_i`. Ground
  speed is `S_i/(βT)`, for example 127 mm/s for the default trot **(est.)**.
- **Swing** (`p ≥ β`): `s = (p−β)/(1−β)`, `x = x0 + S_i(−½ + (1 − cos πs)/2)`,
  `z = H_i − h·sin πs`. The foot has zero velocity at lift-off and touchdown.
- **Turning:** `S_left = S − turn`, `S_right = S + turn`. With `S = 0` Kilo
  turns in place.

**Pose terms.** Signs follow the measured attitude convention (3f, "Sign
conventions"): a positive pitch lowers the nose (raises the back) and a
positive roll raises the right side. In both cases the legs on the raised end
or side get longer (larger `H`, since z points down).

```
H_i = body_h + end_i·(Lb/2)·sin(pitch_total) + side_i·(W/2)·sin(roll_total)
  end_i = −1 front, +1 back;  side_i = −1 left, +1 right
  pitch_total = pose_pitch + corr_pitch;  roll_total = pose_roll + corr_roll   (corr from 3f)
  x0_i += pose_xshift
```

**Head bob and tail wag.** Tilt gets `A_bob·sin(2·2πφ)` (default 4°), which
counteracts the twice-per-cycle body bob. Tail gets `A_wag·sin 2πφ` (default
15°).

**Start and stop.**
- Start at `φ = 0` with the stride ramping from 0 over one period.
- Stop by ramping the stride to 0 over one period, waiting until all feet are
  in stance, then handing over to IDLE STANCE at the same height.
- A pattern change is a stop followed by a start.

**Planar IK.**
- `L1` is the shoulder axis to the knee axis; `L2` is the knee axis to the foot
  contact point.
- `θ1` is the upper link angle from straight down, positive = foot forward.
- `θ2` is the knee bend relative to the upper link.
- `σ_i = ±1` chooses which way the knee points.

```
FK:  x = L1 sin θ1 + L2 sin(θ1+θ2),   z = L1 cos θ1 + L2 cos(θ1+θ2)
IK:  d² = x² + z²   (clamp d to [d_min, 0.95(L1+L2)])
     c  = (d² − L1² − L2²) / (2 L1 L2),   s = σ_i √(1 − c²),   θ2 = atan2(s, c)
     θ1 = atan2(x, z) − atan2(L2·s, L1 + L2·c)
     shoulder_j = sgn_sh_i (θ1 − θ1_0),   knee_j = sgn_kn_i (θ2 − θ2_0_i)
```

- `θ1_0` and `θ2_0` describe the calibration posture. Petoi's guide says it is
  upper leg vertical and lower leg at 90° to it; this must be verified on Kilo.
- The signs `sgn_sh`, `sgn_kn` and `σ` are fixed in P2. A host test checks that
  FK of the legacy `up`, `sit` and `rest` postures puts the feet below the body
  and symmetric left/right. A HIL test nudges one foot +10 mm forward.
- `L1`, `L2`, `Lb`, `W` and the foot offset must be measured (open question 1).
  Appendix B describes how and gives rough guesses.

**Fixed point (summary; derivation in Appendix B).**
- Lengths are `q2`. Squares are int32.
- `c` and `s` are Q12, with one int32 division.
- `atan2` normalises its arguments by a shared shift, then uses a 65-entry
  table. Sine uses a 65-entry Q14 quarter-wave table. `isqrt32` is bitwise.
- Targets: |FK(IK(p)) − p| ≤ 0.25 mm and angle error ≤ 0.25° against a double
  reference.
- Cost **(est.)**: 0.8 ms per tick and ≈2 KB of flash.

### 3e. Keyframe interpolator for tricks

Implemented in P1 as `body/src/core/keyframe.{h,cpp}` (store, upload,
player), wired into `core/body.cpp` (gating, end, balance) and
`core/arbiter.cpp` (the TRICK owner). Fixed point only.

**Frame (14 B):**

| Byte | Field | Meaning |
|---|---|---|
| 0 | `dur` u8 | Ticks from the previous frame (1..255 = 20 ms..5.1 s); 0 is refused |
| 1 | `hold` u8 | Ticks to hold after arriving |
| 2 | `flags` u8 | Bits 0-1 easing: 0 linear, 1 cosine (default), 2 step, 3 reserved (spline, refused). Bit 2: balance on in this frame. |
| 3..13 | `ang[11]` i8 | Degrees, in canonical joint order. **−128 = unchanged.** |

"Unchanged" is resolved as each frame arrives: it becomes the previous
frame's angle (by index). A −128 that survives (never set since frame 0)
means the joint's pose when the trick started. (Inside a loop that differs
from "unchanged" only when `loop_start` leaves a joint unset that the loop
sets; the importer never does.)

**Header (`TRICK_BEGIN`, 11 B):**
- `trick_id`: a Pi tag used for caching (the mind derives it from the
  header and frames, 1..254).
- `n_frames` 1..24 (more is `BAD_ARG`: the store holds `trick_capacity` =
  24 frames since the P1 RAM shrink, 3h; HELLO reports it).
- `loop_start` ≤ `loop_end` < `n_frames`, `repeats`: total plays of
  `loop_start..loop_end`; 0 and 1 = once, 255 = until stopped (legacy
  `loopCycle[2]`, §5).
- `joint_mask` (u16 over the joint ids): joints the trick owns, only the 11
  driven ones and at least one. Unowned joints stay with GAIT/IDLE and the
  head layer.
- `flags`: `BALANCE` (informational: balance is per frame, bit 2), and
  `FALL_OK`, which suppresses fall detection while the trick plays
  (`body_fall_suppressed()`, for the fall reflex).
- `end_posture`: REST, SIT, STAND, CALIB or ZERO (a posture with angles).
- `exp_roll`, `exp_pitch` i8 degrees: the balance reference for balanced
  frames (legacy header bytes; `sit` −30, `str` +15).

**RAM:** 24 × 14 = 336 B for the store (448 B with 32 frames before the
P1 shrink), plus 24 B of header, upload and
player state. There is no `from[11]` array: a segment's start is the
previous frame's (resolved) target, read from the store, and frame 0's is
the arbiter's crossfade snapshot (the output when TRICK took the joint). The
largest Nybble behaviour (`cmh`) has 22 frames, so 24 still holds every
legacy behaviour; the first-version library's largest has 6.

**Timing.**
- Each frame is a MOVE of `dur` ticks, then a HOLD of `hold` ticks at its
  targets. Frame 0 starts from the current output; each later frame from
  the previous frame's targets; the first frame of a repeat from
  `loop_end`'s targets.
- Each tick: `q = from + (to − from)·e(pos/len)` in Q14, `e` per the frame:
  linear `pos/len`; cosine `(1 − cos πs)/2 = sin²(πs/2)` (`fx_ease_cos`,
  the sine table; matches today's `transform()`); step `1` from the first
  tick (the slew limit paces the jump, then it waits). The last tick of
  every MOVE is exactly the target, so consecutive moves meet without a
  jump, and every move is monotone.
- `TRICK_PLAY` takes a speed (Q6, 64 = 1×, 16..255) and a mirror flag; the
  body applies both, so one upload serves every variant. Speed advances
  `pos` by `speed/64` tick per tick: a phase of `n` ticks takes
  `ceil(n·64/speed)` ticks. Mirror swaps the L/R pairs (LF↔RF, RB↔LB,
  shoulders and knees), negates pan and tail, mirrors the joint mask and
  negates the expected roll.
- The trick ends on the tick after its last phase: `EVENT TRICK_DONE`
  (`arg = reason << 8 | trick_id`, reason 0 DONE), and the joints crossfade
  over `XFADE_TICKS` (200 ms) into `end_posture`, which IDLE adopts (joints
  the trick did not own move there over the same time). The mind computes
  the same timing (`mind/kilo/tricks/player.py`).

**Upload** (all acked, de-duplicated like any command):
- `TRICK_BEGIN` empties the store and takes the header.
- `TRICK_FRAMES`, up to 4 frames, strictly in order: `first` must equal the
  number received since `TRICK_BEGIN` (`BAD_ARG` for a gap, a repeat or an
  overrun past `n_frames`; a chunk with a `dur` 0 or easing 3 frame is
  refused whole). Without a header: `NO_TRICK`. The body folds each frame,
  as sent, into a running CRC-16.
- `TRICK_COMMIT` with the CRC-16 of the frame array: `NO_TRICK` (no header,
  other id), `BAD_ARG` (frames missing), `CRC_MISMATCH`, else the store is
  ready. It is idempotent, and that is the cache check: the mind sends
  `TRICK_COMMIT` alone first and uploads only if the answer is not OK (each
  `kilo` command is a new process, so a mind-side cache alone is not enough).
- `TRICK_BEGIN` and `TRICK_FRAMES` while a trick plays are refused with
  `BUSY`.
- A 22-frame trick needs 6 chunks plus a commit, ≈100 ms **(est.)**.

**Play, stop and gating.**
- `TRICK_PLAY` (`E`): `STALE` on an epoch mismatch; `NO_TRICK` if that
  `trick_id` is not committed; `BAD_ARG` for speed < 16 or unknown flags;
  `BUSY` while a trick plays, while a reflex holds the joints (watchdog REST,
  low battery, warm-boot hold) or when the IDLE posture is LIMP or CALIB;
  `REFUSED_LOW_BATT`; `UNCALIBRATED` (config DEFAULTED).
- `TRICK_STOP transition_ticks`: crossfade to the end posture over that
  many ticks (0 = `XFADE_TICKS`), `TRICK_DONE(STOPPED)`. OK also when
  nothing plays. `SET_POSTURE LIMP` stops it at once (STOPPED); other
  postures only move the joints the trick does not own, and the trick's end
  posture replaces them when it ends.
- A reflex taking the joints, or any epoch bump (watchdog stage 1, low
  battery), ends it with `TRICK_DONE(PREEMPTED)` carrying the new epoch; the
  fall reflex calls `body_epoch_bump(b, KF_END_FELL)` for
  `TRICK_DONE(FELL)` (implemented in P1, 3g), unless the trick has
  `FALL_OK`, which keeps fall detection off while it plays.
- TELEMETRY while it plays: mode TRICK (4), flags bit 4, `trick_frame` = the
  frame playing, owner nibbles TRICK for owned joints.

**Built-in sequences** (PROGMEM, ≈100 B): postures `rest`, `sit`, `up`
(stand), `calib`/`zero`, and the recovery trick `rc`. As implemented in P1,
`rc` is three PROGMEM rows in `core/posture.cpp` (39 B) that the REFLEX
mover plays as internal pseudo-postures (`POSTURE_RC + i`, `|
POSTURE_RC_MIRROR`), not through the keyframe store, so a fall never
disturbs the uploaded trick (3g).

### 3f. Balance

The legacy approach is in Appendix C. In short, it is a joint-space,
proportional-only correction with a 5° per-update slew and dead bands. It is
the starting point for the joint-space gain table.

**Controller** (per axis, 50 Hz; the DMP produces ≈100 Hz and the newest
packet is used). Implemented in P1 as `body/src/core/balance.{h,cpp}`, fixed
point throughout:

```
e     = deadband(measured − ref, DB)            DB default 2°
u     = Kp·e + Kd·rate                          rate = DMP gyro on that axis (a16/s)
corr  = −u                                      negative feedback: nose too high (e < 0) → corr > 0 lowers it
corr  = clamp(slew(lowpass(corr, α=1/4), 1°/tick), ±15°)
```

- `ref` is 0 for GAIT and STANCE, and the posture's or trick's expected
  pitch/roll otherwise (table below; `sit` expects −30°, nose up).
- Defaults **(est.)**: Kp = 0.5 (`kp_q8` 128), Kd = 0.03 s (`kd_ms` 30),
  DB 2°, clamp 15°. Kp is Q8, so 255/256 is the largest; Kd·rate is computed
  as `rate·kd_ms·131 >> 17`.
- `−u` is saturated at ±clamp before the low-pass, so the filter state never
  holds more than the clamp lets out. The low-pass keeps 4 extra fraction
  bits and settles exactly; a decaying correction reaches exactly 0.
- There is no integral term: a sustained slope should change the pose, and an
  integrator would wind up.
- `corr_pitch` and `corr_roll` are in the attitude's own frame and enter the
  pose terms in 3d, where a positive value lowers the nose (raises the
  measured pitch) or raises the right side (raises the measured roll). If
  Kilo is nose-high (pitch below ref, `e < 0`), `corr` is positive and the
  front legs shorten.
- Where it runs, and when it is off, is in 3c (layer 2).
- Tuning is live through `SET_BALANCE` (enable, Kp, Kd, dead band, clamp;
  not persisted; dead band and clamp ≤ 30°, else `GOAL_REFUSED BAD_ARG`).
  `kilo balance on|off [--kp X] [--kd S] [--deadband D] [--clamp C]` sends it
  and holds the posture. There are no balance `PARAM_SET` ids yet (3 param
  slots are free if persisted defaults are wanted). Telemetry streams pitch,
  roll and rates at 25 Hz by default (`TELEM_DIV`) for step responses.

**Proportional-only limits H13.** With a 1:1 leg-to-body response the
steady error after a tilt d is (d + Kp·DB)/(1 + Kp): 7.3° for d = 10° at
Kp 0.5, and 5.5° at the largest `kp_q8` (255/256). H13 asks for 3°. If the
hardware confirms that, either the leg response per degree of `corr` is
larger than 1 (the legacy gains were never calibrated to it), or `kp_q8`
needs a coarser scale (Q6 would allow Kp up to 4) or the gain table a
global multiplier. Measured in `docs/p1-hardware-test.md` step 5.

*Measured 2026-10-11* (`docs/p1-hardware-test-results.md`): the leg response
is **≈0.17° of body per degree of `corr`**, not 1. On a board propped 15°
nose up, Kp 0.99 moved the body from −10.2° to −8.8°, three times out of
three; the signs are right, the legs settle. H13 (3° residual) would need
Kp ≈ 20..40 through this map, which the legacy table was never meant for.
**Owner's decision 2026-10-11: keep the P1 layer as the legacy-feel lean and
defer body levelling to P2, where the IK gives a proper leg-length solution
for a target body attitude** (open question 8 below moves to P2).

**Joint-space mapping** (`balance_map`). The per-joint gains are the legacy
`adaptiveParameterArray` {roll, pitch} factors in PROGMEM, which are also
byte-for-byte Kilo's legacy EEPROM `ADAPT_PARAM` @128; units are joint degrees
per 57.3° of tilt:

| Joint | pan | tilt | tail | shoulders LF, RF | shoulders RB, LB | knees LF, RF | knees RB, LB |
|---|---|---|---|---|---|---|---|
| roll factor | −60 | −60 | −120 | 60 | −60 | −90 | 90 |
| pitch factor | 0 | −60 | 0 | 30 | 30 | −45 | −45 |

They are applied exactly as legacy `adjust()` applied them to a posture
(Appendix C3), with legacy's deviations expressed through the corrections
(`dR = −corr_roll`, `dP = −corr_pitch`, each times `LEGACY_*_SIGN`, see
"Sign conventions" below):

```
roll term  = |dR|·rF·(1.2 on one side)    tilt and the legs: a symmetric crouch, 1.2× on one side
           =  dR ·rF                      pan and tail
joint     += (roll term − pF·dP) / 57.3   added to the base angle (legacy: frame + adjust)
```

So 1° of `corr_pitch` moves the knees 0.79°, shoulders 0.52° and head tilt
1.05°; 1° of `corr_roll` crouches every leg (shoulders 1.05°, knees 1.57°)
and the lowering side's legs 1.2× as far, so only a fifth of the roll gain is
differential. With Kp at its maximum and Kd 0 the controller reproduces
legacy `adjust()` (no damper) within 0.16° for every joint (host test). In
physical terms, checked against the posture tables: +`corr_pitch` moves the
knees and back shoulders from STAND away from SIT (the front folds, the back
extends: nose down, measured pitch up), and +`corr_roll` crouches the left
legs further than the right (right side up, measured roll up). Worked
through, Kp 0.5, DB 2°: nose up 10° reads pitch −10 → `e` = −8 → `corr_pitch`
= +4 → `dP` = −4 → front and back knees −3.1°, back shoulders +2.1° (front
folds, back extends) → the nose comes down. Right side up 10° reads roll +10
→ `corr_roll` = −4 → `dR` = −(−4)·(−1) = −4 → every leg crouches, the right
pair 1.2× → the right side comes down.

**Expected attitude per posture** (the balance reference), from the legacy
skill headers in `src/InstinctNybble.h` (frame count, expected roll, expected
pitch, angle ratio), used verbatim (Kilo's attitude is the legacy `ypr`):

| Posture | Header roll, pitch (legacy) | Kilo `ref` roll, pitch | Balance |
|---|---|---|---|
| `rest` | 0, 0 | 0°, 0° | off (lying down) |
| `sit` | 0, −30 | 0°, **−30°** (nose up) | on |
| `up` (STAND, and `balance`) | 0, 0 | 0°, 0° | on |
| `str` (stretch, a P1 trick) | 0, 15 | 0°, **+15°** (nose down) | with the trick's `BALANCE` flag |
| `calib`, `zero` | 0, 0 | 0°, 0° | off |

**Sign test.** Host closed-loop tests (`body/test/test_balance.cpp`) run a
toy plant (in the measured convention: a positive correction raises the
measured value, i.e. lowers the nose or raises the right side): body tilt = disturbance + x, where x follows the correction with a
first-order lag (τ 0.15 s). A 10° step settles at 7.3° = (10 + Kp·DB)/(1 + Kp)
with the defaults; flipping the actuation or the measurement sign drives it
to 18°, and with a large rate gain (Kd 200 ms > τ) the flipped gyro sign
oscillates (12° peak to peak) where the right sign settles. The same loop
closed through `body_tick`, the arbiter and the joint outputs (the plant reads
the knees physically, front fold / back extend = nose down, left crouching
further = right side up, independent of the sign constants) settles at 7.3° pitch / 7.4° roll and runs away to ≈18° with the
sensor flipped. Both axes.

**Sign conventions (verified in P1).**
- **Kilo's attitude is the legacy `ypr`, unchanged.** Legacy `imu.h` reads
  `dmpGetQuaternion(Quaternion*)` (int16/16384), `dmpGetGravity`,
  `dmpGetYawPitchRoll` (the V6_12 active branch, with the reflection) and
  multiplies by `degPerRad`; nothing swaps or negates `ypr` afterwards, and
  `skill.h` uses `ypr[1]` as pitch and `ypr[2]` as roll. The A/B host test
  (`test_attitude.cpp`) runs that path in float32 against the integer
  firmware over 5000 random orientations within ±60° (any yaw): max error
  0.038° pitch, 0.038° roll (the §7 criterion is 1°). TELEMETRY `pitch`/`roll`
  are therefore exactly what the legacy firmware's `ypr` would read.
- **On Kilo nose-DOWN is +pitch and RIGHT-side-up is +roll**, i.e. the
  legacy `ypr` convention. Measured on 2026-10-10 by tilting the whole cat
  by hand with `kilo monitor` (build 975055a): nose up read pitch −74°, left
  side up roll −88°, level +0.9° / −2.7°. (P0 had claimed the opposite on
  both axes without measuring it.)
- **Rates.** `rate_roll = +gyro_x`, `rate_pitch = −gyro_y` are the time
  derivatives of that roll and pitch: checked on the host against the
  quaternion kinematics (`q' = q·(1, ω dt/2)`, body-frame gyro) over 2000
  random orientations within 20° of level (6% worst error, from axis
  cross-coupling at tilt). So the nose rising gives a negative `rate_pitch`
  and the left side rising a negative `rate_roll`, and Kd always damps in the
  direction Kp pushes.
- **Pitch: the legacy skills and gains match.** Every Nybble skill header
  uses nose-down = +pitch: `sit` (nose up) expects −30, the hind-leg `stand`
  −60, `lifted` −75; `buttUp` and `str` (nose down) +15, `dropped` +75.
  Legacy `adjust()` agrees: on a negative (nose-up) pitch deviation it moves
  the legs away from SIT (front folds, back extends), which lowers the nose.
  So the posture references are the header bytes verbatim and the pitch
  gains are used as they are (`LEGACY_PITCH_SIGN = +1` in `core/balance.h`).
- **Roll: the legacy gains are wrong-signed for `ypr`.** Legacy `adjust()`
  crouches every leg by |roll deviation| and the LEFT pair 1.2× harder when
  the deviation is positive. That lowers the left side, which corrects a
  left-side-up tilt, but `ypr` reads left-side-up as NEGATIVE roll. (A
  right-handed sensor cannot have nose-down +pitch and left-up +roll at once;
  on stock legacy the weak 20% roll differential was the wrong-signed axis.)
  So Kilo negates roll before the gains (`LEGACY_ROLL_SIGN = −1`); expected
  roll references stay verbatim (all 0 for the postures). These two
  constants are the only place the convention meets the gains; the host
  plant in `test_balance.cpp` is physical, so flipping either one fails the
  direction and closed-loop tests.

**Attitude without float.** The DMP quaternion is int32 Q30. The firmware takes
the high 16 bits as Q14:

```
gx = 2(qx·qz − qw·qy),  gy = 2(qw·qx + qy·qz),  gz = qw² − qx² − qy² + qz²
```

Each product is Q28 in int32. Each sum is shifted back to Q14 **before**
squaring.

```
pitch = atan2(gx, isqrt(gy² + gz²))     (gy² + gz² is Q28 in int32, isqrt gives Q14)
roll  = atan2(gy, gz)
if gz < 0: pitch = (pitch > 0 ? 180° − pitch : −180° − pitch)    // legacy reflection, kept
```

The reflection matches the active branch of `dmpGetYawPitchRoll` in the
vendored V6_12 header (`USE_OLD_DMPGETYAWPITCHROLL` is not defined). Keeping it
preserves the legacy convention, so the P1 A/B test against today's `ypr` and
the fall-axis classification both behave the same. Rates come from the raw
gyro in the DMP packet.

The DMP-free filter (P5) is in Appendix A.

### 3g. Reflexes and safety

Reflexes are pure state machines in `core/reflex.*`, with full host tests.

**Fall**

| State | Transition / action |
|---|---|
| UPRIGHT → TIPPING | \|roll\| > 75° or \|pitch\| > 75° (`FALL_DEG`) |
| TIPPING → FALLEN | Held for 6 ticks (120 ms). Epoch bump. REFLEX holds the pose. `EVENT FELL(axis, sign)`. Wait until \|gyro\| < 20°/s for 10 ticks. |
| FALLEN → RECOVERING | Roll fall: play `rc`, mirrored by roll sign, up to 3 tries 1 s apart. Pitch fall: REST, then the fall reflex's LIMP timer (2 s), then `RECOVERY_FAILED`. |
| RECOVERING → UPRIGHT | \|roll\|, \|pitch\| < 20° for 25 ticks. IDLE STAND, then `RECOVERED`. |

Fall detection is disabled in CALIB and LIMP, under `IMU_FAULT`, and during
`FALL_OK` tricks.

As implemented in P1 (`core/reflex.{h,cpp}` for the detector,
`fall_step()` in `core/body.cpp` for the state machine, `rc` in
`core/posture.cpp`; host test `body/test/test_reflex.cpp`):

- **Trip.** Every fresh IMU sample with |roll| or |pitch| > 75° counts; one
  under it restarts the count; a tick without a fresh sample neither counts
  nor resets. The 6th consecutive sample trips it, so a single bad or
  handled sample never does (P0 finding 8). A pitch fall is one with the
  long axis within 15° of vertical (75° < |pitch| < 105°, folding the
  legacy pitch reflection); anything else over the threshold (a side, the
  back) is a roll fall. `EVENT FELL` arg ±1 (roll, sign of roll) or ±2
  (pitch, sign of pitch), after the epoch bump, so it carries the new epoch;
  a playing trick ends first with `TRICK_DONE(FELL)`. A short chirp (note
  20) as legacy beeped. REFLEX holds the current output (`POSTURE_HOLD`).
- **Still.** Every body rate < 20°/s on 10 consecutive samples. Until then
  it holds, however long it is handled.
- **rc.** The legacy behaviour (`src/InstinctNybble.h` `rc`, 3 frames)
  converted like the trick importer converts behaviours: dur 11, 5, 18
  ticks (cosine), hold 30, 25, 0 ticks, 89 ticks (1.8 s) in all, played by
  the REFLEX mover from the current output. Legacy `reaction.h` played it
  on |roll| > 90° from a single sample, mirrored **at random**
  (`loadFrame`: `random() % 2` for a name not ending in L/R). Kilo mirrors
  it by the roll sign instead: unmirrored for roll < 0 (right side down;
  the unmirrored frame 1 swings the right legs, which then push on the
  ground), mirrored for roll > 0, and flipped on every other try in case
  that guess is wrong or the body rolled over (**to be verified on Kilo**,
  p1-hardware-test.md "Fall and head").
- **Check, retry.** After rc's last frame REFLEX holds it for up to 1 s;
  25 consecutive samples with |roll|, |pitch| < 20° are RECOVERED: IDLE
  takes STAND (REST if the config is DEFAULTED, where STAND would be
  refused), REFLEX hands back with the usual `XFADE_TICKS` crossfade,
  `EVENT RECOVERED`, detection re-arms. Otherwise the next try (after the
  still wait again, so tries are ≥ 1 s apart), at most 3.
- **Failure.** A pitch fall once still, the third failed try, or an
  `IMU_FAULT` after the trip: REST over 1 s, `EVENT RECOVERY_FAILED` at
  once, LIMP 2 s after the REST move (the fall reflex's own timer), then
  the reflex lets go (detection stays off in LIMP).
- **Owner and goals.** The fall reflex ranks above the warm-boot hold and
  below the watchdog and low-battery reflexes (`RH_FALL`): it does not
  trip while they hold the joints, and if one takes them during a fall the
  fall reflex ends silently (it re-arms when they let go). While it holds
  the joints (TELEMETRY mode FALLEN, or RECOVERING during rc and the
  check): goals from older epochs are refused STALE as usual; current
  `SET_POSTURE` (REST included), `SET_HEAD` mode 1 are refused `FALLEN`,
  `TRICK_PLAY` `REFUSED_FALLEN`; `SET_POSTURE LIMP` (the Pi's emergency
  stop) ends the reflex and goes limp. After RECOVERED the mind's goals
  with the new epoch work again.
- **Cost.** 10 B of RAM (`Fall`); with the head layer, +1268 B of flash
  including the 39 B `rc` table (3h).

**Battery.** A7 reads through a ≈1:2 divider against a 5 V reference, and the
legacy scale is 99 counts per volt, so `LOW_VOLTAGE 650` ≈ 6.57 V (3.28 V per
cell). The scale goes into the EEPROM slot after one meter check. Samples are
taken at 10 Hz and IIR-filtered (α = 1/16, τ ≈ 1.6 s), so motion sag does not
trip thresholds.

| Level | Trip | Clear (hysteresis) | Action |
|---|---|---|---|
| WARN | < 6.9 V for 5 s | > 7.1 V for 10 s | `EVENT LOW_BATTERY(1)` |
| LOW | < 6.6 V for 5 s | > 7.0 V for 10 s | Epoch bump. REFLEX → REST, LIMP after 2 s. Motion goals refused (`REFUSED_LOW_BATT`). Low-battery chirp. `EVENT LOW_BATTERY(2)`. |
| CRITICAL | < 6.3 V for 1 s | (latched until reboot) | LIMP now. `EVENT LOW_BATTERY(3)`. The Pi shuts itself down. |
| no battery | < 3 V | > 5 V | Battery reflexes off (USB bench power), `NO_BATTERY` flag |

**Joint limits** are enforced in the safety stage for every owner. Params can
only narrow them.

**Link watchdog.** Any CRC-valid frame from the Pi resets it, including one
with an unknown type or a wrong length. It arms on the first such frame after
boot: a body nobody has talked to has nothing to stop.

| Since the last valid frame | Action |
|---|---|
| ≥ 0.5 s (`WD_STOP_MS`) | Stage 1. Epoch bump. Stop: gait ramp-down (≤ 1 period, ≤ 2 s), trick stopped, head released. `EVENT WATCHDOG`. |
| ≥ 1.5 s (`WD_REST_MS`), and the ramp-down is done | Stage 2. REFLEX → REST over 1 s, balance on. |
| REST + 10 s | The watchdog reflex's LIMP timer fires: LIMP. |
| Next valid frame | `EVENT LINK_RESTORED`. The body stays in its current state until the Pi sends a goal with the new epoch. |

Worst case, from the Pi going silent mid-walk to lying down, is
0.5 + 2 + 1 = 3.5 s. From standing it is 2.5 s.

**REST and LIMP.**
- **REST** is only a posture: lying down, with servos powered.
- **LIMP** writes every channel full-off. G41 servos then hold no torque, but
  their electronics stay powered (the board appears to have no servo-rail
  switch; to verify).
- REST never limps by itself. Each reflex that ends in LIMP owns its own timer:
  fall 2 s, low battery 2 s, watchdog 10 s. A body that is already LIMP stays
  LIMP when the watchdog or the low-battery reflex would move it to REST:
  powering servos from an unknown pose to lie down is worse than staying
  limp. The Pi can request either state
  directly.

**Boot.**
- A cold boot stays LIMP, because the servo pose is unknown.
- A warm boot (the PCA9685 is found configured) holds the read-back pose, then
  goes to REST under the watchdog timeline. The body never stands without the
  Pi.

**IMU fault.** The fault is raised when no fresh packet arrives for 10 ticks,
or after 3 consecutive I2C errors. Then balance and fall detection are disabled,
`IMU_FAULT` is set, and an event is emitted. Re-initialisation happens only
when LIMP (3a).

**Hardware watchdog.** 250 ms, enabled after the IMU initialises (3a).

### 3h. Memory budget

Itemised estimates, and today's measured symbols for calibration, are in
Appendix D.

| | Flash | RAM |
|---|---|---|
| Estimate (P2 feature set) | ≈15 KB est.; P2 ceiling 20 KB (P0 measured 17.7 KB, below) | globals ≈1.50 KB + stack 350 B ≈ 1.85 KB |
| Limit | 32256 B (32 KB − 512 B optiboot) | 2048 B |
| Headroom | ≥ 14 KB | ≈200 B |

| Phase | Flash ceiling (`body/tools/size_check.sh FLASH_MAX`) | Globals ceiling |
|---|---|---|
| P0 (incl. fx + integer attitude) | 20 KB (20480 B) | 1540 B |
| P1 | 24 KB (24576 B, the `size_check.sh` default since 2026-10-11; 22 KB before) | 1540 B |
| P2 | 24 KB (24576 B) | 1540 B |
| P5 | 24 KB; removing the DMP frees ≈3.5 KB | 1540 B |

**Build flags.** The body is compiled with `-mcall-prologues` (C and C++:
`compiler.cpp.extra_flags` and `compiler.c.extra_flags`, the C++ one also
carrying `-DKILO_BUILD=0x...`) and linked with `-Wl,--relax`
(`compiler.c.elf.extra_flags`), by `kilo flash` (`kilo.tools.flash.compile_cmd`)
and by `body/tools/size_check.sh`, which also fails an ELF built without
them (no `__prologue_saves__`, or a `call`/`jmp` an `rcall`/`rjmp` would
reach). The host tests (g++) do not use them.

**Measured at the P0 integration (default build: ultrasonic and buzzer on,
all P0 messages):** 18164 B flash, 1061 B globals, over the P0 ceiling by
1780 B. The link layer (codec, dispatch, telemetry, events, params) costs
≈4.5 KB, against the 2.3 KB estimated in Appendix D. Turning ultrasonic and
buzzer off saves ≈610 B, `-mrelax -mcall-prologues` ≈430 B; neither closes
the gap. The ceilings are the owner's decision (§8).

**Measured with the P1 balance layer:** 19544 B flash (+1380 B), 1075 B
globals (+14 B for the `Balance` state). `balance_axis_step` is the largest
new function (468 B, plus 116 B for its rounding helper; the mapping is inlined into the tick).

**Measured with the P1 trick player:** 21246 B flash (+1676 B), 1525 B
globals (+450 B: the 448 B store and 24 B of header, upload and player
state, less the arbiter's 22 B `base[]`, gone because crossfades now start
from the output). That leaves 15 B under the globals ceiling: the next RAM
user (head layer, fall reflex) takes the first shrink step below.

**Measured with the P1 fall reflex and head layer** (shrink step 1 taken:
the store is 24 frames, `trick_capacity` 24 in HELLO): **22514 B flash**
(+1268 B; 14 B under the 22528 B P1 ceiling), **1432 B globals** (−93 B:
−112 B store, +10 B `Fall`, +9 B head hold; 108 B under the ceiling, so
616 B for stack and margin, ≈93 B more than before). Flash is now the
tight resource. Getting under the ceiling took: one `under()` helper for
the attitude tests, the head targets copied as sent and clamped by the
safety stage (the layer steps without subtracting the target), the head
hold tagged as owner `PI_HOLD` so a release reuses the hand-over
crossfade, and no saturation on counters that cannot overflow. If the next
feature needs flash, the candidates, in the suggested order (each
a decision for the owner, §8): `-mrelax -mcall-prologues` through
`kilo flash` and `size_check.sh` (≈430 B, a build-flag change everywhere
the body is compiled); raising the P1 flash ceiling toward the P2 one
(24 KB; 32256 B is the hardware limit); dropping the ultrasonic or buzzer
(≈0.4 KB each, features).

**Owner decision 2026-10-11:** the first two are taken. With
`-mcall-prologues` and `-Wl,--relax` the same source is **22092 B flash**
(−422 B), globals unchanged (1432 B); the P1 ceiling is 24 KB (24576 B).
**Measured with the boot epoch** (§3c): **22132 B flash** (+40 B),
**1433 B globals** (+1 B `.noinit`, the epoch in the reset record), 2444 B
under the P1 ceiling.
**Measured with the IR receiver** (2026-10-11, 3a; from 22102 B / 1433 B):
**22548 B flash** (+446 B), **1450 B globals** (+17 B: the decoder state and
the previous `PIND`); `-DKILO_IR=0` gives the bytes back.

The 1540 B globals ceiling keeps ≥ 508 B for stack and margin. Measured stack
use is reported through a painted canary (`stack_free_min` in STATS). RAM is
the tight resource. If needed, shrink in this order:
1. keyframe store 32 → 24 frames (−112 B; the first-version library's
   largest trick has 6 frames, HELLO reports the capacity to the mind).
   **Done** with the fall reflex and head layer (above);
2. RX ring 192 → 128 B;
3. TX ring 96 → 64 B.

---

## 4. Protocol specification

`protocol/spec.toml` is the single source of truth. `protocol/gen.py`
generates `protocol/gen/kilo_protocol.h` and `protocol/gen/kilo_protocol.py`
(committed). `protocol/vendor.py` copies them, byte for byte, into
`body/src/proto/` (with the C codec) and `mind/kilo/protocol/` (with the
Python codec): arduino-cli compiles a copy of the sketch, so the body cannot
include `protocol/` by relative path, and the mind is an installed package.
`protocol/test.sh` fails if a copy is stale. The shared C
and Python codecs and the test vectors are in `protocol/c/`, `protocol/py/`
and `protocol/vectors/` (`protocol/README.md`). This section gives the shape;
Appendix E has every byte layout.

**Link.**
- 328P USART0 ↔ Pi PL011 on GPIO14/15 (`/dev/ttyAMA0`), 8N1, no flow control,
  at **250000 baud**.
- Both ends are exact at that rate: on the 328P with U2X and UBRR = 7, and on
  the PL011 with a 48 MHz UARTCLK and divisor 12.000. 115200 has a 2.1% error
  on the 328P. 500000 would halve the main-loop slack for no need.
- Worst-case load is 19% upstream and 37% downstream (Appendix B).
- Check `vcgencmd measure_clock uart` reads 48 MHz.
- During development the Petoi CH343 USB adapter can stand in for the Pi
  (Appendix F). **Unplug it whenever the Pi drives the UART**, because both
  drive the same RX line.

**Framing:**

```
decoded: type u8 | seq u8 | payload (0..60) | crc16 u16 LE        (4..64 B)
wire:    COBS(decoded) | 0x00                                     (≤ 66 B)
CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection), over type..payload; check("123456789") = 0x29B1
```

- **Why COBS.** 0x00 never occurs inside a frame, so every delimiter is a
  certain resync point, and a dropped byte damages exactly one frame. With a
  start byte plus a length field, a corrupted length can swallow the next good
  frames.
- **Why CRC-16.** It is in the Python standard library as
  `binascii.crc_hqx(data, 0xFFFF)`. On AVR, avr-libc's `_crc_xmodem_update()`
  (inline asm, a few tens of cycles per byte) computes the same polynomial
  without reflection, so with init 0xFFFF it gives CCITT-FALSE. The host build
  uses a portable bitwise version.

**Receiver rules:**
- Read up to 0x00. If more than 66 B arrive first, discard up to the next
  delimiter (`rx_overflow`).
- COBS-decode, check `len ≥ 4`, check the CRC (`rx_crc_err`).
- The payload length must match the type exactly; variable-length types state
  their rule.
- `seq` increments per frame in each direction. A command retry reuses the
  original frame's `seq`, which is what makes de-duplication work.

**Acks and errors.** There is no NACK message. A reply to a command is its
data reply (queries) or an `ACK(seq, type, status)`. Status values:

| Code | Status |
|---|---|
| 0 | OK |
| 1 | BAD_LEN |
| 2 | BAD_ARG |
| 3 | BUSY |
| 4 | UNKNOWN_TYPE |
| 5 | CRC_MISMATCH (trick) |
| 6 | NO_TRICK |
| 7 | REFUSED_LOW_BATT |
| 8 | REFUSED_FALLEN |
| 9 | EEPROM_BUSY |
| 10 | STALE |
| 11 | UNCALIBRATED |

- **Goals** (`SET_POSTURE`, `SET_GAIT`, `SET_HEAD`, `SET_BODY_POSE`,
  `SET_BALANCE`, `SET_EYES`, `BEEP`, `HEARTBEAT`) are never acked. `SET_STREAM`
  is a command. Goals are idempotent, and the mind re-sends active goals at
  5 Hz and immediately on change. Rejections are reported as
  `EVENT GOAL_REFUSED(type << 8 | reason)`, at most one per goal type per
  second. The reason is an ACK status code: STALE 10, BAD_LEN 1, BAD_ARG 2,
  LOW_BATT 7, FALLEN 8, UNCALIBRATED 11 (STAND while the config is
  DEFAULTED), or UNKNOWN_TYPE 4 for a goal this firmware does not implement
  yet (in P0: `SET_GAIT`, `SET_HEAD`, `SET_BODY_POSE`, `SET_BALANCE`,
  `SET_EYES`; `SET_BALANCE` is implemented from P1).
  Telemetry confirms what took effect: `mode`, owners, `epoch`, `last_rx_seq`.
- **Queries** (`HELLO_REQ`, `PING`, `STATS_REQ`, `PCA_READ`, `CAL_GET`,
  `PARAM_GET`, `EEPROM_READ`; `query = true` in the spec) are answered by their
  data reply only (`HELLO`, `PONG`, `STATS`, `PCA_COUNTS`, `CAL_VALUES`,
  `PARAM_VALUE`, `EEPROM_DATA`). They get an ACK only on failure (for example
  `PARAM_GET` of an unknown id is `ACK BAD_ARG`, a `PCA_READ` I2C error is
  `ACK BUSY`). They are never de-duplicated, and the Pi retries a query with a
  fresh `seq` after 50 ms, up to 3 times.
- **Other commands** (trick upload/play/stop, calibration writes, params,
  stream settings, IMU offsets, reboot) get exactly one ACK each. The Pi
  retries after 50 ms, up to 3 times, with the same `seq`. (In P0 the trick
  commands were `ACK UNKNOWN_TYPE`; P1 implements them, §3e.)
- A frame with a bad CRC is never acked, because its `seq` and `type` cannot be
  trusted. A CRC-valid frame with an unknown type is acked `UNKNOWN_TYPE`; a
  known Pi → body type with a wrong payload length is acked `BAD_LEN` (a
  command) or refused `GOAL_REFUSED(type, BAD_LEN)` (a goal). Known body → Pi
  types arriving at the body are ignored.
- **De-duplication.** The body caches the `(seq, type, status)` of the last
  executed non-query command for 500 ms (`dedup_window_ms`). A repeat with the
  same `seq` and `type` within that window is re-acked with the cached status
  and not executed again, so a retry after a lost ACK is harmless. The window
  exists because a new mind session (each `kilo` command is a process) starts
  its `seq` afresh; the mind also starts at a random `seq`.
- `CAL_SAVE` and `PARAM_SET persist` are acked when queued. The write itself is
  reported later by `EVENT EEPROM_SAVED`.

**Message index.** Types 0x01-0x7F go Pi → body; 0x80-0xFF go body → Pi. `E`
marks a payload that starts with an epoch byte.

| Type | Name | Kind | Purpose |
|---|---|---|---|
| 0x01 | HELLO_REQ | cmd | Ask for HELLO |
| 0x02 | HEARTBEAT | goal | Keep the link alive |
| 0x03 | PING | cmd | → PONG, for RTT |
| 0x04 | SET_STREAM | cmd | Telemetry, JOINTS and IMU_RAW divisors |
| 0x05 | STATS_REQ | cmd | → STATS |
| 0x06 | PCA_READ | cmd | → PCA_COUNTS (debug and calibration checks) |
| 0x10 | SET_POSTURE | goal E | LIMP, REST, SIT, STAND, STANCE, CALIB, ZERO + transition |
| 0x11 | SET_GAIT | goal E | Run/stop + gait parameters |
| 0x12 | SET_HEAD | goal E | Hold/release pan, tilt, tail + speed |
| 0x13 | SET_BODY_POSE | goal E | Pitch, roll, height, x-shift |
| 0x14 | SET_BALANCE | goal | Enable, Kp, Kd, dead band, clamp |
| 0x20 | TRICK_BEGIN | cmd | Header |
| 0x21 | TRICK_FRAMES | cmd | Up to 4 frames |
| 0x22 | TRICK_COMMIT | cmd | CRC check |
| 0x23 | TRICK_PLAY | cmd E | Play with speed and mirror |
| 0x24 | TRICK_STOP | cmd | Stop |
| 0x30 | SET_EYES | goal | LED mask + RGB |
| 0x31 | BEEP | goal | Append up to 8 notes to the buzzer queue |
| 0x38 | CAL_GET | cmd | → CAL_VALUES |
| 0x39 | CAL_SET | cmd | Absolute offset for one joint |
| 0x3A | CAL_SAVE | cmd | Queue a slot write |
| 0x3B | CAL_REVERT | cmd | Reload from the slot |
| 0x3C | IMU_OFFSETS_SET | cmd | Write MPU offsets (optionally persist) |
| 0x3D | PARAM_GET | cmd | → PARAM_VALUE |
| 0x3E | PARAM_SET | cmd | Set (optionally persist) a param |
| 0x3F | EEPROM_READ | cmd | → EEPROM_DATA (≤ 56 B) |
| 0x7E | REBOOT | cmd | Watchdog restart of the application (not the bootloader) |
| 0x80 | HELLO | | Versions, build hash, reset cause, last alive tick, capabilities, epoch |
| 0x81 | TELEMETRY | | Layout below |
| 0x82 | JOINTS | | 11 output angles in a16 |
| 0x83 | IMU_RAW | | Raw accel + gyro |
| 0x84 | ACK | | seq, type, status |
| 0x85 | EVENT | | code, arg, tick, epoch |
| 0x86 | CAL_VALUES | | 16 calibration offsets in a16 |
| 0x87 | PARAM_VALUE | | id, value, min, max, default |
| 0x88 | EEPROM_DATA | | addr + bytes |
| 0x89 | PONG | | token, tick |
| 0x8A | STATS | | 19 B of counters |
| 0x8B | PCA_COUNTS | | 16 × u16 OFF counts |
| 0x8F | LOG | | Debug builds only |

**TELEMETRY (0x81), 26 B, 25 Hz by default:**

| Off | Field | Type | Meaning |
|---|---|---|---|
| 0 | tick | u16 | Wraps every 21.8 min; the Pi unwraps it |
| 2 | pitch | i16 a16 | Nose down + (legacy `ypr`), with the legacy reflection |
| 4 | roll | i16 a16 | Right side up + (legacy `ypr`; measured 2026-10-10) |
| 6 | rate_pitch / rate_roll / rate_yaw | 3 × i16 a16/s | |
| 12 | dist_mm | u16 | 0 = no echo |
| 14 | batt_mv | u16 | Filtered |
| 16 | mode | u8 | 0 BOOT, 1 LIMP, 2 IDLE, 3 GAIT, 4 TRICK, 5 FALLEN, 6 RECOVERING, 7 LOW_BATT, 8 CALIB, 9 LINK_LOST |
| 17 | owners | u8 | Low nibble: leg owner. High nibble: head owner (4 = Pi hold). |
| 18 | flags | u16 | bit 0 balance active, 1 IMU_FAULT, 2 servos powered, 3 gait, 4 trick, 5 head held, 6 batt warn, 7 link ok, 8 overrun, 9 NO_BATTERY, 10 EEPROM pending, 11 dist fresh |
| 20 | last_rx_seq | u8 | |
| 21 | trick_frame | u8 | 0xFF = none |
| 22 | gait_phase | u8 | φ >> 8 |
| 23 | tick_load | u8 | Max tick time since the last telemetry, in 100 µs units |
| 24 | rx_err | u8 | Cumulative, mod 256 |
| 25 | epoch | u8 | Current goal epoch |

**TX priority.** The TX ring is 96 B. The worst single-tick burst at maximum
stream rates is 103 B: TELEMETRY 32 + JOINTS 30 + IMU_RAW 20 + ACK 9 +
EVENT 12. When a frame does not fit, the body drops IMU_RAW first,
then JOINTS, then TELEMETRY. ACK and EVENT are never dropped. Events wait in
their 8-deep queue until there is room. Drops are reported by a rate-limited `EVENT TX_DROPPED`.

**Watchdog timing.**
- The Pi's serial writer thread sends `HEARTBEAT` whenever it has sent nothing
  for 100 ms. To stop a hung mind from masking itself, **all** traffic the
  writer generates on its own (heartbeats, the 5 Hz goal refresh, `HELLO_REQ`
  polling, the `HELLO_REQ` on (re)connect) is sent only while the asyncio loop
  has proved alive within the last 1 s (§5). Goals and commands the mind
  issues are sent at once.
- The body stops motion at 0.5 s and moves to REST at 1.5 s (3g).
- The Pi treats the body as lost after 300 ms without telemetry, and polls
  `HELLO_REQ` at 2 Hz until it answers.

**Boot and reboot detection.** On every boot the body sends an unsolicited
`HELLO` (with `reset_cause` from 3a, `last_alive_tick`, build hash, protocol
version and current epoch) and `EVENT BOOT`. The Pi treats an unsolicited HELLO,
a HELLO whose identity (reset cause, `last_alive_tick`, build) differs from
the last one, or a tick that goes backwards, as a reboot (and clears its
motion goals before anything else can be sent; §5 "Contact"). It then:

1. logs the cause;
2. checks `proto_major`, and refuses to drive and offers to flash on a mismatch;
3. re-sends stream settings and non-default params;
4. invalidates its trick cache;
5. adopts the epoch and re-plans its goals.

Flashing over this same UART (GPIO4 reset, optiboot at 115200, verification by
HELLO build hash) is in Appendix F.

---

## 5. Mind software design (Python on the Pi)

The mind runs on the Pi 2 B: 4 × Cortex-A7 at 900 MHz, 1 GB RAM, Raspbian 13
with Python 3.13. Dependencies are pyserial, numpy, OpenCV (apt
`python3-opencv`) and picamera2 (apt). The package is `mind/kilo/`; its layout
and the CLI are in Appendix G.

**Serial link.** `SerialLink` owns `/dev/ttyAMA0` at 250000 with
`ASYNC_LOW_LATENCY`, and runs three threads:

- **Reader:** splits on 0x00, decodes COBS, checks the CRC, parses into
  dataclasses with the generated `struct` formats, and hands the messages to
  the asyncio loop with `call_soon_threadsafe`.
- **Writer:** drains an outbound queue and assigns `seq` (starting at a random
  value). Everything it sends on its own (`HEARTBEAT` after 100 ms of silence,
  the 5 Hz goal refresh, `HELLO_REQ` polling) is gated on the asyncio loop's
  liveness stamp being less than 1 s old. A stalled mind therefore trips the
  body's watchdog instead of being kept alive by the thread.
- **Logger:** writes every frame with a monotonic timestamp to a rotating file,
  off the asyncio loop, so disk stalls never delay control.

`request()` awaits the data reply (queries, retried with a fresh `seq`) or the
ACK (other commands, retried with the same `seq`, which the body de-duplicates)
with a 50 ms timeout and 3 retries.

**Desired-state goals and epoch.** `BodyClient` stores the desired posture,
gait, head, pose, balance and eyes. It sends a goal on change and re-sends all
active goals at 5 Hz, each stamped with the epoch that was current when it
was set (never re-stamped).
Behaviours call methods like:

- `body.walk(stride=30, turn=0, pattern=TROT)`
- `body.look(20, -10)`
- `await body.play("hi", mirror=True)`, which resolves on `TRICK_DONE`
- `body.beep([...])`

On `FELL`, `WATCHDOG` or `LOW_BATTERY`, and on any epoch change seen in
TELEMETRY, an EVENT or HELLO, the client **clears the desired motion goals from
older epochs**, adopts the new epoch, and notifies the behaviour layer. Nothing
from before the change is replayed.

**Contact.** A body that may just have rebooted must not get the old goals
(a reboot restarts the epoch at 0, §3c). After every (re)connect, and once
telemetry has been missing for 300 ms, the link sends and refreshes no
motion goal (`SET_POSTURE` other than REST/LIMP, a `SET_BALANCE` enable, the
other epoch goals) and refuses `TRICK_PLAY` until the body is heard on this
connection: a HELLO that proves a reboot (the goals are cleared first, under
the lock the writer packs goals under), or a TELEMETRY whose tick has been
checked for a restart. A reboot clears the motion goals every time; only its
count is debounced. The behaviour runner forgets its posture/balance cache on
an epoch, `reboots` or `connected` change. What is left (refreshes sent in
the first 300 ms of a reboot on a live connection) is refused STALE by
the body: it boots at a new epoch (§3c "Boot epoch").

**Body state.** `BodyState` holds:
- the latest telemetry, with time unwrapped;
- link health;
- derived values: upright, moving, fallen, filtered distance with approach
  speed, and battery percentage from a resting 2S curve;
- the last events;
- a 10 s numpy ring buffer for detectors.

**Behaviour: a minimal behaviour tree under a utility drive selector, ticked at
10 Hz.**

- **Why a behaviour tree.** Personality is many small reactive behaviours
  (startle, follow a face, nap, greet, explore) that preempt one another.
  Selectors re-check higher-priority conditions every tick, sequences express
  multi-step acts, and decorators give cooldowns, timeouts and chance. A
  hierarchical state machine needs a transition for every interruptible pair,
  which tangles at 10-20 behaviours.
- **Why a utility selector.** A pure behaviour tree is deterministic; a pet
  should not be. The root therefore scores top-level behaviours from **drives**:
  - energy: drains with motion, recovers at rest, tied to battery;
  - curiosity: grows when idle;
  - sociability: grows when alone, drops when a face is seen or Kilo is
    petted;
  - arousal: spikes on a startle, then decays.

  Percepts, personality weights from an editable YAML file, and a little noise
  also feed the scores, with hysteresis. Safety branches (fallen, low battery,
  shutdown) sit above the selector.
- **Engine.** The engine is in-house, ≈300 lines: Sequence, Selector, Parallel,
  Cooldown, Timeout, Action and Condition. Each tick's active path is logged.

**Tricks.**
- Tricks are JSON keyframes named by joint, with a model, validation, mirror
  and time-scaling, and are uploaded with caching by `(trick_id, crc)`.
- The first-version library is imported from `src/InstinctNybble.h`:
  - basics: `sit`, `rest`, `up`, `str` (stretch), `balance`;
  - social: `hi`, `wsf` (wash face), `ck` (check around), `snf` (sniff),
    `scrh` (scratch), `pee`, `pu` (push-up), `buttUp`.

  Athletic tricks (backflip, roll, jump) are not ported. `rc` stays a PROGMEM
  reflex on the body.
- Import rules:
  - `dur = ceil(maxΔ / (speed/4) · LEGACY_STEP_MS / 20)`, where
    `LEGACY_STEP_MS` (≈5-6 ms **(est.)**) is calibrated by filming both
    firmwares doing `hi`.
  - Legacy speed 0 ("jump to") → `dur = 1`; the body's slew limit absorbs the
    jump.
  - `hold = delay·50/20`.
  - `loopCycle` maps to the loop fields. Joints 3-7 are dropped.
  - On mirror, `expectedRollPitch[0]` (roll) is negated.
  - `FALL_OK` is set for tricks whose expected or reached tilt exceeds 45°.
- Found while writing the importer (`mind/kilo/tricks/`, details in
  `mind/README.md` "Tricks"):
  - `loopCycle[2]` counts **total** plays of `loop_start..loop_end`: 0 and 1
    both play it once, N plays it N times, and ≥ 129 wraps the legacy `int8`
    counter negative and loops forever. A `loop_end` of 0 never loops. The
    importer copies the count into `TRICK_BEGIN.repeats` (255 = forever), so
    the body must read `repeats` as total plays, not extra ones.
  - Legacy `perform()` never balances during a behaviour; only postures and
    gaits get `adjust()`. Imported behaviours have `BALANCE` off.
  - Six of the thirteen tricks ask for angles beyond the joint limits (`sit`,
    `hi`, `wsf`, `scrh`: back shoulders to −85/−90; `buttUp`, `wsf`: front
    shoulders to 87/90; `str`: head tilt 70 > 45). Legacy clamps them in
    `calibratedPWM()`; the importer clamps at import and notes it.
  - The legacy file also has a `stand` array: a 6-frame behaviour, not the
    standing posture. "stand" in the first-version list is `up`.
  - Frame 0 moves from wherever the robot is, so its legacy speed has no
    fixed `maxΔ`; the importer sizes it from `up`.
  - `Skill::info()` prints the header as "pitch,roll"; the order is roll,
    pitch (`expectedRollPitch[0]` pairs with `ypr[2]`, roll). Mirroring is
    selected by a trailing `R` on the skill name (or at random for `X`);
    there is no mirror token (`m` is the indexed joint move).
- Uploader and player (P1, implemented; `mind/README.md` "Tricks"):
  `BodyClient.upload_trick()` (cached: a local record, then `TRICK_COMMIT`
  alone as the body-side check), `await body.play(name_or_trick, mirror,
  speed)` resolving on `TRICK_DONE` with its reason (timeout = the expected
  duration from `kilo/tricks/player.py` + 1 s), `stop_trick()`, and `kilo
  trick list|show|upload|play|stop`. `play()` releases the latched posture
  goal: the trick's end posture replaces it.
- Authoring uses a curses keyframe editor that jogs joints live. G41 servos
  have no position feedback, so poses cannot be shaped by hand.

**Perception.**
- **Ultrasonic** (12.5 Hz, from telemetry): median of 3, approach speed, and
  `near`, `approaching` and `clear` events.
- **IMU:** picked up, tap/pet, tilted.
- **Camera:** runs in its own process. It captures 320×240 Y-plane frames with
  picamera2 (CSI) or OpenCV V4L2 (USB) and sends detections over a pipe. Rates
  on a Pi 2 B **(est., measured in P4)**:

  | Detector | Rate |
  |---|---|
  | Motion differencing (160×120) | 15-25 fps |
  | HSV colour blob | 10-20 fps |
  | Haar frontal face (320×240) | 3-6 fps |
  | MobileNet-SSD | ≈0.5-1 fps; only for slow "who is there" checks |

  Head tracking is a P-loop on the detection centre → `SET_HEAD`, so expect
  gentle tracking, not snapping.
- **Microphone (later):** loudness and onset detection only.

**Logging.** Frames go to `~/kilo-logs/*.kilolog.gz` (≈1 MB/h at 25 Hz;
`kilo --kilolog`, `kilo run --kilolog`), replayable through the codec. A JSONL
behaviour log records drives and the active path; the daemon also keeps
`~/kilo-logs/daemon.log`. `kilo log summary|replay|plot` analyses offline. Together these close the loop for
learning control: change a gain, walk, plot.

**Flashing tool.** `kilo flash` builds with arduino-cli, runs the size/symbol
harness and refuses to flash if it fails, embeds the git hash, releases the
port, resets the body over GPIO4, runs avrdude, and verifies by HELLO. Details
are in Appendix F.

**Mind integration backlog (BodyClient).** What the behaviour layer needs from
`BodyClient` that it works around today (2026-10-10):

- `look(pan, tilt)`, `beep(notes)` and gait `walk(...)` methods, once the body
  implements SET_HEAD, BEEP-from-the-mind and SET_GAIT; `BodyAdapter` already
  calls them when present and counts them in `unsupported` otherwise.
- An event callback for epoch change, FELL, WATCHDOG and LOW_BATTERY (and
  connection changes). Done for now on the runner side without touching
  `BodyClient`: `kilo.behaviour.body_events.BodyEvents` diffs successive
  snapshots (epoch, the last 8 EVENTs, connected/alive) and the runner logs
  `body_event` records and calls `on_body_event`. A real callback
  (`SerialLink.subscribe("event"/"goals_cleared")` re-exported by
  `BodyClient`) would remove the 10 Hz polling window.
- The battery level enum (OK/WARN/LOW/CRITICAL, as in EVENT LOW_BATTERY) and
  the current posture id in the snapshot, instead of the millivolt percentage
  and the mode name the behaviours infer them from.
- A cheap snapshot for 10 Hz polling: `BodyState.snapshot()` builds the full
  dict (link stats, HELLO fields, events) every call; a slim variant (attitude,
  mode, epoch, flags, distance, battery, alive) would do for the runner.

---

## 6. Testing and verification strategy

**Host unit tests (body core).** `body/test/` builds `core/` with
`g++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined`, plus the
bare-type lint. On 32-bit ARM (the Pi 2 B) libasan needs `-latomic` for its
64-bit atomics; `body/tools/host_tests.sh` and `protocol/test.sh` probe for that
once and add it only where a plain sanitizer link fails. It covers:
- fx accuracy;
- the per-robot legacy transfer function vs a transcription of the legacy float
  code, fed the dumped EEPROM;
- IK round trips;
- gait phase, swing order and continuity;
- keyframe timing;
- arbitration and crossfades;
- the balance closed-loop sign test;
- reflex timelines, including epoch bumps;
- config-slot torn-write recovery;
- the codec against vectors, plus fuzzing.

Appendix H has the per-unit criteria.

**Exact AVR semantics.** The same tests are built with avr-g++ and run under
simavr. This catches 16-bit `int` and promotion bugs, and gives real cycle
counts that replace the estimates in 3a.

**Virtual body.** `body/test/vbody` links `core/` with stub HALs (scripted IMU,
fake ultrasonic, recorded servo counts) and speaks the real protocol on a pty.
The mind runs against it unchanged on a desktop or the Pi. Integration tests
cover:
- watchdog stages;
- epoch refusal after a simulated fall;
- trick upload and play;
- ACK de-duplication;
- reboot detection.

**Python tests.** pytest covers:
- the codec vs `protocol/vectors/*.json`;
- property-based round trips of every message;
- the CRC check value;
- the legacy importer, including frame counts, speed-0 handling and mirror;
- the behaviour-tree engine;
- link retries over a lossy fake transport;
- the clearing of desired goals on events.

**Protocol single source.** `gen.py` output is committed. CI regenerates it and
fails on any diff.

**Size and symbol harness.** `body/tools/size_check.sh` compiles `body/` (or
checks an existing ELF with `--elf`, which is what `kilo flash` does) against
the phase ceilings `FLASH_MAX` and `GLOBALS_MAX` (3h). It fails if any
forbidden symbol is linked: `HardwareSerial`/`Print`, `TwoWire`, I2Cdev,
`pulseIn`, `tone`, the heap, `String`, soft float or libm. If the sanctioned
float IK fallback is ever used, the float symbols are allowed explicitly
there. It prints the largest symbols on every run.

**Virtual body vs real body.** `mind/tests/test_crosscheck.py` builds the
body's real dispatch (`core/link.cpp` and `core/`) for the host
(`body/tools/build_link_harness.sh`), plays every P0 vector and a set of
sequences (de-dup, params, goals, calibration, reboot) through it and through
`kilo.vbody`, and requires the same replies, statuses and events.

**Hardware-in-the-loop on Kilo.** Checklist H1-H15 (Appendix H) runs per phase,
and the results go in the merge commit message. Highlights:
- H1: flash gives `FLASHED`;
- H2: PCA counts match legacy;
- H5: the watchdog timeline;
- H6: a hang gives `WARM_UNEXPECTED` with no limb jump;
- H7: fall recovery;
- H9: tick timing.

---

## 7. Phased plan

Order: talking and standing under Pi control (P0), today's chosen tricks (P1),
walking (P2). `kilo flash --legacy` stays available throughout. `OpenCat.ino`
and `src/` are deleted after P2 is accepted.

**P0: protocol + skeleton body.**
- **Deliverables:**
  - The `protocol/` spec, generator and vectors.
  - Body:
    - UART, COBS, CRC;
    - I2C, PCA9685 with read-back, DMP init (pre-watchdog) and FIFO read;
    - the **fx library and integer attitude** (no float anywhere);
    - the `.noinit` reset record;
    - config slots with legacy import;
    - built-in postures;
    - HELLO, TELEMETRY (with epoch), EVENT, ACK with de-dup, STATS,
      PCA_COUNTS;
    - calibration, params, `EEPROM_READ`;
    - the two-stage watchdog;
    - battery reflex with hysteresis;
    - the hardware watchdog.
  - Mind: codec, `SerialLink` (three threads), `BodyClient` (postures +
    epoch), CLI `status/monitor/posture/cal/param/flash`.
  - Host tests, virtual body, size harness.
  - **Task: dump the legacy EEPROM 0..282 and commit
    `docs/kilo-eeprom-legacy.md`.**
- **Removed vs today:**
  - the token protocol and parser, and `newCmd` (468 B RAM);
  - the skill system and I2C-EEPROM skills;
  - setup-sketch paths, TaskQueue and ultrasonic reactions;
  - the RGB effects engine;
  - the IR remote library (≈2 KB, 147 B RAM, 20 kHz ISR); replaced
    2026-10-11, at the owner's request, by a ≈450 B NEC decoder on the
    shared pin-change interrupt that relays keys to the mind (`EVENT IR_KEY`, 3a);
  - HardwareSerial, Print, Wire and I2Cdev;
  - the float library, including float quaternion maths (≈1.3 KB) and
    `beep()` floats.
- **Acceptance:** H1-H6, H8, H10. Flash ≤ 20 KB, globals ≤ 1540 B. REST →
  STAND → SIT → REST from the Pi.

**P1: motor loop, arbitration, tricks, balance.**
- **Deliverables:**
  - the full tick;
  - owners, crossfade, head layer, safety stage, goal epoch;
  - keyframe store, player and upload, with mirror and speed;
  - joint-space balance;
  - the fall reflex with `rc`;
  - ultrasonic via PCINT, eyes;
  - the buzzer (`BEEP` queue + reflex chirps);
  - mind: trick model, importer, the first-version library, uploader, editor,
    logger.
  - Optional baseline: legacy `wkF` subsampled to 22 frames as a looping
    trick.
- **Acceptance:** H7, H9 (looping trick), H11-H12 (H13 moved to P2, see
  §3f "Proportional-only limits H13"). Each first-version trick
  matches the old firmware side by side, in the owner's judgement. Attitude
  agrees with the legacy `ypr` within 1° static (A/B). Flash ≤ 24 KB
  (22 KB until 2026-10-11).
- **Status:**
  - Joint-space balance: done on the host (3c layer 2, 3f): controller,
    legacy gain table, gating, `SET_BALANCE`, TELEMETRY `BALANCE_ACTIVE`,
    `kilo balance`, the vbody mirror. Hardware validation:
    `docs/p1-hardware-test.md` (H13 and the sign check).
  - Attitude A/B: the host half passes (0.038° max over 5000 orientations
    within ±60°, against the legacy float path); a static side-by-side on
    Kilo is in the P1 hardware test.
  - Trick player: done on the host (§3e): store, upload with CRC and
    cache check, player with easing, holds, loops, speed and mirror, the
    TRICK owner and its crossfades, end posture, TRICK_DONE reasons,
    gating, per-frame balance, FALL_OK; the mind's uploader and `play()`,
    `kilo trick`, the vbody mirror and the C-vs-vbody cross-check
    (TRICK_DONE on the same tick). `body/test/test_keyframe.cpp` replays all
    13 library tricks (also mirrored and at 1.5×): angles within the joint
    limits, frame timing exactly as the JSON. Hardware: the "P1 tricks
    hardware test" in `docs/p1-hardware-test.md` (side by side with the
    legacy firmware, H9 with `push_up --repeats 255`).
  - Flash 21246 B of 22528 B (the ceiling then); globals 1525 B of 1540 B.
    Now 22132 B of 24576 B, 1433 B globals (§3h).

**P2: gait + IK.**
- **Deliverables:** measured geometry; IK, gait, STANCE, geometric balance, bob
  and wag; `kilo walk` with keyboard driving.
- **Acceptance:** H14, H9 while trotting, the IK/gait tests under simavr. WALK,
  TROT and CRAWL start and stop with no visible foot jump on slow-motion video.
  Flash ≤ 24 KB.
- **Then cut over:** delete the legacy firmware (its hex is kept in
  `mind/firmware/`).

**P3: mind v1 (ultrasonic + IMU personality).**
- **Deliverables:** the behaviour-tree engine, drives, personality YAML, and
  behaviours: explore, nap, startle, greet, pick-up and tap reactions, battery
  wind-down with clean Pi shutdown. A systemd service.
- **Acceptance:** 30 min autonomous with ≤ 2 obstacle contacts; responds to an
  approach within 0.5 s; on CRITICAL battery the Pi shuts down before the body
  limps. No firmware change, or only a P2.x patch.

**P4: camera.**
- **Deliverables:** the detector process and head tracking, with face seen →
  social drive.
- **Acceptance:** measured detector rates documented; tracking with ≤ 0.5 s lag
  at ≥ 4 Hz; Pi CPU < 70%; power logged.

**P5: DMP removal.**
- **Deliverables:** a raw 14 B read and the integer complementary filter
  (Appendix A), behind A/B flag `KILO_IMU_DMP`.
- **Removed:** the DMP image (3062 B) and init/FIFO code (≈0.9 KB); the filter
  adds back ≈0.5 KB. Net ≈ −3.5 KB, and a ≈300 ms faster boot.
- **Acceptance:** static within 1°, trot within 3° RMS of the DMP; fall latency
  no worse; H7, H9 and H13 re-run.

---

## 8. Open questions for the owner

1. **Leg geometry (blocks P2).** Measure L1, L2, the foot contact offset, the
   shoulder spacing Lb and the foot track W (Appendix B). Confirm that the
   calibration posture is "upper leg vertical, lower leg at 90°".
2. **Battery.** Do one meter reading against the reported `batt_mv` to fix the
   scale. Is a higher-drain pack planned? Petoi warns the stock 1000 mAh pack
   struggles with a Pi aboard, and the thresholds depend on it.
3. **UART and reset wiring.**
   - Measure the TX/RX idle levels on the Pi header with a meter. The Pi's
     GPIO15 must never see 5 V.
   - Is there a capacitor between GPIO4 and RESET? The open-drain reset works
     either way.
4. **Camera.** CSI (which version) or USB? Head- or body-mounted?
5. **Autonomy.** How free should Kilo be: "explore the floor" or "only move
   when asked"? Nybble has no cliff sensor, so this decides whether tables are
   off-limits.
6. **Logging.** Keep 50 Hz telemetry plus JOINTS always on for learning
   (≈2-3 MB/h), or only on demand?
7. **I2C EEPROM (64 KB).** Unused. Keep it reserved, or later use it as a
   body-side log of falls and resets?
8. **Network.** The Pi 2 has no Wi-Fi. USB Wi-Fi dongle (+0.5-1 W) or Ethernet
   for development?
9. ~~Flash ceilings.~~ Resolved 2026-10-10: the ceilings were estimates and
   the measured P0 build is 18164 B, so they are now P0 20 KB, P1 22 KB,
   P2 24 KB (still 7.5 KB under the 31.5 KB available before P5's ≈3.5 KB
   DMP saving). Ultrasonic and buzzer stay in P0. 2026-10-11: P1 raised to
   24 KB, and the body is built with `-mcall-prologues -Wl,--relax` (§3h).

---

## Appendix A: DMP-free attitude filter (P5)

Each tick, read 14 B from `ACCEL_XOUT_H` (0x3B), ≈0.4 ms at 400 kHz.

Configuration:
- `CONFIG` DLPF = 3 (44/42 Hz) for anti-aliasing;
- gyro ±500°/s (65.5 LSB per °/s);
- accel ±2 g (16384 LSB/g);
- offsets from the config slot, written to the offset registers at boot.

Angles are held per axis as int32 `a16 << 8`:

```
gyro increment per tick (a16):  Δ = raw · (16/65.5) · 0.02 = raw · 0.004885 ≈ raw · 5/1024   (0.04% error)
accel angle: pitch_acc = atan2(−ax, isqrt(ay² + az²)), roll_acc = atan2(ay, az)   (normalised integer atan2, Appendix B)
             then apply the same gz<0 pitch reflection as 3f
predict:     θ += Δ
correct:     if | |a| − 1 g | < 0.15 g:  θ += (θ_acc − θ) >> 6          // α = 1 − 1/64 ≈ 0.984
```

- The time constant is τ = 0.02·63 ≈ 1.26 s.
- The magnitude gate skips accelerometer correction during impacts.
- Gyro bias is averaged at boot when still. Rates come from the gyro directly.
- Yaw is not estimated (no magnetometer); the Pi integrates `rate_yaw` if it
  needs heading over short periods.
- Cost **(est.)**: 0.5 KB flash, 24 B RAM, ≈0.6 ms per tick.
- The Pi can replay logged IMU_RAW through a Python reference filter to tune α
  and the gate before touching firmware.

---

## Appendix B: Derivations

**B1. PCA9685 burst.** Registers `LED0_ON_L` (0x06) to `LED15_OFF_H` (0x45)
auto-increment. Address + register + 64 data bytes = 66 bytes × 9 bit-times =
594 bits, which is 1.49 ms at 400 kHz, plus ≈0.1 ms overhead. The legacy code
does one `setPWM` transaction per servo through Wire (≈0.2 ms each, 2-3 ms in
total) and runs the bus at 500 kHz, above the MPU6050's 400 kHz rating. The
new driver is polled, with a 2 ms timeout per transaction; on timeout it clocks
9 SCL pulses and a STOP, counts `i2c_err`, and skips the step.

**B2. DMP FIFO read.**
- `SMPLRT_DIV = 4` gives a 200 Hz sample rate, and the image's FIFO divisor 1
  gives ≈100 Hz packets of 28 B. Verify by counting.
- Each tick: read `FIFO_COUNT` (2 B), compute `n = count / 28`, read `n·28`
  bytes in ≤ 28 B transactions, and keep the last packet. There is no
  busy-wait for a packet; if `n = 0`, the previous attitude is reused and
  `imu_stale` increments.
- Reset the FIFO if `count % 28 ≠ 0` for 3 ticks in a row, or if
  `count > 200`.
- At 100 Hz there are ≈2 packets per tick: 2 + 56 bytes plus addressing ≈
  0.6 kbit ≈ 1.5 ms with overhead. If the rate turns out to be 200 Hz (4 packets,
  ≈2.8 ms per tick), raise the divisor in the image config or accept the
  longer tick (still < 7 ms total).

**B3. Ultrasonic.**
1. Every 4th tick (80 ms, above the ≈60 ms ring-down of HC-SR04-style
   modules), set D6 as output with `sbi`, high for 20 µs, then input with
   `PCINT22` enabled.
2. The ISR records the rising and falling `micros()` (4 µs resolution =
   0.7 mm) and disables itself.
3. No falling edge within 25 ms (≈4.3 m) → `dist_mm = 0`.
4. Distance = `echo_us · 1715 / 10000` mm (343 m/s, round trip).

Jitter sources are the UART RX ISR (≈3 µs), the buzzer ISR (≈2-3 µs) and
Timer0 (≈5 µs every 1.024 ms): ≤ ≈2 mm in total. The WS2812 update (180 µs with
interrupts off) is never scheduled during an echo. Legacy `pulseIn` blocked for
up to the echo length (≈38 ms with no obstacle), or 1 s if the echo never
started.

**B4. WS2812.** 6 LEDs × 24 bits × 1.25 µs = 180 µs with interrupts off. The
USART holds ≈2 received bytes. At 250000 baud (40 µs per byte), 2-3 bytes can
be lost, which is one frame, caught by the CRC.

**B5. Bandwidth.** Wire size = payload + 6.

| Direction | Stream | Wire B | Rate | B/s |
|---|---|---|---|---|
| Up | TELEMETRY | 32 | 25 / 50 Hz | 800 / 1600 |
| Up | JOINTS | 30 | 50 Hz max | 1500 |
| Up | IMU_RAW | 20 | 50 Hz max | 1000 |
| Up | ACK/EVENT/LOG | | bursty | ≤ 600 |
| Up | **Worst** | | | **≈4.7 kB/s = 19% of 25 kB/s** |
| Down | GAIT 17 + HEAD 16 + POSE 13 + EYES 10 | | 5 Hz (worst 50 Hz) | 280 (2800) |
| Down | HEARTBEAT | 6 | 10 Hz | 60 |
| Down | Trick upload, stop-and-wait | 64 | ≤ 1 per 10 ms | ≤ 6400 |
| Down | **Worst** | | | **≈9.3 kB/s = 37%** |

At 115200 the downstream worst case would be ≈80%.

**B6. Fixed-point plan (IK and attitude).**
- **Lengths** are `q2`: a 50 mm link = 200. `d² ≤ 2·400² = 320 000` (int32).
- **Knee:** `num = d² − L1² − L2²`, with |num| ≤ 2·L1·L2 ≈ 80 000. Then
  `c = num·4096/den` is Q12; num·4096 ≤ 3.3e8, which fits int32. Clamp `c` to
  ±4096. `s = isqrt(4096² − c²)` is Q12, from an argument of at most 1.7e7.
- **atan2(y, x) → a16** (all int32 inputs):
  1. Take absolute values.
  2. Shift **both** right by the same amount until `max(|y|, |x|) < 2^15`.
  3. `r = (min << 14) / max` is Q14 in [0, 1]. min < 2^15, so `min << 14` <
     2^29 fits.
  4. Index = `r >> 8` (0..64), fraction = `r & 255`; linearly interpolate the
     65-entry atan table (0..720 a16, 130 B).
  5. Rebuild octant and quadrant.

  Maximum interpolation error ≈ (1/64)²/8 · 0.65 rad ≈ 0.001°. The second IK
  atan2 takes `(L2·s, L1·4096 + L2·c)`, values up to ≈3.3e6, which step 2
  normalises. No extra division is needed, since atan2 is scale-invariant.
- **Sine:** a 65-entry Q14 quarter-wave table (130 B), linearly interpolated,
  error < 1e-4. Input is a16.
- **isqrt32:** bitwise, 16 iterations, ≈60 B.
- **Near full extension:** an error in `c` is amplified by `c/s`. The reach
  clamp at 0.95(L1+L2) keeps the knee bend ≥ ≈25°, so the worst θ2 error is
  < 0.05° **(est.)**.
- **Attitude:** quaternion Q14 (int16) → products Q28 (int32) → sums shifted
  `>> 14` to Q14 → squares Q28, with sum ≤ 2^29 → `isqrt` → Q14 → atan2 as
  above.
- **Cost (est.):** per leg, 3 atan2 (one int32 division each, ≈40 µs) + 1 isqrt
  + 1 division ≈ 0.2 ms, so 0.8 ms per tick for four legs. Flash: tables 260 B,
  fx ≈400 B, IK ≈400 B, gait ≈1 KB.
- **Fallback:** float IK costs ≈1.6 KB of library and 2-3 ms per tick
  **(est.)**.

**B7. Measuring leg geometry.**
- Put Kilo in `calib`. With calipers, measure:
  - shoulder axle centre to knee axle centre (L1);
  - knee axle centre to the floor contact point (L2). If the tip is not on the
    knee-to-tip line, also the offset angle δ, added to θ2.
- Measure front-to-back shoulder-axle spacing (Lb) and the left-right foot
  track (W).
- Rough guesses, **not to be shipped:** L1 ≈ L2 ≈ 45-55 mm, Lb ≈ 100-110 mm,
  W ≈ 55-65 mm.

---

## Appendix C: Legacy archaeology

**C1. Leg order LF, RF, RB, LB** for 8-11 and 12-15. `OpenCat.h` does not state
it. Three independent clues:
1. `adjust()` uses `leftQ = (i − 1) % 4 > 1`, which gives 8 = left, 9 = right,
   10 = right, 11 = left.
2. `Skill::mirror()` swaps columns (8,9) and (10,11), so these are the L/R
   pairs.
3. Every posture in `InstinctNybble.h` gives equal values to 8 and 9 and to 10
   and 11 (`rest`: 60, 60, −60, −60), so 8 and 9 are the front pair. The front
   and back limits are mirror images.

**C2. The setup sketch's transfer function.**
- `setPWMFreq(240)` with the 25 MHz nominal oscillator gives prescale 24, so
  244 Hz, a 4.096 ms period and 1 count ≈ 1 µs.
- `ANGLE2PULSE_FACTOR = (2500−500)/180` is integer division (= 11) × 1.0 ×
  1000 = 11000.
- `B_OFFSET = 240/50` (integer division, = 4) × 150 = 600 → stored as 60.
- `zeroPosition = 90 + midShift·dir` and
  `calibratedZeroPosition = zero + calib·dir` are stored as ints.
- So neutral (servo 90°) is ≈1590 counts, not 1500, and the slope is 11
  counts/° rather than 11.1. Kilo's calibration absorbed this, and that is why
  the new firmware imports the stored values instead of recomputing them (3b).
- The `P_SOFT`/`P_HARD` stiffness pulses (`setServoP`) are dropped.

**C3. Legacy balance (`motion.h` `adjust()`).**
- **Error:** `dev = ypr − expectedRollPitch`, with dead bands of 5° roll and 3°
  pitch subtracted using `sign(ypr)`.
- **Gains:** per-joint `{rollFactor, pitchFactor}` / 57.3, giving degrees of
  joint per degree of tilt. Upper legs: roll 1.05, pitch 0.52. Lower legs:
  −1.5×. Head: pan −60, tilt −60, tail −120.
- **Roll term:** uses `|roll|` with signed factors and 1.2× on one side, so it
  is mostly a symmetric crouch with 20% asymmetry (as I read it).
- **Walking:** leg corrections are halved (`POSTURE_WALKING_FACTOR 0.5`).
- **Damping:** the change per update is limited to ±5° (`ADJUSTMENT_DAMPER`).
  This is a slew limit, not a derivative term.
- **Application:** `skill.h` writes `frame + adjust` (`calibratedPWM(j, duty +
  adjust(j))`); `reaction.h`'s `currentAng − currentAdjust` only undoes that
  to recover the unadjusted frame. The new firmware adds its mapping the same
  way (3f).
- **Dead band quirk:** the band is subtracted with the sign of the raw `ypr`,
  not of the deviation; the two differ only when the expected angle is non-zero
  (`sit`). The new firmware uses the sign of the deviation.
- **Sign convention:** the skill headers and the pitch gains assume
  nose-down = +pitch, which is what `ypr` (and Kilo) reads. The roll gains
  correct a left-side-up tilt on positive roll, but `ypr` reads that as
  negative roll, so legacy roll balance was wrong-signed (weakly: only the
  20% left/right differential); Kilo negates it. See 3f "Sign conventions".
- **Rate:** the IMU is read on every 3rd joint write, so the rate depends on
  loop speed.
- It is proportional-only, in joint space.

**C4. Legacy EEPROM items imported by migration (3b).**

| Legacy item | Address | Size |
|---|---|---|
| `CALIBRATED_ZERO_POSITIONS` (CZP) | 96 | int16 × 16, servo degrees |
| `ANGLE2PULSE_FACTOR` (A2P) | 160 | int16 × 16, counts/° × 1000 |
| `B_OFFSET` | 274 | u8, counts / 10 |
| `PCA9685_FREQ` | 275 | u16, kHz |
| `PWM_PIN` | 0 | 16 B |
| `CALIB` | 16 | 16 B |
| `MID_SHIFT` | 32 | 16 B |
| `ROTATION_DIRECTION` | 48 | 16 B |
| `MPUCALIB` | 256 | int16 × 6: accel xyz, gyro xyz offset registers |

**C5. Other legacy facts used.**
- Battery: `LOW_VOLTAGE 650` and the `voltage/99` printout. Full (8.4 V) ≈ 832
  counts, nominal (7.4 V) ≈ 733.
- Behaviour frame counts: `cmh` 22, `dg` 11, `hds` 10, others ≤ 9. No Nybble
  trick uses the IMU-trigger columns.
- Legacy EEPROM usage: 0..282 (config), 320..700 (47 skill entries of
  nameLen + 5 B, 381 B), ≈966..1023 (melodies).

---

## Appendix D: Memory estimates

**D1. Today's build, calibration points** (mega @ 684d0eb: 32016 B, 1620 B
globals):

| Symbol / area | Flash |
|---|---|
| `dmpMemory` | 3062 B |
| `read_IMU()` (float quaternion → ypr) | 1348 B |
| `beep()` (float) | 1236 B |
| `SetRgbEffect` | 970 B |
| Skill loading (4 functions) | ≈1.6 KB |
| `transform` + `calibratedPWM` + `writeAngle` | ≈1.2 KB |
| IR Timer1 ISR | 474 B + decoder; `irparams` 147 B RAM |
| Serial object | 157 B RAM |
| `newCmd` | 468 B RAM |

From the brief: servo path ≈2.1 KB, IMU ≈4.2 KB + image, Wire ≈2.3 KB,
Serial/Print ≈1.4 KB, float library ≈1.6 KB.

**D2. New firmware (all est.; S = soft):**

| Component | Flash | RAM |
|---|---|---|
| Arduino core subset + vectors + crt | 1.0 KB | 15 B |
| UART driver | 0.3 KB | 192 + 96 + 6 B |
| COBS, CRC, codec, dispatch, ACK de-dup cache | 1.5 KB S | 128 + 3 B |
| I2C polled driver | 0.35 KB | 4 B |
| PCA9685 driver + read-back | 0.4 KB | 32 B |
| DMP image / DMP init + FIFO | 3.06 KB / 0.9 KB S | 44 B |
| Attitude (integer) | 0.3 KB | 12 B |
| fx (atan2, sin, isqrt, tables) | 0.7 KB | 0 |
| IK / gait | 0.4 KB / 1.0 KB S | 40 B |
| Keyframe store + player (P1 measured: +1.7 KB flash incl. dispatch; no `from[11]`) | 0.6 KB | 448 + 24 B |
| Arbiter, layers, safety, goal registers + epoch | 0.9 KB S | 120 B |
| Balance | 0.4 KB | 16 B |
| Reflexes | 0.7 KB S | 24 B |
| PROGMEM postures + `rc` | 0.15 KB | 0 |
| Config slots, migration, writer (RAM image) | 0.7 KB | 108 B |
| Params | 0.3 KB | 40 B |
| Ultrasonic / eyes / battery | 0.25 / 0.25 / 0.1 KB | 10 / 18 / 6 B |
| Telemetry, stats / event queue (8 × 5) | 0.5 KB | 30 / 40 B |
| Buzzer (Timer2 ISR, note queue, chirps) | 0.4 KB | 20 B |
| `.noinit` reset record | 0.1 KB | 9 B |
| **Total** | **≈15.4 KB** (S → 18 KB) | **globals ≈1.50 KB** |
| Stack (painted, worst case) | | 350 B |

**D3. Measured at the P0 integration** (avr-gcc 7.3, -Os, LTO): 18164 B flash,
1061 B globals. Of that, the link layer (frame codec ≈0.7 KB, dispatch of every
P0 message, HELLO, TELEMETRY/JOINTS/IMU_RAW, the event queue, STATS, params)
is ≈4.5 KB, about twice the 2.3 KB of the three D2 rows above; the rest
(13.6 KB) was measured before the link existed. See 3h and §8 question 9.

---

## Appendix E: Protocol reference

`protocol/spec.toml` is authoritative. If this table disagrees with it, fix
this table. Offsets are within the payload. `E` = epoch u8.

**Pi → body**

| Type | Payload | B |
|---|---|---|
| 0x01 HELLO_REQ, 0x02 HEARTBEAT, 0x05 STATS_REQ, 0x06 PCA_READ, 0x38 CAL_GET, 0x3A CAL_SAVE, 0x3B CAL_REVERT | (empty) | 0 |
| 0x03 PING | `token u16` | 2 |
| 0x04 SET_STREAM | `telem_div u8` (0 off, default 2 → 25 Hz), `joints_div u8`, `imu_raw_div u8` | 3 |
| 0x10 SET_POSTURE | `E`, `posture u8` (0 LIMP, 1 REST, 2 SIT, 3 STAND, 4 STANCE, 5 CALIB, 6 ZERO), `transition_ticks u8` (0 = param `POSTURE_TICKS`, 25), `flags u8` (bit 0 balance; BALANCE = STAND + this bit) | 4 |
| 0x11 SET_GAIT | `E`, `run u8`, `pattern u8`, `period_ms u16`, `duty u8`, `stride_mm i8`, `turn_mm i8`, `step_h_mm u8`, `body_h_mm u8`, `flags u8` (bit 0 bob, 1 wag, 2 balance) | 11 |
| 0x12 SET_HEAD | `E`, `mode u8` (0 release, 1 hold), `pan i16`, `tilt i16`, `tail i16` (a16; −32768 = not held), `max_speed_dps u16` (0 = safety slew only) | 10 |
| 0x13 SET_BODY_POSE | `E`, `pitch i16`, `roll i16` (a16), `dheight_mm i8`, `xshift_mm i8` | 7 |
| 0x14 SET_BALANCE | `enable u8`, `kp_q8 u8`, `kd_ms u8`, `deadband_deg u8`, `max_corr_deg u8` | 5 |
| 0x20 TRICK_BEGIN | `trick_id u8`, `n_frames u8` (1..24), `loop_start u8`, `loop_end u8`, `repeats u8` (total plays; 0/1 once, 255 forever), `joint_mask u16`, `flags u8` (bit 0 balance, 1 fall_ok), `end_posture u8`, `exp_roll i8`, `exp_pitch i8` (deg, balance reference) | 11 |
| 0x21 TRICK_FRAMES | `first u8`, `count u8` (1..4), `count × 14 B` | 2+14n |
| 0x22 TRICK_COMMIT | `trick_id u8`, `crc16 u16` | 3 |
| 0x23 TRICK_PLAY | `E`, `trick_id u8`, `speed_q6 u8`, `flags u8` (bit 0 mirror) | 4 |
| 0x24 TRICK_STOP | `transition_ticks u8` | 1 |
| 0x30 SET_EYES | `led_mask u8` (0x07 right, 0x38 left, 0x3F all), `r u8`, `g u8`, `b u8` | 4 |
| 0x31 BEEP | `n u8` (1..8), n × (`note i8`, `dur_10ms u8`) | 1+2n |
| 0x39 CAL_SET | `joint u8`, `offset_a16 i16` (absolute, ±25°) | 3 |
| 0x3C IMU_OFFSETS_SET | 6 × `i16`, `save u8` | 13 |
| 0x3D PARAM_GET | `id u8` | 1 |
| 0x3E PARAM_SET | `id u8`, `value i16`, `persist u8` | 4 |
| 0x3F EEPROM_READ | `addr u16`, `len u8` (≤ 56) | 3 |
| 0x7E REBOOT | `magic u16 = 0x4B49` | 2 |

**Body → Pi**

| Type | Payload | B |
|---|---|---|
| 0x80 HELLO | `proto_major u8`, `proto_minor u8`, `fw_build u32`, `reset_cause u8` (0 COLD, 1 FLASHED, 2 REQUESTED, 3 WARM_UNEXPECTED), `last_alive_tick u16`, `caps u16` (bit 0 buzzer, 1 IR receiver, 2 DMP, 3 ultrasonic, 4 eyes), `max_payload u8`, `trick_capacity u8` (0 in P0: no store; 32 from P1), `joint_count u8` (11), `config_state u8` (0 ok, 1 migrated, 2 defaulted), `epoch u8` | 16 |
| 0x81 TELEMETRY | §4 | 26 |
| 0x82 JOINTS | `tick u16`, 11 × `i16` a16 (−32768 = limp) | 24 |
| 0x83 IMU_RAW | `tick u16`, 6 × `i16` | 14 |
| 0x84 ACK | `seq u8`, `type u8`, `status u8` | 3 |
| 0x85 EVENT | `code u8`, `arg i16`, `tick u16`, `epoch u8` | 6 |
| 0x86 CAL_VALUES | 16 × `i16` a16 | 32 |
| 0x87 PARAM_VALUE | `id u8`, `value`, `min`, `max`, `default` (i16) | 9 |
| 0x88 EEPROM_DATA | `addr u16`, bytes | 2+len |
| 0x89 PONG | `token u16`, `tick u16` | 4 |
| 0x8A STATS | `uptime_s u32`, `overruns u16`, `max_tick_us u16`, `stack_free_min u16`, `rx_crc_err u16`, `rx_overflow u16`, `i2c_err u16`, `imu_stale u16`, `dmp_rate_hz u8` | 19 |
| 0x8B PCA_COUNTS | 16 × `u16` OFF counts (bit 12 = full-off) | 32 |
| 0x8F LOG | ASCII ≤ 60 | var |

**Event codes:**

| Code | Event |
|---|---|
| 1 | BOOT (arg = reset cause) |
| 2 | FELL (±1 roll, ±2 pitch) |
| 3 | RECOVERED |
| 4 | RECOVERY_FAILED |
| 5 | LOW_BATTERY (level) |
| 6 | BATTERY_OK |
| 7 | WATCHDOG |
| 8 | LINK_RESTORED |
| 9 | IMU_FAULT |
| 10 | IMU_OK |
| 11 | TRICK_DONE (id \| reason << 8; reason 0 DONE, 1 STOPPED, 2 PREEMPTED, 3 FELL) |
| 12 | GAIT_STOPPED |
| 13 | GOAL_REFUSED (type << 8 \| reason) |
| 14 | CONFIG_MIGRATED |
| 15 | CONFIG_DEFAULTED (reason) |
| 16 | EEPROM_SAVED |
| 17 | TX_DROPPED (rate-limited) |
| 18 | IR_KEY (NEC command byte \| 0x100 for a held-key repeat; proto 1.3) |

**Params** (`[[params]]` in spec.toml; stable ids, appended, never
renumbered; each id < 16 has a persisted slot in the config block, 0x7FFF =
default). The P0 body implements:

| Id | Name | Default | Min..max | Unit | Meaning |
|---|---|---|---|---|---|
| 0 | `WD_STOP_MS` | 500 | 100..5000 | ms | Watchdog stage 1 (stop, epoch bump) |
| 1 | `WD_REST_MS` | 1500 | 200..10000 | ms | Watchdog stage 2 (REST) |
| 2 | `WD_LIMP_S` | 10 | 1..600 | s | Watchdog REST → LIMP |
| 3 | `XFADE_TICKS` | 10 | 0..100 | tick | Crossfade out of TRICK or REFLEX |
| 4 | `POSTURE_TICKS` | 25 | 1..255 | tick | Default IDLE posture transition |
| 5 | `SLEW_MAX_A16` | 160 | 16..800 | a16/tick | Safety-stage slew limit |
| 6 | `BATT_WARN_MV` | 6900 | 5000..8400 | mV | WARN trip |
| 7 | `BATT_WARN_CLEAR_MV` | 7100 | 5000..8400 | mV | WARN clear |
| 8 | `BATT_LOW_MV` | 6600 | 5000..8400 | mV | LOW trip |
| 9 | `BATT_LOW_CLEAR_MV` | 7000 | 5000..8400 | mV | LOW clear |
| 10 | `BATT_CRIT_MV` | 6300 | 5000..8400 | mV | CRITICAL trip (latched) |
| 11 | `TELEM_DIV` | 2 | 0..50 | tick | TELEMETRY divisor at boot (`SET_STREAM` overrides) |
| 12 | `US_PERIOD_TICKS` | 4 | 3..50 | tick | Ultrasonic cadence |
| 13 | `LEG_L1_MM` | 50 (guess) | 20..120 | mm | Upper leg, shoulder axle to knee axle (B7) |
| 14 | `LEG_L2_MM` | 50 (guess) | 20..120 | mm | Lower leg, knee axle to floor contact |
| 15 | `LEG_LB_MM` | 105 (guess) | 50..200 | mm | Front-back shoulder spacing |
| 16 | `LEG_W_MM` | 60 (guess) | 30..150 | mm | Left-right foot track |
| 17 | `LEG_SH_ZERO_DEG` | 0 | -45..45 | deg | Upper link angle at joint angle 0 (θ1_0) |
| 18 | `LEG_KN_ZERO_DEG` | 90 | 30..150 | deg | Knee bend at joint angle 0 (θ2_0) |

The two clear levels replace the earlier single `BATT_HYST_MV`, because the
WARN and LOW hysteresis differ (200 and 400 mV). P1 appends `FALL_DEG` (75),
`BAL_KP_Q8`, `BAL_KD_MS`, `BAL_DB_A16`, `BAL_MAX_A16` and `HEAD_HOLD_MS`; P2
the geometry (`L1_Q2`, `L2_Q2`, `LB_MM`, `W_MM`) and `BOB_A16`, `WAG_A16`. The
legacy per-joint balance gains stay a PROGMEM table.

**Worked frames** (in `protocol/vectors/frames.json`, with one or more frames
per message type, generated by `gen.py`):

```
HELLO_REQ seq=0       decoded: 01 00 3e 2e                  wire: 02 01 03 3e 2e 00
SET_POSTURE seq=1     decoded: 10 01 00 03 00 01 b4 f7      wire: 03 10 01 02 03 04 01 b4 f7 00
  (epoch 0, STAND, transition_ticks 0 = default, flags = balance)
```

---

## Appendix F: Operations

**Pi configuration.**
- Serial login shell off, hardware serial on.
- 1-Wire off. The `w1-gpio` overlay claims GPIO4 and sends 480 µs low reset
  pulses while scanning the bus, and each one resets the NyBoard.
- UART clock 48 MHz (`init_uart_clock=48000000` if `vcgencmd` shows less).
- The I2C switch on the NyBoard stays on "Arduino".
- GPIO4 is left as an **input** (no pull) when idle: `gpio=4=ip,np`.

**Reset via GPIO4 (emulated open-drain).** To reset, drive GPIO4 low for
100 ms (`reset_pulse_ms`), then return it to input. That works whether GPIO4 is AC-coupled to
RESET (the falling edge resets) or wired directly (the low level holds reset).
It never drives the line high against the NyBoard's own reset circuit.

**Flash sequence (`kilo flash`):**
1. Ask the daemon to release `/dev/ttyAMA0` (the socket's `release_port` op,
   with a lease; the daemon takes the port back on `reacquire_port`, when the
   lease runs out, or when the flashing client disconnects). The port's
   exclusive flock keeps any other process off it. This happens after the
   build (step 2), so the body stays supervised while it compiles.
2. `arduino-cli compile --fqbn arduino:avr:uno --output-dir ~/.cache/kilo-build
   --build-property "compiler.cpp.extra_flags=-mcall-prologues -DKILO_BUILD=0x<hash>"
   --build-property compiler.c.extra_flags=-mcall-prologues
   --build-property compiler.c.elf.extra_flags=-Wl,--relax body/`
   (the size flags of §3h). About 1-3 min on a Pi 2 **(est.)**. The build
   hash goes in through
   `-DKILO_BUILD=0x<first 8 hex digits of the commit>`,
   and the body reports it as `HELLO.fw_build`. A plain arduino-cli build
   without it reports an FNV-1a hash of the compile date and time instead.
   The reset record mixes the compile time in either way, so rebuilding a
   dirty tree still boots as `FLASHED`.
3. Run the size/symbol harness on that ELF (`body/tools/size_check.sh --elf`):
   flash ≤ 24576 B, globals ≤ 1540 B, no forbidden symbol, built with the
   size flags. Stop on failure.
4. Pulse GPIO4 (`pinctrl`, libgpiod `gpioset`, or sysfs).
5. Within ≈200 ms, run
   `avrdude -p m328p -c arduino -P /dev/ttyAMA0 -b 115200 -D -U flash:w:body.ino.hex:i`.
   Optiboot only starts on an external reset and waits ≈1 s.
6. Settle the port (three production open/close cycles: link baud, DTR and
   RTS asserted, HUPCL cleared; mind/README.md "DTR and resets"), have the
   daemon reacquire it, and expect a HELLO received after the flash with
   `reset_cause = FLASHED` (any cause over a USB adapter) and `fw_build` equal
   to the build hash within 8 s (`--verify-timeout`; reset, optiboot, DMP init
   and the first HELLO take ≈2.2 s on Kilo). Print the HELLO seen either way.
   Retry once.

`kilo flash --legacy` writes `mind/firmware/opencat-legacy.hex`, which talks
the token protocol at 115200 and finds its EEPROM intact.

**Development over the CH343 USB adapter.**
- The Petoi uploader enumerates as `/dev/ttyACM0` or `/dev/ttyUSB0` on Linux
  (COM9 on the owner's Windows host, passed into WSL2 with usbipd).
- avrdude resets the board through DTR natively.
- DTR is AC-coupled to RESET, so every port open that raises DTR resets the
  328P. The mind clears HUPCL on each open (and again after avrdude, which
  re-enables it), so DTR stays asserted between opens and only the first open
  after a replug or a flash resets the body.
- On a Pi with the adapter plugged in, the mind's port auto-detection picks the
  adapter over the header UART (whose device node always exists) and warns;
  `KILO_PORT` or `--port` selects the header UART.
- The same `kilo` tools work with `--port`.
- The adapter and the Pi header share the 328P's single UART. **Unplug the
  adapter whenever the Pi drives the UART**, and vice versa.

---

## Appendix G: Repository layout and CLI

As of P0. Names in *italics* are planned for later phases.

```
body/                     the NyBoard firmware (arduino-cli sketch)
  body.ino                empty: main() is ours (src/app/main.cpp)
  src/core/               pure C++ logic, no Arduino/AVR headers, host-tested
                          fx  joints  posture  arbiter  attitude  battery  body (tick + reflexes)
                          config (slots, legacy import)  params  events  notes  reset  sched
                          link (dispatch, ACK/de-dup, HELLO, streams, events)  pgm.h
                          *ik  gait  keyframe  balance  fall reflex*
  src/hal/                register-level drivers: uart  i2c  pca9685  mpu6050 (+ dmp_image.h)
                          adc  buzzer  clock  eeprom  gpio.h  sys (.noinit record, WDT, stack paint)
                          ultrasonic  *ws2812*
  src/app/                main.cpp (boot, 50 Hz loop)  link_hal.cpp (core/link's hardware seam)
  src/proto/              VENDORED by protocol/vendor.py: kilo_protocol.h, cobs/crc16/kilo_frame.[ch]
  test/                   host unit tests (test_*.cpp), fakes (EEPROM, link HAL),
                          link_harness.cpp (stdin-driven dispatch for the cross-check)
  tools/                  host_tests.sh  size_check.sh  build_link_harness.sh
protocol/                 spec.toml  gen.py  vendor.py  vector_cases.py  test.sh
  gen/                    kilo_protocol.h  kilo_protocol.py   (generated, committed)
  c/                      cobs  crc16  kilo_frame  test_protocol.cpp
  py/                     kilo_link_codec.py  test_codec.py
  vectors/                frames.json  cobs.json              (generated, committed)
mind/                     the Python mind (pip install -e mind)
  pyproject.toml  README.md  firmware/ (opencat-legacy.hex)
  kilo/
    cli.py  log.py
    protocol/             VENDORED by protocol/vendor.py: kilo_protocol.py, kilo_link_codec.py
    link/                 codec.py (framing adapter)  messages.py (dataclass adapter)  consts.py
                          serial_link.py (reader/writer threads, goals, commands)  transport.py  ports.py
    body/                 state.py (BodyState)  client.py (BodyClient)
    tools/                flash.py  gpio.py (GPIO4 reset)
    vbody/                sim.py (virtual body on a pty)  __main__.py
    *daemon.py  tricks/  behaviour/  perception/*
  tests/                  pytest: codec, link rules, state, CLI, flash, vbody,
                          cross-check vs the C dispatch, CLI end to end on a pty
docs/                     architecture.md  raspberry-pi.md  p0-hardware-test.md  p1-hardware-test.md
OpenCat.ino, src/         the legacy firmware, until the P2 cutover
```

arduino-cli compiles `src/` recursively, which is why the logic lives under
`body/src/`.

**CLI.** `kilo` talks to the daemon over `/run/kilo/kilo.sock` (JSON lines), or
opens the port directly if the daemon is not running. As built
(`kilo/daemon.py`, `kilo/remote.py`): the socket is
`$XDG_RUNTIME_DIR/kilo/kilo.sock` for the systemd user service
(`/run/kilo/kilo.sock` only as root); protocol and ops in mind/README.md
"Daemon and systemd".

| Command | Purpose |
|---|---|
| `kilo status`, `kilo monitor` | HELLO, stats; live curses telemetry |
| `kilo posture rest\|sit\|stand\|limp\|balance` | Postures (`balance` = STAND + flag) |
| `kilo balance on\|off [--kp X] [--kd S] [--deadband D] [--clamp C] [--posture stand\|sit] [--for S]` | `SET_BALANCE` + hold the posture, a pitch/roll line per second |
| `kilo walk --pattern trot --stride 30 --turn 0 --period 500 [--keys]`, `kilo stop` | Gait |
| `kilo look PAN TILT [--tail T] [--speed DPS] [--for S] [--posture P]`, `kilo eyes RRGGBB [--side left]`, `kilo beep NOTE:DUR …` | Head (P1, `mind/kilo/cli_head.py`: `SET_HEAD` hold, released on exit), eyes, sound |
| `kilo trick list\|show NAME\|upload NAME\|play NAME [--mirror] [--speed 1.5] [--repeats N] [--for S] [--posture P] [--then-watch T]\|stop` | Tricks (P1, `mind/kilo/cli_trick.py`); `import-legacy FILE` and `edit NAME` (jogs joints live and captures frames) later |
| `kilo cal`, `kilo cal imu` | Joint calibration; IMU offset iteration on the Pi from IMU_RAW (keeps `CalibrateAccel/Gyro`, ≈1 KB, off the body) |
| `kilo param get\|set\|list [--persist]` | Live tuning (names or ids from the spec's param table) |
| `kilo eeprom dump 0 283 > docs/kilo-eeprom-legacy.md` | The P0 legacy dump |
| `kilo flash [--legacy] [--no-build] [--port P]` | Appendix F |
| `kilo log plot FILE --fields pitch,roll,corr` | Offline plots |
| `kilo run` | Start the mind (normally via systemd) |

---

## Appendix H: Test tables

**H-unit. Host and simavr criteria**

| Unit | Criteria |
|---|---|
| fx | atan2 vs `std::atan2` over a 1000² grid incl. axes, zero and large magnitudes (≤ 0.02°); sin over all a16 (≤ 2e-4); isqrt exact (edges + sampled) |
| joints | Transfer function vs a transcription of legacy float `writeAngle`, using the dumped EEPROM: all 16 joints × every whole degree in limits; exact or ±1 count (listed). Limits, slew. |
| config | Migration accept/reject paths; torn write at every byte offset recovers to the other slot; sequence wrap |
| ik | Round trip on a 1 mm grid ≤ 0.25 mm; ≤ 0.25° vs double; reach clamp; legacy postures plausible |
| gait | Swing order and pairs per pattern; stance speed constant ±2%; ≤ 3°/tick output step at defaults; clean start/stop; turn sign; no phase jump on period change |
| keyframe | Tick-exact dur/hold; easing endpoints; −128; loops; mirror (incl. roll ref); speed. P1: `body/test/test_keyframe.cpp` (also upload edge cases, crossfades, end posture, TRICK_DONE reasons, gating, per-frame balance, and all 13 library tricks within the joint limits with the JSON's timing) |
| arbiter | Per-joint priority; crossfade length/monotonicity; head and balance gating; epoch refusal and LIMP/REST exemption. P1 head layer in `body/test/test_reflex.cpp`: slew, timeout, release crossfade, epoch release, no balance on held joints, precedence with TRICK and REFLEX, TELEMETRY |
| balance | Toy-plant closed loop converges; every sign flip diverges; dead band; saturation; slew. Gain table = legacy `adaptiveParameterArray` = Kilo's EEPROM; mapping vs legacy `adjust()` ≤ 0.07° (grid), controller vs legacy ≤ 1° (0.16°); physical direction vs the posture tables; gating by owner, posture, request, epoch bump, IMU_FAULT |
| reflex | Fall 6-tick timing, hysteresis, retries, mirror by roll sign, per-reflex LIMP timers; battery sag spikes don't trip, hysteresis clears; watchdog 0.5 / 1.5 s stages; epoch bump on each. P1 fall: `body/test/test_reflex.cpp` (6 consecutive samples, a dip restarts, single samples never trip, both sides, back, pitch falls, still wait, rc frames tick by tick and mirrored, 3 tries, RECOVERED/RECOVERY_FAILED, FALL_OK, gating, TRICK_DONE(FELL), STALE/FALLEN refusals, LIMP ends it, watchdog takes over) |
| attitude | vs float reference incl. the gz<0 reflection (≤ 0.1°); A/B vs the legacy float32 `ypr` path within ±60° (≤ 1°; 0.038°); gyro rates = d/dt of pitch/roll (sign, ≤ 10%) |
| codec | protocol/vectors both ways; 10⁶-iteration fuzz under ASan; resync on next 0x00; ACK de-dup |

**H-HIL. On Kilo**

| # | Check | Pass |
|---|---|---|
| H1 | Flash from the Pi | HELLO `FLASHED` with matching build, 3/3 |
| H2 | Under legacy firmware put Kilo in `kup`; flash the new firmware (the PCA keeps running); `PCA_READ`; then `SET_POSTURE STAND` (`up`) and `PCA_READ` again | Counts identical ±1 per joint; `CAL_VALUES` = legacy CALIB |
| H3 | Each joint ±10° from STAND | Moves as documented, returns |
| H4 | Command beyond a limit | JOINTS shows the clamp |
| H5 | `kill -STOP` the daemon while walking / standing | Motion stops ≤ 0.5 s + one period; lying down ≤ 3.5 s / 2.5 s; LIMP 10 s later; LINK_RESTORED and epoch refusal on resume |
| H6 | Debug param that hangs the loop | HELLO `WARM_UNEXPECTED` with `last_alive_tick` near the hang; pose held with no jump, then REST |
| H7 | Tip onto each side | FELL, `rc` mirrored correctly, RECOVERED ≥ 4/5; old goals refused as STALE until the mind re-plans |
| H8 | Bench supply 8.4 → 6.0 V in 0.1 V / 10 s steps, then back up | Trips and clears within ±0.1 V of the meter; no sag trips while trotting at 7.2 V |
| H9 | 10 min trot, 50 Hz telemetry + JOINTS | `max_tick_us` < 12000, 0 overruns, `stack_free_min` > 150 B |
| H10 | 30 min soak, max streams + 5 Hz goals | `rx_crc_err` ≈ 0 (eye updates explain any); seq gaps < 0.1% |
| H11 | Ultrasonic at 10 / 30 / 100 cm | ±1 / ±2 / ±4 cm |
| H12 | Eyes R/G/B/W per side; BEEP scale | Correct LEDs; clean tones; no ultrasonic degradation while beeping |
| H13 | Board tilted 0 → 10° pitch and roll | Body within 3° of level after 1 s. **Not met by the P1 joint-space layer (measured 2026-10-11: ≈0.17° body per degree of correction); deferred to P2 IK levelling by the owner.** |
| H14 | Walk 2 m (P2) | Drift < 20 cm **(target)**; 360° turn in place < 20 s; 30 s trot without a fall |
| H15 | `kilo flash --legacy` | Old `kup`, `kwkF` work with the old calibration |

Results so far (docs/p0-hardware-test-results.md):
- 2026-10-10, from the Pi over the CH343 adapter: **H1 passed** (3 flashes,
  HELLO FLASHED with the build hash), **H2 passed** for the import side
  (`cal show` = legacy CALIB; STAND counts recorded, no legacy read-back to
  compare against yet), **H5 passed** with a held posture and the passive
  watch (stop 0.5 s, REST 1.5 s, LIMP 12.5 s), **H15 passed** (`kup`,
  `krest`). H2's legacy count comparison and H6 are still open.
