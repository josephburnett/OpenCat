# Kilo architecture: body firmware and mind software

Design document for rebuilding the firmware and control software of Kilo, a
Petoi Nybble on a NyBoard V1_2 with a Raspberry Pi 2 Model B on its back.

Status: draft for implementation, 2026-10-10. Ground truth used: `OpenCat.ino`,
`src/` on branch `mega` at 684d0eb, and `docs/raspberry-pi.md`. Numbers marked
**(est.)** are estimates. Everything else was read from the code or measured on
that commit (`arduino-cli compile -b arduino:avr:uno`: 32016 B flash, 1620 B of
globals).

Units used throughout:

| Name | Meaning | Type |
|---|---|---|
| `a16` | angle in 1/16 degree (0.0625°) | `int16_t`, range ±2047.9° |
| `q2` | length in 1/4 mm | `int16_t` |
| `Q14` / `Q12` | fixed-point fraction with 14 / 12 fractional bits | `int16_t` / `int32_t` |
| tick | one 20 ms motor-loop period (50 Hz) | |
| count | one PCA9685 step, 1/4096 of the PWM period (about 1 µs, see 3b) | |

All multi-byte protocol fields are little-endian.

---

## 1. Goals and non-goals

### Goals

1. **A companion with personality.** Kilo should explore, react to people and
   obstacles, rest when tired or low on battery, and show mood with eyes, head,
   tail and tricks. Personality lives on the Pi, where it can grow without
   fighting a 32 KB flash limit.
2. **A platform for learning control.** The firmware has a fixed-rate loop, an
   explicit owner model, a parametric gait with real inverse kinematics, and a
   tunable balance controller. Gains and parameters can be changed live over the
   protocol, and every tick can be logged at 50 Hz on the Pi.
3. **Capability over compatibility.** The Pi is always onboard. The firmware
   uses the NyBoard for hard real-time work (servo timing, IMU, reflexes,
   ultrasonic timing) and nothing else.
4. **Safety stays on the body.** Falls, low battery, joint limits and a lost or
   crashed Pi are handled by the firmware within one tick. They never depend on
   the Pi.
5. **Verifiable.** Firmware logic is plain C++ that builds and runs on a
   desktop. The protocol has one specification with cross-language test vectors.
   Flash and RAM are budgeted and checked on every build.

### Non-goals

- No compatibility with Petoi's token protocol (`k`, `m`, `i`, `K`...), the
  Petoi mobile app, `serialMaster`, or the setup sketch (`MAIN_SKETCH` off).
- No skill storage in EEPROM. The 64 KB I2C EEPROM is not used in v1 (see 8).
  Tricks live on the Pi and are sent into RAM when needed.
- No on-board behaviour: no random mind, no task queue, no ultrasonic reactions,
  no eye effects engine. The body has reflexes, not opinions.
- No legacy gait tables (`wkF`, `trF`, ...) on the body. The gait generator
  replaces them. A subsampled legacy walk may be used once as a P1 baseline
  (see 7).
- No vision or other heavy compute on the NyBoard. No machine learning on the Pi
  2 beyond classical computer vision.
- No 12-DoF (Bittle-style shoulder roll) support. Joints 4 to 7 are kept as
  indices so the joint table stays legacy-shaped, but they are never driven.

---

## 2. System overview

```
             +-------------------------- Raspberry Pi 2 B ("MIND", Python) --------------------------+
             |  perception (camera, later mic)   behaviour tree + drives   trick library + authoring |
             |  telemetry log      CLI / daemon socket      flash tool (arduino-cli + avrdude)         |
             |                         kilo.link.SerialLink  (/dev/ttyAMA0, 250000 8N1)              |
             +---------------------------------------|---------------------------------------------+
                         goals: posture, gait params,| head target, body pose, eyes, tricks, params
                         telemetry 25-50 Hz, events,  | acks                           GPIO4 -> RESET (flashing only)
             +---------------------------------------|---------------------------------------------+
             |                        NyBoard V1_2, ATmega328P @ 16 MHz ("BODY", C++)                 |
             |  UART/COBS/CRC  ->  goal registers  ->  50 Hz tick:                                   |
             |   IMU (DMP) -> reflexes -> owners {REFLEX > TRICK > GAIT > IDLE} -> crossfade          |
             |   -> head layer -> balance layer -> limits + slew -> angle->counts -> PCA9685 (I2C)    |
             |  side work: ultrasonic (pin-change ISR), battery ADC, eyes (WS2812), buzzer, EEPROM     |
             +-----------------------------------------------------------------------------------------+
                 I2C 400 kHz: PCA9685 (0x40), MPU6050 (0x68), [AT24C512 0x54 unused]
```

### What runs where

| Function | Where | Why |
|---|---|---|
| Servo PWM generation | PCA9685 hardware | Autonomous. It holds the last pose even while the AVR resets. |
| 50 Hz motor loop, interpolation, arbitration | Body | Needs deterministic timing. Linux on a Pi 2 gives no hard deadline. |
| Gait generator + IK | Body | Generates angles at 50 Hz. Streaming them from the Pi would use bandwidth and miss deadlines. |
| Balance (pitch/roll) | Body | Needs IMU-to-servo latency of a few ms. |
| Reflexes (fall, battery, limits, link watchdog) | Body | Must keep working when the Pi hangs, reboots or is absent. |
| Ultrasonic ranging | Body | Echo timing needs µs resolution. Results go to the Pi as telemetry. |
| Eye colours, buzzer notes | Body executes, Pi decides | Simple actuators. |
| Trick storage, authoring, mirroring, legacy import | Mind | Plenty of storage, easy to edit. |
| Perception, personality, behaviour selection | Mind | Python, camera, more CPU. |
| Logging, flashing, calibration UI | Mind | Filesystem, network, tools. |

### Latency expectations

| Path | Latency | Notes |
|---|---|---|
| IMU sample -> servo command | 3-5 ms after the tick starts, plus up to 10 ms DMP sample age **(est.)** | IMU is read at the start of the tick and the PCA9685 is written about 4 ms later. |
| PCA9685 register write -> new pulse on the wire | ≤ 4.1 ms | One PWM period at about 244 Hz. |
| Pi goal frame -> applied to the next tick | 1-3 ms on the wire + ≤ 20 ms tick phase | Worst case is about 23 ms. |
| Body state -> Pi Python callback | ≤ 20 ms tick phase + 1.2 ms wire + 1-10 ms Linux tty **(est.)** | Set `ASYNC_LOW_LATENCY` on the tty. |
| Behaviour tree reaction to telemetry | 100 ms tick of the tree | The tree runs at 10 Hz. |
| Camera perception -> head motion | 150-350 ms **(est.)** | Pi 2 at 3-10 fps, see 5. |
| Servo mechanical response (G41) | about 100-150 ms per 60° **(est.)** | Usually dominates everything above. |

Design rule that follows: **anything that must react within about 100 ms runs on
the body.** That covers balance, falls and the link watchdog. Everything slower
belongs to the mind.

---

## 3. Body firmware design

### Repository layout

```
body/
  body.ino                 # setup(), loop(): glue only, no logic
  src/                     # arduino-cli compiles src/ recursively
    core/                  # plain C++11, no Arduino headers, built on host too
      fx.h fx.cpp          # fixed point: sin, atan2, isqrt, tables
      joints.h joints.cpp  # joint table, angle->counts, limits, slew
      ik.h ik.cpp          # planar 2-link IK/FK
      gait.h gait.cpp      # phase oscillator + foot trajectories
      keyframe.h .cpp      # trick store + interpolator
      arbiter.h .cpp       # owners, crossfade, head + balance layers
      balance.h .cpp       # attitude controller
      reflex.h .cpp        # fall, battery, watchdog state machines
      attitude.h .cpp      # quaternion -> pitch/roll (integer)
      cobs.h crc16.h codec.h codec.cpp   # protocol
      protocol_gen.h       # GENERATED from protocol/spec.toml
      params.h params.cpp  # tunable parameter table
      config_block.h .cpp  # EEPROM block layout, CRC, legacy migration
    hal/                   # Arduino/AVR-specific, thin
      uart.* i2c.* pca9685.* mpu6050_dmp.* ultrasonic.* ws2812.* adc.* eeprom_writer.* buzzer.* clock.*
  test/                    # host tests (g++), Makefile, golden vectors
  budget.json              # per-phase flash/RAM ceilings for the size harness
protocol/
  spec.toml                # single source of truth for message layouts
  gen.py                   # emits body/src/core/protocol_gen.h and mind/kilo/protocol_gen.py
  vectors.json             # cross-language codec test vectors (generated + hand-written)
mind/
  kilo/ ...                # Python package (section 5)
```

Rules for `core/`:

- Use fixed-width types only (`int16_t`, `int32_t`, `uint8_t`). On AVR `int` is
  16 bits, on the host it is 32 bits. A bare `int` product that overflows on the
  328P passes silently on the host. A grep-lint in the test Makefile rejects
  `\bint\b|\bunsigned\b|\blong\b` in `core/`.
- Widen products explicitly: `(int32_t)a * b`.
- No floats, no heap, no virtual functions, no `Arduino.h`. Time is passed in as
  an argument (tick number). Sensors and outputs pass through plain structs.
- Use C++11 (`-std=gnu++11` is what avr-gcc 7.3 runs under arduino-cli).

The firmware does **not** use `HardwareSerial`, `Wire`, `I2Cdev`/`MPU6050`
classes, `Print`, `tone()` or `EEPROM.h`. Each is replaced by a small HAL module.
Together these libraries cost about 6 KB of flash and about 330 B of RAM today
(Serial object 157 B, Wire buffers about 165 B). The Arduino core stays for
`init()`, Timer0 `millis()`/`micros()`, `analogRead` and the build system.
**Implementation check:** confirm with `avr-nm` that `HardwareSerial0.o` is not
linked. Our `USART_RX_vect` would collide with it.

### 3a. Timing: the 50 Hz tick

**Recommendation: deadline polling on `micros()` in `loop()`, not a timer
interrupt.** Nearly all tick work is I2C (IMU read, PCA9685 write), which cannot
run inside an ISR. A timer ISR would only set a flag, which is the same as
polling a deadline. Polling also leaves Timer1 free (the IR library needs it if
the IR remote is kept) and Timer2 free for the buzzer.

```
loop():
  uart_poll()                       // decode complete frames from the RX ring, apply to goal registers
  if ((int32_t)(micros() - next_tick) >= 0):
      lag = micros() - next_tick
      next_tick += 20000            // fixed grid, no drift
      if lag > 40000: next_tick = micros() + 20000; stats.overruns++   // resync, never "catch up"
      run_tick()
  else:
      background()                  // EEPROM writer, LED show, eye/buzzer queues, TX, ultrasonic bookkeeping
```

Order of work inside `run_tick()`:

| # | Step | Time **(est.)** |
|---|---|---|
| 1 | Read IMU: FIFO count (2 B) + newest 28 B DMP packet | 1.0-1.5 ms |
| 2 | Quaternion -> pitch/roll/gyro rates (integer) | 0.15 ms |
| 3 | `uart_poll()` (yield point) | <0.1 ms |
| 4 | Reflex state machines, owner selection | 0.05 ms |
| 5 | Owner outputs: gait + IK for 4 legs, or trick interpolation, or posture | 0.8 ms (gait), 0.1 ms (others) |
| 6 | Crossfade, head layer, balance layer, limits, slew, angle->counts | 0.3 ms |
| 7 | `uart_poll()` (yield point) | <0.1 ms |
| 8 | PCA9685 burst write, all 16 channels in one transaction | 1.6 ms |
| 9 | Ultrasonic trigger if due (20 µs pulse); battery ADC if due (104 µs) | 0.13 ms |
| 10 | Telemetry encode into TX ring if due; `wdt_reset()`; record tick duration | 0.15 ms |
| | **Total** | **about 4-5 ms of 20 ms (20-25%)** |

The real figure is reported live as `tick_load` in telemetry and as a max in
`STATS`. Acceptance threshold: max tick time < 12 ms over a 10-minute walk.

**I2C cost.** The PCA9685 auto-increments, so channels 0-15 (`LED0_ON_L` =
0x06 through 0x45) can be written in one transaction: address + register + 64
data bytes = 66 bytes × 9 bit-times = 594 bits, which is 1.49 ms at 400 kHz plus
about 0.1 ms overhead. Today's firmware does one `setPWM` transaction per servo
(6 bytes plus the Wire library overhead, about 0.2 ms each), so 11-16 servos take
2-3 ms. Today's sketch runs the bus at 500 kHz (`Wire.setClock(500000L)`), but
the MPU6050 is specified to 400 kHz, so the new firmware uses 400 kHz.

The custom I2C driver is polled (busy-waits on `TWINT`) and has a 2 ms timeout
per transaction. On timeout it does a bus recovery (9 SCL clocks + STOP),
increments `stats.i2c_errors`, and skips the step. Three consecutive IMU
failures set the `IMU_FAULT` flag (see 3g). The driver has no 32-byte buffer
limit, unlike `Wire`.

**Rules for what must never block:**

- No `delay()`, no `pulseIn()`, no busy-wait longer than 50 µs, except inside an
  I2C transaction (bounded at 2 ms).
- No single non-yielding section longer than about 2 ms. `uart_poll()` is called
  between the large steps. At 250000 baud a byte arrives every 40 µs, so a
  192-byte RX ring gives 7.7 ms of slack.
- EEPROM writes go through a background writer. It writes one byte when `EEPE`
  is clear, about 3.4 ms per byte, so a 64-byte block takes about 220 ms in the
  background. It never calls the blocking `eeprom_write_byte` in a loop.
- WS2812 eye updates disable interrupts for 6 LEDs × 24 bits × 1.25 µs = 180 µs.
  That can drop UART bytes (the USART buffers only about 2 bytes) and can skew an
  ultrasonic echo edge. Mitigation: eye updates happen only on change (rare),
  only from `background()`, and never while an ultrasonic echo is in flight. Any
  frame that loses bytes fails its CRC. Goals are re-sent idempotently and
  commands are acked and retried (section 4), so a lost frame costs nothing.

**Ultrasonic.** Today `RgbUltrasonic::GetUltrasonicDistance()` calls
`pulseIn()`. It blocks for up to the echo length (about 38 ms with no obstacle)
or for `pulseIn`'s default 1 s timeout if the echo never starts. The module uses
one signal pin (D6) for both trigger and echo. New design:

1. Every 4th tick (80 ms, 12.5 Hz), drive D6 high for 20 µs (busy-wait is fine),
   then set it to input. Enable pin-change interrupt `PCINT22` (PCMSK2 bit 6).
2. The ISR timestamps the rising edge and the falling edge with `micros()`
   (4 µs resolution, which is 0.7 mm) and disables itself after the falling
   edge.
3. At the next trigger time, if no falling edge was seen within 25 ms of the
   trigger (about 4.3 m), report `distance_mm = 0` ("no echo") and re-arm.
   Distance = echo_us × 343 / 2000 mm, computed as `echo_us * 1715 / 10000` in
   int32.
4. The 80 ms cadence is above the 60 ms ring-down that HC-SR04-style modules
   need.

ISR latency jitter from the UART, TWI and Timer0 ISRs is about 10-20 µs, which
is 2-3 mm. That is acceptable. Cost: about 0.25 KB flash, 10 B RAM, zero
blocking.

**Hardware watchdog.** `wdt_enable(WDTO_250MS)` after boot, reset once per
completed tick. A hung I2C bus or a firmware bug reboots the AVR within 250 ms.
The PCA9685 keeps outputting the last pulses, so the robot freezes rather than
going limp. On boot the firmware **does not reset the PCA9685** if it is already
configured (MODE1 shows it awake and the prescaler is the expected value).
Instead it reads back the 16 `LEDn_OFF` registers to initialise the current
joint angles, so a watchdog reboot causes no jump. Today's `servoSetup()` always
calls `pwm.begin()`, which resets the chip, and then `shutServos()`.

### 3b. Joint model

**Joint table** (verified against `src/OpenCat.h` arrays `pwm_pin`,
`rotationDirection`, `middleShift`, `angleLimit`):

| Idx | Joint | PCA ch | dir | midShift (°) | limits (°) | Nybble |
|---|---|---|---|---|---|---|
| 0 | head pan | 12 | +1 | 0 | -120..120 | yes |
| 1 | head tilt | 11 | -1 | 15 | -80..45 | yes |
| 2 | tail | 4 | -1 | 0 | -120..120 | yes |
| 3 | (unused) | 3 | +1 | 0 | -120..120 | no |
| 4-7 | (shoulder roll on 12-DoF) | 13,10,5,2 | +1,-1,+1,-1 | -45 | -90..60 / -90..90 | no |
| 8 | LF shoulder | 14 | +1 | 10 | -100..80 | yes |
| 9 | RF shoulder | 9 | -1 | 10 | -100..80 | yes |
| 10 | RB shoulder | 6 | -1 | -10 | -80..100 | yes |
| 11 | LB shoulder | 1 | +1 | -10 | -80..100 | yes |
| 12 | LF knee | 15 | -1 | -30 | -60..120 | yes |
| 13 | RF knee | 8 | +1 | -30 | -60..120 | yes |
| 14 | RB knee | 7 | +1 | 30 | -120..60 | yes |
| 15 | LB knee | 0 | -1 | 30 | -120..60 | yes |

**Leg order verified as LF, RF, RB, LB** for both 8-11 and 12-15, from three
independent clues in the code (OpenCat.h has no comment that states it):

1. `adjust()` in `motion.h` computes `leftQ = (i - 1) % 4 > 1`, which gives
   8 → left, 9 → right, 10 → right, 11 → left.
2. `Skill::mirror()` swaps columns (8,9) and (10,11), so those are the left/right
   pairs.
3. Every posture in `InstinctNybble.h` gives 8 and 9 the same value and 10 and 11
   the same value (for example `rest`: 60, 60, -60, -60), so 8 and 9 are the
   front pair. The angle limits agree: front shoulders are -100..80, back
   shoulders are -80..100.

The populated PCA channels are {0,1,4,6,7,8,9,11,12,14,15}. They span 0..15, so
one 16-channel burst write is the cheapest option. Unused channels are written
as full-off.

Canonical order on the wire for the 11 driven joints is
**[0, 1, 2, 8, 9, 10, 11, 12, 13, 14, 15]**. It is used by keyframes, `JOINTS`
telemetry and calibration-sized arrays where noted.

**The angle -> PWM path (reproduces the legacy transfer function exactly).**
Today's path (`calibratedPWM` → `writeAngle`) works as follows:

- `setPWMFreq(240)` with a 25 MHz nominal oscillator gives prescale
  `round(25e6/(4096·240)) - 1 = 24`. The real frequency is 25e6/(4096·25) =
  244 Hz, a 4.096 ms period, so one count is 1.0 µs nominal.
- The setup sketch stored `ANGLE2PULSE_FACTOR = 11000`. That is
  `(2500-500)/180` evaluated in **integer** division (= 11) times 1.0 count/µs,
  times 1000. It also stored `B_OFFSET = 60`, from `240/50` in integer division
  (= 4) × 150 = 600, divided by 10.
- `servo_deg = 90 + dir·midShift + dir·calib + dir·angle`.
- `counts = 11 · servo_deg + 600`, clipped to 500..2500.
- So the legacy neutral (servo 90°) is **1590 counts ≈ 1590 µs, not 1500 µs**,
  and the slope is 11 counts/° instead of the nominal 11.1. Kilo's stored
  calibration offsets absorb this offset.

The new firmware keeps the identical function so that migrated calibration
offsets stay valid:

```
servo_a16 = 90*16 + dir[j] * (midShift[j]*16 + calib_a16[j] + angle_a16)       // int16
counts    = 600 + (11 * (int32_t)servo_a16 + 8) / 16                            // round to nearest
counts    = clamp(counts, 500, 2500)
```

The sub-degree `a16` resolution is finer than the hardware's 1 count = 0.09°,
so it costs nothing. A host differential test checks this against a transcript
of the legacy float code for every joint and every whole-degree angle in range
(section 6).

`dir`, `midShift`, PCA channel and limits are compile-time PROGMEM constants
copied from OpenCat.h, not EEPROM data. They describe the robot model, and
nothing writes them at runtime. Only per-robot data (calibration, IMU offsets,
tunables) lives in EEPROM.

**EEPROM: a new block in a hole of the legacy layout.** The legacy layout uses
internal EEPROM 0..282 (config), 320..700 (skill index; 47 entries of
`nameLen + 5` bytes = 381 B) and about 966..1023 (melodies). The new firmware
uses **704..959 (256 B)**. That region is unused by the old firmware, so
flashing back the old OpenCat firmware works with its EEPROM intact. This is the
fallback guarantee.

| Offset (from 704) | Size | Field |
|---|---|---|
| 0 | 2 | magic `'K','L'` |
| 2 | 1 | layout version (1) |
| 3 | 1 | reserved |
| 4 | 32 | `calib_a16[16]`, int16 |
| 36 | 12 | IMU offsets: accel x,y,z, gyro x,y,z (MPU6050 offset-register values, int16) |
| 48 | 2 | battery scale: mV per 64 ADC counts (default 646 ≈ 1000/99·64) |
| 50 | 64 | persisted params (32 × int16, indexed by param id; 0x7FFF = "use default") |
| 114 | 140 | reserved |
| 254 | 2 | CRC-16/CCITT-FALSE over bytes 0..253 |

**Migration on first boot.** If the magic or CRC is bad, the firmware imports
legacy values:

- `CALIB` (16 × int8 at 16) → `calib_a16 = calib · 16`.
- `MPUCALIB` (6 × int16 at 256) → IMU offsets.
- Defaults for everything else.

It then writes the block and emits `EVENT CONFIG_MIGRATED`. If the legacy data
also looks invalid (calib outside ±25°), it uses zeros and emits
`CONFIG_DEFAULTED`.

Today's firmware also sends 3064/3160 µs "P_SOFT/P_HARD" pulses (`setServoP`).
That is a Petoi convention for switching servo stiffness. Whether Kilo's G41
servos honour it is unknown (open question 8.6). v1 drops it.

### 3c. Arbitration

**Owners and priority.** Each owner is a producer with an `active` flag, a joint
mask (`uint16_t`, bit = joint index) and an output `q[j]` in `a16` for the
joints in its mask.

| Priority | Owner | Mask | Active when |
|---|---|---|---|
| 3 | REFLEX | all driven joints | fall response, recovery trick, low-battery rest, watchdog rest |
| 2 | TRICK | mask from `TRICK_BEGIN` (default all 11) | a trick is playing |
| 1 | GAIT | legs 8-15 | gait running, or stopping (stride ramp-down) |
| 0 | IDLE | all driven joints | always: the current posture (REST, SIT, STAND, CALIB, ZERO, LIMP, or IK `STANCE`) |

Per tick, per joint `j`:

```
o*      = highest-priority active owner whose mask contains j
base[j] = q_o*[j]
if o*(j) != prev_owner[j]:                       // ownership changed
    fade_src[j] = last_out_pre_layers[j]         // snapshot what the joint was doing
    fade_n[j]   = crossfade_ticks(prev_owner, o*) // see table
    fade_i[j]   = 0
if fade_i[j] < fade_n[j]:
    base[j] = fade_src[j] + (base[j] - fade_src[j]) * ease_cos(++fade_i[j] / fade_n[j])
```

| Transition | Crossfade | Reason |
|---|---|---|
| any → TRICK | 0 ticks | The trick's first frame interpolates from the current pose over its own `dur`. |
| any → REFLEX fall/recover | 0 ticks | The reflex sequence starts from the current pose. |
| TRICK/REFLEX → GAIT/IDLE | 10 ticks (200 ms), param `XFADE_TICKS` | |
| GAIT ↔ IDLE | handled by gait start/stop (see 3d), plus 10 ticks | |
| IDLE posture A → posture B | the `SET_POSTURE` transition time (default 25 ticks) | |

The snapshot approach does not run the old owner during the fade. That keeps
the cost low and makes the behaviour deterministic.

**Blend layers on top of the base pose:**

1. **Head layer** (joints 0, 1, 2). It applies when the head's owner is IDLE or
   GAIT, never during TRICK or REFLEX.
   - If the Pi holds a head target (`SET_HEAD mode=1`), the output is that
     target, slew-limited to the requested speed. Otherwise the output is the
     posture's head angles plus the gait's head bob and tail wag (3d).
   - The Pi's hold is released by `SET_HEAD mode=0`, by 2 s without a refreshed
     `SET_HEAD` (param `HEAD_HOLD_MS`), or by the link watchdog.
2. **Balance layer** (legs, optionally head tilt). The balance controller (3f)
   produces body-attitude corrections `(corr_pitch, corr_roll)` in `a16`. It is
   consumed in one of two ways:
   - For GAIT and IDLE `STANCE`, the corrections are added to the body pose
     offsets **before IK**. That is geometric and posture-independent.
   - For joint-space owners (built-in postures, tricks with the `BALANCE` flag),
     they are mapped through a per-joint gain table (the OpenCat
     `adaptiveParameterArray` approach) and added after crossfade.

   It is disabled for REFLEX, for tricks without the flag, for CALIB/LIMP, and
   when `IMU_FAULT` is set.
3. **Safety stage** (always last): clamp to joint limits, then slew-limit to
   `SLEW_MAX` per tick (default 10°/tick = 500°/s, about the G41 no-load speed
   **(est.)**), then angle→counts. The value written is recorded as
   `out[j]` (telemetry `JOINTS`).

**How the Pi expresses goals.** The Pi never sends joint angles, except as
keyframes inside a trick. Goals are latched registers on the body that the Pi
overwrites:

- posture: `SET_POSTURE`
- gait: `SET_GAIT` (run/stop + parameters)
- head: `SET_HEAD`
- body pose offsets: `SET_BODY_POSE`
- balance: `SET_BALANCE`
- eyes: `SET_EYES`
- tricks: `TRICK_*`

Goals are idempotent, so re-sending one is harmless. Reflexes can override them.
When a reflex ends, the body goes to `STAND` (fall) or stays in `REST` (battery,
watchdog). It does not resume the previous gait on its own. The Pi decides what
happens next. Telemetry reports `mode`, the leg owner, the head owner and the
last received sequence number, so the Pi can confirm that its goal took effect.

### 3d. Gait generator

**Parameters** (from `SET_GAIT`, slew-limited per tick to avoid foot jumps):

| Param | Unit | Range | Default walk / trot / crawl **(est., to tune)** |
|---|---|---|---|
| `pattern` | enum | WALK, TROT, CRAWL, (later PACE, BOUND) | |
| `period_ms` | ms | 300..2000 | 800 / 500 / 1200 |
| `duty` | /256 | 128..230 (0.5..0.9) | 192 (0.75) / 140 (0.55) / 218 (0.85) |
| `stride_mm` | mm, signed | -60..60 (negative = backward) | 30 / 35 / 25 |
| `turn_mm` | mm, signed | -40..40 (positive = turn left) | 0 |
| `step_h_mm` | mm | 0..40 | 15 / 12 / 18 |
| `body_h_mm` | mm | within IK workspace | 0.80·(L1+L2) / 0.80 / 0.70 |
| `flags` | bits | bob head, wag tail, balance | all on |

Body pose offsets (`SET_BODY_POSE`: pitch, roll, height delta, x-shift) apply on
top. They are also used by IDLE `STANCE`.

**Phase oscillator.**
- Global phase `φ` is a `uint16_t` (65536 = one cycle). Each tick adds
  `65536·20/period_ms`. Changing the period never makes the phase jump.
- Leg phase `φ_i = φ + offset_i` (wraps for free in uint16).
- Offsets, in leg order LF, RF, RB, LB, as fractions of a cycle:

| Pattern | LF | RF | RB | LB | Footfall sequence |
|---|---|---|---|---|---|
| WALK (lateral-sequence) | 0.25 | 0.75 | 0.50 | 0.00 | LB → LF → RB → RF |
| CRAWL | same as WALK, higher duty, lower body | | | | |
| TROT | 0.00 | 0.50 | 0.00 | 0.50 | diagonals together |

**Foot trajectory** in the leg frame (origin at the shoulder axis, x forward,
z down). Per leg: neutral foot `x0_i` (param, default 0 = under the shoulder),
height `H_i = body_h + pose terms`, and stride `S_i`. With `β = duty` and
`p = φ_i/65536`:

- Stance (`p < β`): `s = p/β`, `x = x0 + S_i·(1/2 − s)`, `z = H_i`. The foot
  sweeps back linearly. Ground speed is `S_i/(β·T)`, for example
  35 mm / (0.55 × 0.5 s) = 127 mm/s for the default trot **(est.)**.
- Swing (`p ≥ β`): `s = (p − β)/(1 − β)`,
  `x = x0 + S_i·(−1/2 + (1 − cos πs)/2)`, `z = H_i − h·sin(πs)`. The foot lifts
  with a half-sine and moves forward with cosine easing, so its velocity is zero
  at lift-off and touchdown.
- Turning: `S_left = S − turn`, `S_right = S + turn`. With `S = 0` and
  `turn ≠ 0`, Kilo turns in place.

**Pose terms** (Nybble legs are planar, so roll comes only from leg-length
differences):

```
H_i = body_h + side_i·(W/2)·sin(roll_total) − end_i·(Lb/2)·sin(pitch_total)
  side_i = +1 left, −1 right;   end_i = +1 front, −1 back
  pitch_total = pose_pitch + corr_pitch(balance),   roll_total = pose_roll + corr_roll
  x0_i += pose_xshift
```

The signs are fixed by host tests and the HIL check "positive pitch command
raises the nose".

**Head bob and tail wag** (head layer input when the Pi does not hold the head):
- tilt += `A_bob·sin(2·2πφ)`, which counteracts the twice-per-cycle body bob of
  a trot or walk, default 4°.
- tail = posture tail + `A_wag·sin(2πφ)`, default 15°.

This replaces today's `10·sin(frame·(j+2)·π/period)`.

**Start and stop.**
- Start: GAIT becomes active at `φ = 0` with stride ramping from 0 to `S` over
  one period.
- Stop: stride ramps to 0 over one period. Then the generator waits until every
  leg is in stance, deactivates, and IDLE (`STANCE` at the same body height)
  takes over with no visible jump.
- A pattern change goes through stop → start (about 1 cycle). Morphing phase
  offsets smoothly is a later experiment.

**Planar IK for one leg.** Upper link `L1` runs from the shoulder axis to the
knee axis. Lower link `L2` runs from the knee axis to the foot contact point.
Geometric angles:

- `θ1` = upper link from straight down, positive = foot forward.
- `θ2` = knee bend relative to the upper link, 0 = straight.
- `σ_i = ±1` selects which way the knee points (elbow back or forward). It is a
  per-leg parameter.

Forward kinematics:
```
x = L1·sin θ1 + L2·sin(θ1+θ2)
z = L1·cos θ1 + L2·cos(θ1+θ2)
```

Inverse kinematics (closed form):
```
d² = x² + z²                                     clamp d to [d_min, 0.95·(L1+L2)]
c  = cos θ2 = (d² − L1² − L2²) / (2·L1·L2)
s  = σ·√(1 − c²)
θ2 = atan2(s, c)
θ1 = atan2(x, z) − atan2(L2·s, L1 + L2·c)
```

Joint angles sent to the safety stage:
`shoulder_j = sgn_sh_i·(θ1 − θ1_0)` and `knee_j = sgn_kn_i·(θ2 − θ2_0_i)`.
`θ1_0`, `θ2_0` are the geometric angles at joint angle 0 (the calibration
posture). Per Petoi's calibration guide that posture is upper leg vertical and
lower leg at 90° to it, so `θ1_0 = 0` and `θ2_0 = ±90°`. **Verify on Kilo.**

The signs `sgn_sh`, `sgn_kn` and `σ` are found in P2 by two checks:
- a host test: FK of the legacy `up`, `sit` and `rest` postures must give
  plausible feet (below the body, symmetric left/right);
- a HIL check: command foot +10 mm forward on one leg at a time.

**What must be measured** (open question 8.1): `L1`, `L2`, the foot-contact
offset (if the foot tip is not on the knee-to-tip line, add a constant angle
`δ` to `θ2`), the shoulder-axis spacing front-to-back `Lb`, and the foot track
width `W`.

How to measure: put Kilo in `calib` and use calipers from axle centre to axle
centre, and from the knee axle to the floor contact point. Rough estimates from
Nybble's size: L1 ≈ L2 ≈ 45-55 mm, Lb ≈ 100-110 mm, W ≈ 55-65 mm. These are
**guesses; do not ship them.** They are compile-time constants in
`core/ik.h`, overridable by params.

**Fixed-point plan.**

- Lengths in `q2` (0.25 mm), so a 50 mm link is 200. Squares go in int32
  (d² ≤ 2·400² = 320 000).
- `c` in Q12 via one int32 division: `num·4096/den`, with
  |num| ≤ 2·L1·L2 ≈ 80 000, so num·4096 ≤ 3.3e8, which fits.
  `s = isqrt(4096² − c²)` in Q12.
- The second atan2 takes `(L2·s, L1·4096 + L2·c)` directly. atan2 is
  scale-invariant, so no division is needed there.
- `atan2(y, x)` → `a16`: reduce to the first octant, `r = (min << 14) / max`
  (Q14), then look up a **65-entry atan table** (0..45° in a16, 130 B PROGMEM)
  with linear interpolation, then rebuild the quadrant. Max interpolation error
  ≈ h²/8·max|f''| ≈ (1/64)²/8·0.65 rad ≈ 0.001°.
- `sin`/`cos` from a **65-entry quarter-wave table** (Q14, 130 B), linearly
  interpolated, error < 1e-4.
- `isqrt32`: bitwise, 16 iterations, about 60 B.
- Precision near full extension: `s = √(1−c²)` amplifies errors in `c` as
  `c/s`. With the reach clamp at 0.95·(L1+L2), the knee bend stays ≥ about 25°
  and the worst-case θ2 error is < 0.05° **(est.)**.
- Acceptance: host test over a 1 mm grid of the workspace, |FK(IK(p)) − p| ≤ 0.25
  mm and angle error vs a double-precision reference ≤ 0.25°.
- Cost per leg: 3 atan2 (one int32 division each, about 40 µs **(est.)**) + 1
  isqrt + 1 division ≈ 0.2 ms. That is 0.8 ms per tick for four legs.
- Flash: tables 260 B + atan2/sin/isqrt about 400 B + IK about 400 B + gait
  generator about 1.0 KB, so **about 2.0 KB total (est., soft)**.

### 3e. Keyframe interpolator for tricks

**Data model.** A trick has a header and up to **32 frames** held in one RAM
buffer. Frame layout, 14 bytes:

| Byte | Field | Meaning |
|---|---|---|
| 0 | `dur` u8 | ticks to move from the previous frame to this one (1..255 = 20 ms..5.1 s) |
| 1 | `hold` u8 | ticks to hold after arriving (0..255) |
| 2 | `flags` u8 | bits 0-1 easing: 0 linear, 1 cosine (default), 2 step; bit 2 balance on during this frame; others 0 |
| 3..13 | `ang[11]` i8 | degrees, canonical joint order; **-128 = "unchanged"**, meaning keep the previous target for this joint |

Header (from `TRICK_BEGIN`):
- `trick_id` u8: a Pi-chosen tag, so the Pi can cache what is loaded.
- `n_frames`.
- `loop_start`, `loop_end`, `repeats`: 0 = play once, 255 = repeat until
  stopped. This is the same idea as OpenCat's `loopCycle`.
- `joint_mask` u16: joints the trick owns. The others stay with GAIT/IDLE and
  the head layer, which lets the Pi play, say, a tail-only or head-only trick.
- `flags`: BALANCE (all frames), END_POSTURE (posture to switch IDLE to
  afterwards).

**RAM budget.** 32 × 14 B = 448 B, plus about 16 B of player state. That is the
largest elastic item in the RAM budget (3h). The biggest Nybble behaviour today
is `cmh` with 22 frames, followed by `dg` (11) and `hds` (10), so 32 frames
covers every existing trick with room left over. Postures are single-frame
tricks. None of the Nybble tricks use the IMU-trigger columns (all zero), so
triggers are not supported.

**Timing and easing.**
- Frame k starts from the joint targets of frame k−1. Frame 0 starts from the
  current `out[]`.
- Each tick `t` of `dur`: `q = from + (to − from)·e(t/dur)`, where
  `e(s) = s` (linear) or `(1 − cos πs)/2` (cosine).
- Cosine is computed as `sin²(πs/2)` from the sine table, all in Q14.
- Cosine matches today's `transform()` easing and suits pose-to-pose motion.
  Linear suits dense keyframes that should flow without stopping at every frame.
- A later option is Catmull-Rom through keyframes, flag value 3 reserved.
- `TRICK_PLAY` takes a `speed` (Q6, 64 = 1.0×, range 16..255 = 0.25×..4×) that
  scales `dur` and `hold`, and a `mirror` flag. Mirror swaps the L/R joint pairs
  (8↔9, 10↔11, 12↔13, 14↔15) and negates pan and tail, as `Skill::mirror()`
  does. It costs about 60 B and lets the body mirror the built-in recovery
  trick by fall direction.

**Upload.** Uploads are chunked and acked, 4 frames (56 B) per `TRICK_FRAMES`,
then `TRICK_COMMIT` with a CRC-16 of the whole frame array. Uploading while a
trick is playing is refused with `BUSY`. Tricks are short, and a second buffer
would cost another 448 B. A 22-frame trick needs 6 chunks plus a commit, which
is about 100 ms of stop-and-wait **(est.)**.

**Built-in sequences in PROGMEM** (needed without the Pi): postures `rest`,
`sit`, `up` (stand), `calib`/`zero`, and the recovery trick `rc` (3 frames).
About 100 B.

**Legacy import** (mind-side, section 5): each OpenCat behaviour row has
16 angles, then speed, delay, trigger axis and trigger angle.
- `dur = ceil(maxΔ / (speed/4) × LEGACY_STEP_MS / 20)`. `LEGACY_STEP_MS` is the
  time today's `transform()` takes per step (16 servo writes), about 5-6 ms
  **(est.)**. Calibrate it by filming both firmwares doing `hi`.
- `hold = delay × 50 ms / 20`.
- Joints 3-7 are dropped. `loopCycle` maps directly to
  `loop_start/loop_end/repeats`.

### 3f. Balance

**Today (`motion.h` `adjust()` + `skill.h` `perform()`):**

- **Error:** `dev = measured − expectedRollPitch[skill]`. A dead band of 5° roll
  and 3° pitch is subtracted. Note that `sign(ypr)` is used, not `sign(dev)`.
- **Per-joint gains:** `adaptiveParameterArray[j] = {rollFactor, pitchFactor}`
  in units of 1/57.3, so the effective gain is factor·radPerDeg degrees of joint
  per degree of tilt. Examples:
  - upper legs: roll 60 → 1.05, pitch 30 → 0.52;
  - lower legs: −1.5× the upper-leg values;
  - head pan and tilt: −60, tail −120.
- **Roll term:** it uses `|roll|` times a signed per-joint factor, with a 1.2×
  multiplier on the downhill side. As I read it, most of the roll response is a
  symmetric front/back crouch, and only 20% is left/right asymmetric.
- **Walking factor:** leg corrections are halved while walking
  (`POSTURE_WALKING_FACTOR 0.5`).
- **Damping:** the change per update is limited to ±5° (`ADJUSTMENT_DAMPER`).
  This is a slew limit, not a derivative term.
- **Rate:** the IMU is read on every 3rd joint update in `perform()`
  (`IMU_SKIP 3`), so the update rate depends on loop speed **(est. tens of Hz)**.
  It is proportional-only, in joint space.

**New design:**

- **Rate:** 50 Hz, synchronous with the tick. The DMP produces packets at about
  100 Hz (`SMPLRT_DIV = 4` → 200 Hz sample rate, FIFO divisor 1 → 100 Hz;
  **verify** by counting packets), and we take the newest.
- **Controller,** per axis:
  ```
  e     = deadband(measured − ref, DB)        DB default 2° (param)
  u     = Kp·e + Kd·rate                      rate = DMP raw gyro on that axis, deg/s
  u_f   = u_f + (u − u_f)·α                   α = 1/4, about 70 ms smoothing
  corr  = clamp(slew(u_f, 1°/tick), ±15°)
  ```
  - `ref` is 0 for GAIT and STANCE, and the posture's or trick's expected
    pitch/roll otherwise (for example `sit` = −30° pitch, from the legacy
    header).
  - Starting gains **(est.)**: Kp = 0.5 (Q8 128), Kd = 0.03 s.
  - The gyro rate gives real damping, which today's slew-limit "damper" does
    not. There is no integral term: a sustained slope should change the pose,
    and an integrator would wind up.
- **Application:** see 3c. Geometric through IK for GAIT and STANCE, so a 5°
  roll correction changes left/right leg heights by `W·sin 5°`. Legacy-style
  joint gains for joint-space postures and tricks, and those gains are params
  initialised from `adaptiveParameterArray`.
- **Tunable:** `SET_BALANCE` and `PARAM_SET` expose Kp, Kd, DB, clamp and α.
  Telemetry streams pitch, roll and rates at 50 Hz for step-response
  experiments. This is the main learning surface.
- v1 applies corrections to all legs. Applying them only to stance legs during
  swing is a later experiment.

**Attitude without float.** The DMP packet holds the quaternion as int32 Q30. We
take the high 16 bits as Q14 and compute the gravity vector:

```
gx = 2(qx·qz − qw·qy)
gy = 2(qw·qx + qy·qz)
gz = qw² − qx² − qy² + qz²
pitch = atan2(gx, √(gy² + gz²))
roll  = atan2(gy, gz)
```

This uses the integer atan2 and isqrt from 3d. It removes the float library and
`dmpGetYawPitchRoll` (today's `read_IMU()` alone is 1348 B). Yaw is not
computed; the yaw rate comes from the raw gyro. Axis signs must be checked
against today's `ypr` on Kilo (P1 differential HIL test).

The DMP-free filter is in the appendix.

### 3g. Reflexes and safety

Reflex state machines live in `core/reflex.*`, are pure functions of
(tick, sensors, params) and have full host tests.

**Fall detection and recovery.**

| State | Condition / action |
|---|---|
| UPRIGHT | |
| → TIPPING | \|roll\| > 75° or \|pitch\| > 75° (param `FALL_DEG`) |
| TIPPING → FALLEN | the condition holds for 6 consecutive ticks (120 ms). Today a single sample over 90° roll triggers. |
| FALLEN | REFLEX takes all joints and holds the current pose (with the gait stopped). Emit `EVENT FELL(axis, sign)`. Wait until \|gyro\| < 20°/s for 10 ticks (it has stopped rolling). |
| RECOVERING | Roll fall: play PROGMEM `rc`, mirrored by roll sign. Pitch fall (on the nose or tail): go to REST then LIMP and emit `RECOVERY_FAILED`, because `rc` does not handle pitch. |
| → UPRIGHT | \|roll\| < 20° and \|pitch\| < 20° for 25 ticks. Hand over to IDLE `STAND`, emit `RECOVERED`. |
| retry | Up to 3 attempts, 1 s apart. Then LIMP + `RECOVERY_FAILED`. |

Fall detection is disabled in CALIB, LIMP, while the IMU is faulted, and while a
trick runs that declares `FALL_OK` (header flag, for rolls).

**Battery.**
- The ADC on A7 uses AVcc (5 V) as reference: 1023 counts = 5 V at the pin.
  Today `LOW_VOLTAGE 650` and the printout `voltage/99` imply **99 counts per
  battery volt**. The divider ratio is 99·5/1023 ≈ 0.48 (about 1:2), so 650
  counts = **6.57 V = 3.28 V per cell**. Full charge (8.4 V) is about 832 counts
  and nominal (7.4 V) about 733.
- The "/99" is Petoi's empirical scale and depends on the actual 5 V rail and
  resistor tolerances. Kilo's real scale should be measured once with a meter
  and stored (EEPROM field, open question 8.5).
- New handling: sample every 5 ticks (10 Hz), IIR filter with α = 1/16 (time
  constant about 1.6 s). Servo current spikes sag the pack briefly, so raw
  samples are never compared to thresholds. Reported as `batt_mv`.

| Level | Default | Action |
|---|---|---|
| WARN | < 6.9 V for 5 s | `EVENT LOW_BATTERY(level=1)`. The Pi should wind down. |
| LOW | < 6.6 V for 5 s | REFLEX → REST, then LIMP after 2 s. Motion goals are refused with `REFUSED_LOW_BATT` until the voltage is > 7.0 V (hysteresis). `EVENT LOW_BATTERY(2)`. Buzzer melody if the buzzer is kept. |
| CRITICAL | < 6.3 V for 1 s | LIMP immediately. `EVENT LOW_BATTERY(3)`. The Pi should `shutdown -h` now, because it runs off the same pack through the buck converter. |
| USB-only | < 3 V | No battery (USB power): battery reflexes disabled, flag `NO_BATTERY`. |

**Joint limits.** Always enforced in the safety stage (3c), whatever the owner.
The limits are the PROGMEM table above, which can only be narrowed by params.
The slew limit (10°/tick default) also applies to reflexes.

**Link watchdog.** "Valid message" means any CRC-valid frame from the Pi. The
Pi sends `HEARTBEAT` at 10 Hz when it has nothing else to send.

| Time since last valid frame | Action |
|---|---|
| < 500 ms (`WATCHDOG_MS`) | normal |
| ≥ 500 ms | Emit `EVENT WATCHDOG` (queued; the Pi sees it on reconnect). Release the head hold. Stop the gait (stride ramp-down, 1 period) and stop any trick. Then REFLEX moves to the REST posture over 1 s with **balance on**. |
| ≥ 10 s more | LIMP (servos unpowered). |
| next valid frame | The watchdog clears. The body stays in REST/LIMP until the Pi sends a goal. |

Boot counts as link-lost: the body boots into REST then LIMP, and never stands
up without the Pi. When `WATCHDOG_MS = 0` the link watchdog is off, for bench
debugging only.

**Servo power semantics.**
- **LIMP:** every channel is written full-off (`LED_OFF_H` bit 4). The G41s get
  no pulses and hold no torque. The servo electronics stay powered from the
  pack: as far as I can tell the NyBoard has no switch on the servo rail
  (verify), so LIMP saves motor current but not quiescent current.
- **REST:** the lying-down posture (`rest` from InstinctNybble.h), then LIMP
  after a 1 s settle. This is the same as today's `d`.

**AVR watchdog and brown-out.** `MCUSR` is read at boot and reported in `HELLO`
(power-on, external reset = Pi GPIO4 or the button, brown-out, watchdog). A
watchdog reset means a firmware bug. A brown-out means the battery sagged under
load, which is diagnostic for the Pi-on-battery problem in
`docs/raspberry-pi.md`.

**IMU fault.** If no fresh DMP packet arrives for 10 ticks, or 3 I2C errors in a
row: balance and fall detection are disabled, the `IMU_FAULT` flag is set,
`EVENT IMU_FAULT` is emitted, and the IMU is re-initialised in background at
most once per 5 s.

### 3h. Memory budget

**Calibration from today's build** (mega @ 684d0eb: 32016 B flash, 1620 B
globals). Selected symbols from `avr-nm --size-sort`:

| Item today | Flash |
|---|---|
| `dmpMemory` (DMP image) | 3062 B |
| `read_IMU()` (float quaternion → ypr) | 1348 B |
| `beep()` (float duration + tone) | 1236 B |
| `RgbUltrasonic::SetRgbEffect` (effects engine) | 970 B |
| `Skill::loadFrame` + `lookupAddressByName` + `formatSkill` + `loadDataFromI2cEeprom` | about 1.6 KB |
| `transform()` + `calibratedPWM()` + `writeAngle()` | about 1.2 KB |
| IR timer ISR (`TIMER1_COMPA`) | 474 B (+ decoder, `irparams` 147 B RAM) |
| Serial object / `newCmd` buffer | 157 B / 468 B RAM |
| `main()` (everything inlined) | 10.5 KB |

From the brief: servo driver path about 2.1 KB, IMU with DMP about 4.2 KB + 3 KB
image, Wire about 2.3 KB, Serial/Print about 1.4 KB, float library about 1.6 KB.

**New firmware estimate** (all **est.**; **S** = soft, could be 50% off):

| Component | Flash | RAM | Notes |
|---|---|---|---|
| Arduino core subset (init, Timer0 millis/micros, analogRead, pin I/O) + vectors + crt | 1.0 KB | 15 B | |
| UART driver (RX/TX ISRs, rings) | 0.3 KB | 192 + 96 B | rings sized in 3a |
| COBS + CRC-16 + codec + dispatch (generated tables) | 1.5 KB S | 64 + 64 B | RX frame + TX frame scratch |
| I2C polled driver + bus recovery | 0.35 KB | 4 B | replaces Wire (2.3 KB) |
| PCA9685 driver, legacy transfer, read-back on boot | 0.4 KB | 32 B | counts buffer |
| MPU6050 DMP: image | 3.06 KB | | removed in P5 |
| MPU6050 DMP: init + FIFO read (no I2Cdev/MPU6050 classes) | 0.9 KB S | 28 + 16 B | |
| Attitude (Q14 quaternion → pitch/roll, rates) | 0.3 KB | 12 B | replaces float path (1.3 KB + float library 1.6 KB) |
| Fixed-point math (atan2, sin, isqrt, 2 tables) | 0.7 KB | 0 | |
| IK | 0.4 KB | 0 | |
| Gait generator | 1.0 KB S | 40 B | |
| Keyframe player + store | 0.6 KB | 448 + 16 B | store is the elastic item |
| Arbiter, crossfade, head layer, limits, slew | 0.8 KB S | 90 B | 11 joints × about 8 B |
| Balance controller | 0.4 KB | 16 B | |
| Reflexes (fall/recover, battery, link watchdog, IMU fault) | 0.7 KB S | 24 B | |
| PROGMEM postures + `rc` | 0.15 KB | 0 | |
| EEPROM block, migration, background writer | 0.5 KB | 70 B | RAM copy of calib, IMU offsets, live params |
| Params table (ids, defaults, min/max) | 0.3 KB | 40 B | |
| Ultrasonic (PCINT) | 0.25 KB | 10 B | |
| WS2812 eyes, solid colours only | 0.25 KB | 18 B | replaces RGBLed + effects (about 1.6 KB) |
| Battery ADC + filter | 0.1 KB | 6 B | |
| Telemetry, stats, events queue | 0.5 KB | 30 B | |
| Buzzer, Timer2 note queue (optional) | 0.35 KB | 20 B | |
| IR remote (optional, **not recommended**) | about 2.0 KB | about 150 B | + 20 kHz Timer1 ISR |
| Stack (worst case, measured by stack painting) | | 350 B | |
| **Total without IR** | **about 14.8 KB** (S: up to 19 KB) | **about 1.72 KB** | |
| **Limit** | **32256 B** (32 KB − 512 B optiboot) | **2048 B** | |
| Headroom | about 13-17 KB | about 330 B | |

**Flash and RAM by phase (est.):**

| Phase | Flash | RAM |
|---|---|---|
| P0 | about 10 KB | about 1.1 KB |
| P1 | about 13 KB | about 1.65 KB |
| P2 | about 15 KB | about 1.72 KB |
| P5 | about 11.5 KB | about 1.70 KB |

RAM is tighter than flash. If it gets tight, these levers apply in order:
1. keyframe store 32 → 24 frames (−112 B);
2. RX ring 192 → 128 (−64 B; then decoding must poll more often);
3. TX ring 96 → 64.

`STATS` reports the stack low-water mark from a painted canary
(`0xA5` fill at boot), so RAM headroom is measured, not guessed.

---

## 4. Protocol specification

### Physical link and baud

The link is the 328P USART0 ↔ Pi PL011 on GPIO14/15 (`/dev/ttyAMA0`). On the
Pi 2 B, PL011 is the primary UART because there is no Bluetooth. 3.3 V levels on
the header. 8N1, no flow control.

**Recommendation: 250000 baud.**

| Baud | 328P @16 MHz error | Pi PL011 (48 MHz UARTCLK) | Throughput |
|---|---|---|---|
| 115200 | +2.1% (U2X, UBRR=16 → 117647) | 26.04 → 115176 (−0.02%) | 11.5 kB/s |
| **250000** | **0.0%** (U2X, UBRR=7) | **12.000 exact** | **25 kB/s** |
| 500000 | 0.0% (U2X, UBRR=3) | 6.000 exact | 50 kB/s |

Why 250000:
- Both ends are exact.
- It more than doubles the headroom over 115200. The worst-case load below is
  18% upstream and 36% downstream at 250000; at 115200 the downstream worst case
  would be 78%.
- It keeps 40 µs per byte, which with a 192 B ring gives 7.7 ms of slack in the
  main loop.

500000 halves that slack and gains nothing we need. It can be revisited if
streaming needs grow.

Pi notes:
- Check `vcgencmd measure_clock uart`; it should read 48000000. With the old
  3 MHz default the maximum is 187500 baud; fix it with
  `init_uart_clock=48000000` in `config.txt`.
- pyserial on Linux sets non-standard rates through `termios2`/`BOTHER`, so
  `baudrate=250000` works.
- Disable the serial console (`raspi-config` → serial login shell No, hardware
  Yes), or kernel messages land on the body. CRC would reject them, but they
  would still use bandwidth.
- The bootloader still runs at 115200 (see Flashing).

### Framing: COBS + CRC-16

```
decoded frame:  type u8 | seq u8 | payload (0..60 B) | crc16 u16 LE      (4..64 B)
wire:           COBS(decoded) | 0x00                                       (≤ 66 B)
CRC:            CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout)
                over type..payload. Check value: "123456789" → 0x29B1.
```

**Why COBS over start-byte + length:**
- With SOF+LEN, a corrupted length byte can swallow the next good frames, and a
  SOF value inside a payload produces false starts. Resync is guesswork, and
  with only a CRC-8 about 1 in 256 false candidates passes.
- With COBS, 0x00 never appears inside a frame, so every delimiter is a certain
  resync point. A dropped byte damages exactly one frame.
- Cost: at most 1 byte of overhead per frame (frames are < 254 B), about 80 B of
  AVR code, and 15 lines of Python.

**Why CRC-16:**
- Python has it in the standard library: `binascii.crc_hqx(data, 0xFFFF)`.
- AVR has a 1-instruction-per-bit helper: `_crc_xmodem_update` from
  `<util/crc16.h>` with init 0xFFFF computes the same CRC. The host build uses a
  portable bitwise version.
- It costs one byte more than CRC-8, which this link can easily afford. It
  detects all 1- and 2-bit errors and all bursts ≤ 16 bits.

**Receiver rules:**
- Bytes accumulate until 0x00. More than 66 bytes before a delimiter → discard
  until the next 0x00 and count `rx_overflow`.
- Then COBS-decode, check `len ≥ 4`, check CRC (failure → `rx_crc_err`).
- Dispatch on `type`. The payload length must exactly match the type's spec;
  variable types state their rule. A mismatch → `NACK BAD_LEN`.
- Unknown type → `NACK UNKNOWN_TYPE`.
- `seq` increments per frame per direction (wraps). The Pi uses body `seq` gaps
  to count lost telemetry. The body echoes Pi `seq` in `ACK` and in
  telemetry `last_rx_seq`.

**Worked examples** (computed with `binascii.crc_hqx`; these become entries in
`protocol/vectors.json`):

```
HELLO_REQ seq=0       decoded: 01 00 3e 2e              wire: 02 01 03 3e 2e 00
SET_HEAD seq=7 mode=1 pan=20° tilt=-10° tail=0 speed=180°/s
                      decoded: 12 07 01 40 01 60 ff 00 00 b4 00 97 4f
                      wire:    08 12 07 01 40 01 60 ff 01 02 b4 03 97 4f 00
```

### Acknowledgement model

- **Goals** (`SET_POSTURE`, `SET_GAIT`, `SET_HEAD`, `SET_BODY_POSE`,
  `SET_BALANCE`, `SET_EYES`, `HEARTBEAT`, `BEEP`) are **not acked**. They are
  idempotent and the mind re-sends active goals at 5 Hz and immediately on
  change. A lost goal is corrected within 200 ms. Confirmation comes from
  telemetry (`mode`, owners, `last_rx_seq`).
- **Commands** (everything else: trick upload, calibration, params, EEPROM,
  stream config, reboot) are **acked** with `ACK(seq, type, status)`. The Pi
  retries after 50 ms, up to 3 times. Commands must be safe to repeat: for
  example `TRICK_FRAMES` carries its frame index, and `CAL_SET` is absolute.
- A goal that is refused (for example while low on battery) produces an
  `EVENT GOAL_REFUSED(type, reason)`, rate-limited to 1/s.

### Message catalogue

Payload offsets start after `type, seq`. Types 0x01-0x7F are Pi → body; types
0x80-0xFF are body → Pi.

**Pi → body**

| Type | Name | Payload | Bytes | Ack |
|---|---|---|---|---|
| 0x01 | HELLO_REQ | (empty) | 0 | replies HELLO |
| 0x02 | HEARTBEAT | (empty) | 0 | no |
| 0x03 | PING | `token u16` | 2 | replies PONG |
| 0x04 | SET_STREAM | `telem_div u8` (ticks per TELEMETRY, 0=off, default 2 → 25 Hz), `joints_div u8` (default 0), `imu_raw_div u8` (default 0) | 3 | yes |
| 0x05 | STATS_REQ | (empty) | 0 | replies STATS |
| 0x10 | SET_POSTURE | `posture u8` (0 LIMP, 1 REST, 2 SIT, 3 STAND, 4 STANCE(IK), 5 CALIB, 6 ZERO), `transition_ticks u8` (0 = default 25), `flags u8` (bit0 balance) | 3 | no |
| 0x11 | SET_GAIT | `run u8` (0 stop, 1 run), `pattern u8`, `period_ms u16`, `duty u8` (/256), `stride_mm i8`, `turn_mm i8`, `step_h_mm u8`, `body_h_mm u8`, `flags u8` (bit0 head bob, bit1 tail wag, bit2 balance) | 10 | no |
| 0x12 | SET_HEAD | `mode u8` (0 release, 1 hold), `pan a16 i16`, `tilt a16 i16`, `tail a16 i16`, `max_speed_dps u16` | 9 | no |
| 0x13 | SET_BODY_POSE | `pitch a16 i16`, `roll a16 i16`, `dheight_mm i8`, `xshift_mm i8` | 6 | no |
| 0x14 | SET_BALANCE | `enable u8`, `kp Q8 u8`, `kd_ms u8` (Kd in ms), `deadband_deg u8`, `max_corr_deg u8` | 5 | no |
| 0x20 | TRICK_BEGIN | `trick_id u8`, `n_frames u8` (1..32), `loop_start u8`, `loop_end u8`, `repeats u8`, `joint_mask u16`, `flags u8` (bit0 balance, bit1 fall_ok), `end_posture u8` | 9 | yes (BUSY if playing) |
| 0x21 | TRICK_FRAMES | `first u8`, `count u8` (1..4), then `count × 14 B` frames (3e) | 2+14n ≤ 58 | yes |
| 0x22 | TRICK_COMMIT | `trick_id u8`, `crc16 u16` over all frame bytes | 3 | yes (CRC_MISMATCH) |
| 0x23 | TRICK_PLAY | `trick_id u8`, `speed Q6 u8` (64 = 1×), `flags u8` (bit0 mirror) | 3 | yes (NO_TRICK, REFUSED_*) |
| 0x24 | TRICK_STOP | `transition_ticks u8` | 1 | yes |
| 0x30 | SET_EYES | `led_mask u8` (bits 0-5 = LEDs; 0x07 right, 0x38 left, 0x3F all), `r u8`, `g u8`, `b u8` | 4 | no |
| 0x31 | BEEP | `n u8` (1..8), then n × (`note i8`, `dur_10ms u8`), appended to a non-blocking queue | 1+2n | no |
| 0x38 | CAL_GET | (empty) | 0 | replies CAL_VALUES |
| 0x39 | CAL_SET | `joint u8`, `offset a16 i16` (absolute, ±25°). Enters CALIB posture if not already in it. | 3 | yes |
| 0x3A | CAL_SAVE | (empty): queues the EEPROM block write | 0 | yes after the write completes (about 0.3 s) |
| 0x3B | CAL_REVERT | (empty): reloads from EEPROM | 0 | yes |
| 0x3C | IMU_OFFSETS_SET | `ax ay az gx gy gz i16 ×6`, `save u8` | 13 | yes |
| 0x3D | PARAM_GET | `id u8` | 1 | replies PARAM_VALUE |
| 0x3E | PARAM_SET | `id u8`, `value i16`, `persist u8` | 4 | yes (BAD_ARG if out of range) |
| 0x3F | EEPROM_READ | `addr u16`, `len u8` (≤ 56) | 3 | replies EEPROM_DATA |
| 0x7E | REBOOT | `magic u16 = 0x4B49` ("KI"): restart the application by watchdog (does **not** enter the bootloader) | 2 | yes, sent before the reset |

**Body → Pi**

| Type | Name | Payload | Bytes |
|---|---|---|---|
| 0x80 | HELLO | `proto_major u8`, `proto_minor u8`, `fw_build u32` (git short hash), `reset_cause u8` (MCUSR bits: PORF 1, EXTRF 2, BORF 4, WDRF 8), `caps u16` (bit0 buzzer, bit1 IR, bit2 DMP, bit3 ultrasonic, bit4 eyes), `max_payload u8` (60), `trick_capacity u8` (32), `joint_count u8` (11), `config_state u8` (0 ok, 1 migrated, 2 defaulted) | 13 |
| 0x81 | TELEMETRY | see below | 25 |
| 0x82 | JOINTS | `tick u16`, `out a16 i16 × 11` (canonical order, after the safety stage) | 24 |
| 0x83 | IMU_RAW | `tick u16`, `ax ay az gx gy gz i16 × 6` (raw sensor LSB) | 14 |
| 0x84 | ACK | `seq u8`, `type u8`, `status u8` (0 OK, 1 BAD_LEN, 2 BAD_ARG, 3 BUSY, 4 UNKNOWN_TYPE, 5 CRC_MISMATCH, 6 NO_TRICK, 7 REFUSED_LOW_BATT, 8 REFUSED_FALLEN, 9 EEPROM_BUSY) | 3 |
| 0x85 | EVENT | `code u8`, `arg i16`, `tick u16` | 5 |
| 0x86 | CAL_VALUES | `calib a16 i16 × 16` (all 16 joint indices) | 32 |
| 0x87 | PARAM_VALUE | `id u8`, `value i16`, `min i16`, `max i16`, `default i16` | 9 |
| 0x88 | EEPROM_DATA | `addr u16`, `bytes[len]` | 2+len |
| 0x89 | PONG | `token u16`, `tick u16` | 4 |
| 0x8A | STATS | `uptime_s u32`, `overruns u16`, `max_tick_us u16`, `stack_free_min u16`, `rx_crc_err u16`, `rx_overflow u16`, `i2c_err u16`, `imu_fail u16`, `dmp_rate_hz u8` | 21 |
| 0x8F | LOG | ASCII text, ≤ 60 B (debug builds only, `-DKILO_LOG`) | var |

**TELEMETRY (0x81) payload:**

| Off | Field | Type | Unit / meaning |
|---|---|---|---|
| 0 | tick | u16 | tick counter (wraps every 21.8 min; the Pi unwraps it) |
| 2 | pitch | i16 | a16, nose up positive |
| 4 | roll | i16 | a16, left side up positive (convention fixed in P1) |
| 6 | rate_pitch | i16 | a16/s (±2048 °/s) |
| 8 | rate_roll | i16 | a16/s |
| 10 | rate_yaw | i16 | a16/s |
| 12 | dist_mm | u16 | last ultrasonic range; 0 = no echo |
| 14 | batt_mv | u16 | filtered |
| 16 | mode | u8 | 0 BOOT, 1 LIMP, 2 IDLE, 3 GAIT, 4 TRICK, 5 FALLEN, 6 RECOVERING, 7 LOW_BATT, 8 CALIB, 9 LINK_LOST |
| 17 | owners | u8 | low nibble: leg owner (0-3), high nibble: head owner (0-3, 4 = Pi hold) |
| 18 | flags | u16 | bit0 balance active, 1 IMU_FAULT, 2 servos powered, 3 gait active, 4 trick playing, 5 head held, 6 low batt warn, 7 link ok, 8 tick overrun since last telemetry, 9 NO_BATTERY, 10 EEPROM write pending, 11 dist fresh (new sample since last telemetry) |
| 20 | last_rx_seq | u8 | seq of the last valid Pi frame |
| 21 | trick_frame | u8 | 0xFF = none |
| 22 | gait_phase | u8 | φ >> 8 |
| 23 | tick_load | u8 | max tick duration since last telemetry, units of 100 µs |
| 24 | rx_err | u8 | CRC+overflow errors, cumulative mod 256 |

**EVENT codes:** 1 BOOT (arg = reset cause), 2 FELL (arg = ±1 roll / ±2 pitch),
3 RECOVERED, 4 RECOVERY_FAILED, 5 LOW_BATTERY (arg = level 1-3), 6
BATTERY_OK, 7 WATCHDOG, 8 LINK_RESTORED, 9 IMU_FAULT, 10 IMU_OK, 11
TRICK_DONE (arg = trick_id), 12 GAIT_STOPPED, 13 GOAL_REFUSED (arg = type <<
8 | reason), 14 CONFIG_MIGRATED, 15 CONFIG_DEFAULTED, 16 EEPROM_SAVED.
Events are queued (8 deep) and sent from `background()`. Events that happened
during link loss are delivered once the Pi talks again.

**Params** (ids are stable; full list in `protocol/spec.toml`; examples):
`WATCHDOG_MS` (500), `XFADE_TICKS` (10), `SLEW_MAX_A16` (160), `FALL_DEG` (75),
`BATT_WARN_MV` (6900), `BATT_LOW_MV` (6600), `BATT_CRIT_MV` (6300),
`BAL_KP_Q8`, `BAL_KD_MS`, `BAL_DB_A16`, `BAL_MAX_A16`, `HEAD_HOLD_MS` (2000),
`US_PERIOD_TICKS` (4), `L1_Q2`, `L2_Q2`, `LB_MM`, `W_MM`, `BOB_A16`, `WAG_A16`,
`LEGACY_GAIN_j_ROLL/PITCH`.

### Rates and bandwidth

Wire size = payload + 6 (type, seq, CRC × 2, COBS overhead, delimiter).

| Direction | Stream | Rate | Wire B | B/s |
|---|---|---|---|---|
| Body → Pi | TELEMETRY | 25 Hz default, 50 Hz max | 31 | 775 / 1550 |
| | JOINTS | off default, 50 Hz max | 30 | 1500 |
| | IMU_RAW | off default, 50 Hz max | 20 | 1000 |
| | EVENT / ACK / LOG | bursty | ≤ 66 | about 500 worst |
| | **worst case total** | | | **about 4.5 kB/s = 18% of 25 kB/s** |
| Pi → body | goal refresh (GAIT 16, HEAD 15, POSE 12, EYES 10) | 5 Hz + on change; worst 50 Hz each | 53 | 265 / 2650 |
| | HEARTBEAT | 10 Hz | 6 | 60 |
| | trick upload (stop-and-wait, ≤ 1 frame per 10 ms) | burst | 64 | ≤ 6400 |
| | **worst case total** | | | **about 9.1 kB/s = 36%** |

Single-frame wire time: 66 B max = 2.6 ms; telemetry = 1.2 ms.

### Watchdog timing

- The Pi sends `HEARTBEAT` every 100 ms. Any frame counts.
- The body declares link loss at 500 ms, after 5 missed heartbeats. Actions are
  in 3g.
- The Pi declares the body lost after 300 ms without telemetry (at the 25 Hz
  default that is 7 missed frames). It then marks state stale and keeps trying
  `HELLO_REQ` at 2 Hz.

### Boot and reboot detection

On every boot the body sends `HELLO` (unsolicited) and `EVENT BOOT`, then starts
telemetry at the default rate. The Pi treats an unsolicited `HELLO`, or a
`tick` that goes backwards, as a body reboot. It then:

1. logs `reset_cause` (WDRF = firmware bug, BORF = power problem);
2. checks `proto_major` (mismatch → refuse to drive and offer to flash);
3. re-sends stream settings and any non-default params it owns;
4. invalidates its trick cache;
5. re-sends current goals.

### Flashing from the Pi over the same UART

- The NyBoard runs the Uno optiboot at **115200**. Optiboot only starts on an
  **external reset** (`EXTRF`). A watchdog reset (`REBOOT`) goes straight back
  to the application, so a reset pulse on the RESET line is required.
- Pi GPIO4 (header pin 7) lands on the NyBoard's DTR/reset line. On the USB
  adapter's socket that line is normally AC-coupled through a capacitor to
  RESET; whether the header path has the capacitor is unverified (open question
  8.7). Driving GPIO4 **low for 50 ms then high** produces a reset in either
  case: through a capacitor the falling edge resets the chip, and with a direct
  connection the low level holds reset and releasing it starts the bootloader.
- In idle, GPIO4 is an output held **high** (`gpio=4=op,dh` in `config.txt`),
  so Pi boot activity or a floating pin cannot reset the body.
- **1-Wire must stay off.** The `w1-gpio` overlay claims GPIO4 by default and
  periodically drives 480 µs low reset pulses while searching the bus. Each
  pulse resets the NyBoard, and it also stops our tool from claiming the pin.

Flash sequence (`kilo flash`, section 5):

1. Tell the mind daemon to release `/dev/ttyAMA0` (lock file
   `/run/kilo/serial.lock`).
2. Build (`arduino-cli compile --fqbn arduino:avr:uno body/`). On a Pi 2 a full
   build takes about 1-3 min **(est.)**, so cache the build directory.
3. `gpioset` (libgpiod) GPIO4 low 50 ms → high.
4. Within about 200 ms, run
   `avrdude -p m328p -c arduino -P /dev/ttyAMA0 -b 115200 -D -U flash:w:body.ino.hex:i`.
   Optiboot waits about 1 s after reset.
5. Reopen at 250000 and wait ≤ 3 s for `HELLO` with `fw_build` equal to the
   built hash. Success means the new firmware booted. Retry the reset+avrdude
   once if sync fails.

The legacy OpenCat hex is kept in `mind/firmware/opencat-legacy.hex`, so the
same tool can roll back (`kilo flash --legacy`). The legacy firmware talks at
115200 using the token protocol, and its EEPROM is untouched (3b).

---

## 5. Mind software design (Python on the Pi)

Constraints: Pi 2 B, 4 × Cortex-A7 at 900 MHz, 1 GB RAM, Raspbian 13
(Python 3.13), armv7 32-bit wheels. Keep dependencies to `pyserial`, `numpy`,
`opencv-python-headless` or apt `python3-opencv`, `picamera2` (apt), and
optionally `py_trees`.

### Package layout

```
mind/
  pyproject.toml
  kilo/
    protocol_gen.py     # GENERATED from protocol/spec.toml: type ids, struct formats, enums, params
    codec.py            # COBS, CRC (binascii.crc_hqx), encode/decode to dataclasses
    link.py             # SerialLink: port, reader thread, framing, acks/retries, heartbeat, stats
    body.py             # BodyState (latest telemetry, derived), BodyClient (goal API, desired-state refresh)
    tricks/
      model.py          # Trick, Frame dataclasses, JSON/YAML I/O, mirror, time scaling, validation
      legacy_import.py  # parse src/InstinctNybble.h → tricks/*.json (behaviours + postures)
      library/          # *.json tricks
      uploader.py       # chunked upload + commit, cache by (trick_id, crc)
    behaviour/
      bt.py             # minimal behaviour-tree engine
      drives.py         # personality drives + utility selection
      trees/            # explore.py, rest.py, greet.py, startle.py, play.py, ...
    perception/
      ultrasonic.py     # filtering, approach/recede detection
      imu_events.py     # pick-up, tap/pet (accel spikes), tilt
      camera.py         # capture process (picamera2 or V4L2) + detectors
      audio.py          # later
    log.py              # telemetry/event logger (binary-framed or JSONL, rotating)
    daemon.py           # `kilo run`: owns the link, runs the mind, exposes a UNIX socket
    cli.py              # `kilo ...` subcommands
    flash.py            # build + reset + avrdude + verify
  tests/                # pytest: codec vs vectors.json, tricks, importer, BT, link against the virtual body
  firmware/opencat-legacy.hex
  systemd/kilo-mind.service
```

### Serial link

`SerialLink` opens `/dev/ttyAMA0` at 250000 with `ASYNC_LOW_LATENCY` and owns
one reader thread.

- The reader splits on 0x00, decodes COBS, checks CRC, and parses into
  dataclasses using the generated struct formats (`struct.Struct('<HhhhhhHHBBHBBBBB')`
  for telemetry, for example).
- Messages go to the asyncio loop with `call_soon_threadsafe`.
- `send(msg)` assigns `seq`. `request(msg, expect=ACK)` awaits the matching ack
  with a 50 ms timeout and 3 retries.
- A heartbeat task runs at 10 Hz.
- Stats: CRC errors, seq gaps, RTT from `PING`.
- `on_hello` triggers the reboot handling in section 4.

Threads plus asyncio is chosen over `pyserial-asyncio` to keep dependencies
minimal. The reader thread is about 60 lines.

**Desired-state goals.** `BodyClient` stores the current goals (posture, gait,
head, pose, balance, eyes). Changing one sends it immediately, and a 5 Hz task
re-sends all active goals. Callers write
`body.walk(stride=30, turn=0, pattern=TROT)`, `body.look(pan=20, tilt=-10)`,
`body.stop()`, `body.posture(SIT)`, `await body.play("hi", mirror=True)`
(resolves on `TRICK_DONE`) and `body.eyes(0x3F, 0, 0, 40)`. Behaviours never
build frames.

### Body-state model

`BodyState` holds:
- the last telemetry, with time unwrapped to a monotonic `t_body`;
- link health;
- derived quantities: `upright`, `moving`, `fallen`, filtered distance with
  approach velocity, battery percentage (2S Li-ion curve, from voltage at rest),
  an `is_stale` flag, and the last N events;
- a 10 s ring of telemetry for detectors (numpy arrays).

### Behaviour layer

**Recommendation: a minimal behaviour tree, ticked at 10 Hz, with a
utility-scored "drive" selector at the root.**

- Personality needs many small reactive behaviours (startle at a sudden
  approach, follow a face, nap when bored, greet). They must preempt each other
  cleanly and be composable and reusable. A BT gives exactly that:
  - Selectors re-check higher-priority conditions every tick, so "obstacle
    close" or "fallen" interrupts "explore" without hand-written transitions.
  - Sequences express multi-step acts (look, approach, sit, wag).
  - Decorators handle cooldowns, timeouts and probability.
- A hierarchical state machine needs explicit transitions between every pair of
  states that can interrupt each other. With 10-20 behaviours that turns into a
  tangle, and adding a behaviour means touching many transitions.
- Pure BTs are deterministic, though, and a pet should not be.
- So the root is a **utility selector over drives**:
  - **energy**: drains with motion, recovers while resting, tied to battery;
  - **curiosity**: grows when idle, drops while exploring;
  - **sociability**: grows when alone, drops when a face is seen or Kilo is
    petted;
  - **arousal**: spikes on a startle, then decays.

  Each top-level behaviour has a score function of the drives and percepts plus
  a little noise and a personality weight vector. The highest score wins, with
  hysteresis so it does not flip every tick. Personality traits are the weight
  vectors, saved in a YAML file the owner can edit.
- Safety-style responses (fallen → wait for the body's recovery; low battery →
  rest and shut down) sit in a higher-priority branch above the utility
  selector.

The engine is in-house, about 300 lines (Sequence, Selector, Parallel,
Inverter, Cooldown, Timeout, Action/Condition with RUNNING). It stays readable
for learning, and it is easy to log every tick's active path (`log.py` records
it). `py_trees` is an acceptable swap if visualisation matters more than
minimalism.

### Perception adapters (realistic on a Pi 2 B)

- **Ultrasonic** (from telemetry, 12.5 Hz): median-of-3 filter, approach speed
  from the slope, events `near(<15 cm)`, `approaching(>20 cm/s)`, `clear`.
- **IMU events** (from 50 Hz telemetry with `IMU_RAW` on): picked up (sustained
  accel change with legs unloaded), tap/pet (short accel spike while still),
  tilted.
- **Camera.** The camera runs in its own process (multiprocessing) to use a
  second core and avoid the GIL. It sends compact detections (bbox, centre,
  confidence, timestamp) over a pipe. Capture with `picamera2` (libcamera,
  CSI camera) at 320×240 YUV420, using only the Y plane; a USB webcam uses
  OpenCV V4L2 at 320×240.

  Expected rates on a Pi 2 B **(est., measure in P4)**:

  | Detector | Resolution | Rate |
  |---|---|---|
  | Motion (frame differencing + blur) | 160×120 | 15-25 fps |
  | Colour blob (HSV threshold + moments, e.g. a toy) | 160×120 | 10-20 fps |
  | Haar frontal face (`detectMultiScale` with scaleFactor 1.2, minSize 30) | 320×240 | 3-6 fps |
  | DNN face/person (MobileNet-SSD) | | about 0.5-1 fps; only for slow "who is there" checks, not tracking |

  Head tracking closes the loop on the Pi with a P-controller on the bbox
  centre → `SET_HEAD` target at the detector rate, with the body slew-limiting.
  At 5 fps that is a 200 ms loop, so expect gentle tracking, not snapping.
- **Microphone (later):** USB mic, RMS loudness and onset detection
  (clap/name-call level) at 20 Hz with numpy. Keyword spotting is a stretch on a
  Pi 2.

### Logging

`log.py` writes every received frame (raw decoded bytes + Pi monotonic
timestamp) to `~/kilo-logs/YYYYMMDD-HHMMSS.kilolog`. The format is a length
prefix + timestamp + frame; logs are replayable through the same codec. At
25 Hz telemetry plus events this is about 1 MB/h. It also writes a JSONL
behaviour log (active BT path, drive values, chosen behaviour) at 1 Hz and on
change. `kilo log plot FILE --fields pitch,roll,corr` draws matplotlib plots
off-robot. That covers the control-learning loop: change a gain, walk, plot the
step response.

### CLI (`kilo`)

`kilo` talks to the daemon over `/run/kilo/kilo.sock` (JSON lines) if it is
running, else it opens the serial port directly.

| Command | Purpose |
|---|---|
| `kilo status` / `kilo monitor` | HELLO info, stats; live curses telemetry view |
| `kilo posture rest\|sit\|stand\|limp` | |
| `kilo walk --pattern trot --stride 30 --turn 0 --period 500` ; `kilo stop` | gait goals; `--keys` drives with arrow keys |
| `kilo look PAN TILT [--tail T]` ; `kilo eyes RRGGBB [--side left]` ; `kilo beep NOTE:DUR ...` | |
| `kilo trick list\|play NAME [--mirror] [--speed 1.5]\|import-legacy src/InstinctNybble.h` | |
| `kilo trick edit NAME` | Curses keyframe editor. Select a joint, jog it ±1°/±5° (the robot shows the pose live via a 1-frame trick), capture a frame, set dur/hold/easing, preview from frame N, mirror, save JSON. The G41 servos have no position feedback, so poses cannot be authored by moving limbs by hand. |
| `kilo cal` | Interactive joint calibration (CAL_SET/SAVE/REVERT, shows a reference image), plus `kilo cal imu` (Pi-side offset iteration using IMU_RAW with the robot flat; writes IMU_OFFSETS_SET) |
| `kilo param get\|set\|list [--persist]` | Live tuning |
| `kilo flash [--legacy] [--no-build]` | Section 4 |
| `kilo run` | Start the mind (normally via systemd `kilo-mind.service`) |

Moving IMU calibration to the Pi removes `CalibrateAccel/Gyro` (about 1 KB) from
the body. The Pi has the time and the floats.

### Flashing tool

`flash.py`:

1. `arduino-cli compile --fqbn arduino:avr:uno --build-path ~/.cache/kilo-build body/`
2. Run the size check (the budget harness, section 6). Refuse to flash if over
   budget.
3. Embed the git hash: `--build-property compiler.cpp.extra_flags=-DKILO_BUILD=0x<hash>`.
4. Release the port, reset via GPIO4, run avrdude, verify via HELLO (section 4).

All of it is logged, and it works over SSH.

---

## 6. Testing and verification strategy

**1. Host unit tests for body core** (`body/test/`, `make test`). Build with
`g++ -std=c++11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined`, plus
the bare-type lint (3, layout rules).

| Unit | Tests |
|---|---|
| fx | atan2 vs `std::atan2` over a 1000×1000 grid incl. axes and zero (≤ 0.02°); sin over all a16 (≤ 2e-4); isqrt exact for 0..2³²−1 sampled + edges |
| joints | **Differential vs legacy:** a C++ transcription of today's `calibratedPWM`+`writeAngle` float math (stored counts 11000/60, PROGMEM tables) against the new integer path, for all 16 joints × every whole degree in limits × calib −10..10. Counts must match exactly, or within ±1 where float rounding differs (counts listed). Plus limit clamping and slew. |
| ik | FK(IK(p)) round trip on a 1 mm grid (≤ 0.25 mm); angles vs a double reference (≤ 0.25°); reach clamp behaviour; FK of legacy postures plausible (feet below shoulders, L/R symmetric). |
| gait | Per-leg phase offsets; stance foot speed constant within 2%; continuity (no output step > 3°/tick at default params); start/stop ends with all feet down; turn asymmetry sign; period change mid-cycle has no phase jump. Golden trajectories generated by a Python float reference (`body/test/ref/gait_ref.py`). |
| keyframe | Exact tick timing of dur/hold; linear and cosine endpoints exact; −128 "unchanged"; loops and repeats; mirror; speed scaling; reject malformed headers. |
| arbiter | Priority resolution per joint mask; crossfade length and monotonicity; head layer gating; balance gating per owner. |
| balance | Step response in a toy simulation (body pitch = integrated correction with lag) is stable with no overshoot beyond 20% at default gains; deadband; saturation; slew. |
| reflex | Fall timing (6 ticks), hysteresis, retries, recovery mirror by roll sign; battery filter and thresholds with synthetic sag spikes (a 200 ms dip must not trip LOW); link watchdog timeline. |
| codec | All `protocol/vectors.json` entries both ways; fuzz with random/truncated/bit-flipped streams (libFuzzer or a 10⁶-iteration random loop under ASan): no crash, no accept of corrupted frames in practice, resync on the next 0x00. |
| attitude | Quaternion → pitch/roll vs float reference for random unit quaternions (≤ 0.1°). |

**2. Exact-semantics run on AVR.** The same tests compiled with avr-g++ and run
under `simavr` (Debian package), for fx/ik/gait/joints/codec. This catches
int-promotion and 16-bit `int` bugs the host build cannot. It also yields real
cycle counts for the tick budget in 3a, replacing the estimates.

**3. Virtual body.** A host executable `body/test/vbody` links `core/` with stub
HALs: a fake IMU from a scripted attitude, a fake ultrasonic, a servo array
recording counts. It exposes the real protocol on a pty (`openpty`). The mind's
`SerialLink`, CLI and behaviour trees run against it unchanged, on the
desktop, in CI and on the Pi. Integration tests:
- link watchdog → REST;
- trick upload/play/TRICK_DONE;
- reboot detection;
- goal refusal on low battery.

**4. Python tests** (pytest, run on desktop and Pi):
- codec vs `vectors.json`;
- hypothesis round-trip of every message type;
- CRC check value;
- legacy importer on `src/InstinctNybble.h` (all 25 behaviours + postures
  import, frame counts ≤ 32, `cmh` = 22 frames);
- trick mirror/time-scale;
- BT engine semantics;
- link retry logic with an injected lossy transport.

**5. Protocol single source.** `protocol/gen.py` generates both
`protocol_gen.h` and `protocol_gen.py` from `spec.toml`. The generated files are
committed, so the Arduino build needs no Python. CI regenerates them and fails
on diff.

**6. Size harness** (`tools/size.sh`, also run by `kilo flash`):
- compiles `body/` with arduino-cli and parses flash/RAM;
- runs `avr-nm --size-sort -C` and writes a per-module breakdown, grouped by
  object file, to `build/size-report.txt`;
- checks `body/budget.json` (ceilings per phase, e.g. P1: flash ≤ 16 KB,
  globals ≤ 1700 B) and fails above the ceiling;
- checks a forbidden-symbol list: `HardwareSerial`, `TwoWire`, `__addsf3`,
  `__mulsf3`, `pulseIn`, `malloc`. If any is linked, the build fails. This
  enforces the "no float, no Wire, no pulseIn" decisions mechanically.

**7. Hardware-in-the-loop checklist on Kilo** (run per phase; results are noted
in the merge commit message):

| # | Check | Pass criterion |
|---|---|---|
| H1 | Flash from Pi; HELLO with matching build and `reset_cause` = EXTRF | 3/3 |
| H2 | Calibration migration: compare `CAL_VALUES` to the legacy EEPROM bytes 16..31 via EEPROM_READ; `calib` posture looks identical to the old firmware's `kcalib` (photo side by side) | exact values, visual match |
| H3 | Each joint +10° then −10° from STAND, one at a time | moves the documented direction, returns |
| H4 | Limits: command beyond limit | clamped (JOINTS shows the limit) |
| H5 | Watchdog: `kill -STOP` the daemon while standing | REST within 1.5 s, LIMP after 10 s more; recovers on resume |
| H6 | AVR WDT: debug param that hangs the loop | reboot, no limb jump, HELLO with WDRF |
| H7 | Fall: tip Kilo onto each side by hand | FELL event, `rc` mirrored correctly, RECOVERED in ≥ 4/5 tries |
| H8 | Battery: bench supply 8.4 → 6.0 V in 0.1 V steps, 10 s each | WARN/LOW/CRIT at threshold ±0.1 V per the meter; no trips from motion sag at 7.2 V while trotting |
| H9 | Timing: 10 min of trot with 50 Hz telemetry + JOINTS | `max_tick_us` < 12000, 0 overruns, `stack_free_min` > 150 B |
| H10 | Serial soak: 30 min, max streams + 5 Hz goals | `rx_crc_err` = 0 (or explained by eye updates), Pi seq gaps < 0.1% |
| H11 | Ultrasonic: box at 10 / 30 / 100 cm | within ±1 cm / ±2 cm / ±4 cm |
| H12 | Eyes: red/green/blue/white each side | correct LEDs and colours |
| H13 | Balance: Kilo standing on a board tilted 0→10° pitch and roll | body stays within 3° of level after 1 s |
| H14 | Walk (P2): 2 m straight on the floor at default walk/trot | lateral drift < 20 cm **(target, est.)**; turn in place 360° < 20 s; 30 s trot without falling |
| H15 | Fallback: `kilo flash --legacy`; old firmware works with its calibration | old `kup`, `kwkF` work |

---

## 7. Phased plan

Order principle: get Kilo **standing and talking under Pi control in P0**,
**doing everything it does today in P1**, and **walking under Pi control in
P2**, while the old firmware stays one command away (`kilo flash --legacy`).
`OpenCat.ino` and `src/` are deleted only after P2's acceptance passes
("cutover").

### P0: protocol + skeleton body

- **Deliverables:**
  - `protocol/` (spec, generator, vectors).
  - `body/` with: UART/COBS/CRC; HAL (I2C, PCA9685 with read-back, DMP read for
    pitch/roll telemetry, ADC, EEPROM block + migration + writer); PROGMEM
    postures with plain interpolation; HELLO/TELEMETRY/EVENT/ACK/STATS;
    calibration and param messages; link watchdog → REST; battery reflex; AVR
    WDT.
  - `mind/`: codec, `SerialLink`, `BodyClient` (posture only), CLI
    `status/monitor/posture/cal/param/flash`.
  - Host tests for joints (legacy differential), codec, reflex (battery,
    watchdog). Size harness. Virtual body.
- **Removed from flash vs today:**
  - the token protocol and parser (`io.h`, `reaction.h`, `newCmd` 468 B RAM);
  - the skill system and I2C-EEPROM skill storage;
  - the setup-sketch paths;
  - TaskQueue;
  - ultrasonic reactions and the RGB effects engine;
  - IR (pending decision);
  - `HardwareSerial`/Print, `Wire`/I2Cdev, `beep()` floats;
  - the float library, if the integer attitude path lands in P0, else in P1.
- **Acceptance:** H1, H2, H3, H4, H5, H6, H8, H10. Flash ≤ 12 KB. Kilo goes
  REST → STAND → SIT → REST from `kilo posture` on the Pi.

### P1: motor loop, arbitration, tricks, balance (parity with today's tricks)

- **Deliverables:**
  - The fixed 50 Hz tick as specified.
  - Owners + crossfade + head layer + safety stage.
  - Keyframe store/player/upload, mirror, speed.
  - Balance controller (joint-space mode).
  - Fall detection + `rc` recovery.
  - Integer attitude.
  - Ultrasonic via PCINT, eyes, buzzer queue (if kept).
  - Mind: trick model, legacy importer for all Nybble behaviours and postures,
    uploader with cache, `kilo trick ...`, keyframe editor v1, logger.
  - Optional baseline: legacy `wkF` subsampled to 22 frames as a looping trick,
    to compare against P2.
- **Removed:** DMP float math (`dmpGetYawPitchRoll`, helper_3dmath float types),
  if not already done.
- **Acceptance:** H7, H9 (on a looping trick), H11, H12, H13. Every imported
  trick plays and looks right to the owner side by side with the old firmware.
  Attitude matches the old `ypr` within 1° static (A/B flash). Flash ≤ 16 KB.

### P2: gait generator + IK (walking under Pi control)

- **Deliverables:**
  - Measured link lengths and body dimensions (open question 8.1).
  - `fx` tables, IK, gait generator, STANCE posture, geometric balance through
    IK, head bob / tail wag.
  - Mind: `kilo walk` with keyboard driving, gait parameter presets per pattern.
- **Acceptance:** H14, H9 while trotting, IK/gait host tests including simavr.
  WALK, TROT and CRAWL each start and stop cleanly with no foot jump visible on
  slow-motion video. Flash ≤ 18 KB.
- **Cutover:** delete `OpenCat.ino` and `src/` (the legacy hex is kept in
  `mind/firmware/`).

### P3: mind v1 (personality with ultrasonic + IMU)

- **Deliverables:**
  - BT engine, drives, personality YAML.
  - Behaviours: explore (walk + turn away from obstacles), rest/nap (low
    energy), startle (approach), greet (tricks + eyes), react to pick-up and
    taps, battery wind-down + clean Pi shutdown on CRITICAL.
  - systemd service, behaviour log.
- **Acceptance:** 30 min autonomous on the floor; ≤ 2 obstacle contacts; rests
  when energy is low; responds to approach within 0.5 s; on CRITICAL battery the
  Pi shuts down cleanly before the body goes limp; no unhandled exceptions.
  No firmware change is required. If one is, it is a P2.x patch.

### P4: camera

- **Deliverables:** camera process, motion/blob/face detectors, head tracking,
  "social" drive inputs (a face seen counts as company).
- **Acceptance:** measured detector rates documented. Head tracks a face moving
  across the field of view with ≤ 0.5 s lag at ≥ 4 Hz. Total Pi CPU < 70% and
  power draw logged (USB meter) for the battery budget.

### P5: DMP removal

- **Deliverables:** raw accel/gyro read (14 B burst from 0x3B), integer
  complementary filter (appendix), boot-time gyro bias capture, A/B build flag
  `KILO_IMU_DMP`.
- **Removed:** the DMP image (3062 B) + DMP init/FIFO code (about 0.9 KB). Added
  back: about 0.5 KB for the filter. **Net about −3.5 KB flash**. Boot is
  faster (no 3 KB image upload over I2C).
- **Acceptance:** logged side-by-side (both estimators in a debug build or
  back-to-back runs): static pitch/roll within 1°, dynamic (trot) within 3° RMS,
  fall detection latency no worse than with the DMP. H7, H9, H13 re-run.

---

## 8. Open questions for the owner

1. **Leg geometry** (blocks P2): measure L1, L2, the foot contact offset, the
   front-back shoulder-axis spacing Lb and the foot track width W (3d). Also
   confirm that the calibration posture is "upper leg vertical, lower leg at
   90°" on Kilo.
2. **Which tricks matter?** All 25 Nybble behaviours import mechanically. Which
   ones should be polished, and which new ones do you want (for example stretch
   on waking, a sleepy head-down nap)?
3. **IR remote:** keep or drop? Cost: about 2 KB flash, about 150 B RAM, and a
   20 kHz Timer1 interrupt (an estimated 5-10% CPU plus timing jitter).
   Recommendation: **drop**; the Pi is
   always there, and a phone or SSH replaces it.
4. **Buzzer:** keep? Cost: about 0.35 KB. Recommendation: **keep.** It gives the
   body a voice without the Pi (low-battery alarm, fall chirp), and the Pi
   composes the melodies.
5. **Battery scale:** measure the pack voltage with a meter against the
   reported `batt_mv` once, to fix the "/99" scale. Do you plan a higher-drain
   pack? Petoi warns the stock 1000 mAh pack struggles with a Pi aboard. The
   battery thresholds depend on it.
6. **Servo stiffness pulses:** did the old `setServoP` P_SOFT/P_HARD pulses ever
   visibly change Kilo's G41 servos? If not, they stay dropped.
7. **GPIO4 → reset wiring:** is there a capacitor on the header path (as on the
   USB adapter socket)? The reset procedure works either way, but it matters if
   GPIO4 is ever left low.
8. **Camera hardware:** CSI camera (which version) or USB webcam? Where is it
   mounted? The head moves, so a head-mounted camera makes tracking
   self-referential (simpler), but adds weight and cabling on the head.
9. **Personality:** which traits and how much autonomy? For example: "never
   walk off unless I say so", or "explore freely on the floor". This also
   decides whether table-edge risk needs a cliff sensor, which Nybble lacks.
10. **Telemetry by default:** 25 Hz is the default. Do you want 50 Hz logging
    always on for learning? It costs about 2 MB/h of SD writes.
11. **I2C EEPROM (64 KB):** unused in this design. Keep it reserved, or use it
    later (for example a persistent body-side log of falls and brown-outs)?
12. **Network:** the Pi 2 has no Wi-Fi. A USB Wi-Fi dongle (+0.5-1 W) or
    tethered Ethernet for development?

---

## Appendix A: DMP-free attitude filter (P5)

Read 14 bytes from `ACCEL_XOUT_H` (0x3B) each tick at 400 kHz, which takes
about 0.4 ms.

Configuration:
- `CONFIG` DLPF = 3 (44 Hz accel / 42 Hz gyro): anti-aliasing for 50 Hz
  sampling.
- `GYRO_CONFIG` ±500 °/s (65.5 LSB per °/s).
- `ACCEL_CONFIG` ±2 g (16384 LSB/g).
- Offsets from the EEPROM block, written to the MPU offset registers at boot as
  today.

Per axis, with angles held as int32 `a16 << 8` so that sub-LSB increments
accumulate:

```
gyro increment per tick in a16:  Δ = raw_gyro · (16 / 65.5) · 0.02 = raw · 0.004885 ≈ raw · 5 / 1024   (0.04% error)
accel angle:  pitch_acc = atan2(−ax, isqrt(ay² + az²)),  roll_acc = atan2(ay, az)     (integer atan2 from 3d)
predict:      θ += Δ
correct:      if | |a| − 1 g | < 0.15 g:  θ += (θ_acc − θ) >> 6          // α = 1 − 1/64 ≈ 0.984
```

- Time constant: τ = dt·α/(1−α) = 0.02·63 ≈ 1.26 s. The gyro dominates below
  τ and the accelerometer pins drift above it.
- The magnitude gate skips the accelerometer during impacts and footfalls,
  which would otherwise tilt the estimate.
- Gyro bias: at boot, if the variance is low for 1 s, average 50 samples
  in-place and subtract. The `kilo cal imu` offsets handle the rest.
- Rates for telemetry and the balance D-term come straight from the gyro.
- Yaw is not estimated (no magnetometer). The Pi integrates `rate_yaw` if it
  needs heading for short periods.
- Cost **(est.)**: 0.5 KB flash, 24 B RAM, about 0.6 ms per tick including the
  read.

This filter is also a learning exercise: the Pi can log `IMU_RAW` at 50 Hz,
replay it through a Python reference filter, and tune α and the gate offline
before changing firmware params.

## Appendix B: decisions taken where the brief left a choice

| Topic | Decision | Section |
|---|---|---|
| Tick scheduling | `micros()` deadline polling, fixed grid, resync on > 2 tick lag | 3a |
| Ultrasonic | pin-change ISR on D6, 80 ms cadence, 25 ms cap | 3a |
| I2C | custom polled driver at 400 kHz, single 16-channel PCA9685 burst | 3a |
| Serial/Wire/float | none linked; enforced by the size harness | 3, 6 |
| Calibration layout | new 256 B EEPROM block at 704 with migration; legacy layout untouched for fallback; legacy angle→counts function reproduced exactly | 3b |
| Framing | COBS + CRC-16/CCITT-FALSE, max 60 B payload | 4 |
| Baud | 250000 | 4 |
| Acks | goals unacked and refreshed at 5 Hz; commands acked with retry | 4 |
| Trick store | 32 × 14 B frames in RAM, int8 degrees, −128 = unchanged, cosine/linear easing | 3e |
| Balance | PD with gyro damping at 50 Hz; geometric via IK for gait/stance, legacy joint gains for joint-space owners | 3f |
| Watchdog | 500 ms → REST with balance; +10 s → LIMP | 3g |
| IMU calibration | moved to the Pi | 5 |
| Behaviour | in-house BT under a utility drive selector | 5 |
| Testing | host + simavr + virtual body on a pty | 6 |
