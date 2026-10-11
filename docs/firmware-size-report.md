# Body firmware flash size: deep dive (report only)

2026-10-11, on `mega` at 44a961d. Nothing in the firmware was changed for
this report. **(measured)** means a number from a real build or from the ELF.
**(est.)** means an estimate.

The question: how do we fit the roughly 2 KB of P2 walking code (gait
generator and per-leg 2-link IK in fixed point, plus geometric balance through
the IK) without dropping the IR remote, buzzer, ultrasonic, eyes or the DMP?

## Short answer

1. **The 24576 B limit is a budget we chose, not a hardware limit.** The chip
   and bootloader allow **32256 B**, and Kilo has already run a 32016 B image.
   Raising the budget gives 6-7.7 KB with no code change (section 2).
2. Without raising the budget, the only capability-preserving change that
   frees ≥ 2.5 KB is **moving the 3062 B DMP image out of AVR flash**. The
   best place for it is the NyBoard's unused I2C EEPROM. Every other saving we
   found adds up to under 1 KB (section 4).
3. Today there are 2028 B free under 24576 B. That is about the size of the P2
   estimate, so P2 might just fit or might not. There is no margin either way.

## 1. Measured breakdown

Build: `body/tools/size_check.sh`, which runs arduino-cli with
`--fqbn arduino:avr:uno`, avr-gcc 7.3.0 (arduino:avr 1.8.8), `-Os -flto`,
`-ffunction-sections -fdata-sections -Wl,--gc-sections`, plus the repo's own
`-mcall-prologues` and `-Wl,--relax`.

| Section | Bytes (measured) | Counts against |
|---|---|---|
| `.text` (code + PROGMEM tables + vectors) | 22520 | flash |
| `.data` (initialised globals: `g_param` 26 B, `prev_pind` 1 B, padding) | 28 | flash **and** RAM |
| `.bss` | 1412 | RAM |
| `.noinit` (reset record) | 10 | RAM |
| **Flash** (`.text` + `.data`) | **22548** | budget 24576 |
| **Globals** (`.data` + `.bss` + `.noinit`) | **1450** | budget 1540 |

### By module (measured)

LTO inlines most of the firmware into two big symbols, `main` (8592 B) and
`link_poll` (2970 B), so symbol sizes alone do not show which module costs
what. The table below gives every `.text` byte to the source file of its
instruction, using the DWARF line table (`avr-objdump -dl`). PROGMEM tables
count toward the module that owns them. Code inlined from a header counts
toward that header's module.

| Module | Bytes | Notes |
|---|---|---|
| hal/dmp_image.h (DMP firmware blob, PROGMEM) | 3062 | 13.6 % of flash |
| core/body | 2350 | reflexes, fall, trick hand-off, tick glue |
| core/link | 2138 | dispatch, HELLO, TELEMETRY/JOINTS/STATS emitters |
| core/config | 1250 | of which ≈544 B is the legacy EEPROM import (measured, below) |
| core/keyframe | 1176 | trick store, upload, player |
| core/fx | 1070 | atan2, sin, isqrt, ease; SIN_Q14 182 B + ATAN_Q8DEG 130 B tables |
| core/arbiter | 1054 | movers, crossfades, layers, safety stage |
| core/balance | 948 | |
| hal/mpu6050 | 842 | DMP upload state machine, FIFO, init SCRIPT 48 B |
| app/main | 794 | |
| proto (frame codec, COBS, CRC-16, generated length table) | 770 | of which 144 B is the payload-length table |
| core/attitude | 718 | quaternion → pitch/roll |
| hal/i2c | 712 | |
| core/joints | 705 | |
| libgcc (lib1funcs.S) | 428 | details below |
| core/battery | 400 | |
| hal/uart | 396 | |
| core/posture | 340 | |
| core/ir_nec | 328 | |
| hal/pca9685 | 302 | |
| core/params | 296 | |
| hal/ultrasonic | 286 | |
| core/notes | 286 | |
| core/reflex | 252 | |
| app/link_hal, core/sched | 166 each | |
| hal/buzzer | 163 | |
| core/reset, hal/clock | 152, 150 | |
| avr-libc inline code (`_delay_*`, `wdt_*`) | 134 | |
| crt (26-entry vector table 104 B + startup) | 132 | |
| core/events, hal/eeprom, hal/pcint2, hal/sys, hal/adc, hal/ir, hal/gpio, avr-libc `eeprom_read_byte` | 114, 110, 100, 90, 64, 50, 10, 16 | |
| **Total `.text`** | **22520** | |

By layer: core 13743, hal 6337 (3062 of it the DMP blob), app 960,
proto 770, libgcc 428, avr-libc 150, crt 132.

**Arduino core: 0 B (measured).** None of the 139 symbols in the ELF comes
from the Arduino core: no `init()`, no `millis`/`micros` timer ISR from
wiring.c, no `digitalWrite`/`pinMode`, no `digital_pin_to_port_PGM` and
similar tables, no HardwareSerial, Print or Wire. The only vectors are the
firmware's own (`__vector_5` PCINT2 174 B, `__vector_7` Timer2 buzzer,
`__vector_16` Timer0 overflow from hal/clock, `__vector_18`/`19` UART). The
design's claim checks out, so the Arduino core has nothing left to give.

### Top 30 flash symbols (measured, `avr-nm --size-sort`)

| Bytes | Symbol |
|---|---|
| 8592 | `main` (LTO merge: body, arbiter, attitude, keyframe player, battery, the loop) |
| 3062 | `dmp_image` |
| 2970 | `link_poll()` (LTO merge: dispatch, trick upload, frame codec) |
| 658 | `config_migrate(ConfigSlot&)` (includes the inlined `config_defaults`) |
| 434 | `mpu_init_step(unsigned long)` |
| 432 | `balance_axis_step(...) [clone .constprop.55]` |
| 406 | `fx_atan2(long, long)` |
| 308 | `ir_nec_edge(IrNec&, unsigned long)` |
| 276 | `emit(unsigned char, void const*, unsigned char, bool)` |
| 182 | `SIN_Q14` |
| 174 | `__vector_5` (PCINT2: ultrasonic echo + IR) |
| 166 | `joint_counts(JointCal const&, unsigned char, int)` |
| 154 | `fx_sin(int)` |
| 144 | `t.2030` (= `kilo_payload_len_check`'s table) |
| 130 | `send_hello()` |
| 130 | `ATAN_Q8DEG` |
| 128 | `__vector_18` (UART RX) |
| 116 | `shr_round(long, unsigned char)` |
| 116 | `posture_angle(unsigned char, unsigned char)` |
| 112 | `config_slot_valid(ConfigSlot const&)` |
| 100 | `mover_angle(Mover const&, unsigned char)` |
| 100 | `mpu_set_offsets(int const*)` |
| 100 | `i2c_recover()` |
| 94 | `__vector_19` (UART UDRE) |
| 91 | `POSTURES` |
| 90 | `mpu_init_begin(int const*) [clone .constprop.20]` |
| 86 | `fx_ease_cos(unsigned int, unsigned int)` |
| 84 | `body_mode(Body const&) [clone .constprop.100]` |
| 84 | `i2c_end()` |
| 82 | `pca_read_all(unsigned int*)` |

Largest RAM users: `g_body` 660, `rx_buf` 192, `g_config` 135, `L` (link
state) 103, `tx_buf` 96, `wire` 66.

### libgcc arithmetic (measured)

| Helper | Bytes | Called from |
|---|---|---|
| `__prologue_saves__` + `__epilogue_restores__` | 110 | `-mcall-prologues` (saves 192 B net, measured) |
| `__udivmodsi4` | 68 | fx.cpp:82 (atan2 ratio), fx.cpp:114 (ease), keyframe.cpp:140 (linear ease), ultrasonic.cpp:65 (µs → mm) |
| `__divmodsi4` | 40 | joints.cpp:67 (`num / 16000`), buzzer.cpp:24 (`F_CPU/2 / hz`), config.cpp, link.cpp |
| `__udivmodhi4` | 40 | params.h:30 (×4), main.cpp, mpu6050.cpp, link.cpp, body.cpp |
| `__mulsi3`, `__umulhisi3`, `__muluhisi3`, `__mulhisi3`, `__usmulhisi3`, `__negsi2` | 120 | balance, attitude (11 call sites), joints, fx |
| `__do_copy_data`, `__do_clear_bss` | 38 | crt |

The 32-bit divide helpers cost 108 B in total. Removing them would mean
rewriting all eight call sites. The IK will almost certainly divide in 32 bits
again (the law of cosines, atan2), so they would come back. **Not worth
doing.** The helpers are shared, so each extra call costs only 4-8 B.

### Generated protocol code (measured)

- `kilo_field_table` (string names, near line 1053) and `kilo_msg_table` (line
  1204) are inside `#ifdef KILO_PROTOCOL_FIELD_TABLE`, which only the host
  tools define. **They are not in the AVR image**: no message-name strings
  appear in the ELF.
- Used on the AVR: `kilo_payload_len_check` (a 144 B PROGMEM table plus about
  40 B of code), and `kilo_msg_is_goal` and `kilo_msg_is_query` (bit masks,
  inlined). With the inlined parts of `kilo_protocol.h`, that is 230 B of code
  inside `link_poll`, plus the 144 B table. `kilo_msg_kind` and
  `kilo_msg_has_epoch` are not referenced. The table is 0xFF for 0x40..0x7D,
  so 62 of its 144 bytes are filler.

### PROGMEM tables (measured)

DMP image 3062, SIN_Q14 182, payload-length table 144, ATAN_Q8DEG 130,
POSTURES 91, JOINTS 80, PARAM_TABLE 78, mpu SCRIPT 48, TOP_OCTAVE 24, GAIN 22,
CANONICAL 11, buzzer SHIFT 7, posture ROW 7, pca all_off 4. Apart from the DMP
image, they total 729 B. We found no duplicated table. The trick library is
not in flash (tricks are uploaded by the Pi into the 24-frame RAM store). The
legacy joint rows (`JOINTS`, 80 B) are needed at runtime.

### Code-structure checks

- **Jump tables:** there are none. `__tablejump2__` is not linked, and
  `-fno-jump-tables` and `-fno-tree-switch-conversion` change nothing
  (measured 0 B).
- **Inlining:** `KILO_NOINLINE` is on 21 small shared helpers (`under`,
  `lerp`, `take_reflex`, `emit`, ...). Every helper we looked at is used from
  several call sites, and the code already shares the REST-then-LIMP logic
  (`rest_then_limp`, `limp_now`, `limp_timer_step`) across reflexes. Forcing
  less inlining globally makes things bigger (`-fno-inline-small-functions`
  +50 B, `-finline-limit=3` +28 B, measured). So the inlining choices are
  already about right.
- **Duplicated paths:** there is one mover type in the arbiter, and crossfades
  go through `mover_start`/`mover_angle`. We found no second copy worth
  merging. Earlier shrink work (§3h) already did this kind of cleanup, and the
  remaining wins would be tens of bytes per item and need host-test re-runs.
- **`.data`:** 28 B, of which 26 B are the live params initialised from their
  defaults. Nothing to gain.

## 2. The real ceiling

**Finding: the usable flash is 32256 B. 24576 B is a project budget, about
7.5 KB below the hardware.**

Evidence:

1. `size_check.sh` builds for `arduino:avr:uno`. In arduino:avr 1.8.8
   `boards.txt`: `uno.upload.maximum_size=32256`, `uno.bootloader.file=
   optiboot/optiboot_atmega328.hex`, `high_fuses=0xDE`. That fuse value sets
   BOOTSZ to 256 words = 512 B. 32768 − 512 = 32256.
2. `docs/architecture.md` §3h says it directly: "Limit | 32256 B (32 KB − 512 B
   optiboot)". The phase table then sets the P0/P1/P2 ceilings at 20/24/24 KB
   as owner budgets (§8 question 9: "the ceilings were estimates"; "still
   7.5 KB under the 31.5 KB available").
3. **Kilo has already run an image larger than a 2 KB-bootloader board would
   allow.** `mind/firmware/opencat-legacy.hex` covers addresses up to 32016
   (measured from the hex records). On 2026-10-10 it was flashed with
   `avrdude -c arduino -b 115200` and verified ("32016 B verified",
   `docs/p0-hardware-test-results.md` row 9, H15 passed), and it ran.
   Afterwards the P0 firmware was flashed back over the same bootloader. With
   the classic 2 KB bootloader the application area would end at 30720 B, so
   bytes 30720..32015 would either fail to write (lock bits) or overwrite the
   bootloader and break the next flash. Neither happened. The 115200 baud
   upload speed also matches Optiboot; the old 2 KB ATmegaBOOT runs at 57600.
4. The firmware already assumes Optiboot elsewhere (`hal/sys.cpp` and
   `hal/uart.cpp`: Optiboot clears MCUSR and leaves U2X0 set).

Optiboot never reads its own fuses, so the BOOTSZ value has not been read
directly. Reading it would need an ISP programmer, and is not needed, given
evidence 3.

**What raising the budget means:** `FLASH_MAX` in `body/tools/size_check.sh`
(and the matching lines in §3h, §7 P2 acceptance "Flash ≤ 24 KB", §8 q9) is
the only thing that enforces 24576. `kilo flash` uses the same check. Setting
it to 30720 leaves 1.5 KB of margin under the hardware limit and gives
**+6144 B**. Setting it to 32256 gives **+7680 B**. P5's planned DMP removal
(≈3.5 KB) would add to that later.

## 3. Compiler and linker flags (all measured, same source)

Flags in effect: `-Os -g -flto -fno-fat-lto-objects -ffunction-sections
-fdata-sections -fno-exceptions -fno-threadsafe-statics -fpermissive`, link
`-Os -flto -fuse-linker-plugin -Wl,--gc-sections`, plus `-mcall-prologues`
and `-Wl,--relax` from the repo. `-mrelax` is in effect through
`-Wl,--relax`; `size_check.sh` checks that no `call`/`jmp` is left within
`rcall` reach. The arduino:avr platform has no `-O2`; it is already `-Os`.
Because of LTO, code is generated at link time, so the experiment flags were
passed to both compile and link (`compiler.c.elf.extra_flags`).

| Extra flag | Flash Δ |
|---|---|
| `-mstrict-X` | **−40** |
| `-fno-reorder-blocks` | **−48** |
| `-mstrict-X -fno-reorder-blocks` | **−88** |
| `-fno-tree-switch-conversion`, `-fno-jump-tables`, `--param inline-unit-growth=0`, `-fno-ipa-cp-clone`, `-fno-caller-saves`, `-fno-optimize-sibling-calls`, `-fno-tree-pre`, `-fno-if-conversion`, `-fmerge-all-constants` | 0 |
| `-fno-tree-ch` / `-fno-move-loop-invariants` / `-fno-split-wide-types` | +2 / +6 / +12 |
| `-finline-limit=3` / `-fno-inline-small-functions` / `-fno-gcse` | +28 / +50 / +52 |
| `-fno-tree-ter` / `-fno-tree-loop-optimize` / `-fno-ivopts` | +70 / +180 / +358 |
| `-mno-call-prologues` (the current flag's value) | +192 |
| `-fno-lto` (the value of LTO) | +2254 |

Both useful flags only change code generation. They do not change C
semantics or the ABI. The host tests (g++) do not use them, so the host tests
prove nothing here. After adopting them, re-run the simavr tests and a short
hardware smoke test (H1/H5), because tick timing changes slightly. The flags
would go into `CFLAGS`/`LDFLAGS` in `size_check.sh` and into
`mind/kilo/tools/flash.py` (`compile_cmd`), and §3h's flag check could check
them too.

A newer avr-gcc (12-14) sometimes gives a few percent, in either direction.
We did not test it: it would mean installing new tooling, which is the
owner's call.

## 4. Options

Ordered by bytes saved per unit of risk. Effort and risk: L = low, M = medium,
H = high.

| # | Option | Saving | Effort | Risk | What it affects |
|---|---|---|---|---|---|
| 1 | **Raise `FLASH_MAX` from 24576 to 30720 (or the 32256 hardware limit)** | **+6144 B (+7680 B)** headroom (measured limit, see 2) | L: one number plus doc lines | L: proven on Kilo by the 32016 B legacy image; costs only budget discipline | `size_check.sh`, §3h/§7/§8 text. No firmware change. |
| 2 | `-mstrict-X -fno-reorder-blocks` | **88 B** (measured) | L | L: valid code, re-run simavr + H1/H5 | build flags in `size_check.sh` and `kilo flash` |
| 3 | Move the legacy EEPROM import (`check_legacy` + import loop in `config_migrate`) to the Pi: the mind reads 0..282 with `EEPROM_READ`, then sends `CAL_SET` ×11, `IMU_OFFSETS_SET`, `CAL_SAVE` | **544 B** (measured with the import stubbed out; the body still writes defaults) | M: mind-side importer and tests; the body keeps `config_defaults` | L-M: Kilo already has valid migrated slots, so this only matters if both slots are lost. Legacy per-joint `a2p` (10963 on Kilo vs default 11000, 0.3 %) and the PCA frequency (24916 vs 25000, stored only) are not settable today. Accepting the defaults costs nothing. A message to set them adds about 50-80 B (est.). | first boot on a fresh board; CFG_MIGRATED state; `config_revert` falls back to defaults |
| 4 | **DMP image in the on-board I2C EEPROM** (0x54, "unused" in §8 q7): the upload state machine reads each 16 B chunk from the EEPROM into a RAM buffer instead of from PROGMEM; a CRC-16 of the image is kept in flash (2 B) and checked before upload; a failed check means `IMU_FAULT`, as an MPU failure does today | **≈2.9 KB**: −3062 B (measured array size), +100-150 B (est.) for a 16-bit-address EEPROM read and the CRC pass | M: loader change, a one-time provisioning step (a small provisioning sketch, or the legacy-style writeConst approach, so the write path is not in the production image), host test of the chunking | M-L: boot stays self-contained (no Pi needed). Boot gets about 80 ms longer (3 KB at 400 kHz). **To check first:** chip size and free region. §8 says 64 KB, but the legacy code uses `EEPROM_SIZE (65535/8)`, i.e. a 64 Kbit = 8 KB part, and stores its instinct skills there. The image must not overlap them, or `kilo flash --legacy` loses its skills. | IMU init path (boot and the LIMP re-init); provisioning procedure; the I2C EEPROM is no longer free (§8 q7) |
| 5 | Zero-run compression of the DMP image in flash (banks 0-3 are 58 % zeros) | ≈220 B: 274 B measured compressible (3062 → 2788), minus a ≈50 B decoder (est.) | M | L-M: the decoder must reproduce the image byte for byte (`test_dmp_image.cpp` can check it on the host); the read-back verify must compare against decoded bytes | `hal/mpu6050.cpp` upload. Cannot be combined with #4. |
| 6 | Shrink the generated payload-length table (two ranges, 0x00..0x3F and 0x7E..0x8F, instead of 0x00..0x8F) | ≈50-60 B (est.; 62 B of the 144 are filler) | L-M: change in `protocol/gen.py`, re-vendor | L | generated code only |
| 7 | DMP image sent by the Pi over the UART at boot (new message) | ≈2.8 KB (est.) | M-H: protocol message, mind code | H: IMU, fall detection and balance would depend on the Pi at every boot and every LIMP re-init; cold boot without the Pi has no IMU | boot sequence, protocol |
| 8 | P5 early: replace the DMP with the integer complementary filter (Appendix A) | ≈3.5 KB (doc est.: image 3062 + DMP init/FIFO − 0.5 KB filter) | H: new filter, tuning, A/B against the DMP | M-H | **drops the DMP**, so it is outside the owner's constraint; listed for completeness |
| 9 | Remove the 32-bit divides (8 sites) | ≤ 108 B (est.) | M | M: fixed-point rounding changes, host tests to update | IK will likely bring them back; not recommended |
| — | For reference, not options: `-DKILO_IR=0` −422, `-DKILO_BUZZER=0` −374, `-DKILO_ULTRASONIC=0` −350 (all measured) | | | | each drops a capability |

There is nothing left to win from the Arduino core (0 B), LTO (already on),
`-mcall-prologues`/`--relax` (already on), `-Os` (already on) or jump tables
(none).

## 5. Recommendation

- **Freeing ≥ 2.5 KB while keeping every capability is possible.**
- **Simplest, and enough on its own: option 1.** Raise the P2 flash budget to
  30720 B. That leaves 8.1 KB free for P2's ≈2 KB, with a 1.5 KB guard under
  the 32256 B hardware limit, and no code change at all. The 24 KB figure was
  set when the ceilings were estimates (§8 q9). The doc's own headroom line
  already says the hardware allows 32256. This is an owner decision.
- **If the 24 KB budget should stay:** the code-only options (2 + 3 + 5 + 6)
  add up to roughly 0.9 KB (88 measured + 544 measured + ≈220 + ≈55 est.).
  That is **not enough for 2.5 KB**. Reaching 2.5 KB under 24 KB without
  dropping a capability needs the DMP image out of flash, and the I2C EEPROM
  (option 4) is the only way to do that which keeps boot independent of the
  Pi. **Options 2 + 3 + 4 ≈ 3.5 KB** (88 + 544 measured, ≈2.9 KB est.),
  giving ≈5.5 KB free under 24576 B. First check the EEPROM chip size and
  where the legacy skills sit.
- In both cases, take option 2 now: it is free.
- Keep option 4 in reserve for after P2 (P3+ features, or if the budget
  stays).

## 6. RAM

Globals are 1450 of 1540 B (measured), so 90 B are left. The measured
minimum free stack on Kilo was 353 B, taken at P1 before the IR receiver added
17 B of globals, so it is likely ≈335 B now (est.). P2's IK and gait state is
estimated at ≈40 B of globals (Appendix D2), which fits. The IK's
intermediates live on the stack in a 50 Hz tick with no recursion, so they
should stay in the tens of bytes. Measure them with the painted canary
(`stack_free_min`) in the first P2 build. If RAM gets tight, the order in §3h
still applies: RX ring 192 → 128 B (−64 B), then TX ring 96 → 64 B (−32 B).
Option 4 needs a 16 B chunk buffer only while the IMU is being
(re)initialised, which happens in the LIMP-only state machine, not in the
walking tick.
