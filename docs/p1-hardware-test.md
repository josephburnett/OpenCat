# P1 balance hardware test

The first run of the joint-space balance layer (docs/architecture.md §3c
layer 2, §3f) on Kilo, from the Raspberry Pi. It checks, in this order: that
the attitude signs are what the firmware assumes, that each leg answers a tilt
the right way, that the balance switches on and off when it should, the
closed-loop residual on a slope (H13), the live tuning, and the static
attitude against the legacy firmware (the §7 A/B). Allow about an hour.

Everything the host can check already passed (`body/tools/host_tests.sh`:
toy-plant and through-the-joints closed loops, every sign flip diverges,
legacy `adjust()` reproduced within 0.16°, attitude A/B 0.038°). What only
Kilo can tell is whether the conventions the host tests assume are the real
ones: which way the IMU is mounted, and which way each joint moves the body.

**Conventions under test.**
- Kilo reports **nose down as +pitch** and **right side up as +roll**: the
  legacy `ypr` convention, measured on 2026-10-10 (nose up read −74°, left
  side up −88°). The P0 doc had both backwards.
- The posture references are the legacy header bytes verbatim (`sit`
  expects pitch −30°, nose up). The legacy pitch gains are used as they are
  (`LEGACY_PITCH_SIGN = +1`); the legacy roll gains assume left-up = +roll,
  so roll is negated before them (`LEGACY_ROLL_SIGN = -1`), both in
  `body/src/core/balance.h`. If a leg answers the wrong way in step 4, those
  two constants are what to flip.

**Before you start:**
- The battery is charged (USB alone cannot power the servos).
- The CH343 adapter is in one of the Pi's USB ports, and the Pi is not on the
  NyBoard's header UART.
- Steps 3, 4, 6 and 7: Kilo on its stand, legs and head free. Steps 5 and 8:
  Kilo on its feet on a board you can tilt by 10° (a book under one edge),
  with a hand ready to catch it.

## 0. Setup (Pi, once)

```sh
cd ~/OpenCat
git pull                                   # the branch with the balance layer
export PATH="$HOME/OpenCat/.venv/bin:$PATH" KILO_PORT=/dev/ttyACM0
~/OpenCat/.venv/bin/pip install -e mind    # picks up the new `kilo balance`
systemctl --user stop kilo 2>/dev/null     # if the daemon is installed: these steps run direct
```

With the daemon stopped every `kilo` command opens the port itself, as these
steps assume (`kilo daemon` must say "no daemon listening").

## 1. Host checks

```sh
body/tools/host_tests.sh                   # ends "host tests: ALL PASSED"
body/tools/size_check.sh                   # flash ~19.5 KB of 22528, PASS
(cd mind && python -m pytest -q)
```

## 2. Flash

```sh
kilo -v flash
kilo status
```

`verified: HELLO ...` with the new build, then `config OK`, `balance off`,
`overruns 0`. Kilo lies down, then limps (the P0 watchdog timeline).

**No reset after the flash** (the post-flash DTR settle, mind/README.md "DTR
and resets"). Run the flash verbose and read the uptime twice:

```sh
kilo -v flash --no-build                   # prints "settled /dev/ttyACM0: 3 open/close cycles ..."
                                           # and one line per cycle (HUPCL was set / clear, DTR 1 RTS 1)
kilo status | grep uptime                  # right after: uptime continuing (≥ 2 s), not 0
sleep 5; kilo status | grep -E 'uptime|reset'   # uptime ≈ 5 s more, reset cause unchanged
```

`kilo status` right after `kilo flash` must show the uptime continuing from the
verify's boot, not 0 (0 means the status open reset the body: the settle did
not take; note what the per-cycle lines said). If it fails, try
`kilo -v flash --no-build --settle-cycles 5` and report which cycle first shows
`HUPCL was clear`.

## 3. Attitude and rate signs (stand, no motion needed)

```sh
kilo monitor
```

Kilo is limp on the stand; tilt the whole stand slowly:

| Tilt | `pitch/roll` row | `rates` row (first two: pitch, roll) while moving |
|---|---|---|
| nose up | pitch goes **negative** | pitch rate negative while tilting up |
| nose down | pitch positive | pitch rate positive |
| left side up | roll goes **negative** | roll rate negative while tilting |
| right side up | roll positive | roll rate positive |

The rate must have the sign of the change of the angle (the host checked the
maths, this checks the gyro). **If any row is the other way round, stop here
and report it**: a wrong attitude sign makes balance push the wrong way.

## 4. Each leg answers a tilt the right way (stand, open loop)

On the stand the legs move but the body does not, so nothing closes the loop:
each tilt gives a steady leg offset you can watch. Use a strong gain and a
small dead band so the movement is obvious:

```sh
kilo balance on --kp 0.9 --deadband 1 --for 120
```

Kilo stands, and a line per second shows pitch, roll and `balance ACTIVE`
(read the attitude signs from these lines: only one `kilo` command can hold
the port, so no `kilo monitor` alongside). The default-gain equivalent is
`kilo posture balance --for 30`; it moves the legs only about half as far.
Tilt the stand about 15° each way, hold 2 s, and come back to level:

| Tilt (what `kilo` prints) | Legs should | Why |
|---|---|---|
| nose up (pitch −15) | **front legs fold** (shorter), **back legs straighten** (longer); the head tilts down | lower the nose |
| nose down (pitch +15) | front legs straighten, back legs fold; head tilts up | raise the nose |
| left side up (roll −15) | all four legs bend a little (the body crouches), the **left pair clearly more** than the right; the tail swings | lower the left side |
| right side up (roll +15) | all four bend, the **right pair more** | lower the right side |
| level | everything returns to STAND within about a second | dead band 1° |

Roll is a crouch with only a 20% left/right difference (legacy behaviour,
§3f), so look at the knees from the front: the side that should go down bends
further. At Kp 0.9 and 15° the knees move ≈ 10°, the shoulders ≈ 7°.

Sign check summary: the legs must always act to bring the raised end or side
**down**. If pitch is the wrong way round (nose up makes the front legs
straighten), flip `LEGACY_PITCH_SIGN` in `body/src/core/balance.h`; if roll is
(left side up makes the right pair bend further), flip `LEGACY_ROLL_SIGN`.
Such a flip also means the host plant in `body/test/test_balance.cpp` (which
models the leg-to-body physics from the posture tables) is wrong for that
axis and fails: report it rather than just flipping, then fix the plant,
re-run `body/tools/host_tests.sh`, `kilo -v flash`, and repeat this step.

Ctrl-C ends the hold; the watchdog then lays Kilo down.

## 5. Closed loop on a slope (H13, Kilo on its feet)

Put Kilo on the board, level, and hold balance with the defaults (Kp 0.5,
Kd 0.03 s, dead band 2°, clamp 15°):

```sh
kilo --log balance on --for 30
```

This is `kilo posture balance` (STAND with the balance flag) plus the default
gains sent explicitly, and it prints the body's pitch and roll once a second.
Only one `kilo` command can use the port at a time, so do not start a
`kilo monitor` next to it; `--log` writes every TELEMETRY (25 Hz) to
`~/kilo-logs/kilo-<time>.jsonl` for a closer look afterwards.

Tilt the board nose up by ≈10°, hold 5 s, back to level; then nose down,
left up, right up. Record the body's pitch/roll 1 s after each tilt:

- The body must lean **less** than the board: with Kp 0.5 and a leg response
  of about one degree of body per degree of correction, expect ≈7° for a 10°
  board (the §3f formula `(d + Kp·DB) / (1 + Kp)`). Less is better.
- H13 asks for 3°. If it is not reached at Kp 0.5, repeat with
  `kilo balance on --kp 0.99 --for 30` (the largest Kp the protocol carries).
  Report the residuals: if even that is above 3°, the gain scale needs a
  firmware change (§3f, "Proportional-only limits H13").
- No oscillation: the legs settle, they do not hunt. If they hunt, lower Kp or
  raise Kd (step 7).

## 6. Gating (stand)

```sh
kilo balance on --for 10            # balance ACTIVE in every line
kilo balance off --for 10           # balance off; tilting moves nothing
kilo posture stand --for 10         # off: no flag, no enable
kilo posture balance --for 5 --then-watch 15
```

The last one holds STAND with balance, then goes silent: the timeline must
show `balance_active` drop **at the watchdog stop** (0.5 s; the epoch bump
clears the posture flag) and stay off through REST and LIMP.

Also: `kilo balance on --posture sit --for 10` on the stand. SIT expects
−30° (nose up), the stand holds the body level, so the correction runs to
its clamp (15°) and the legs move toward a deeper sit: that is expected on a
stand. Do the real SIT test on the floor: the body should settle near −30°
and hold it when the board tilts.

## 7. Tuning

Each run holds the posture and prints pitch, roll and rates once a second;
add `--log` and plot the JSONL to see step responses at 25 Hz.

```sh
kilo balance on --kp 0.75 --for 30                  # stiffer
kilo balance on --kp 0.5 --kd 0.08 --for 30         # more damping
kilo balance on --deadband 1 --clamp 20 --for 30    # finer, larger range
```

Limits: Kp 0..0.99, Kd 0..0.255 s, dead band and clamp 0..30°. Nothing is
persisted: a reboot returns to the defaults. Note the values that work best.

## 8. Attitude A/B against the legacy firmware (§7, optional)

Put Kilo on a fixed wedge (or flat), not touched between the two readings.

```sh
kilo status | grep attitude                         # new firmware: pitch, roll
kilo flash --legacy
python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200); time.sleep(3); [s.write(b'v\n') or time.sleep(0.5) for _ in range(5)]; print(s.read(s.in_waiting).decode(errors='ignore'))"
kilo -v flash                                       # back to the new firmware
```

Legacy `v` prints `pitch<TAB>roll` as whole degrees (truncated). They must
agree with `kilo status` within 1° (2° allowing for the truncation). Repeat on
a second wedge angle if you can.

## What to report

- Step 3: the four signs (pitch, roll, both rates).
- Step 4: for each tilt, which legs moved which way; whether a constant had to
  be flipped.
- Step 5: the residual pitch/roll after 1 s for each tilt, at Kp 0.5 and 0.99;
  any hunting.
- Step 6: the `--then-watch` timeline.
- Step 7: the gains you settled on.
- Step 8: the two readings side by side.
