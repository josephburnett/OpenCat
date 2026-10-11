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

A failed verify now exits non-zero: `kilo flash` checks for the HELLO twice
(`verifying once more`), then prints `verify FAILED: ...` with the last HELLO
it saw and exits 1, so `kilo -v flash && kilo status` stops there. A port
still held by a daemon `kilo flash` could not reach (`--direct`, root's, or
another user's) is refused before avrdude runs ("... is in use"); stop it,
or pass `--force`.

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

---

# P1 tricks hardware test

The first run of the keyframe trick player (docs/architecture.md §3e, the
TRICK owner of §3c) on Kilo: every first-version trick uploaded and played
from the Pi, first on the stand and then on the floor, each compared side by
side with the legacy firmware, then the H9 looping trick. Do it after the
balance steps above (same setup, same branch). Allow about two hours.

What the host already checked (`body/tools/host_tests.sh`, test `keyframe`):
the upload rules (order, CRC, 24-frame limit), tick-exact timing and easing
(the last tick of every move lands on its target, every move is monotone),
holds, loops (0/1 = once, N, 255 = until stopped), speed, mirror, the
crossfades in and out, the end posture, TRICK_DONE reasons, gating and owner
precedence, and a replay of all 13 library tricks (as the mind encodes them,
also mirrored and at 1.5×) with every angle inside the joint limits and the
frame timing exactly as the JSON says. What only Kilo can tell is whether the
tricks look right: the imported durations (`LEGACY_STEP_MS` 5.5 ms, an
estimate), the clamped angles, and how the moves feel at full speed.

**Before you start:** as for the balance test (charged battery, CH343 on the
Pi, `KILO_PORT` set), plus a phone on a tripod or propped up, filming from
the side at the same spot for both firmwares.

## T0. Host checks and flash

```sh
body/tools/host_tests.sh                   # ends "host tests: ALL PASSED" (incl. "keyframe ... 0 failed")
body/tools/size_check.sh                   # flash ~22.5 KB of 22528, globals 1432 of 1540, PASS
(cd mind && python -m pytest -q)
kilo -v flash
kilo status                                # `proto 1.2`
kilo trick list                            # 13 tricks, no body needed
kilo trick show hi                         # header, notes, keyframes
```

## T1. Upload and the cache

```sh
kilo trick upload hi                       # "uploaded"
kilo trick upload hi                       # "already on the body (cached)": one TRICK_COMMIT only
kilo trick upload hi --force               # uploaded again
```

## T2. Every trick on the stand

Kilo on its stand, legs and head free. For each trick, basics first, then
social:

```sh
for t in sit rest stand stretch balance  hi wash_face check_around sniff scratch pee push_up butt_up; do
  kilo trick play $t --posture stand; sleep 1
done
kilo trick play hi --posture stand --mirror          # mirrored: the other paw
kilo trick play hi --posture stand --speed 0.5       # half speed: easy to watch the easing
```

Each line ends `TRICK_DONE DONE after X s (expected Y s)`, X within ~0.1 s of
Y. Watch for:

- **No jump at the start.** The first frame moves from wherever the joints
  are (the stand posture). A joint that snaps is a bug (except frame 0 of
  `scratch`, `dur 1`: a legacy "jump to", paced by the 10°/tick slew limit).
- **No jump between frames**: cosine moves slow down into each keyframe and
  start again from rest.
- **No jump at the end**: the joints crossfade over 200 ms (`XFADE_TICKS`)
  into the trick's end posture (STAND for most, SIT after `sit` and
  `wash_face`, REST after `rest`), which then holds.
- Clamped joints (`sit`, `hi`, `wash_face`, `scratch`: back shoulders at
  −80°; `butt_up`, `wash_face`: front shoulders at 80°; `stretch`: head
  tilt at 45°) stop at the limit without buzzing.
- `kilo monitor` in a second terminal shows mode TRICK, owners `TRICK`, the
  `trick` flag and the frame number while it plays.

## T3. Stop and preemption (stand)

```sh
kilo trick play push_up --posture stand --repeats 255 --for 3   # stops after 3 s: TRICK_DONE STOPPED
kilo trick play wash_face --posture stand --then-watch 15       # then the watchdog timeline
```

The first ends with a 200 ms crossfade to STAND from wherever the stop caught
it. The second: after `TRICK_DONE DONE` the CLI goes silent; the timeline
shows `WATCHDOG` at 0.5 s, REST at 1.5 s, LIMP 10 s later. Also, during a
long play, pull the CH343 cable (or Ctrl-Z the CLI): the trick must stop at
the watchdog's 0.5 s (TRICK_DONE PREEMPTED in the log, epoch +1), crossfade
to its end posture, then REST.

## T4. Every trick on the floor

Repeat T2 with Kilo on its feet on the floor, a hand ready. Now the weight
is on the legs: note any trick that tips Kilo, slides, or where a foot drags.
The balancing tricks (`sit`, `stand`, `stretch`, `balance`, `butt_up`: the
`balance` flag on their frame) hold their expected attitude (sit −30°,
stretch and butt_up +15° nose down) when you tilt the board, like step 5;
the social tricks never balance (legacy `perform()` did not either).

## T5. Side by side with the legacy firmware

For each trick, film Kilo doing it on the new firmware (T4), then on the
legacy one, same spot and camera:

```sh
kilo flash --legacy
python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200); time.sleep(3); [s.write(t.encode()+b'\n') or time.sleep(6) for t in ('kup','khi','kup','kwsf','kup','kck','kup','ksnf','kup','kscrh','kup','kpee','kup','kpu','kup','kbuttUp','kup','ksit','kup','kstr','kbalance','krest')]"
kilo -v flash                                        # back to the new firmware
```

(Legacy tokens: `k` + the legacy name; a trailing `R` mirrors, e.g. `khiR`.)
Compare, per trick: the poses at each keyframe, the overall duration and
rhythm (the new firmware's durations come from `LEGACY_STEP_MS` = 5.5 ms; if
the legacy one is consistently faster or slower, measure the ratio from the
video and change `LEGACY_STEP_MS` in `mind/kilo/tricks/legacy.py`, re-run the
importer, `python3 body/tools/gen_trick_library.py`), the loop count, and how
it ends (legacy goes back to whatever it did before; Kilo goes to the end
posture).

## T6. H9: the looping trick

```sh
kilo trick play push_up --posture stand --repeats 255 --for 60
kilo status                                         # overruns 0, max tick, stack free
```

60 s of push-ups (the loop section until stopped), then `TRICK_DONE STOPPED`
and a crossfade to STAND. H9's numbers: `max_tick_us` < 12000, 0 overruns,
`stack_free_min` > 150 B (`kilo status` after the run). Watch for the
loop restart: the first move of each repeat starts from the last frame of
the loop, with no jump.

## The owner's judgement (§7 P1 acceptance)

For each first-version trick (basics: sit, rest, stand, stretch, balance;
social: hi, wash face, check around, sniff, scratch, pee, push-up, butt-up):

- [ ] it is recognisably the same trick as on the legacy firmware;
- [ ] its speed and rhythm match (or the difference is noted);
- [ ] no visible jump at its start, between frames, or into the end posture;
- [ ] it ends in a sensible posture and holds it;
- [ ] on the floor it does not tip Kilo over (or `FALL_OK` is needed: note it);
- [ ] mirrored (`--mirror`) it is the other side's version.

And once: H9 passes (60 s loop, numbers above), and a stop or a link loss in
the middle of a trick always ends in a smooth crossfade.

## What to report (tricks)

- T2/T4: per trick, DONE times vs expected, any jump, buzzing or tipping.
- T3: the two timelines.
- T5: per trick, same/different, the measured speed ratio if any.
- T6: `kilo status` after the 60 s loop.
- The checklist above, ticked per trick.

---

# P1 fall and head hardware test

The first run of the fall reflex (docs/architecture.md §3g "Fall", "As
implemented in P1") and the head layer (§3c layer 1) on Kilo, and the H7
acceptance. Do it after the tricks test (same setup and branch). Allow about
an hour.

What the host already checked (`body/tools/host_tests.sh`, test `reflex`):
the trip needs 6 consecutive samples over 75° (5 and a dip do not trip, a
single bad sample never does, missing samples neither count nor reset),
roll falls on both sides and on the back, pitch falls (nose or tail down),
the still wait, the `rc` frames tick by tick (dur 11/5/18, hold 30/25/0,
mirrored for roll > 0 and flipped on every other try), RECOVERED after
0.5 s upright, three tries then REST, RECOVERY_FAILED and LIMP, FALL_OK,
LIMP/CALIB/IMU_FAULT gating, a trick ending with TRICK_DONE(FELL), STALE and
FALLEN refusals, LIMP ending the reflex, the watchdog taking over; the head
hold's slew, timeout, release, epoch release, balance exclusion, owner
precedence and TELEMETRY. What only Kilo can tell: whether `rc` gets it back
on its feet, and **which way it must be mirrored** (the legacy firmware
mirrored it at random; Kilo's choice, unmirrored for the right side down, is
a reading of the legacy frames, not a measurement).

**Before you start:** as for the balance test (charged battery, CH343 on the
Pi, `KILO_PORT` set), plus a soft surface big enough for Kilo to roll on (a
bed, a sofa cushion or foam), and a phone filming each fall from the front.
Kilo must not fall off the surface.

## F0. Host checks and flash

```sh
body/tools/host_tests.sh                   # ALL PASSED, incl. "reflex ... 0 failed"
body/tools/size_check.sh                   # flash ~22.5 KB of 22528, globals 1432 of 1540, PASS
(cd mind && python -m pytest -q)
kilo -v flash
kilo status                                # stack free: note it (was ≈730 B before the trick store)
```

## F1. Three terminals through the daemon

The fall test needs the link fed the whole time (a silent link means the
watchdog's REST takes the joints from the fall reflex after 1.5 s) and more
than one client, so run the daemon in the foreground, with no idle posture
of its own:

```sh
# terminal A
kilo run --idle-posture none
# terminal B: mode, owners, epoch, roll/pitch and the last events, live
kilo monitor
# terminal C: the goals
kilo posture stand                         # holds STAND until the body clears it
```

## F2. Fall on each side (H7)

Kilo standing on the soft surface. Push it over onto its **left** side (right
side up: roll ≈ +90°) and let go.

Expected, in terminal B: within ≈0.1 s of passing 75° `FELL(1)`, the epoch
+1, mode FALLEN, owners REFLEX/REFLEX, a short chirp; terminal C prints
"posture goal cleared by the body" and exits (the mind dropped its old-epoch
goal: that is "old goals refused as STALE until the mind re-plans"; a
`GOAL_REFUSED` with reason STALE may also appear if a refresh was in flight).
Once Kilo lies still (0.2 s), mode RECOVERING and `rc` plays (≈1.8 s: head
turns, the legs on the ground side sweep and push). If it ends upright,
0.5 s later `RECOVERED(0)`, mode IDLE and it stands (STAND). If not, after
1 s the next try, mirrored the other way; after three failed tries REST,
`RECOVERY_FAILED` and LIMP 3 s later.

Then the **right** side (roll ≈ −90°, `FELL(-1)`), then once on its **back**
(roll ≈ ±180°, `FELL(±1)`). Re-run `kilo posture stand` in terminal C before
each fall. Five falls per side:

| Fall | FELL arg | try that recovered (1, 2, 3 or none) | RECOVERED? | notes |
|---|---|---|---|---|
| left 1..5 | | | | |
| right 1..5 | | | | |
| back | | | | |

**H7 passes** with RECOVERED in ≥ 4 of 5 on each side, mostly on try 1. If a
side keeps failing try 1 and succeeding on try 2, the mirror convention is
backwards: report it (the fix is the `b.att.roll > 0` in `fall_step()`,
`body/src/core/body.cpp`).

## F3. Nose down, handling, and goals while fallen

- **Nose down.** Lift Kilo and hold it nose down, vertical, still: `FELL(2)`,
  then REST, `RECOVERY_FAILED` at once and LIMP 3 s later (no `rc` for a
  pitch fall). Tail down: `FELL(-2)`, the same.
- **Handling.** Pick Kilo up while it stands and tilt it by hand to ≈60°
  and back, a few times, quickly: no FELL (P0 finding 8). Past 75° for more
  than 0.12 s is a fall by design.
- **Goals while fallen.** During a FALLEN or RECOVERING phase run
  `kilo posture sit` in terminal C: refused (`GOAL_REFUSED`, reason FALLEN,
  in terminal B). `kilo posture limp` ends the reflex at once (LIMP).
- **A trick.** `kilo trick play push_up --posture stand --repeats 255`, then
  push Kilo over: the CLI prints `TRICK_DONE FELL`, and the fall goes on as
  in F2.
- **LIMP and CALIB.** `kilo posture limp`, lay Kilo on its side: nothing.

## F4. Head

Stop the daemon (Ctrl-C in terminal A) or keep it; either way:

```sh
kilo look 30 0 --posture stand --for 3     # pans to 30° at 120°/s (≈0.25 s), holds 3 s, back
kilo look -40 -20 --tail 30 --speed 0 --for 3    # at the safety slew (500°/s): a quick snap
kilo look 0 30 --for 3                     # tilt up (the limit is 45°)
kilo look 100 0 --for 5                    # Ctrl-C after 1 s: released at once, crossfade back
```

Expected: the head moves at the asked speed, stays put while the command
runs (the goal is refreshed at 5 Hz), and returns to the posture's head
smoothly (a 0.2 s crossfade) when the command ends. In `kilo monitor`
(through the daemon): head owner `PI_HOLD`, flag `head_held`. With
`kilo balance on` running in another terminal through the daemon, tilt the
board: the legs correct, the held head does not move.

## What to report (fall and head)

- F2: the table, and the video of one fall per side.
- F3: the four outcomes.
- F4: whether the speeds and the return look right, and any buzzing at the
  hold.
- `kilo status` at the end: overruns, max tick, **stack free**.
