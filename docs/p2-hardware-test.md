# P2 walking hardware test

The first run of the IK, STANCE and the gait generator on Kilo
(docs/architecture.md §3d, "As implemented in P2"; §3f for the geometric
balance). In order: measure the legs and store them, check the joint signs
with the foot nudge, check STANCE's height and levelling (H13), then the
first steps on the floor, then tuning. Work at your own pace; every step
waits for you, nothing here is timed, and any step can be stopped and
resumed later.

Everything the host can check already passed (`body/tools/host_tests.sh`:
`test_ik` round trips within 0.11 mm / 0.07°, `test_gait` swing order,
start/stop, landing, turning, pre-emption, the geometric balance closed
through a plant). What only Kilo can tell: the real leg lengths, which way
each joint really moves a foot (there is no position feedback), and how the
gait behaves with real servos, friction and weight.

**Conventions under test** (`body/src/core/ik.h`):
- `IK_SGN_SH = -1`: a positive OpenCat shoulder angle swings the foot
  BACKWARD, on every leg.
- `IK_SGN_KN = -1`: a positive OpenCat knee angle reduces the knee bend of
  the front legs (and, mirrored, of the back legs).
- `IK_SIGMA_FRONT = 1`: the front "elbows" point backward and the back knees
  forward (as in a cat); `ik_sigma()` gives the back legs the opposite.
- `LEG_SH_ZERO_DEG = 0`, `LEG_KN_ZERO_DEG = 90`: at the calibration posture
  (all joints 0) the upper leg is vertical and the lower leg at 90° to it.

**Safety, always:**
- Walking happens **on the floor, never on the desk**: Kilo will fall off.
  Use a soft surface (a rug or a yoga mat) with room around it.
- Keep a way to stop at hand: `kilo posture rest` (or `limp`) in a terminal,
  or simply stop the sending process: the body's link watchdog stops the
  gait within half a second plus one period and lies down after 1.5 s.
- The battery is charged; the CH343 adapter is in the Pi's USB port.

**Names of the Pi-side commands.** The mind's walking commands are being
written alongside this firmware; below they are described by the goal fields
they send (`SET_GAIT`: run, pattern WALK/TROT/CRAWL, period_ms, duty /256,
stride_mm, turn_mm (+ = left), step_h_mm, body_h_mm, flags bob/wag/balance;
`SET_BODY_POSE`: pitch, roll (degrees), dheight_mm, xshift_mm). Use whatever
`kilo walk` / `kilo pose` (or the names they end up with) offer for those
fields. `SET_POSTURE STANCE` is `kilo posture stance` if the CLI lists it.

## 0. Setup (Pi)

```sh
cd ~/OpenCat
git pull                                   # the branch with P2
export PATH="$HOME/OpenCat/.venv/bin:$PATH" KILO_PORT=/dev/ttyACM0
~/OpenCat/.venv/bin/pip install -e mind
systemctl --user stop kilo 2>/dev/null     # these steps open the port directly
body/tools/host_tests.sh                   # ends "host tests: ALL PASSED"
kilo -v flash
kilo status                                # uptime keeps counting, config OK
```

## 1. Measure the legs and store them

Put Kilo in the calibration posture (`kilo posture calib`) on its stand, so
the joints sit at their zero. With calipers, on one front leg and one back
leg (and a second of each if they look different):

| Param | What to measure | Default (a guess) |
|---|---|---|
| `LEG_L1_MM` | Shoulder axle centre to knee axle centre | 50 |
| `LEG_L2_MM` | Knee axle centre to the point of the foot that touches the floor | 50 |
| `LEG_LB_MM` | Front shoulder axle to back shoulder axle, along the body | 105 |
| `LEG_W_MM` | Left foot to right foot, across the body (the track) | 60 |
| `LEG_SH_ZERO_DEG` | Angle of the upper leg from vertical in `calib`, + = foot forward | 0 |
| `LEG_KN_ZERO_DEG` | Angle between upper and lower leg in `calib` (90 = square) | 90 |

If the foot's contact point is not on the line from the knee axle through the
foot, note the offset angle too: there is no param for it yet; write it down
in the results and it will be added to `LEG_KN_ZERO_DEG`'s meaning or as a new
param.

Then, whole millimetres and degrees:

```sh
kilo param set LEG_L1_MM 48 --persist      # your values
kilo param set LEG_L2_MM 55 --persist
kilo param set LEG_LB_MM 104 --persist
kilo param set LEG_W_MM 62 --persist
kilo param set LEG_SH_ZERO_DEG 0 --persist
kilo param set LEG_KN_ZERO_DEG 90 --persist
kilo param list                            # check them
```

The params are live: no reboot needed. Front and back legs share one set;
if they differ by more than a millimetre, note it.

## 2. Foot nudge: the joint signs

On the stand first (legs free), then on the floor.

1. `SET_POSTURE STANCE` (`kilo posture stance`). Every foot should end up
   roughly under its shoulder, knees bent the way they are in `up`
   (front knees behind the foot line, back knees in front, like `kilo
   posture stand`). Compare with STAND side by side: STANCE is the same
   shape, slightly lower.
2. `SET_BODY_POSE` with `xshift_mm = +10` (everything else 0). **Every foot
   must move about 10 mm FORWARD** (toward the head), without rising or
   sinking. Then `xshift_mm = 0`: they come back.
3. `SET_BODY_POSE` with `dheight_mm = -10`: every foot rises 10 mm toward
   the body (Kilo crouches); `+10`: it stands taller. Back to 0.

What to flip (in `body/src/core/ik.h`, then rebuild and flash), and tell
the session working on the code:
- The feet move **backward** on +10 mm x-shift: flip `IK_SGN_SH` to `1`.
- STANCE has the knees bent the **wrong way** (front knees in front of the
  feet), or a leg folds up instead of standing: flip `IK_SGN_KN`, and if
  that alone does not do it, `IK_SIGMA_FRONT`.
- Front and back knees of Kilo point the **same way** (not mirrored): the
  back legs need the front's sigma; that is a code change in `ik_sigma()`.
- Only the height is off (feet move the right way, but STANCE is visibly
  higher or lower than 80% of the stretched leg): recheck `LEG_L1_MM`,
  `LEG_L2_MM`, `LEG_KN_ZERO_DEG`.

Record in the results which constants were changed, if any.

## 3. STANCE height and levelling (H13)

Kilo on its feet on a board on the floor, a phone with a level app lying on
its back (or `kilo monitor` for the IMU's pitch and roll).

1. STANCE on the level board. Read pitch and roll: both should be within
   about 1-2° of 0. If one end sits lower, note how much: a consistent
   offset is a geometry error (Lb or the legs) or calibration.
2. Pitch: `SET_BODY_POSE pitch = +5`: the nose goes DOWN 5° (the level
   reads it). `-5`: nose up. Roll `+5`: the RIGHT side goes UP. Back to 0.
3. Geometric balance: STANCE with the balance flag (`SET_POSTURE STANCE`
   flags bit 0, or `kilo balance on --kp 0.99` holding STANCE if the CLI
   offers it), at the full gain `kp_q8` 255 (Kp 0.99). The default 0.5
   only levels half the slope by design (§3f).
4. Tilt the board 10° nose up (a book under the front edge), wait for Kilo
   to settle, read the body pitch: **H13 passes if it is within 3° of
   level**. Then nose down, then each side up. Note each reading, and
   whether it got there smoothly or oscillated.
5. If the body overshoots or wobbles, lower `kd` or `kp`; if it levels only
   partly at Kp 0.99, the legs move the body less than the model says
   (geometry params) or the correction hit its clamp (15°; raise it with
   `--clamp` up to 30).

## 4. First steps (on the floor)

Soft surface, room around, Kilo starts in STANCE (or anywhere: a run goal
moves it to STANCE first). Start small and slow:

1. TROT in place: `run 1, pattern TROT, period 800, duty 0 (default),
   stride 0, turn 0, step_h 8, body_h 0 (default), flags bob|wag|balance`.
   The diagonal pairs (left front with right back) lift together. Watch
   for a foot scuffing, a leg hitting its limit, or the body rocking.
2. Stop (`run 0`): the stepping fades over one period, every foot lands,
   STANCE holds, the mind receives one `GAIT_STOPPED`. No foot should jump
   at the start or the stop (H-P2 acceptance: slow-motion video if you can).
3. TROT forward: stride 10, then 20, then the default 35, period 500. Then
   backward (stride -15).
4. Turning in place: stride 0, turn +15 (left), then -15.
5. WALK (period 800, stride 20, step_h 12) and CRAWL (period 1200, stride
   15, step_h 15). One foot in the air at a time, order left back, left
   front, right back, right front.

If Kilo falls, the fall reflex takes over as before (the gait ends, `rc`,
STANCE is lost): pick it up, `kilo posture rest`, and note what happened
just before.

## 5. Tuning

One parameter at a time, in this order, and note the values that felt best:

1. **Period**: the slowest that still looks like walking; faster when the
   rest is stable.
2. **Duty**: higher (more feet on the ground) when it wobbles; lower for a
   livelier trot.
3. **Stride**: up until the feet start to slip or the legs hit a limit.
4. **Step height**: just enough to clear the surface; too high makes the
   swing fast (the legs move up to ~9°/tick at the defaults).
5. **Body height** (`body_h_mm`): lower is more stable, higher gives room
   to step.
6. **Balance** while walking (flags bit 2, `kilo balance` gains): only
   after the gait is stable without it.

Then the acceptance runs, as far as they go:
- **H14**: walk 2 m straight (drift under 20 cm), a 360° turn in place
  (under 20 s), 30 s of trot without a fall.
- **H9**: 10 minutes of trot with 50 Hz telemetry and JOINTS: `kilo status`
  afterwards, `max_tick_us` under 12000, no overruns, `stack_free_min`
  over 150 B.

## 6. Record the results

In `docs/p2-hardware-test-results.md`:
- the build (`kilo status` build hash) and the date;
- the measured L1, L2, Lb, W, the zero angles, any foot offset, and whether
  front and back legs differed;
- the foot nudge result per leg, and which constants (if any) were flipped;
- STANCE's level reading, the pose checks, the H13 readings per direction
  with the gains used;
- for each gait tried: the parameters, what it looked like, any scuffing,
  slipping or falls; the tuned parameters you liked best;
- H14 and H9 numbers (`max_tick_us`, overruns, `stack_free_min`);
- anything surprising, with a video file name if you took one.
