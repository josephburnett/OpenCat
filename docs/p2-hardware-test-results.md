# P2 hardware test: results

Procedure: `docs/p2-hardware-test.md`. Kilo = Petoi Nybble, NyBoard V1_2,
Raspberry Pi 2 B. Owner: Joseph Burnett.

## 1. Leg geometry (measured offline, 2026-10-11)

Measured by the owner with calipers, Kilo not powered. Now the firmware
defaults (`protocol/spec.toml` params 13-16).

| Quantity | Measured | Param | Note |
|---|---|---|---|
| Upper leg, axle to axle (x4) | 45 mm | `LEG_L1_MM` 45 | |
| Lower leg, axle to tip (x4) | 56 mm | | the foot is a disc of radius 6 mm |
| Lower leg, axle to foot-disc centre | 50 mm | `LEG_L2_MM` 50 | the IK controls the disc centre; the floor is 6 mm lower |
| Body, shoulder axle to hip axle | 123 mm | `LEG_LB_MM` 123 | |
| Foot track, left centre to right centre | 65 mm | `LEG_W_MM` 65 | |
| Knee directions | front knees bend back, back knees bend forward | `IK_SIGMA_FRONT` 1 | matches the firmware's convention (`body/src/core/ik.h`) |

All four legs measured the same. Default body height from these: 0.8 × 95 =
76 mm to the foot centres (82 mm above the floor); crawl 0.7 × 95 = 66 mm.

Still to measure on Kilo in `calib`: `LEG_SH_ZERO_DEG` (0?) and
`LEG_KN_ZERO_DEG` (90?), procedure section 1.

## 2. Foot nudge (joint signs)

**2026-10-11 17:1x, build ded3c1d8, Kilo on the desk.** `kilo posture
stance`, then `kilo pose set --xshift 20 --posture stance --for 3` alternating
with x-shift 0, twice. With the feet planted the body is what moves: the
owner saw the body slide **back, then forward** for +20 / 0 mm, i.e. the
feet went forward in the body frame. **Joint signs confirmed**
(`IK_SGN_SH`, `IK_SGN_KN`, `IK_SIGMA_FRONT` unchanged). The first try with
10 mm was too small to see.

STANCE itself: stood cleanly; on the desk it read pitch 6.6° nose down (the
desk's slope, STAND read the same) and roll 0.00°. Tick time stayed within
budget (no overruns; the cumulative max tick, 7.7-7.9 ms, did not rise while
in STANCE).

## 3. STANCE height and levelling (H13)

**2026-10-11 17:1x-17:2x, build ded3c1d8.** A background client held
STANCE with the posture balance flag and `SET_BALANCE` enable, kp 0.99
(kp_q8 253), the other gains default (kd 30 ms, dead band 2°, max 15°).
Board angles from the owner's phone level on the board.

| Surface | Balance off | Balance on | H13 (≤ 3°) |
|---|---|---|---|
| Desk alone (slopes nose down) | pitch 6.6°, roll 0.0° | pitch 2.1°, roll 0.6-1.7° | pass |
| Board 13° nose up | pitch 10.3° | pitch 2.1°, roll 0.7° | pass |
| Board 13° sideways, right side up | roll 17.9° (13° + Kilo's own ≈5° list, finding B4) | roll 5.0° first, 0.8° after re-engaging | pass |

- Owner, hand-tilting the board before the propped readings: "Balance was
  quite good. It's a flimsy board so Kilo was shaking it while I held it."
- The residual sits at the 2° dead band, as the host model predicted.
- **Two falls**, both onto the left side (P1 results doc, "Fall and head
  test"). Fall 1: the owner was moving the board around by hand; Kilo rolled
  over. Fall 2: during the sideways A/B, right side uphill; the owner did
  not describe the moment. The agent's inference (unverified): STANCE
  **without** balance on a 13° side slope plus Kilo's ≈5° list is 18° of
  roll, and with a 65 mm track at a 76 mm foot-centre height the static
  tipping roll is only about 22°. **Lesson either way:** never hold STANCE
  without balance on a side slope above ≈8°, and do slope tests on the
  floor; the procedure doc was updated.
- Stack free fell from 355 B (P1 build) to 248 B with the gait code in;
  battery 8.38 → 8.11 V over the 17 min session.

Still to do on Kilo: `LEG_SH_ZERO_DEG` / `LEG_KN_ZERO_DEG` in `calib`,
first steps on the floor, tuning, H14, H9.

## 4. First steps (2026-10-11 17:34-17:40, build ded3c1d8)

Kilo on a wood floor (owner), tethered to the Pi by the USB serial cable
(the Pi lives in its own enclosure with its own supply, not on Kilo).
Daemon started by hand (`--idle-posture none`); every run through
`kilo walk ... --for S`, which stops and waits for GAIT_STOPPED. Attitude
lines are the command's 1 Hz printout (firmware readings).

| # | Command | Firmware | Owner |
|---|---|---|---|
| 1 | `walk march --pattern trot --period 800 --step-h 8` (balance on, body 76) | roll 1.4 → 4.5 → −16.5° over 3 s, then **FELL +1** at 17:34:56, GAIT_STOPPED, RECOVERED 2.9 s later | "wobbled until it fell over" (left side); **got up on its own**: the first verified self-recovery |
| 2 | `walk march --pattern walk --period 1200 --step-h 8 --body-h 60 --no-balance --for 6` | pitch 1.8-4.2°, roll within ±3.8°, clean stop | "Looked good. Crouched down." |
| 3 | `walk forward --pattern walk --stride 10 --period 1200 --step-h 8 --body-h 60 --no-balance --for 6` | within ±5°, clean stop | moved "maybe like a centimeter" (5 cm expected) |
| 4 | `walk forward --pattern walk --stride 25 --period 1200 --step-h 8 --body-h 70 --no-balance --for 8` | pitch 1.7-3.0°, roll within ±1.8°, clean stop | **"15 cm exactly"** (15 cm expected), "slight to the left" |

- **Kilo walks.** Run 4 is the first measured match between the gait maths
  and the floor.
- Trot in place with balance on fell within 3 s. Inference (unverified):
  the diagonal-pair support on a 65 mm track rocks, and the geometric
  balance may feed the rocking; to test trot again, start with
  `--no-balance` and a lower body, and compare.
- 10 mm steps on wood from a 60 mm crouch mostly slid (1 of 5 cm); 25 mm
  steps at 70 mm did not slip measurably.
- Left drift: inference, the right-leaning list (left-front knee, P1 finding
  B4) loads the right feet more; recalibrate the knee before judging.
- Body stats at the end: max tick 7.9 ms (cumulative; never rose during
  walking), overruns 0, stack free 238 B (355 B before P2, 248 B after the
  flash: the gait and its stop sequence use about 10 B more stack than
  STANCE). Battery 8.38 V at 17:07 → 7.99 V at 17:41.
- Not run: turning, the default (faster, higher) walk, trot forward, crawl,
  H14, H9, start/stop video.

## 5. Tuning

Not started. First candidates from §4: trot stability (balance off, body
height, step height), stride vs slip on wood, the left drift after the knee
calibration.
