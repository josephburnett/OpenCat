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

Not started.

## 3. STANCE height and levelling (H13)

Not started.

## 4. First steps

Not started.

## 5. Tuning

Not started.
