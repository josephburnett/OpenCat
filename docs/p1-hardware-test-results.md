# P1 hardware test results

Log of `docs/p1-hardware-test.md` on Kilo. First session 2026-10-11, from
the home server over the LAN to the Pi, firmware build **49f23f4f**
(`mega` 49f23f4), Kilo on a desk that slopes about 4° nose down. The
owner's hands did the tilts; nothing walked.

## Balance test

| Step | Result |
|---|---|
| 0 Setup | Pi pulled to 49f23f4, mind reinstalled, `kilo.service` stopped. Battery 8176 mV (89 %) at the start, 7927 mV (75 %) after 70 min. |
| 1 Host checks | All four suites pass on the server (host tests ALL PASSED, size 22132 B PASS, protocol all passed, mind 346 passed). |
| 2 Flash | `kilo -v flash`: verified HELLO proto 1.2 build 49f23f4f reset COLD. **No reset after the flash**: settle reported 3 cycles, all `HUPCL was clear; DTR 1 RTS 1`; `kilo status` right after read uptime 2 s and 10 s five seconds later, reset cause unchanged. The settle fix is verified on hardware. Stack free 353 B (was 732 B before the trick store). |
| 3 Attitude and rate signs | **All four correct.** Kilo limp, hand tilted about 30°: nose up pitch −31° with negative pitch rate; nose down +41°, positive; left side up roll −46°, negative; right side up +35°, positive (from the 25 Hz telemetry log, rates summed over the tilt). |
| 4 Leg response (open loop) | **All four correct**, done in the hand instead of on the stand, `kilo balance on --kp 0.9 --deadband 1`: nose up → front legs fold, back straighten; nose down → the reverse; left up → left pair bends more; right up → right pair bends more; back to STAND within a second after each. `LEGACY_PITCH_SIGN` and `LEGACY_ROLL_SIGN` stay as they are. |
| 5 Slope residual (H13) | **Not reached, by a wide margin.** Hand-tilted board, four tilts: balance off (baseline) body 16°/13°/13°/13°; Kp 0.5: 26°/18°/18°/17°; Kp 0.99: 26°/20°/15°/18° (hand tilts vary, so these are not comparable). Fixed-angle A/B through the daemon, board propped 15° nose up (phone), balance toggled off/on three times at Kp 0.99: body **−10.2° off, −8.8° on**, every time. So 1.4° of body for a computed correction of about 8°: **≈0.17° of body per degree of correction**, against the ≈1:1 the design assumed (§3f). The legs settled, no hunting. The owner: the legs move the right way but not far enough to keep the centre of gravity; tipping further would topple Kilo. |
| 6 Gating | `kilo balance on` ACTIVE every line; `balance off` holds STAND with no correction; `posture balance --for 5 --then-watch 15`: `balance_active` dropped at the 0.5 s watchdog stop (epoch 204→205), REFLEX REST at 1.5 s, servos off (LIMP) at 12.5 s. **Finding B1** below: a plain `posture stand` through the daemon showed `balance_active` after a killed `kilo balance on`. |
| 7 Tuning | Not done: moot until the gain scale changes (step 5). |
| 8 Attitude A/B | Done against a phone instead of the legacy firmware. Board 15° nose up, 6° listing right (phone). New firmware, REST held: pitch −12.8°, roll −7.3° (left side up); limp: −13.1°/−8.4°. Legacy firmware, its boot REST, `v` 5 s after boot: `-34  0`, which disagrees with the phone. **The new firmware is the one that is right**; the legacy reading soon after boot is not a usable reference (its DMP has probably not converged). Also, the step as written compares a limp body with a held REST; use a phone. |

### Findings

- **B1. SET_BALANCE enable survives the epoch bump.** `body_epoch_bump()`
  clears `balance_req` (the posture flag) but not `bal.enable`, so after a
  `kilo balance on` client dies, every later STAND balances until something
  sends `SET_BALANCE enable 0`. The spec calls SET_BALANCE a goal, and goals
  go stale at an epoch bump. Fix: clear `b.bal.enable` in
  `body_epoch_bump()`; add the case to `test_body.cpp`.
- **B2. The joint-space balance is about six times too weak for H13.** The
  legacy gain table (knee 0.45, shoulder 0.3 per degree of correction) plus
  the leg geometry give ≈0.17° of body per degree of correction. For a 3°
  residual on a 10° slope, `(d + X·DB)/(1 + X) = 3` with `X = G·Kp` needs
  `X = 7` at DB 2° (Kp ≈ 40) or `X = 3.5` at DB 1° (Kp ≈ 20): far outside
  the protocol's Kp ≤ 0.99 and, at the knee, outside what the legacy map
  was ever meant to do. Options are in §8 of the architecture (gain scale
  change) or a proper body-levelling through the P2 IK. Decision pending.
- **B3. Step 8 protocol.** Flashing moves the legs and a limp body sags, so
  "not touched between the readings" cannot hold. A phone level on the
  board is the better reference, and it showed the legacy readout to be the
  wrong side anyway.
- **B4. Left front knee at REST.** The owner: the left front knee (joint 12,
  legacy calibration −21°, right front +7°) does not retract as far as the
  right one at REST, so Kilo lists right (the roll reading agreed: −6°).
  Not a clamp (REST asks −45°, the limit is −60°) and not the pulse maths
  (verified against the dump). To check with `kilo cal set 12` and the
  owner's eye; the legacy REST roll printed 0 but that readout is suspect
  (B3).
- **B5. Background `kilo` clients ignore SIGINT** when started with `nohup
  … &` from a non-interactive shell (the shell sets SIGINT to ignore).
  SIGTERM ends them. Not a mind bug; worth a note in mind/README.md for
  remote use.
- `imu_stale` rose from 3 to 16 over 700 s (P0 saw 3 over 153 s). Watch.
- `kilo monitor --json` lines carry pitch/roll but not the rates; the
  `--log` TELEMETRY file has them (`rate_pitch`, `rate_roll`, 1/16°/s).

## Tricks test

**2026-10-11 evening, build ded3c1d8 (main, P2 firmware), Kilo on the
desk on battery (8.38 V at the start), owner watching, daemon started by
hand with `--idle-posture none`.** T2 basics only:

| Step | Result |
|---|---|
| STAND from LIMP | OK (owner). Pitch read 6.5° nose down: the desk slopes; STANCE later read the same. |
| SIT, STAND, REST | OK (owner). |
| `kilo trick play stretch` | **Too fast** (owner). Root cause: the legacy `str` is a single-frame posture; the legacy firmware held it until the next command, the import moves there in 25 ticks and the trick ends at once with end posture STAND, so Kilo flicks into the stretch and straight back. Fix: the importer gives single-frame legacy postures a hold (see the commit after this session); `kilo trick play --for S` already holds longer. |
| REST list | Kilo still leans right at REST (roll −8° to −11°): finding B4, the left-front knee (joint 12). Not jogged yet. |

Social tricks, `--speed 0.5`, push-up looping (H9), `kilo look`: not run.

## Fall and head test

**Two unplanned roll falls during the P2 levelling test (2026-10-11,
build ded3c1d8), both onto the LEFT side**, see
`docs/p2-hardware-test-results.md` §3:

| | Fall 1 | Fall 2 |
|---|---|---|
| Time | 17:15:46 | 17:21:35 |
| EVENT FELL arg | +1 (roll) | +1 (roll) |
| Side (owner) | left | left (right side was uphill) |
| RECOVERED after | 3.6 s | 5.7 s |
| Self-recovery | **not verified**: the owner picked Kilo up; the get-up routine ran in the air | same |

- Detection: works; `FELL +1` for a fall onto the left side agrees with the
  firmware's roll convention (right side up = positive).
- Recovery: the sequence runs after the fall, but whether it gets Kilo up
  on its own is unknown; both times the owner lifted Kilo while it played.
  The planned five-per-side test on a soft surface is still to do.
- `kilo look`: not run.
