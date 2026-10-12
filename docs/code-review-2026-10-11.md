<!-- Written by a fresh-context Claude Fable 5.1 review agent on 2026-10-11 at the owner's request: a read-only review of main at f019646. Kept verbatim; the main session's notes are at the end. -->

# Kilo repository review (main, working tree as of 2026-10-11 evening)

Suites run locally on this tree: `body/tools/host_tests.sh` ALL PASSED (15 binaries); `protocol/test.sh` passed (1,144,542 C checks incl. 1e6 fuzz round trips, 159 Python tests, avr-gcc objects build); `.venv/bin/python -m pytest mind/tests -q` 392 passed in 242.8 s. `size_check.sh` not run (needs arduino-cli; handoff: 26938 B flash of 30720, 1517 B globals of 1540). The tree includes uncommitted edits (the `--balance` default in `mind/kilo/daemon.py`, `cli.py`, `ir.py` and their tests); they pass.

Sizes for orientation: body `src/` ≈ 7.5 k lines (of which `proto/kilo_protocol.h` 1283 generated), tests ≈ 4.4 k; mind `kilo/` ≈ 16 k (2557 generated), tests ≈ 7.3 k; docs 4.7 k lines, `mind/README.md` alone 1475.

## 1. What is well architected

1. **One protocol source, enforced in both directions.** `protocol/spec.toml` → `gen.py` → committed C header and Python module → byte-identical vendored copies (`protocol/vendor.py --check` runs in `test.sh`; `mind/tests/test_codec.py` also fails on drift). On the body, `body/src/core/link.cpp:14-70` is a wall of `static_assert`s tying every firmware enum (events, goal reasons, postures, owners, reset causes, config states, easing, gait pattern rows) to the generated constants, so the firmware cannot silently disagree with the spec. On the mind, `kilo/link/consts.py` derives everything from the generated module. Cross-language vectors plus a 1e6-iteration ASan fuzz on the codec. This is the strongest part of the repo and the reason the P0/P1/P2 hardware sessions found so few protocol bugs.

2. **The core/hal/app split and the host-test discipline.** `body/src/core/*` is plain C++ with no AVR headers, linted for bare `int` and for `float`/`double` (`host_tests.sh:14-27`), built with `-fsanitize=address,undefined -Werror`. The one hardware seam for the link (`link_hal_*`, `core/link.h:46-67`) is implemented by `app/link_hal.cpp` on the AVR and `test/fake_link_hal.cpp` on the host, so the exact dispatch that runs on Kilo runs in `test_link.cpp` and in the `link_harness` used by `mind/tests/test_crosscheck.py`. `size_check.sh` fails the build on forbidden symbols (Serial/Wire/heap/String/soft-float) and on missing size flags. `main()` is owned (`app/main.cpp`), so no Arduino core machinery is linked.

3. **The body's safety model is coherent and small.** Fixed 20 ms grid with resync and overrun counting (`core/sched.cpp`), per-joint owner arbitration with crossfades that start from the last *output* (`core/arbiter.cpp:120-170`), a single `ReflexHolder` priority (`core/body.h:46`, `take_reflex` in `body.cpp`) so battery > watchdog > fall > warm-hold never fight, the epoch byte that makes stale goals refusable after any bump, the two-stage link watchdog, limits-then-slew as the last stage, and the rule that LIMP/REST are never stale. Blocking work (DMP upload) only happens in LIMP. The `.noinit` reset record gives honest reset causes.

4. **Cross-checks against the legacy firmware rather than against intent.** Joint transfer vs the legacy float path with Kilo's real EEPROM dump (1.32 M checks), attitude A/B vs the legacy `ypr` path, balance gains vs `adaptiveParameterArray`, built-in postures vs `InstinctNybble.h`, the DMP image bytes vs the vendored header. These are the tests that caught the tail sign and the pitch sign.

5. **The mind's link rules are explicit and testable.** Liveness gating (`SerialLink.mark_alive`; autonomous traffic only while the main loop proved alive), the "contact" rule (no motion goal into a body that may have rebooted until it is heard), epoch stamping at set time, retries with same vs fresh seq. `conftest.Peer` lets every rule be tested on a loopback with the real codec. The daemon's hold/lease ownership model (`goal_owner`, `_end_hold`) is a reasonable answer to "who owns the body when a CLI dies".

6. **Operational care.** Logging off the control threads (QueueListener, kilolog writer thread), port settle after avrdude with a modelled test (`test_settle.py`), hot-plug retry, hardware procedures written as documents with observations separated from inference.

## 2. Weak points (ranked by impact)

1. **RAM on the 328P is at the edge, and stack is the unmeasured part.** Globals 1517/1540 (23 B of headroom); `stack_free_min` was 355 B before P2 and **238 B while walking** (`docs/p2-hardware-test-results.md §4`). P1 leftovers still to land (WS2812 eyes need a frame buffer; `kilo cal imu`) and P3/P4 will want more. The shrink list (§3h: keyframe store, RX ring 192→128, TX 96→64) is the only plan, and the worst-case stack (trick playing + gait stopping + EEPROM save + `PCA_READ` reply + two ISRs) has never been provoked deliberately. Nothing in the host tests can measure this; see §3 and §5 (simavr).

2. **Two blocking spins inside the 50 Hz core.** `core/link.cpp` `emit(..., must=true)` busy-waits `while (link_hal_tx_free() < n) {}` for ACKs/replies/events; it relies on the UDRE ISR draining (documented as < 4 ms, fine) but if TX ever stalls the only exit is the hardware WDT reboot. `hal/i2c.cpp act()` spins up to ~1 ms per TWI action; the `if (err) return` short-circuit keeps a dead bus from eating the tick, but a *slow* (not dead) bus is unbounded per tick. Both are acceptable for now; they should be stated as failure modes and covered by the `tick_load`/overrun telemetry in a soak test (H9/H10 have not run).

3. **The daemon mixes four concurrency models.** asyncio loop + `SerialLink` reader/writer threads + an 8-thread pool + a behaviour thread + the remote's lock + logging threads (`daemon.py:218-290`). It is carefully done (ops serialised per session, `_blocking` for body calls, `_percept_events` checks `threading.get_ident()`), and `test_hardening.py` covers the races found so far, but: every body request is funnelled through `SerialLink._req_lock` (one in flight, up to 4×50 ms), so any client's slow request delays every other op, the remote and the runner; `op_call` (`daemon.py:1046`) is a reflection proxy to any public `BodyClient` method, a wide surface for a privileged socket; and `daemon.py` is 1326 lines with a 139-line docstring that duplicates the README. The design is sound; the size and the implicit thread-affinity rules are the fragility.

4. **The safety timeline exists twice.** The body already does stop→REST→LIMP when the mind goes silent; the daemon re-implements REST-then-LIMP as the idle latch (`_check_leases`, `idle_stage`) because it keeps the heartbeat alive on behalf of dead clients. Differences are documented (no 0.5 s stage) but it is two policies for one behaviour, plus a third copy in the vbody. Related semantic drift: README/daemon say "SET_BALANCE is not an epoch goal, so the link keeps refreshing it across a reboot", while `body.cpp body_epoch_bump` sets `b.bal.enable = 0` (P1 finding B1). It works only because the mind refreshes at 5 Hz, i.e. balance is off for up to 200 ms after every bump. No test pins this cross-layer assumption.

5. **Tuning constants are compiled in where they will need live tuning.** Gait pattern offsets and default heights (`gait.cpp PAT[]`), bob/wag amplitudes, IK sign macros (`ik.h`, now confirmed), `LEGACY_STEP_MS = 5.5` for trick timing (estimated, uncalibrated), the fall thresholds. Period/duty/stride are live, which is good; trot tuning (the one gait that fell) will want offsets and body height live too.

6. **Documentation drift.** `docs/architecture.md §6` promises simavr runs and a `body/test/vbody` (real core on stub HALs) that do not exist; §3h's first table still says "P2 ceiling 20 KB"; Appendix G is "as of P0"; the handoff says 345 tests (now 392). `mind/README.md` (1475 lines) repeats module docstrings nearly verbatim. With one developer and an AI doing most edits, three copies of each paragraph is where errors will hide.

7. Minor: `.gitignore` still carries legacy entries; the uncommitted `--balance` feature changes `_end_hold` semantics (balance returns to *on* when a client's hold ends) and should be checked against the owner's expectation that `kilo balance off` holds only while the client lives.

## 3. Under-tested

Ranked by payoff.

1. **No test runs the real C core as a body against the mind.** `kilo/vbody/sim.py` (1054 lines) is a second implementation of the body's state machines; `test_crosscheck.py` compares it with the C dispatch for the P0 vectors and a dozen sequences, but tick-level behaviour (watchdog timing, gait ramp and landing, trick preemption, fall timeline) is checked only against the Python copy's reading of the docs. The `link_harness` already has `reset`/`rx` commands; adding `tick N` (run `body_tick` + `link_tick` with scripted attitude/battery) and streaming its TX would give the promised `body/test/vbody` cheaply, let pytest drive the firmware core on a pty, and let the Python sim shrink.

2. **The HAL has zero host coverage** (~1100 lines: `uart.cpp` ring arithmetic and ISR contract, `i2c.cpp`, `mpu6050.cpp` FIFO/packet handling, `pca9685.cpp`, `ultrasonic.cpp` PCINT timing, `ir.cpp` edge capture, `sys.cpp` reset record and stack paint, `eeprom.cpp`). `uart_tx_free()` and the IR capture are pure arithmetic over a few registers and could be tested with register fakes. `main.cpp` (boot sequence, warm/cold, IMU re-init backoff) is also untested except on Kilo.

3. **Stack and cycle budget.** Nothing measures AVR stack depth or tick cost except `stack_free_min` read back from Kilo. `avr-gcc` is already installed; `simavr` (apt) would run `test_gait`/`test_reflex`-style scenarios with real 16-bit `int` semantics and give stack high-water marks before a flash. This was in §6's plan and is the single most valuable addition given point 2.1.

4. **Timing-dependent mind tests.** `test_link.py` asserts counts over wall-clock sleeps (`5 <= len(sends) <= 8` in 1 s; `n >= 4` heartbeats in 0.6 s; `time.sleep(0.35/0.5)` around a 0.3 s liveness window), `test_daemon.py` sleeps 1.5-1.6 s for leases, `test_vbody.py:177` sleeps 1.2 s. They pass here but will flake on a loaded Pi 2 or CI runner. `SerialLink` already takes an injectable `clock`; the writer's `_txq.get(timeout=housekeeping_s)` is the only thing that needs a step hook to make these deterministic. The suite takes 4 min; 60 s of it is `test_hardening.py::test_link_stats_snapshot_under_concurrent_counter_growth`, whose `join(30)` lets it pass even if the reader threads never finished (it grows a dict without bound while copying it 6000 times, so it is quadratic and effectively always hits the join timeout). It should bound the growth and assert the readers completed.

5. **Perception and behaviour on real traces.** `test_perception.py` uses synthetic traces; the P1/P2 sessions produced real kilologs (taps, pets, the two falls, the trot wobble) that are not fixtures. `kilo log replay` exists, so turning three captures into regression fixtures is cheap and would also catch the `imu_stale` drift the handoff flags.

6. **Weak or dead bits.** `test_e2e_cli._kilo(timeout=15)` never uses `timeout`. `docs/architecture.md §6`'s simavr and `body/test/vbody` items describe tests that do not exist. The gait cross-check used to be skipped "until the firmware implements SET_GAIT" and now runs (the README still says it is skipped). The `size_check.sh` suite is not invoked by `host_tests.sh` and there is no CI, so the flash/RAM ceilings are only enforced when someone remembers (or flashes).

## 4. Is the code DRY?

Mostly yes where it matters (protocol constants, event names, trick timing via the shared `kilo.tricks.player`). The duplication that exists is deliberate but worth naming:

- **Gait defaults in five places:** `body/src/core/gait.cpp PAT[]` (period, duty, height fraction, offsets), `mind/kilo/body/client.py:61-63 GAIT_DEFAULTS` (period, duty, stride, step, height fraction), `vbody/sim.py Gait`, `docs/architecture.md §3d` table, `mind/README.md` "Walking". The mind needs period/duty for `stop_timeout` and the body needs them for "0 = default"; a `[gait_patterns]` table in `spec.toml` generated into both would make one source.
- **Joint limits:** `mind/kilo/tricks/model.py:39-41 JOINT_LIMITS` is a copy of `body/src/core/joints.cpp`, guarded by `test_joint_limits_match_body`. Same remedy: into the spec.
- **Balance/head defaults:** `balance.h` `BAL_KP_Q8 128` vs `client.py BALANCE_KP 0.5` (and `BALANCE_MAX_DEG 30` vs `BAL_MAX_DEG_LIMIT`), `HEAD_SPEED_DPS`; not cross-checked.
- **The watchdog/idle timeline** (point 2.4): spec constants, body params, vbody `REST_MOVE_S`, daemon idle latch.
- **The vbody itself** is the largest duplicate: a Python re-implementation of dispatch, watchdog, trick gating, gait state machine and fall timeline. It earns its keep today via the cross-check; the C-core vbody (3.1) would retire most of it.
- **Scripts:** the `-latomic` probe and sanitizer flags appear in both `host_tests.sh` and `protocol/test.sh`; `host_tests.sh` and `build_link_harness.sh` both enumerate the core sources and proto objects.
- **Docs:** README ↔ docstrings ↔ architecture.md ↔ handoff repeat the link rules, holds, remote key table and CLI synopsis (also in `cli.py`'s module docstring and Appendix G's table).
- In the firmware the `goal()` switch in `link.cpp` repeats the `r = body_goal_x(); if (r != GOAL_OK) refuse(t, r)` pattern five times; trivial.

## 5. Ideas for development

**Simulator.** The virtual body answers "does the mind speak to the body correctly"; it cannot answer the questions P2 now has (trot fell in 3 s, 10 mm steps slid, left drift, 22° static tipping on a 65 mm track). Before reaching for MuJoCo/PyBullet, two cheaper steps would pay off: (a) the C-core vbody (3.1), so the mind is tested against the real gait maths; (b) a quasi-static Python model fed by the real foot trajectories (export `gait_legs()` x/z per leg per tick through the harness): COM projection vs the support polygon per phase, tipping margin vs body height, roll/pitch induced by the pose terms, swing timing vs duty. That is deterministic, unit-testable, and would have predicted the trot rocking (diagonal support line passes under the COM on a 65 mm track) and the tipping angle. A full physics sim is worth it *only if* trot tuning stalls after that: with 45/50 mm legs, servo backlash, foot-disc friction on wood and the ≈5° knee list dominate, and none of those is in a model without identification work. If you do go there, PyBullet (pip, URDF, Python API) is the lower-friction choice; drive it from the same exported foot trajectories, calibrate friction from the "10 mm slides, 25 mm sticks" observation, and treat its numbers as ranking, not prediction.

**Tooling and CI.** One `tools/check.sh` (or `make test`) that runs the four suites and marks slow tests; GitHub Actions on push to `main` (g++ + venv; arduino-cli install is a one-liner so `size_check.sh` can gate flash/RAM in CI; `FUZZ=10000`); a pre-commit hook for `gen.py --check && vendor.py --check && gen_trick_library.py --check`; `pytest-xdist` and a fake clock to get the mind suite under a minute; simavr job for stack high-water marks.

**Refactors next.** Move gait defaults, joint limits and balance/head defaults into `spec.toml`; split `daemon.py` (session/holds, ops, remote, logging); narrow or remove `op_call`; make `mind/README.md` an index that points at docstrings; fix §6/§3h/Appendix G in architecture.md; make the per-pattern offsets and heights live params (or a `SET_GAIT_PATTERN` command) before trot tuning; add a `kilo tune` loop (set param, walk N s, summarise the kilolog) since `kilo log summary|plot` already exists; a knee re-calibration pass before judging drift.

## 6. Legacy leftovers

Read by new code or tests (keep, or move into a `legacy/` reference folder with the source commit noted and update the paths):
- `src/InstinctNybble.h`: `body/test/test_misc.cpp:127`, `mind/kilo/tricks/legacy.py` (default source), `import_legacy.py`, `mind/tests/test_tricks.py` (library freshness).
- `src/motion.h`: `body/test/test_balance.cpp:39`.
- `src/imu.h`, `src/skill.h`: `body/test/test_attitude.cpp:137,142`.
- `src/mpu6050/MPU6050_6Axis_MotionApps_V6_12.h`: `body/test/test_misc.cpp:114` (DMP image bytes). The rest of `src/mpu6050/` is not read.
- **`src/infrared.h`: `mind/tests/test_ir.py:20`** (not in the list you gave; the IR key table is checked against it).
- `mind/firmware/opencat-legacy.hex`: used by `kilo flash --legacy` (`mind/kilo/tools/flash.py:67`), the rollback path (H15). Keep. Once `OpenCat.ino` and `src/` go it cannot be rebuilt from the tree (it can from commit 64b9447; the md5 is in `mind/firmware/README.md`).
- Comment/doc references only (no code dependency): `src/OpenCat.h` (architecture.md, raspberry-pi.md), `src/PCA9685servo.h` (`test_joints.cpp` comment), `serialMaster/ardSerial.py` (`kilo/link/ports.py:19` comment, raspberry-pi.md).

Not referenced by anything under `body/`, `mind/`, `protocol/` or `docs/` (grep over code, tests, scripts and docs): `pyUI/` (87 files), `ModuleTests/` (74), `SkillLibrary/` (17), `serialMaster/` (11), `Resources/` (9), `ChangeLog.md`, `OpenCat.ino`, and the rest of `src/` (`io.h`, `reaction.h`, `randomMind.h`, `taskQueue.h`, `sound.h`, `ultrasonic.h`, `eeprom.h`, `tools.h`, `OpenCat.h`, `PCA9685servo.h`, `Adafruit-PWM-Servo-Driver-Library/`, `ir/`, `MemoryFree/`, `QList/`, `rgbUltrasonic/`, `mpu6050/` except the one header). All deletable; also prune the legacy lines from `.gitignore` and `docs/raspberry-pi.md`'s serialMaster notes. The `src/` name is also a footgun today: `body/src/` and the legacy `src/` share a name in every grep.

**Caveats.** I did not run on hardware or inspect the Pi; `size_check.sh` numbers are from the docs. The concurrency observations are from reading `daemon.py` and `serial_link.py`, not from a found bug; the 60 s test's vacuous-pass claim is from its `join(30)` structure and its measured duration, not from forcing a failure. The simulator advice is a judgement call, not a measurement.

---

## Main session notes (2026-10-11)

- Done the same evening (ccfb3d8): `pyUI/`, `ModuleTests/`, `SkillLibrary/`, `serialMaster/`, `Resources/`, `ChangeLog.md` removed, `.gitignore` pruned. `OpenCat.ino` and `src/` kept by the owner's decision (the headers listed in §6 are read by tests and the trick importer).
- §2.7: the `--balance` semantics are as intended and documented in `mind/README.md` "Balance by default": with `kilo run --balance`, a client's `kilo balance off` holds only while that client lives; the remote's CH+ is the way to switch it off for longer.
- §2.4: the body does clear `bal.enable` on an epoch bump; the mind's 5 Hz refresh brings it back within 200 ms. Worth a test, not yet written.
