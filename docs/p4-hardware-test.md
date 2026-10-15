# P4: the planned walk on Kilo

First run of protocol 2.0 on the real cat: the mind plans every step and
streams foot positions (docs/walking-and-balance.md §9-10, architecture
§3d). Everything the host can check passes (four suites, the simulated body
with real tipping, the review fixes). What only Kilo can tell: the servo
lag (assumed 150 ms), the masses (estimated), and whether the walk and the
crawl stay up on the floor.

Nothing here is timed; every step waits. Results go in
`docs/p4-hardware-test-results.md`.

**Safety:** walking on the floor only. Kilo switched on and charged before
the cable goes into the Pi (docs/pi-outages-2026-10-13.md). A way to stop
at hand: B6 on the remote (stand), `kilo walk stop`, or Ctrl-C in the
walking terminal (the body settles by itself within half a second when the
stream stops).

## 0. Weigh Kilo (any time, switched off, no Pi needed)

Kitchen scale and a block the same height as the scale platform. Kilo
switched off, limp. Four pair readings, each with the other pair on the
block, body level:

| reading | on the scale | on the block |
|---|---|---|
| front share | both front feet | both back feet |
| back share | both back feet | both front feet |
| left share | both left feet | both right feet |
| right share | both right feet | both left feet |

Then the same four without the battery. Also note with a ruler: front feet
to back feet (foot centre to foot centre) and left to right, as he stood.
Optional, for the centre's height: front feet on the scale raised on a
second block of known height (say 30 mm), back feet on the floor-level
block, one more front reading.

Send me the numbers; they replace the estimates in
`mind/kilo/planner/model.py`.

## 1. Flash

Kilo on the floor, switched on, plugged in:

```sh
~/OpenCat/mind/systemd/update.sh --flash      # ~1 min, Kilo goes limp meanwhile
kilo status                                   # proto 2.0, build = main's hash
```

If the Pi drops off the network during the flash, wait a minute (it reboots
clean in safe mode) and run `kilo status`, then `kilo flash` again if the
build is still the old one.

## 2. Servo lag (standing, 1 minute)

Kilo standing on the floor, telemetry at 50 Hz recorded, then a 10° pitch
step held 2 s, twice:

```sh
kilo monitor --json --rate 50 --duration 60 > ~/kilo-logs/p4-lag-$(date +%H%M%S).jsonl &
kilo pose --pitch 10 --posture stance --for 2; sleep 2; kilo pose --pitch -10 --posture stance --for 2
```

The delay between the command and the pitch starting to move, and the time
to reach 90 % of 10°, are the servo lag and rise time. Compare with the
150 ms the planner assumes (`servo_lag_ms` in the model).

## 3. Floor runs, telemetry recorded

```sh
~/kilo-logs/record.sh        # 30 min of telemetry to ~/kilo-logs/p3-<time>.jsonl
```

In this order, each from a stand, ready to catch:

1. Trot forward 6 s: `kilo walk forward --for 6` (or A3 then B7 on the
   remote; B6 stops). Expected: as yesterday or better.
2. Walk forward 8 s: `kilo walk forward --pattern walk --for 8` (or C4,
   B7). Expected: a visible lean away from each lifting leg, the body
   within a few degrees of the plan, no fall. `kilo walk` prints the model
   error (offset, lag, largest error) at the end: note them.
3. Crawl forward 8 s: `--pattern crawl` (or B4, B7).
4. Speed: during a trot, C1 twice (faster), C5 twice (slower). The change
   is gradual.
5. Walk backward and a turn: `kilo walk back --pattern walk --for 8`,
   `kilo walk left --for 6`.
6. One gentle sideways push during a walk.

## 4. Read-outs

Per run: the `kilo walk` error summary; from the jsonl, roll and pitch
min/max/sd (`~/kilo-logs/rollwin.py`), the `walk` row in `kilo monitor`
(reference, error, correction, feet down, ticks queued); `kilo status` for
feet_late / feet_underruns after the runs (STATS). Falls as FELL in the
daemon log.

## 5. Tuning knobs (mind/kilo/planner/plan.py PlannerConfig, model.py)

- Leaves the plan toward the lifting leg: raise `margin_mm` (14) or
  `servo_lag_ms` (150) if the lean arrives late.
- Lift gate holds too often (the walk stutters): raise `gate_deg` (5).
- Catches fire in a normal walk: raise `catch_deg` (7).
- The walk's lean looks excessive: lower `margin_mm` once the masses are
  measured.
