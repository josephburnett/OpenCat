# P3 walking-with-balance hardware test

The first run on Kilo of the stance-aware walking model
(docs/walking-and-balance.md; docs/architecture.md §3d "Support state",
"Sway", "Early touchdown", "Forward placement"; §3f "Stance-aware under
the gait"). Everything the host can check has passed (`body/tools/host_tests.sh`:
the sway's sign and amplitude read off the stance feet, the balance
following the planned sway within the dead band, the two-foot hold, the
reflex landing the right leg on a shove in a plant, the cost and the servo
slew budget). What only Kilo can tell: whether the sway's 8° of roll is
enough on the real cat, whether the servo lag is near the 150 ms the plant
assumes, whether the reflex ever fires in normal walking, and whether the
crawl and the walk now stay up.

Work at your own pace; nothing here is timed, every step waits, and any
step can be stopped and resumed later. Results go in
`docs/p3-hardware-test-results.md` (section 6).

**Safety, always:** walking on the floor only, never on the desk; a soft
surface with room around it; a way to stop at hand (`kilo walk stop`,
`kilo posture rest`, or stop the sending process: the link watchdog stops
the gait within half a second plus one period). Kilo switched on and
charged before the cable goes into the Pi (docs/pi-outages-2026-10-13.md:
the adapter ties the two 5 V rails; with Kilo off the Pi powers Kilo's
board and dips).

## 0. Flash and check the versions

On the Pi (or from rtb over ssh), Kilo on its stand or on the floor,
switched on, plugged in:

```sh
~/OpenCat/mind/systemd/update.sh --flash      # git pull, pip, restart, kilo flash (~1 min)
kilo status                                   # firmware build = the main commit's; proto 1.5
```

Expected: `kilo status` shows the new build id, `proto 1.5`, mode IDLE,
`link_ok`. If the Pi dropped off the network during the flash (the 5 V dip),
wait a minute, it reboots clean (safe mode), then run `kilo status` again
and `kilo flash` once more if the build id is still the old one.

## 1. On the stand: the sway is visible and the right way round

Kilo on his stand, feet in the air (nothing to tip over), balance on:

```sh
kilo walk march --pattern walk --for 8        # one leg at a time, sway on by default
```

Watch the body, not the legs. As each leg lifts, the body should lean
AWAY from it: when the left front leg is up the body rolls to the right and
drops its back end a little; when a back leg is up, the nose dips. About
8° of roll, visible by eye. Then:

```sh
kilo walk march --pattern walk --no-sway --for 8   # the body stays level
kilo walk march --pattern trot --for 8             # diagonal pairs: no lean (the sway cancels)
```

If the lean goes TOWARD the lifted leg, stop: the sign is wrong on the
real cat (the host test agrees with §3f's convention, so this would mean
the attitude convention is off; record it, do not go to the floor).

## 2. On the stand: the reflex lands a leg

Still on the stand, `kilo monitor` in a second terminal. Start a slow
walk and, while a leg is in the air, tip the whole stand quickly toward
that leg (a few degrees in a fraction of a second, the way a push would):

```sh
kilo walk march --pattern walk --speed 8 --for 20
```

Expected: the lifted leg drops to the ground height at once instead of
finishing its arc; the others carry on. A slow, gentle tilt (under the
90°/s threshold) does nothing. `--no-catch` turns it off for comparison.

## 3. Floor: the crawl and the walk stay up

Floor, balance on (the remote's default), telemetry recorded:

```sh
kilo monitor --json --rate 25 --duration 600 > ~/kilo-logs/p3-floor-$(date +%H%M%S).jsonl &
```

Then, with the remote or the CLI:

1. Trot forward and back (the gait that already held within ±3.5° of roll
   on 2026-10-13): expect the same or better.
2. Crawl marching in place (`0` key after `100+`, or
   `kilo walk march --pattern crawl --for 8`): this fell to the left within
   2 s twice without the sway. Expected now: stays up; roll trace within
   about ±10° (the sway itself is 8°).
3. Walk forward (`kilo walk forward --pattern walk --for 8`): expected to
   stay up; note the body's lean and whether the feet scuff.
4. Speed keys (9 faster, EQ slower) during a trot: two presses each way;
   the swing stays at 150 ms, the stride and the cadence change.
5. One gentle push sideways during a trot, from the side: expected a leg
   lands early and he stays up or at least falls less; `FELL` /
   `RECOVERED` in the daemon log if not.

Stop between runs with the stand key; `kilo walk stop` from the CLI.

## 4. What to read off the telemetry

From the jsonl (`roll_deg`, `pitch_deg`, `mode`, `balance`,
`flags` with `BALANCE_HELD` under the trot), per run: roll min/max and
standard deviation, pitch min/max, falls. The script used on 2026-10-13
(`~/kilo-logs/rollwin.py` on the Pi; takes the jsonl path and a list of
windows) prints these per time window. Compare against section 7 of
docs/walking-and-balance.md.

## 5. Tuning, if needed

- Crawl or walk still tips toward the lifted leg: raise `sway_roll_deg`
  (spec `[gait]`; 8 → 10 or 12; the walk's windows overlap to 12° already
  at the defaults), or raise `sway_lead_ms` if the lean arrives late (the
  servo lag on Kilo may be more than the plant's 150 ms).
- The reflex fires during normal walking (legs dropping without a push):
  raise `touchdown_rate_dps` (90 → 120); if it never fires on a real push,
  lower it.
- Trot wobble: `kilo balance on --kp 0.99` is what the remote uses; at the
  default Kp 0.5 a 5° slope leaves enough lean for the reflex to fire on
  every step (the host plant showed 121°/s), so keep Kp near 1 for the
  trot.

## 6. Record the results

`docs/p3-hardware-test-results.md`: build id, date, surface; one row per
run with the command, the roll/pitch numbers from section 4, what it
looked like in the owner's words, falls; the tuning changes made and the
spec values they ended at.
