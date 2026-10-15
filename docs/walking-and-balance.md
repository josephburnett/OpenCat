# Walking and balance: what we have, why the trot tips, what the model should be

Written 2026-10-13 after the first floor drives (trot at 1000 ms with
balance on: "wobbles so much when walking that he tipped over"; tricks fine).
Numbers use Kilo's measured geometry: legs 45 + 50 mm, body 123 mm long,
feet 65 mm apart, body height 0.80 × 95 = 76 mm, centre of mass about 80 mm
up. Code: `body/src/core/gait.cpp`, `balance.cpp`, `body.cpp` (the wiring);
design: `docs/architecture.md` §3d, §3f.

## 1. What we have

**The gait** is open loop. Each leg follows a fixed foot path in its own
plane (back along the ground in stance, a cosine arc in swing), timed by a
global phase. Nothing about it depends on what the body is doing. It knows
the body height and a commanded pose (pitch, roll) and that is all.

**The balance** is a slow leveller. Once per 20 ms tick it takes the measured
pitch and roll, subtracts its own correction and the commanded pose to get a
"ground slope", and asks for leg-length changes (front/back, left/right)
that would level the body on that slope. Between the sensor and the legs
there is a 2° dead band, a low-pass (about 60 ms), a slew of 1° per tick
(50°/s), a clamp of 15°, and the servos themselves (about 150 ms). The loop
takes roughly 250 ms to act on what it sees.

**The coupling** is one number pair: the balance's pitch/roll corrections
are added to the gait's pose terms, so the stance feet get longer or shorter.
The balance does not know which feet are on the ground, where the gait is in
its cycle, or what the body is about to do. The gait does not know the
balance exists. The balance also cannot tell the body's own rocking (caused
by the gait) from a slope: it levels against both, late.

That design is right for standing on a slope, which is what it was built and
tested for (H13: level within the dead band at Kp 0.99). It was never a
walking stabiliser, and the numbers below say it cannot be one.

## 2. Why the trot tips, in numbers

A trot stands on a diagonal pair of feet while the other pair swings. On a
diagonal pair the body is an inverted pendulum pivoting about the line
between those two feet: nothing holds it up sideways. For a centre of mass
h ≈ 80 mm above the ground the fall grows as e^(t/τ) with

    τ = sqrt(h / g) = sqrt(0.08 / 9.81) ≈ 90 ms.

Every 90 ms on two feet multiplies the tilt by 2.7. The diagonal-only time
per step is (1 − duty) × period:

| trot setting                | two feet only | tilt growth | result (observed)           |
|-----------------------------|---------------|-------------|-----------------------------|
| legacy, ≈250 ms, duty 0.70  | ≈75 ms        | × 2.3       | shuffles, stays up          |
| 800 ms, duty 0.55 (P2 test) | 360 ms        | × 55        | fell in 3 s                 |
| 500 ms, duty 0.55 (evening) | 225 ms        | × 12        | wobbled, fell once          |
| 1000 ms, duty 0.70 (now)    | 300 ms        | × 28        | wobbles, tips over          |

A 1° lean at lift-off becomes 28° by touchdown. The next pair catches it,
over-corrects, and the rock grows: that is the wobble. The balance loop,
at ≈250 ms from seeing to acting, is three times slower than the fall; and
on a diagonal pair, changing the lengths of the two stance legs does not
produce a torque about the line they stand on. Through the gains it does
not matter: no gain on this loop stabilises a 300 ms diagonal stance. The
balance can only level the body during the four-foot phases, and with its
delay it is often pushing the wrong way by the time the legs move, which is
the "balance may feed the rocking" of the P2 results.

**So slowing the trot down made it less stable, not more.** Period is the
wrong knob. The thing that must stay short is the time on two feet.

## 3. What "speed" should change

Keep the swing short and fixed, and get speed from the stride and the
stance time:

    swing time   T_sw  fixed, 150–200 ms (short enough for τ = 90 ms, long
                       enough for the 10°/tick servo slew: a 25 mm stride
                       in 150 ms is about 6°/tick at the shoulder)
    stance time  T_st  = stride / speed
    period       = T_sw + T_st,   duty = T_st / period

At walking pace (say 50 mm/s, stride 25 mm) that is a 500 ms stance, period
650 ms, duty 0.77. At a crawl (10 mm/s) a 2.5 s stance, duty 0.94: a slow
gait with quick steps and long four-foot phases, which is what the stock
firmware's shuffle looks like. Speed then modulates continuously, and the
two-foot time never grows. The current remote maps the key to a period;
that mapping is what the mind should change (the body already accepts any
period/duty per goal, with phase continuity).

Walk and crawl (one leg at a time, three feet down) have no diagonal phase
and can be as slow as you like, if the centre of mass is inside the
triangle of the three stance feet. On a cat 65 mm wide that triangle's edge
runs right under the body's centre line, so an unmoved body sits on the
edge: marginal. That is where sway comes in (section 4).

## 4. The model that should exist

One shared state, written by the gait and read by the balance:

- **Support state**: which feet are on the ground this tick and where they
  are (the gait knows: stance/swing per leg, foot x and z). From it, the
  support polygon (four feet, a triangle, or a diagonal line).
- **Expected attitude**: what the body will do because of the gait itself
  (the planned sway and the bob), so the balance corrects deviations from
  the plan, not the plan.

And two couplings instead of one:

**Gait → balance.** The balance becomes stance-aware. It corrects only
through feet that are down. Its reference is the planned attitude, not 0.
Its "slope" estimate is frozen while on two feet and updated on four. Its
delay chain (dead band, low-pass, slew) is sized for levelling, which is
fine; it is simply not asked to catch falls.

**Balance → gait.** Three things the gait does for balance:

1. **Sway (feedforward).** Before a foot lifts, roll the body toward the
   feet that stay down, and pitch toward the end that keeps both feet. On a
   cat whose legs move only fore-aft (two servos per leg, no sideways foot
   motion) rolling the body is the only way to move the centre of mass
   sideways: 10° of roll moves it about 80 mm × sin 10° ≈ 14 mm, from the
   triangle's edge to well inside it. The gait already has the pose terms to
   do this (the same leg-length channel the balance uses); what is missing
   is a roll/pitch pattern locked to the leg phases. This is what makes a
   slow walk statically stable, and what lets "balance on" and "walking" be
   one thing.
2. **Early touchdown (feedback).** If the body starts tipping toward the
   swinging side faster than a threshold, end the swing now: put the foot
   down where it is. A foot on the ground is the only thing that stops a
   sideways fall on this cat; nothing else can. This is a reflex at tick
   rate (20 ms), inside the fall's 90 ms, and it is what the fast legacy
   trot gets for free from its 75 ms swings.
3. **Forward placement (capture point).** Fore-aft falls can be caught by
   landing the swing foot further ahead or behind, by about
   τ × forward velocity of the tilt. The legs can do this (it is the one
   axis they move in). Lower priority: pitch falls are rarer here than roll
   falls because the body is long and narrow.

**Terrain** (the goal: balance on all the time, uneven ground) falls out of
the same state. A swing foot that meets the ground early, or late, changes
the measured attitude at touchdown; the stance-aware balance then adjusts
that leg's length in the next four-foot phase, which is the slope
correction we already have, applied per step. Without foot contact sensing
(Kilo has none) the touchdown height is a guess; the IMU jolt at contact
and the attitude change after it are the available signals, and the early
touchdown reflex above doubles as "stop pushing when the foot has found
ground" if it also watches pitch.

## 5. What this means in practice, in order

*Since 2026-10-14 (protocol 2.0) the steps are planned in the mind and the
firmware's gait generator is gone (sections 9 and 10). Items 2 to 6 below
were built into that generator and left with it; what became of each in
the mind: the speed mapping (2) is `kilo.planner.patterns`; the sway (3)
became the planner's lean, a move of the whole body over the three feet
that stay down, finished a servo lag before each lift; the stance-aware
balance (4) is the body's loop with the plan's prediction as its reference,
holding while any foot is up; the early touchdown (5) is the planner's
catch from the error the body reports; forward placement (6) was not
carried over (the plant showed no benefit). The mind's tests of all this
are tests/test_planner.py and tests/test_walk_sim.py (section 10).*

1. **Parameters only, next drive** (no code): a trot with a fixed short
   swing. Period 600 ms, duty 192 (0.75): swing 150 ms, four-foot phases of
   150 ms, stride 20–25 mm. Expect much less wobble than 1000/0.70. This is
   a test of section 2, not a fix. Measure: roll amplitude per cycle from
   telemetry (`kilo monitor`), held in the hand and on the floor.
2. **Speed mapping in the mind** (small): the remote's speed keys and
   `kilo walk` set stride and stance time with a fixed swing, instead of a
   period. Continuous speed, two-foot time constant.
   **Done (2026-10-13)**: `kilo.body.client.gait_timing(pattern, speed)`,
   `kilo walk --speed MM_S`, and the remote's C5 / C1 (slower / faster, 0.5x
   to 2x of the default speed). Each pattern keeps its default's swing time
   (trot 150 ms; walk 500 and crawl 297 ms, which their higher steps need
   under the slew limit), stance = stride / speed, period = swing + stance.
   Below the longest stance (duty 230/256 or period 2000 ms; the trot's is
   1327 ms) the stride shrinks to as little as 8 mm; above the default
   stride at the shortest stance (duty 0.65 trot, 0.75 walk/crawl) the speed
   is capped. Trot 6..90 mm/s, default 56 (600 / 192 / 25 mm). The trot's
   0.65 keeps three ticks on four feet, which the stance-aware balance needs
   to run at all (review 2026-10-13).
3. **Sway in the gait** (firmware, moderate): planned roll/pitch locked to
   the leg phases, amplitude from the geometry (about 8–12° roll for the
   walk, less for the trot), ramped with the stride. Makes slow walk and
   crawl statically stable with balance on. *Done 2026-10-13* (§3d "Sway",
   not yet on the floor): SET_GAIT flag `SWAY`, on by default for walk and
   crawl from the mind; 8° roll and 4° pitch away from the lifting leg at
   its mid-swing (WALK peaks at 12° between its two same-side swings), 0
   for the trot by construction. The gait also publishes the support state
   (`Gait::down`, feet on the ground) and the planned attitude
   (`plan_pitch`, `plan_roll`) for item 4.
4. **Stance-aware balance** (firmware, moderate): reference = planned
   attitude; correct through stance feet only; hold the slope estimate on
   two feet. *Done 2026-10-13* (§3f "Stance-aware under the gait", not yet
   on the floor): in a host plant that falls on a diagonal, the walk with
   sway at Kp ≈ 1 now follows its plan within 1° with corrections ≤ 1.8°
   (they were 10.8°, fighting the sway), and on a trot the corrections hold
   on two feet and 2 ticks after (measured: the lean per step stops
   improving past 2) while a 10° slope is still levelled to 2.65° at every
   lift-off.
5. **Early touchdown reflex** (firmware, small once 4 exists): tipping
   toward the swing side ends the swing. The piece that lets the trot
   survive a misstep or a bump. *Done 2026-10-13* (§3d "Early touchdown",
   SET_GAIT flag `CATCH`, on by default, `kilo walk --no-catch`): at 90°/s
   toward a swinging leg; in the plant normal walking peaks at 40°/s (walk)
   and 65°/s (trot on a 5° slope) and never triggers. Since the same day it
   waits for a second tick over the threshold and lowers the caught foot
   over a few ticks with its x held; a 60°/s shove on the trot then catches
   the leg on the falling side late in its swing, and the lean is 12.2° vs
   15.5° in the plant that counts a foot down on its command, unchanged
   (19.6°) when the foot must first reach the ground through the servo lag
   (the earlier 7.3° assumed a caught foot carried the body at once).
6. **Forward placement and terrain** later, on the same state.
   *Forward placement done 2026-10-13* (§3d "Forward placement", SET_GAIT
   flag `PLACE`, off by default until a floor run compares `kilo walk --place` with the default): at a lift-off the
   leg's touchdown moves 7 mm per rad/s of pitch rate (τ·h), forward on a
   nose-down rate, clamped to 3 mm (the trot's 150 ms swing has no more
   slew to give). In a capture plant with a rigid stance it neither helps
   nor hurts the pitch after a fore-aft shove (three or four feet stop the
   shove before any foot lifts); normal walking shifts the feet by at most
   1.25 mm. Terrain: not started.
   Also from the plant (**fixed** 2026-10-13, the "sway finding" of §3f):
   the walk's sway crossed zero at the lift-offs where it changes sides, so
   with the servo lag the body leaned up to 3.8° toward the lifting leg; the
   sway now runs 225 ms ahead (`[gait] sway_lead_ms`) and the body leans at
   least 4.0° (walk) and 7.5° (crawl) away from every leg as it lifts.

Each step is testable in the hand first (legs in the air: cadence, servo
slew, sway pattern) and then on the floor with the roll trace as the
number that must go down.

## 6. What stays as it is

The leveller itself (controller, gains, sign conventions, the geometric
pose terms) is correct and measured; it is the four-foot and standing part
of the model, and in a walk it trims what the plan did not predict. The
IK and the leg-length pose terms stay in the body. The gait's phase
machinery and foot paths moved to the mind (kilo.planner.plan) as slots of
a shift and a swing with the same cosine foot paths. The legacy firmware
never had any of this: it stayed up by being fast. The mind's test of the
whole loop is the virtual body's physics (tests/test_walk_sim.py): the
planned walk follows its plan within 2.5°, the same steps without the lean
tip over the edge every swing.

## 7. Measured on the floor, 2026-10-13 16:50 PDT (step 1 of section 5)

Remote drive, balance on, firmware 74347362 (no sway yet), the mind at
df578c4 (trot period 600, duty 192, stride 25, step 8 = 150 ms on two feet).
Roll from `kilo monitor --json` at 25 Hz (`~/kilo-logs/monitor-20261013-164821.jsonl`):

| what                          | roll (deg)       | pitch (deg)  | result                                   |
|-------------------------------|------------------|--------------|------------------------------------------|
| standing before               | 0.6 .. 1.0       | 2.5 .. 2.7   |                                          |
| trot forward, 5 s             | -1.5 .. 3.4 (sd 1.2) | 0.4 .. 2.7 | "much less wobble, stayed on his feet"   |
| trot backward, 6 s            | -3.4 .. 3.6 (sd 1.6) | 0.2 .. 4.5 | walked                                   |
| crawl, marching in place (x2) | +9.8 then -124   |              | FELL to the left within 2 s, both times  |

The trot at 150 ms on two feet holds within ±3.5° of roll where the 300 ms
trot reached -16.5° and fell (P2 results, 800 ms / 0.55). Section 2's
arithmetic holds on the robot.

The crawl fell as section 3 predicts for a three-foot gait with no sway:
the centre of mass sits on the edge of the support triangle, and a 300 ms
swing (period 2000, duty 0.85) with the body low and the step 18 mm high
tips it. That is what the sway (section 4, item 1; merged 405d139, not yet
flashed) is for. Until it is flashed, crawl and walk are expected to tip;
the trot is the gait to drive with.

## 8. Measured on the floor, 2026-10-14 (the sway, flashed as 56768228)

Full table in docs/p3-hardware-test-results.md. The short version: the walk
pattern fell within 2-3 s with the balance on, sway on or off, every time
(four runs), always with the same growing 3 Hz side-to-side swing; with the
balance **off** it rocked ±15° and stayed up for the whole 6 s. The trot
held in every run. So the balance loop is unstable on three feet (the body
tips about the far diagonal in under 0.1 s, the loop answers 0.25 s later,
after the body has turned round), and section 4's item 1 as built, a roll
reference for that loop, cannot work: it asks the unstable loop for more.

## 9. Direction agreed 2026-10-14: the mind plans, the body plays and corrects the residual

Discussed with the owner on the evidence above. Four parts:

1. **A model of the body in the mind.** Masses and where they are (body,
   battery, leg segments, later the Pi): the centre of mass for any leg
   configuration. Servo speed under load and lag (assumed 150 ms, never
   measured). From these, how fast a tip grows (today's traces: 30° in
   0.2 s) and the lead the servos need. To measure: weight per foot on a
   kitchen scale, with and without the battery; a servo step test with the
   IMU recording.
2. **The mind plans every step through the whole body.** Not "lift a leg
   and hope": the plan is a path for the centre of mass that stays inside
   the support that remains, with the body already moving before the lift
   (the way a person walks: the fall is planned and the next foot is on its
   way). The mind streams foot positions and the predicted attitude for
   each 20 ms tick, about five ticks (100 ms) ahead. It replans from the
   body's reported actual position, never from what it last asked for.
3. **The body keeps a self-contained correction loop.** Its reference is
   the predicted attitude for this tick, so it sees only model error. It
   always moves from where it is toward the next point at a limited rate
   (no snap). It keeps its own limits that no plan can override: servo
   slew, fall reflex, the link watchdog (a gap of ~100 ms slows to a stand
   on the current feet; 500 ms stops the gait as now).
4. **Error telemetry every tick**: predicted attitude, measured attitude,
   the correction applied. A steady offset = mass or IMU placement to
   learn once; a lag between predicted and measured = servo lag, which
   drifts with battery voltage (track it against batt_mv and adjust the
   lead); a spike = a bump, a slope, a slipped foot: the body has handled
   the first 0.1 s, the mind decides whether to carry on, slow down, widen
   or stop.

Consequence: the gait generator leaves the firmware (which also frees the
flash and RAM the plan buffer needs; the firmware has 4 bytes of globals
free today). The firmware becomes a pose player with reflexes and the
residual loop. Can the Pi keep up: a 50 Hz Python loop on the Pi 2 with
the daemon running fired 1500 ticks with the latest 0.44 ms late (20:49);
a 10-minute run under SD-write and Wi-Fi load is in
docs/p3-hardware-test-results.md. The per-tick work (4 legs of IK plus the
mass model) is well under 1 ms; a frame of foot positions plus predicted
attitude is ~35 bytes, ~1.5 ms on the wire. The 100 ms lookahead in the
body covers garbage-collector pauses and SD stalls; if Python itself
becomes the limit the planner can be rewritten in Rust (owner).

Not started: the owner asked to discuss before any balance work, and did.

**Body side, 2026-10-14 (protocol 2.0, branch fw/stream).** The gait
generator left the firmware; the body plays the mind's `FEET` stream
(docs/architecture.md §3d) and its balance takes the predicted attitude as
the reference and corrects through the feet on the ground (§3f). The host
plant now tips: a point mass on the four legs, standing on the feet whose
triangle holds it and turning about the nearest edge as an inverted
pendulum when they do not (2 to 30° in 180 ms, the floor's ~0.2 s). In it a
walk that rolls the body 10° away from each lifting side stays up with
corrections of 1.4°; the same steps without the roll sink 8° toward every
lifting foot. One finding changed the body: a three-foot stance shoved
onto its lifted foot leaned 19.2° with the loop updating on three feet and
12.9° with it off, because the corrections act through the two feet the
body pivots on. The body now holds its corrections whenever a foot is up;
the loop trims on four feet only, and the plan's lean has to hold the mass
on three. The mass model, the servo step test and the planner are still
the mind's to do.
Next concrete steps, in order: weigh Kilo per foot; servo step test; put
both into the host plant (which today treats three feet as safe, which is
why the sway passed there); then the protocol change (streamed poses,
error telemetry) and the planner.

## 10. The planner, as built (the mind, 2026-10-14)

Code: `mind/kilo/planner/` (model.py, patterns.py, plan.py, errors.py,
walker.py); the virtual body it is tested on: `mind/kilo/vbody/feet.py`
and `physics.py`; how to use it: mind/README.md "Walking".

**What it computes, every 20 ms tick.** Where each foot is (fore-aft x and
depth z under its shoulder), which feet are on the ground, and the
attitude the body will have. The pattern's legs take turns in slots: a
*shift* with all four feet down, then a *swing*. Feet on the ground slide
back at the stride speed; a swinging foot lifts on a cosine arc to half a
stride ahead of its shoulder. For the walk and the crawl, before every lift
the planner solves for the smallest move of the body (a fore-aft shift of
every foot, a roll and a little pitch by making legs longer or shorter)
that puts the centre of mass at least 10 mm inside the triangle of the
three feet that stay down, at lift-off and at touchdown, and moves the body
there during the shift: done a servo lag before the lift, held until the
foot is down, dropping a side no faster than gravity can follow. The trot
stands on diagonal pairs for 150 ms and does not lean. The predicted
attitude for a tick is the plane through the support feet the plan
commanded a servo lag earlier; it goes to the body as the balance
reference. From the body's reports: no lift while the body is more than 5°
off the plan (the shift waits, up to 0.5 s); a swinging leg lands at once
where it is when the body tips toward it by 7°. A stop takes every leg home
in one more step, levels the body and hands over to the STANCE posture.

**The parameters, and which are estimates.** Measured: leg 45 + 50 mm,
foot disc radius 6 mm, shoulders 123 mm apart, track 65 mm. Estimates
(BodyModel, one place): body 700 g with its centre 5 mm ahead of the
middle and 5 mm above the shoulder axles, battery 150 g 15 mm below,
upper leg 20 g (its centre at 0.8 of the link: the knee servo), lower leg
8 g, the Pi not carried; 962 g in all; servo lag 150 ms. Chosen: margin
10 mm, shift at most 18 mm, roll 14°, pitch 6°, lift gate 5°, catch 7°,
patterns and swing times as in mind/README.md. Replace the masses with the
per-foot scale readings and the lag with the servo step test.

**The stream's timing.** A thread in the process that walks (the daemon or
`kilo walk`) wakes every 20 ms. The body's clock is estimated from the
TELEMETRY arrival times (50 Hz while walking): body time = local time + the
largest (tick x 20 ms - arrival) of the last 2 s, i.e. the least delayed
frame. Each wake sends every planned tick from the last one sent up to the
body's tick + 5 (100 ms ahead), so a late wake (a garbage-collector pause,
an SD stall) catches up in one go; a tick already played is planned but not
sent. The body queues 8 ticks and holds the last pose on an underrun, then
settles. The thread never writes to disk (its records go to the frame
log's queue), freezes the existing objects for the garbage collector and
raises its young-generation threshold while it runs, and sends nothing
while the mind's main loop looks dead. FEET goes out at once on the
caller's thread (SerialLink.send_stream): no ack, refresh or retry, outside
the goals' epoch machinery; no SET_POSTURE goal is latched while it
streams (a refresh would end GAIT on the body).

**What the virtual body showed.** In its physics (a rigid body on point
feet, servo lag, tipping over an edge as an inverted pendulum) the planned
walk and crawl follow their plan within 2.5° with the body leaning 6-9°
on purpose; the same steps without the lean put the mass up to 4 mm past
the edge and the body tips 9-11° onto the swinging foot every swing (the
rigid catch stops it short of a fall; on the floor the balance loop of the
old firmware took it over); the trot stays within 2°; speed, direction and
pattern changes mid-walk stay within 4°; a 60 ms stall of the producer is
caught up with no underrun; a slower servo (200 ms) and a tilted IMU show
in the error estimates (lag +40..60 ms, the offset). The real body is the
next test.

