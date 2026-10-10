#include "body.h"
#include <string.h>
#include "fx.h"
#include "posture.h"
#include "pgm.h"

void body_init(Body& b, Config* cfg, const int16_t* warm_angles, bool imu_ok) {
  // Everything starts at zero (queues empty, counters 0, epoch 0, filter
  // unseeded, watchdog unarmed) except what is set below.
  memset(&b, 0, sizeof b);
  b.cfg = cfg;
  b.att.gz = Q14_ONE;  // upright until the first packet says otherwise
  b.imu_fault = imu_ok ? 0 : 1;
  // Warm boot: the PCA9685 kept driving the servos through our reset; keep
  // exactly that pose until we know which way is up. Cold boot: the servo
  // pose is unknown, so stay limp until the Pi asks.
  arbiter_init(b.arb, warm_angles);
  b.holder = warm_angles ? RH_WARM_HOLD : RH_NONE;
}

KILO_NOINLINE static void event(Body& b, uint8_t code, int16_t arg) { events_push(b.ev, code, arg, b.tick, b.epoch); }

void body_epoch_bump(Body& b) {
  // A new epoch makes every goal the Pi sent before this moment stale, so
  // a goal re-sent from before a fall or a link loss cannot restart motion.
  // Clearing the latched motion goals (gait, head, body pose, trick) goes
  // here once those exist (P1/P2). The IDLE posture is not a motion goal.
  b.epoch++;
}

// Take the REFLEX owner for `who`, unless a more important reflex holds it.
KILO_NOINLINE static bool take_reflex(Body& b, uint8_t who, uint8_t posture, uint8_t ticks) {
  if (b.holder > who) return false;
  b.holder = who;
  arbiter_reflex(b.arb, posture, ticks);
  return true;
}

KILO_NOINLINE static void release_reflex(Body& b, uint8_t who) {
  if (b.holder != who) return;
  b.holder = RH_NONE;
  arbiter_reflex_release(b.arb);
}

// Lie down, then limp after `limp_ticks` (each reflex owns its timer). If
// already limp, stay limp: powering servos from an unknown pose to lie down
// would be worse than doing nothing.
static void rest_then_limp(Body& b, uint8_t who, uint16_t* timer, uint16_t limp_ticks) {
  if (arbiter_idle_posture(b.arb) == POSTURE_LIMP) {
    take_reflex(b, who, POSTURE_LIMP, 0);
    *timer = 0;
    return;
  }
  // The IDLE posture follows, so that releasing the reflex later is seamless.
  arbiter_set_idle(b.arb, POSTURE_REST, REST_MOVE_TICKS);
  *timer = take_reflex(b, who, POSTURE_REST, REST_MOVE_TICKS) ? (uint16_t)(REST_MOVE_TICKS + limp_ticks) : 0;
}

static void limp_now(Body& b, uint8_t who) {
  arbiter_set_idle(b.arb, POSTURE_LIMP, 0);
  take_reflex(b, who, POSTURE_LIMP, 0);
}

// Count a reflex's LIMP timer down; when it fires, go limp.
static void limp_timer_step(Body& b, uint8_t who, uint16_t* timer) {
  if (*timer && --*timer == 0 && b.holder == who) limp_now(b, who);
}

void body_link_frame(Body& b) {
  if (b.wd.stage) {
    event(b, EV_LINK_RESTORED, 0);
    // Stay in the current posture (IDLE already holds it); the Pi must send
    // a goal with the new epoch to move again.
    release_reflex(b, RH_WATCHDOG);
  }
  b.wd.armed = 1;
  b.wd.stage = 0;
  b.wd.since = 0;
  b.wd.limp_timer = 0;
}

static void watchdog_step(Body& b) {
  LinkWatchdog& w = b.wd;
  if (!w.armed) return;  // never heard from the Pi since boot: nothing to stop
  if (w.since < 0xFFFF) w.since++;
  if (w.stage == 0 && w.since >= WD_STOP_TICKS) {
    w.stage = 1;
    body_epoch_bump(b);  // also stops gait/trick/head from P1 on
    event(b, EV_WATCHDOG, 0);
  }
  // Stage 2 also waits for the gait ramp-down (P2 hook; nothing to wait for yet).
  if (w.stage == 1 && w.since >= WD_REST_TICKS) {
    w.stage = 2;
    rest_then_limp(b, RH_WATCHDOG, &w.limp_timer, WD_LIMP_TICKS);
  }
  if (w.stage == 2) {
    limp_timer_step(b, RH_WATCHDOG, &w.limp_timer);
    if (!w.limp_timer) w.stage = 3;
  }
}

static void warm_hold_step(Body& b, bool imu_fresh) {
  if (b.holder != RH_WARM_HOLD) return;
  b.hold_ticks++;
  if (!(imu_fresh || b.hold_ticks >= WARM_HOLD_TICKS)) return;
  if (b.wd.armed && b.wd.since < WD_STOP_TICKS) {
    // The Pi is already talking to us: hand the held pose to IDLE (which
    // holds the same snapshot) and let the Pi decide.
    release_reflex(b, RH_WARM_HOLD);
    return;
  }
  // Nobody is talking to us: lie down under the watchdog policy (REST, then
  // LIMP 10 s later). The body never stands without the Pi.
  b.holder = RH_NONE;
  b.wd.armed = 1;
  b.wd.stage = 2;
  b.wd.since = WD_REST_TICKS;
  rest_then_limp(b, RH_WATCHDOG, &b.wd.limp_timer, WD_LIMP_TICKS);
}

static void battery_step(Body& b, uint16_t counts) {
  BattLevel prev = b.batt.level;
  BattLevel lvl = battery_sample(b.batt, battery_counts_to_mv(counts, b.cfg->img.batt_mv_per_64));
  if (lvl == prev) return;
  if (lvl > prev) {
    if (lvl == BATT_CRITICAL) {
      limp_now(b, RH_BATTERY);
      b.batt_limp_timer = 0;
    } else if (lvl == BATT_LOW) {
      body_epoch_bump(b);
      rest_then_limp(b, RH_BATTERY, &b.batt_limp_timer, BATT_LIMP_TICKS);
      // Low-battery chirp: the legacy melodyLowBattery opening, short.
      notes_push(b.notes, 15, 10);
      notes_push(b.notes, 11, 10);
      notes_push(b.notes, 13, 10);
    }
    event(b, EV_LOW_BATTERY, lvl);
    return;
  }
  // Recovering (CRITICAL never does: it is latched).
  if (prev >= BATT_LOW && lvl < BATT_LOW) {
    b.batt_limp_timer = 0;
    release_reflex(b, RH_BATTERY);  // IDLE holds REST or LIMP, as set above
  }
  if (lvl == BATT_OK) event(b, EV_BATTERY_OK, 0);
}

static void imu_step(Body& b, const TickIn& in) {
  if (in.imu == IMU_FRESH) {
    b.att = in.att;
    b.imu_stale_run = 0;
    b.imu_err_run = 0;
    return;
  }
  if (in.imu == IMU_ERR) {
    if (b.imu_err_run < 255) b.imu_err_run++;
  } else {
    if (b.imu_stale_run < 255) b.imu_stale_run++;
    if (b.imu_stale_total < 0xFFFF) b.imu_stale_total++;
  }
  if (!b.imu_fault && (b.imu_stale_run >= IMU_STALE_TICKS || b.imu_err_run >= IMU_ERR_RUN)) {
    // Balance and fall detection switch off (P1); re-init only while LIMP.
    b.imu_fault = 1;
    event(b, EV_IMU_FAULT, 0);
  }
}

void body_imu_recovered(Body& b) {
  b.imu_fault = 0;
  b.imu_stale_run = b.imu_err_run = 0;
  event(b, EV_IMU_OK, 0);
}

void body_tick(Body& b, const TickIn& in, uint16_t counts[JOINT_COUNT]) {
  b.tick++;
  // Step 2 happened in the caller (attitude_from_packet); keep the result.
  imu_step(b, in);
  // Step 4: reflexes and epoch bookkeeping, most important last so that it
  // wins the REFLEX owner within this same tick.
  warm_hold_step(b, in.imu == IMU_FRESH);
  watchdog_step(b);
  if (in.batt_valid) battery_step(b, in.batt_counts);
  limp_timer_step(b, RH_BATTERY, &b.batt_limp_timer);
  // Steps 5-6: owners, crossfade, layers, limits, slew, counts.
  arbiter_run(b.arb, b.cfg->img.cal, counts);
}

GoalResult body_goal_posture(Body& b, uint8_t epoch, uint8_t posture, uint8_t ticks, uint8_t flags) {
  if (posture >= POSTURE_COUNT || posture == POSTURE_STANCE) return GOAL_BAD_ARG;
  // LIMP and REST are always allowed: they are how the Pi makes Kilo safe.
  bool safe = posture == POSTURE_LIMP || posture == POSTURE_REST;
  if (!safe && epoch != b.epoch) return GOAL_STALE;
  if (!safe && b.batt.level >= BATT_LOW) return GOAL_LOW_BATT;
  if (posture == POSTURE_STAND && b.cfg->state == CFG_DEFAULTED) return GOAL_UNCALIBRATED;
  b.balance_req = flags & 1;
  if (b.holder == RH_WARM_HOLD) release_reflex(b, RH_WARM_HOLD);  // the Pi takes over
  if (posture == POSTURE_LIMP && b.holder != RH_NONE) arbiter_reflex(b.arb, POSTURE_LIMP, 0);
  arbiter_set_idle(b.arb, posture, ticks);
  return GOAL_OK;
}

uint8_t body_mode(const Body& b) {
  if (b.batt.level >= BATT_LOW) return KILO_MODE_LOW_BATT;
  if (b.wd.stage) return KILO_MODE_LINK_LOST;
  if (arbiter_idle_posture(b.arb) == POSTURE_LIMP && !b.arb.reflex_active) return KILO_MODE_LIMP;
  return KILO_MODE_IDLE;
}
