#include "body.h"
#include <string.h>
#include "fx.h"
#include "posture.h"
#include "pgm.h"

void body_init(Body& b, Config* cfg, const int16_t* warm_angles, bool imu_ok, uint8_t epoch) {
  // Everything starts at zero (queues empty, counters 0, filter unseeded,
  // watchdog unarmed) except what is set below. The epoch is the boot epoch
  // from the reset record (core/reset.h), never the previous run's.
  memset(&b, 0, sizeof b);
  b.epoch = epoch;
  b.cfg = cfg;
  b.att.gz = Q14_ONE;  // upright until the first packet says otherwise
  b.imu_fault = imu_ok ? 0 : 1;
  balance_init(b.bal);
  kf_init(b.kf);
  gait_init(b.gait);
  // Warm boot: the PCA9685 kept driving the servos through our reset; keep
  // exactly that pose until we know which way is up. Cold boot: the servo
  // pose is unknown, so stay limp until the Pi asks.
  arbiter_init(b.arb, warm_angles);
  b.holder = warm_angles ? RH_WARM_HOLD : RH_NONE;
}

KILO_NOINLINE static void event(Body& b, uint8_t code, int16_t arg) { events_push(b.ev, code, arg, b.tick, b.epoch); }

// The gait lets go of the legs at once (a reflex or trick took them, or
// LIMP): no ramp, the new owner starts from the current output.
static void gait_kill(Body& b) {
  if (!b.gait.state) return;
  b.gait.state = GS_OFF;
  event(b, EV_GAIT_STOPPED, 0);
}

// The trick has stopped playing (or is stopped now): hand back to IDLE.
KILO_NOINLINE static void trick_finish(Body& b, uint8_t reason, uint8_t ticks) {
  if (!ticks) ticks = (uint8_t)PARAM(XFADE_TICKS);
  kf_halt(b.kf, ticks);
  // IDLE adopts the end posture; joints the trick did not own move there
  // over the same time, the trick's joints crossfade from their output.
  arbiter_set_idle(b.arb, b.kf.end_posture, ticks ? ticks : 1);
  // The corrections restart from zero under the new owner: the crossfade
  // starts from the corrected output, so nothing jumps.
  balance_reset(b.bal);
  event(b, EV_TRICK_DONE, (int16_t)(((uint16_t)reason << 8) | b.kf.id));
}

void body_trick_end(Body& b, uint8_t reason, uint8_t ticks) {
  if (b.kf.run) trick_finish(b, reason, ticks);
}

void body_epoch_bump(Body& b, uint8_t trick_reason) {
  // A new epoch makes every goal the Pi sent before this moment stale, so
  // a goal re-sent from before a fall or a link loss cannot restart motion.
  // Latched motion goals stop here: the head hold, the trick, the gait
  // (it ramps down and lands; a reflex taking the legs cuts it short) and
  // the body pose (it slews back to neutral). The IDLE posture is not a
  // motion goal, but its balance flag is: the Pi re-asserts it with the new
  // epoch. The trick's TRICK_DONE carries the new epoch.
  b.epoch++;
  Gait& g = b.gait;
  g.run = 0;
  g.pose_g[0] = g.pose_g[1] = 0;
  g.dh_g = g.xs_g = 0;
  b.balance_req = 0;
  // SET_BALANCE's enable is a goal too (spec: goals go stale at a bump); a
  // `kilo balance on` client that died must not leave every later STAND
  // balancing (P1 hardware finding B1). The gains stay: not motion.
  b.bal.enable = 0;
  arbiter_head_release(b.arb);
  body_trick_end(b, trick_reason, 0);
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

// ---- The fall reflex (doc §3g; the state machine is described in reflex.h) ----
// rc frame i of this try: the REFLEX mover moves there from the current
// output over the frame's dur, then holds it.
KILO_NOINLINE static void rc_frame(Body& b, uint8_t i) {
  Fall& f = b.fall;
  f.frame = i;
  f.timer = (uint16_t)(rc_dur(i) + rc_hold(i));
  arbiter_reflex(b.arb, (uint8_t)(POSTURE_RC + i + f.mirror), rc_dur(i));
}

static void fall_failed(Body& b) {
  b.fall.state = FS_FAILED;
  rest_then_limp(b, RH_FALL, &b.fall.timer, FALL_LIMP_TICKS);
  event(b, EV_RECOVERY_FAILED, 0);
}

static void fall_recovered(Body& b) {
  // IDLE takes STAND (REST without a calibration: STAND would be refused
  // UNCALIBRATED) from the current output; the REFLEX hands back to it
  // with the usual XFADE_TICKS crossfade.
  arbiter_set_idle(b.arb, b.cfg->state == CFG_DEFAULTED ? POSTURE_REST : POSTURE_STAND, 0);
  release_reflex(b, RH_FALL);
  b.fall.state = FS_UPRIGHT;
  event(b, EV_RECOVERED, 0);
}

static void fall_step(Body& b, bool fresh) {
  Fall& f = b.fall;
  // A more important reflex (watchdog REST, low battery) took the joints:
  // it owns them now; detection re-arms once they are released.
  if (f.state != FS_UPRIGHT && b.holder != RH_FALL) f.state = FS_UPRIGHT;
  if (f.state == FS_UPRIGHT) {
    uint8_t p = arbiter_idle_posture(b.arb);
    if (b.imu_fault || b.holder != RH_NONE || body_fall_suppressed(b) || p == POSTURE_LIMP ||
        p == POSTURE_CALIB) {
      f.over = 0;
      return;
    }
    if (!fresh) return;
    int8_t arg = fall_detect(f.over, b.att);
    if (!arg) return;
    // Fallen: every goal sent before now is stale (a trick ends with
    // TRICK_DONE FELL), the REFLEX holds the pose until the body is still.
    body_epoch_bump(b, KF_END_FELL);
    b.holder = RH_FALL;
    arbiter_reflex(b.arb, POSTURE_HOLD, 0);
    f.state = FS_FALLEN;
    f.axis = arg;
    f.tries = f.still = 0;
    event(b, EV_FELL, arg);
    notes_push(b.notes, 20, 15);  // a chirp (legacy beeps note 20 on a fall)
    return;
  }
  if (f.state == FS_FAILED) {
    limp_timer_step(b, RH_FALL, &f.timer);
    if (!f.timer) {  // limp: let go (detection stays off in LIMP)
      release_reflex(b, RH_FALL);
      f.state = FS_UPRIGHT;
    }
    return;
  }
  // Without the IMU (which also stops fresh samples) nothing can be judged.
  if (b.imu_fault) return fall_failed(b);
  if (f.state == FS_FALLEN) {
    if (!fresh) return;
    f.still = fall_still(b.att) ? (uint8_t)(f.still + 1) : 0;
    if (f.still < FALL_STILL_TICKS) return;
    if (f.axis == 2 || f.axis == -2) return fall_failed(b);  // no recovery from nose or tail down
    // rc as legacy wrote it pushes with the right legs (unmirrored for the
    // right side down, roll < 0); every other try flips it, in case the
    // body lies the other way round by then.
    f.mirror = (uint8_t)(((b.att.roll > 0) ^ (f.tries & 1)) ? POSTURE_RC_MIRROR : 0);
    f.tries++;
    f.state = FS_RC;
    return rc_frame(b, 0);
  }
  if (f.state == FS_CHECK && fresh) {
    // (No saturation needed: CHECK ends at FALL_LEVEL_TICKS.)
    f.level = fall_level(b.att) ? (uint8_t)(f.level + 1) : 0;
    if (f.level >= FALL_LEVEL_TICKS) return fall_recovered(b);
  }
  if (--f.timer) return;
  if (f.state == FS_RC) {
    if (f.frame + 1 < RC_FRAMES) return rc_frame(b, (uint8_t)(f.frame + 1));
    f.state = FS_CHECK;  // hold rc's last frame, waiting for upright
    f.timer = FALL_CHECK_TICKS;
    f.level = 0;
    return;
  }
  // CHECK timed out: not upright. The next try (after the body is still
  // again), or give up.
  if (f.tries >= FALL_TRIES) return fall_failed(b);
  f.state = FS_FALLEN;
  f.still = 0;
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
  // Stage 2 also waits for the gait's ramp-down and landing (stage 1's
  // epoch bump stopped it), so Kilo does not lie down mid-stride.
  if (w.stage == 1 && w.since >= WD_REST_TICKS && !body_gait_running(b)) {
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
  fall_step(b, in.imu == IMU_FRESH);
  watchdog_step(b);
  if (in.batt_valid) battery_step(b, in.batt_counts);
  limp_timer_step(b, RH_BATTERY, &b.batt_limp_timer);
  // The keyframe player (doc §3e) steps first. A reflex that took the
  // joints preempts it.
  if (b.kf.run && b.arb.reflex_active) body_trick_end(b, KF_END_PREEMPTED, 0);
  if (kf_step(b.kf)) trick_finish(b, KF_END_DONE, 0);
  // The gait (doc §3d). A reflex or a trick that owns a leg pre-empts it at
  // once; the goal stays latched (an epoch bump clears it), so it restarts
  // from STANCE when the legs are free again. A run goal waits for IDLE
  // STANCE (moving there first) to settle, then starts at phi = 0.
  Gait& g = b.gait;
  bool legs_taken = b.arb.reflex_active || (b.kf.run && (b.kf.mask & 0xFF00u));
  if (legs_taken) gait_kill(b);
  if (gait_step(g) && !g.run) event(b, EV_GAIT_STOPPED, 0);  // landed: IDLE STANCE has the legs
  uint8_t p = arbiter_idle_posture(b.arb);
  if (!g.state && g.run && !legs_taken) {
    if (p != POSTURE_STANCE) {
      arbiter_set_idle(b.arb, POSTURE_STANCE, 0);
      gait_stance_default(g);
      p = POSTURE_STANCE;
    } else if (!arbiter_idle_moving(b.arb)) {
      gait_start(g);
    }
  }
  bool gait_on = g.state != GS_OFF;
  // The balance (doc §3c layer 2, §3f). Joint-space for IDLE SIT and STAND
  // once their move has settled; never under REFLEX (which also covers the
  // warm-boot hold) and never in LIMP, CALIB, ZERO or REST (lying on the
  // belly there is nothing to balance). Outside those the corrections are
  // zeroed at once: the reflex or the new posture's mover starts from the
  // corrected output, so nothing jumps. Inside, a request turned off or an
  // IMU_FAULT lets them decay through the filter and slew.
  //
  // While a trick plays, balance follows the frame's balance bit alone,
  // against the trick's expected attitude (doc §3e); between balanced
  // frames the corrections decay through the filter and slew.
  //
  // The gait and IDLE STANCE balance geometrically (§3f): the corrections
  // tilt the body through the IK's pose terms instead of being added to
  // joints. The controller sees the ground slope rather than the body's
  // tilt: what the body would read with no correction and no commanded
  // pose (measured - corr - pose). That removes the correction's own effect
  // from the loop, so corr = -Kp * slope: at Kp ~1 the body ends level to
  // within the dead band whatever the slope (H13), with no integrator to
  // wind up (corr is bounded by the clamp), and the loop stays stable for
  // any leg response between 0 and 2x the model's. Leaving it for a
  // joint-space owner zeroes the corrections (they mean something else
  // there).
  bool geo = !b.kf.run && !b.arb.reflex_active && (gait_on || (p == POSTURE_STANCE && !arbiter_idle_moving(b.arb)));
  if (g.geo && !geo) balance_reset(b.bal);
  g.geo = geo;
  if (b.kf.run && !b.arb.reflex_active) {
    balance_step(b.bal, kf_balance(b.kf) && !b.imu_fault, b.att, kf_ref_roll(b.kf), kf_ref_pitch(b.kf));
  } else if (geo) {
    bool want = (gait_on ? (g.flags & KILO_GAIT_FLAGS_BALANCE) : (b.balance_req || b.bal.enable)) && !b.imu_fault;
    Attitude slope = b.att;
    slope.pitch = (int16_t)(slope.pitch - b.bal.ax[BAL_PITCH].corr - g.pose[0]);
    slope.roll = (int16_t)(slope.roll - b.bal.ax[BAL_ROLL].corr - g.pose[1]);
    balance_step(b.bal, want, slope, 0, 0);
  } else if (b.arb.reflex_active || arbiter_idle_moving(b.arb) || !(p == POSTURE_SIT || p == POSTURE_STAND)) {
    balance_reset(b.bal);
  } else {
    bool want = (b.balance_req || b.bal.enable) && !b.imu_fault;
    balance_step(b.bal, want, b.att, posture_roll(p), posture_pitch(p));
  }
  // The legs' IK angles for this tick (the gait's feet, or STANCE's).
  if (gait_on || p == POSTURE_STANCE) gait_legs(g, b.bal.ax[BAL_ROLL].corr, b.bal.ax[BAL_PITCH].corr);
  // Steps 5-6: owners, crossfade, layers, limits, slew, counts.
  arbiter_run(b.arb, b.cfg->img.cal, counts, geo ? 0 : &b.bal, &b.kf, &g);
}

GoalResult body_goal_posture(Body& b, uint8_t epoch, uint8_t posture, uint8_t ticks, uint8_t flags) {
  if (posture >= POSTURE_COUNT) return GOAL_BAD_ARG;
  // LIMP and REST are always allowed: they are how the Pi makes Kilo safe.
  bool safe = posture == POSTURE_LIMP || posture == POSTURE_REST;
  if (!safe && epoch != b.epoch) return GOAL_STALE;
  if (body_fallen(b)) {
    // The fall reflex holds the joints until RECOVERED; only LIMP (the Pi's
    // emergency stop) ends it.
    if (posture != POSTURE_LIMP) return GOAL_FALLEN;
    b.fall.state = FS_UPRIGHT;
    release_reflex(b, RH_FALL);
  }
  if (!safe && b.batt.level >= BATT_LOW) return GOAL_LOW_BATT;
  if ((posture == POSTURE_STAND || posture == POSTURE_STANCE) && b.cfg->state == CFG_DEFAULTED)
    return GOAL_UNCALIBRATED;
  // Any posture but STANCE ends the gait goal: LIMP at once, the others
  // through the ramp (IDLE moves to the posture meanwhile; the legs
  // crossfade there when the gait lets go). STANCE from elsewhere starts at
  // the default height.
  Gait& g = b.gait;
  if (posture != POSTURE_STANCE) g.run = 0;
  if (posture == POSTURE_LIMP) gait_kill(b);
  if (posture == POSTURE_STANCE && arbiter_idle_posture(b.arb) != POSTURE_STANCE) gait_stance_default(g);
  b.balance_req = flags & 1;
  if (b.holder == RH_WARM_HOLD) release_reflex(b, RH_WARM_HOLD);  // the Pi takes over
  // LIMP is the Pi's emergency stop: it ends a trick at once. Any other
  // posture only moves the joints the trick does not own; the trick's end
  // posture replaces it when the trick ends.
  if (posture == POSTURE_LIMP) body_trick_end(b, KF_END_STOPPED, 1);
  if (posture == POSTURE_LIMP && b.holder != RH_NONE) arbiter_reflex(b.arb, POSTURE_LIMP, 0);
  arbiter_set_idle(b.arb, posture, ticks);
  return GOAL_OK;
}

GoalResult body_goal_balance(Body& b, const uint8_t p[5]) {
  if (p[0] > 1 || p[3] > BAL_MAX_DEG_LIMIT || p[4] > BAL_MAX_DEG_LIMIT) return GOAL_BAD_ARG;
  Balance& s = b.bal;
  s.enable = p[0];
  s.kp_q8 = p[1];
  s.kd_ms = p[2];
  s.db_deg = p[3];
  s.max_deg = p[4];
  return GOAL_OK;
}

uint8_t body_trick_play(Body& b, const uint8_t p[4]) {
  if (p[0] != b.epoch) return GOAL_STALE;
  uint8_t s = kf_check_play(b.kf, p[1], p[2], p[3]);
  if (s) return s;
  if (body_fallen(b)) return GOAL_FALLEN;
  if (b.batt.level >= BATT_LOW) return GOAL_LOW_BATT;
  uint8_t idle = arbiter_idle_posture(b.arb);
  if (b.holder != RH_NONE || b.arb.reflex_active || idle == POSTURE_LIMP || idle == POSTURE_CALIB) return GOAL_BUSY;
  if (b.cfg->state == CFG_DEFAULTED) return GOAL_UNCALIBRATED;
  kf_start(b.kf, p[2], p[3] & KILO_TRICK_PLAY_FLAGS_MIRROR);
  // The trick's first segment starts from the corrected output; the
  // corrections restart from zero (only balanced frames bring them back).
  balance_reset(b.bal);
  return GOAL_OK;
}

GoalResult body_goal_head(Body& b, const uint8_t* p) {
  // Payload (kilo_set_head_t, little-endian like both targets): epoch, mode,
  // pan, tilt, tail (i16 a16), max_speed_dps (u16).
  uint8_t mode = p[1];
  if (mode > KILO_HEAD_MODE_HOLD) return GOAL_BAD_ARG;
  Arbiter& a = b.arb;
  if (mode == KILO_HEAD_MODE_RELEASE) {  // calming down is never stale
    arbiter_head_release(a);
    return GOAL_OK;
  }
  if (p[0] != b.epoch) return GOAL_STALE;
  if (body_fallen(b)) return GOAL_FALLEN;
  if (b.batt.level >= BATT_LOW) return GOAL_LOW_BATT;
  // Targets as sent (the safety stage clamps them to the joint limits);
  // JOINT_OFF (-32768) leaves that joint to its owner.
  memcpy(a.head_tgt, p + 2, sizeof a.head_tgt);
  // deg/s -> a16 per 20 ms tick (x 16 / 50), rounded up so a slow head
  // still moves. 0, or anything over 2500 deg/s (SLEW_MAX_A16's maximum),
  // leaves only the safety slew.
  uint16_t dps = (uint16_t)(p[8] | (p[9] << 8));
  a.head_step = (dps && dps <= 2500u) ? (int16_t)((dps * 8u + 24u) / 25u) : 0x3FFF;
  a.head_ttl = HEAD_HOLD_TICKS;
  return GOAL_OK;
}

GoalResult body_goal_gait(Body& b, const uint8_t* p) {
  if (gait_check(p) != GOAL_OK) return GOAL_BAD_ARG;
  if (!p[1]) {  // stopping is never stale
    b.gait.run = 0;
    return GOAL_OK;
  }
  if (p[0] != b.epoch) return GOAL_STALE;
  if (body_fallen(b)) return GOAL_FALLEN;
  if (b.batt.level >= BATT_LOW) return GOAL_LOW_BATT;
  if (b.cfg->state == CFG_DEFAULTED) return GOAL_UNCALIBRATED;
  if (b.holder == RH_WARM_HOLD) release_reflex(b, RH_WARM_HOLD);  // the Pi takes over
  gait_goal(b.gait, p);
  return GOAL_OK;
}

GoalResult body_goal_pose(Body& b, const uint8_t* p) {
  if (p[0] != b.epoch) return GOAL_STALE;
  if (body_fallen(b)) return GOAL_FALLEN;
  if (b.batt.level >= BATT_LOW) return GOAL_LOW_BATT;
  gait_pose_goal(b.gait, p + 1);
  return GOAL_OK;
}

void body_trick_stop(Body& b, uint8_t ticks) { body_trick_end(b, KF_END_STOPPED, ticks); }

uint8_t body_mode(const Body& b) {
  if (b.batt.level >= BATT_LOW) return KILO_MODE_LOW_BATT;
  if (b.wd.stage) return KILO_MODE_LINK_LOST;
  if (b.fall.state == FS_RC || b.fall.state == FS_CHECK) return KILO_MODE_RECOVERING;
  if (b.fall.state) return KILO_MODE_FALLEN;
  if (b.kf.run) return KILO_MODE_TRICK;
  if (b.gait.state || b.gait.run) return KILO_MODE_GAIT;
  if (arbiter_idle_posture(b.arb) == POSTURE_LIMP && !b.arb.reflex_active) return KILO_MODE_LIMP;
  return KILO_MODE_IDLE;
}
