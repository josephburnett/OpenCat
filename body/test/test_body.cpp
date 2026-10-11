// The tick logic: arbiter (IDLE postures, crossfade, limits, slew), goal
// epoch, battery reflex with its LIMP timer, link watchdog stages, warm and
// cold boot, IMU health (Appendix H "arbiter", "reflex").
#include <cstdlib>
#include "../src/core/body.h"
#include "../src/core/posture.h"
#include "../src/core/reset.h"
#include "check.h"
#include "fake_eeprom.h"

static Config cfg;
static Body b;
static uint16_t counts[16];
static TickIn in;

static void tick(int n = 1) {
  for (int i = 0; i < n; i++) body_tick(b, in, counts);
}
static bool all_off() {
  for (int c = 0; c < 16; c++)
    if (!(counts[c] & PCA_FULL_OFF)) return false;
  return true;
}
static bool at_posture(uint8_t p) {
  for (int k = 0; k < DRIVEN_COUNT; k++) {
    uint8_t j = joint_id((uint8_t)k);
    int16_t a = joint_clamp(j, posture_angle(p, (uint8_t)k));
    if (counts[joint_channel(j)] != joint_counts(cfg.img.cal, j, a)) return false;
  }
  for (int j = 3; j <= 7; j++)
    if (!(counts[joint_channel((uint8_t)j)] & PCA_FULL_OFF)) return false;
  return true;
}
static bool has_event(uint8_t code, int16_t arg = -1) {
  for (int i = 0; i < b.ev.count; i++) {
    const Event& e = b.ev.q[(b.ev.head + i) % EVENT_QUEUE_LEN];
    if (e.code == code && (arg < 0 || e.arg == arg)) return true;
  }
  return false;
}
static void clear_events() { events_init(b.ev); }
static void reset(const int16_t* warm = nullptr) {
  config_defaults(cfg.img);
  cfg.state = CFG_MIGRATED;
  body_init(b, &cfg, warm, true);
  in = TickIn();
  in.imu = IMU_FRESH;
  in.att.gz = 16384;
}

int main() {
  // ---- Cold boot: LIMP until asked; REST is never stale.
  reset();
  tick();
  CHECK(all_off() && body_mode(b) == 1);
  CHECK(body_goal_posture(b, 99, POSTURE_REST, 0, 0) == GOAL_OK);
  tick();
  // From limp there is no pose to interpolate from: REST is commanded at
  // once, and the slew reference (REST) makes that a no-op.
  CHECK(at_posture(POSTURE_REST));

  // ---- REST -> STAND over the default 25 ticks: monotonic, slew-bounded.
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);
  int16_t prev[DRIVEN_COUNT];
  for (int k = 0; k < DRIVEN_COUNT; k++) prev[k] = b.arb.out[k];
  int moved_ticks = 0;
  for (int t = 0; t < 30; t++) {
    tick();
    bool moved = false;
    for (int k = 0; k < DRIVEN_COUNT; k++) {
      int16_t d = (int16_t)(b.arb.out[k] - prev[k]);
      CHECK(std::abs(d) <= PARAM(SLEW_MAX_A16));
      int16_t dir = (int16_t)(posture_angle(POSTURE_STAND, (uint8_t)k) - posture_angle(POSTURE_REST, (uint8_t)k));
      CHECK((long)d * dir >= 0);  // never moves backwards
      if (d) moved = true;
      prev[k] = b.arb.out[k];
    }
    if (moved) moved_ticks++;
  }
  CHECKF(moved_ticks == 25, "stand transition took %d ticks", moved_ticks);
  CHECK(at_posture(POSTURE_STAND));
  // Re-sending the same goal (5 Hz refresh) must not restart anything.
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);
  tick();
  CHECK(at_posture(POSTURE_STAND));

  // ---- SIT: RB/LB shoulders ask for -90, the safety stage clamps to -80.
  CHECK(body_goal_posture(b, b.epoch, POSTURE_SIT, 10, 0) == GOAL_OK);
  tick(20);
  CHECK(at_posture(POSTURE_SIT));
  CHECK(b.arb.out[5] == -80 * 16 && b.arb.out[6] == -80 * 16);

  // ---- Epoch: stale goals refused, LIMP/REST exempt; bad args.
  CHECK(body_goal_posture(b, (uint8_t)(b.epoch + 1), POSTURE_STAND, 0, 0) == GOAL_STALE);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, 0) == GOAL_BAD_ARG);
  CHECK(body_goal_posture(b, b.epoch, 9, 0, 0) == GOAL_BAD_ARG);
  // Uncalibrated (CONFIG_DEFAULTED) robots may not stand.
  cfg.state = CFG_DEFAULTED;
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_UNCALIBRATED);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_CALIB, 0, 0) == GOAL_OK);
  cfg.state = CFG_MIGRATED;

  // ---- Battery LOW: epoch bump, REFLEX -> REST over 1 s, LIMP 2 s later.
  reset();
  body_goal_posture(b, 0, POSTURE_STAND, 1, 0);
  in.batt_valid = 1;
  in.batt_counts = 740;  // ~7.47 V
  tick(10);
  CHECK(at_posture(POSTURE_STAND));
  in.batt_counts = 640;  // ~6.46 V: below LOW
  int t = 0;
  uint8_t e0 = b.epoch;
  while (b.batt.level < BATT_LOW && t < 2000) {
    tick();
    t++;
  }
  CHECKF(b.batt.level == BATT_LOW && t > 50 * 5 / 5 && t < 400, "LOW after %d ticks", t);
  CHECK(b.epoch != e0 && has_event(EV_LOW_BATTERY, BATT_LOW) && has_event(EV_LOW_BATTERY, BATT_WARN));
  CHECK(b.arb.reflex_active && b.notes.count > 0);  // REFLEX owns the joints, chirp queued
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_LOW_BATT);
  tick(REST_MOVE_TICKS);
  CHECK(at_posture(POSTURE_REST) && !all_off());
  tick(BATT_LIMP_TICKS - 2);
  CHECK(!all_off());
  tick(3);
  CHECK(all_off());  // the battery reflex's own LIMP timer fired
  // CRITICAL: LIMP immediately (already limp) and latched.
  in.batt_counts = 600;
  tick(200);
  CHECK(b.batt.level == BATT_CRITICAL && has_event(EV_LOW_BATTERY, BATT_CRITICAL) && all_off());

  // ---- Link watchdog: stop at 0.5 s, REST at 1.5 s, LIMP 10 s later.
  reset();
  body_goal_posture(b, 0, POSTURE_STAND, 1, 0);
  tick(5);
  body_link_frame(b);  // the Pi is talking
  for (int i = 0; i < 24; i++) tick();
  CHECK(!has_event(EV_WATCHDOG) && at_posture(POSTURE_STAND));
  e0 = b.epoch;
  tick();  // 25 ticks = 0.5 s without a frame
  CHECK(has_event(EV_WATCHDOG) && b.epoch == (uint8_t)(e0 + 1) && body_mode(b) == 9);
  CHECK(body_goal_posture(b, e0, POSTURE_SIT, 0, 0) == GOAL_STALE);  // pre-trip goal refused
  tick(49);
  CHECK(at_posture(POSTURE_STAND));  // still standing until 1.5 s
  tick(1 + REST_MOVE_TICKS);
  CHECK(at_posture(POSTURE_REST));
  tick(WD_LIMP_TICKS - 2);
  CHECK(!all_off());
  tick(2);
  CHECK(all_off());
  clear_events();
  body_link_frame(b);
  CHECK(has_event(EV_LINK_RESTORED) && !b.arb.reflex_active);
  tick(5);
  CHECK(all_off());  // stays in its current state until a new goal
  CHECK(body_goal_posture(b, b.epoch, POSTURE_REST, 0, 0) == GOAL_OK);
  tick();
  CHECK(at_posture(POSTURE_REST));

  // ---- Warm boot: hold the read-back pose exactly, then REST, then LIMP.
  {
    config_defaults(cfg.img);
    int16_t warm[DRIVEN_COUNT];
    uint16_t want[16];
    for (int k = 0; k < DRIVEN_COUNT; k++) {
      uint8_t j = joint_id((uint8_t)k);
      uint16_t c = joint_counts(cfg.img.cal, j, posture_angle(POSTURE_STAND, (uint8_t)k));
      want[joint_channel(j)] = c;
      warm[k] = joint_angle_from_counts(cfg.img.cal, j, c);
    }
    reset(warm);
    in.imu = IMU_NONE;  // the IMU is not ready yet
    for (int i = 0; i < WARM_HOLD_TICKS - 1; i++) {
      tick();
      bool same = true;
      for (int k = 0; k < DRIVEN_COUNT; k++) {
        uint8_t ch = joint_channel(joint_id((uint8_t)k));
        if (counts[ch] != want[ch]) same = false;
      }
      CHECKF(same, "warm hold moved at tick %d", i);
    }
    in.imu = IMU_FRESH;
    tick(1 + REST_MOVE_TICKS);
    CHECK(at_posture(POSTURE_REST));
    tick(WD_LIMP_TICKS);
    CHECK(all_off());
    // Warm boot with the Pi already talking: the pose is handed to IDLE.
    reset(warm);
    body_link_frame(b);
    tick(3);
    CHECK(!b.arb.reflex_active && b.holder == RH_NONE);
    for (int i = 0; i < 100; i++) {
      body_link_frame(b);  // the Pi keeps talking (heartbeats)
      tick();
    }
    bool same = true;
    for (int k = 0; k < DRIVEN_COUNT; k++) {
      uint8_t ch = joint_channel(joint_id((uint8_t)k));
      if (counts[ch] != want[ch]) same = false;
    }
    CHECK(same);
  }

  // ---- Boot epoch (doc §3c): a goal stamped before a warm reboot is STALE
  //      after it, consecutive boots never repeat an epoch.
  {
    ResetRecord rec = {0, 0, 0, 0, 0};
    uint16_t last;
    reset_classify(rec, 5, last);  // cold boot (invalid magic)
    config_defaults(cfg.img);
    cfg.state = CFG_MIGRATED;
    body_init(b, &cfg, nullptr, true, rec.epoch);
    in = TickIn();
    in.imu = IMU_FRESH;
    in.att.gz = 16384;
    CHECK(b.epoch == 1);
    for (int boot = 0; boot < 3; boot++) {
      uint8_t old = b.epoch;
      CHECK(body_goal_posture(b, old, POSTURE_STAND, 0, 0) == GOAL_OK);
      tick();
      rec.alive_tick = b.tick;  // what the firmware's tick writes
      rec.epoch = b.epoch;
      CHECK(reset_classify(rec, 5, last) == RESET_WARM_UNEXPECTED);  // watchdog/DTR reset
      int16_t warm[DRIVEN_COUNT] = {0};
      body_init(b, &cfg, warm, true, rec.epoch);
      CHECK(b.epoch != old && b.epoch == (uint8_t)(old + 1));
      // the refresh the UART buffered during the init: refused
      CHECK(body_goal_posture(b, old, POSTURE_STAND, 0, 0) == GOAL_STALE);
      CHECK(body_goal_posture(b, old, POSTURE_REST, 0, 0) == GOAL_OK);  // REST is never stale
    }
    // a runtime bump (watchdog stage 1, fall, low battery) just before the reset:
    // the record holds the bumped epoch, the boot goes past it.
    body_epoch_bump(b);
    uint8_t bumped = b.epoch;
    rec.epoch = b.epoch;
    reset_classify(rec, 5, last);
    body_init(b, &cfg, nullptr, true, rec.epoch);
    CHECK(b.epoch == (uint8_t)(bumped + 1));
    CHECK(body_goal_posture(b, bumped, POSTURE_STAND, 0, 0) == GOAL_STALE);
    // wrap: 255 -> 1
    rec.epoch = 255;
    reset_classify(rec, 5, last);
    body_init(b, &cfg, nullptr, true, rec.epoch);
    CHECK(b.epoch == 1 && body_goal_posture(b, 255, POSTURE_STAND, 0, 0) == GOAL_STALE);
    CHECK(body_goal_posture(b, 0, POSTURE_STAND, 0, 0) == GOAL_STALE);
  }

  // ---- IMU health: 10 ticks without a packet -> IMU_FAULT; recovery.
  reset();
  in.imu = IMU_NONE;
  tick(9);
  CHECK(!b.imu_fault);
  tick();
  CHECK(b.imu_fault && has_event(EV_IMU_FAULT));
  body_imu_recovered(b);
  CHECK(!b.imu_fault && has_event(EV_IMU_OK));
  reset();
  in.imu = IMU_ERR;
  tick(3);
  CHECK(b.imu_fault);
  return report("body");
}
