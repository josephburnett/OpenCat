// The balance layer (doc §3f, Appendix H "balance" and the balance gating of
// "arbiter"):
//   1. the per-joint gain table == legacy adaptiveParameterArray (motion.h
//      constants) == Kilo's EEPROM ADAPT_PARAM;
//   2. balance_map() reproduces legacy adjust() (float, transcribed from
//      src/motion.h + skill.h) over a grid, and the whole controller at
//      Kp ~ 1, Kd 0 reproduces it for a set of tilts, dead bands included;
//   3. the mapping's physical direction, checked against the posture tables;
//   4. toy-plant closed loops (corr space and through body_tick + joints):
//      converge, and every sign flip diverges;
//   5. dead band, saturation/clamp, slew;
//   6. gating: owner (REFLEX), posture (LIMP/REST/CALIB/ZERO, transitions),
//      request (posture flag, SET_BALANCE enable, epoch bump), IMU_FAULT.
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include "../src/core/body.h"
#include "../src/core/posture.h"
#include "check.h"
#include "fake_eeprom.h"

static std::string slurp(const char* rel) {
  std::ifstream f(std::string(REPO_ROOT) + "/" + rel);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

// ---- Legacy reference (src/motion.h, src/skill.h), in double -------------
static double LEG[16][2];  // adaptiveParameterArray as stored (int8 in EEPROM)
static double LRF;         // LEFT_RIGHT_FACTOR
static double TOL[2];      // levelTolerance {roll, pitch}

static void load_legacy() {
  std::string m = slurp("src/motion.h");
  std::map<std::string, std::string> def;
  // "#define NAME value  // comment" lines.
  std::istringstream lines(m);
  for (std::string ln; std::getline(lines, ln);) {
    if (ln.compare(0, 8, "#define ") != 0) continue;
    size_t n0 = ln.find_first_not_of(' ', 8), n1 = ln.find(' ', n0);
    if (n1 == std::string::npos) continue;
    std::string v = ln.substr(ln.find_first_not_of(' ', n1));
    v = v.substr(0, v.find("//"));
    while (!v.empty() && (v.back() == ' ' || v.back() == '\r')) v.pop_back();
    def[ln.substr(n0, n1 - n0)] = v;
  }
  CHECK(def["lRF"] == "(-1.5 * uRF)" && def["lPF"] == "(-1.5 * uPF)");
  double panF = std::stod(def["panF"]), tiltF = std::stod(def["tiltF"]);
  double sRF = std::stod(def["sRF"]), sPF = std::stod(def["sPF"]);
  double uRF = std::stod(def["uRF"]), uPF = std::stod(def["uPF"]);
  double lRF = -1.5 * uRF, lPF = -1.5 * uPF;
  LRF = std::stod(def["LEFT_RIGHT_FACTOR"]);
  TOL[0] = std::stod(def["ROLL_LEVEL_TOLERANCE"]);
  TOL[1] = std::stod(def["PITCH_LEVEL_TOLERANCE"]);
  // The array exactly as motion.h writes it.
  CHECK(m.find("{ -panF, 0 },\n  { -panF, -tiltF },\n  { -2 * panF, 0 },\n  { 0, 0 },\n  { sRF, -sPF },") !=
        std::string::npos);
  const double t[16][2] = {{-panF, 0}, {-panF, -tiltF}, {-2 * panF, 0}, {0, 0}, {sRF, -sPF}, {-sRF, -sPF},
                           {-sRF, sPF}, {sRF, sPF},      {uRF, uPF},      {uRF, uPF}, {-uRF, uPF}, {-uRF, uPF},
                           {lRF, lPF},  {lRF, lPF},      {-lRF, lPF},     {-lRF, lPF}};
  // Kilo's legacy EEPROM, ADAPT_PARAM @128 (docs/kilo-eeprom-legacy.md).
  static const uint8_t EE[32] = {0xc4, 0x00, 0xc4, 0xc4, 0x88, 0x00, 0x00, 0x00, 0x32, 0xf4, 0xce,
                                 0xf4, 0xce, 0x0c, 0x32, 0x0c, 0x3c, 0x1e, 0x3c, 0x1e, 0xc4, 0x1e,
                                 0xc4, 0x1e, 0xa6, 0xd3, 0xa6, 0xd3, 0x5a, 0xd3, 0x5a, 0xd3};
  for (int i = 0; i < 16; i++)
    for (int j = 0; j < 2; j++) {
      LEG[i][j] = (int8_t)std::lround(t[i][j]);  // writeConst(): (int8_t)round(...)
      CHECKF((int8_t)EE[i * 2 + j] == LEG[i][j], "EEPROM ADAPT_PARAM[%d][%d]", i, j);
    }
}

// motion.h adjust(i) without the damper (its steady state), for a posture
// (postureOrWalkingFactor 1, slope 1). devR/devP: RollPitchDeviation.
static double legacy_adjust(int i, double devR, double devP) {
  double rollAdj;
  if (i == 1 || i > 3) {
    bool leftQ = (i - 1) % 4 > 1;
    double f = ((leftQ && devR > 0) || (!leftQ && devR < 0)) ? LRF : 1;
    rollAdj = std::fabs(devR) * LEG[i][0] * f;
  } else {
    rollAdj = devR * LEG[i][0];
  }
  return M_PI / 180 * (rollAdj - LEG[i][1] * devP);
}

// skill.h: RollPitchDeviation[i] = sign(ypr) * max(|ypr - expected| - tol, 0).
static double legacy_dev(double ypr, double expected, double tol) {
  double s = ypr > 0 ? 1 : ypr < 0 ? -1 : 0;
  return s * std::fmax(std::fabs(ypr - expected) - tol, 0.0);
}

static const uint8_t KJ[DRIVEN_COUNT] = {0, 1, 2, 8, 9, 10, 11, 12, 13, 14, 15};

// ---- 1-2. Gains and the legacy mapping -----------------------------------
static void test_legacy_mapping() {
  load_legacy();
  // balance_map() alone, over a grid of corrections: the legacy formula with
  // RollPitchDeviation = -corr converted to the legacy convention.
  double worst = 0;
  for (int cr = -240; cr <= 240; cr += 12)
    for (int cp = -240; cp <= 240; cp += 12)
      for (int k = 0; k < DRIVEN_COUNT; k++) {
        double devR = LEGACY_ROLL_SIGN * (-cr / 16.0), devP = LEGACY_PITCH_SIGN * (-cp / 16.0);
        double want = legacy_adjust(KJ[k], devR, devP);
        double got = balance_map((uint8_t)k, (int16_t)cr, (int16_t)cp) / 16.0;
        worst = std::fmax(worst, std::fabs(got - want));
        CHECKF(std::fabs(got - want) <= 0.07, "map k=%d cr=%d cp=%d: %.3f vs legacy %.3f", k, cr, cp, got, want);
      }
  // The whole controller in STAND (expected 0/0), Kp = 255/256 (the largest
  // kp_q8), Kd 0, clamp 30 deg, settled: the legacy adjust() for the same
  // tilt, with legacy's own dead bands (roll 5, pitch 3 deg) where only one
  // axis is tilted, and a shared one otherwise. Within 1 deg (doc §7).
  struct Case {
    double roll, pitch;  // Kilo convention, degrees
    uint8_t db;
    bool legacy_tol;
  } cases[] = {{10, 0, 5, true},   {-10, 0, 5, true},  {22, 0, 5, true},   {0, 8, 3, true},
               {0, -8, 3, true},   {0, 20, 3, true},   {0, -25, 3, true},  {4, 0, 5, true},
               {0, 2.5, 3, true},  {7, 12, 3, false},  {-12, -6, 2, false}, {15, -15, 2, false},
               {-20, 18, 4, false}};
  double worst_ctl = 0;
  for (const Case& c : cases) {
    Balance bal;
    balance_init(bal);
    bal.kp_q8 = 255;
    bal.kd_ms = 0;
    bal.db_deg = c.db;
    bal.max_deg = 30;
    Attitude att = {};
    att.roll = (int16_t)std::lround(c.roll * 16);
    att.pitch = (int16_t)std::lround(c.pitch * 16);
    for (int t = 0; t < 400; t++) balance_step(bal, true, att, posture_roll(POSTURE_STAND), posture_pitch(POSTURE_STAND));
    double tolR = c.legacy_tol ? TOL[0] : c.db, tolP = c.legacy_tol ? TOL[1] : c.db;
    double yprR = LEGACY_ROLL_SIGN * c.roll, yprP = LEGACY_PITCH_SIGN * c.pitch;
    double devR = legacy_dev(yprR, 0, tolR), devP = legacy_dev(yprP, 0, tolP);
    for (int k = 0; k < DRIVEN_COUNT; k++) {
      double want = legacy_adjust(KJ[k], devR, devP);
      double got = balance_joint(bal, (uint8_t)k) / 16.0;
      worst_ctl = std::fmax(worst_ctl, std::fabs(got - want));
      CHECKF(std::fabs(got - want) <= 1.0, "roll %.1f pitch %.1f joint %d: %.2f vs legacy %.2f", c.roll, c.pitch,
             KJ[k], got, want);
    }
  }
  std::printf("balance: map vs legacy adjust() max err %.3f deg; controller (Kp 255/256) vs legacy max err %.3f deg\n",
              worst, worst_ctl);
}

// ---- 3. Physical direction against the posture tables ---------------------
static int sgn(int v) { return (v > 0) - (v < 0); }

static void test_directions() {
  // Measured convention: nose DOWN = +pitch, RIGHT side up = +roll. A
  // positive correction must raise the measured value on its axis.
  // +corr_pitch lowers the nose: from STAND, the knees and back shoulders
  // move AWAY from SIT (SIT = nose up: back folded, front extended), i.e.
  // the front folds and the back extends. -corr_pitch: toward SIT.
  for (int k : {5, 6, 7, 8, 9, 10}) {
    int toward = sgn(posture_angle(POSTURE_SIT, (uint8_t)k) - posture_angle(POSTURE_STAND, (uint8_t)k));
    CHECKF(toward != 0 && sgn(balance_map((uint8_t)k, 0, 80)) == -toward, "+pitch corr, joint %d", KJ[k]);
    CHECKF(sgn(balance_map((uint8_t)k, 0, -80)) == toward, "-pitch corr, joint %d", KJ[k]);
  }
  // +corr_roll raises the right side: every leg crouches toward REST, the
  // left legs (8, 11, 12, 15) 1.2x as far as the right (9, 10, 13, 14), so
  // the left side drops. -corr_roll: the right legs crouch further.
  for (int sign : {1, -1}) {
    for (int k = 3; k < DRIVEN_COUNT; k++) {
      int toward = sgn(posture_angle(POSTURE_REST, (uint8_t)k) - posture_angle(POSTURE_STAND, (uint8_t)k));
      CHECKF(sgn(balance_map((uint8_t)k, (int16_t)(80 * sign), 0)) == toward, "roll corr %d, joint %d", sign, KJ[k]);
    }
    const int pairs[4][2] = {{3, 4}, {6, 5}, {7, 8}, {10, 9}};  // {left, right}: LF/RF, LB/RB shoulders and knees
    for (auto& p : pairs) {
      int l = std::abs(balance_map((uint8_t)p[0], (int16_t)(160 * sign), 0));
      int r = std::abs(balance_map((uint8_t)p[1], (int16_t)(160 * sign), 0));
      CHECKF(sign > 0 ? l > r : r > l, "roll corr %d: left %d right %d", sign, l, r);
    }
  }
}

// ---- 4. Toy plant, corr space ----------------------------------------------
// Body tilt = d + x, where x follows the commanded correction with a
// first-order lag tau (the legs move, the body follows). The controller sees
// meas_sign * tilt and a gyro rate rate_sign * meas_sign * dtilt/dt.
struct Loop {
  double final_err, late_max, late_pp;  // |tilt| at the end; max |tilt| and peak-to-peak over the last 100 ticks
};
static Loop toy(double d, double tau, uint8_t kp, uint8_t kd, int plant_sign, int meas_sign, int rate_sign) {
  Balance bal;
  balance_init(bal);
  bal.kp_q8 = kp;
  bal.kd_ms = kd;
  BalAxis s = {0, 0};
  const double dt = 0.02;
  double x = 0, rate = 0, late = 0, lo = 1e9, hi = -1e9, tilt = d;
  for (int t = 0; t < 600; t++) {
    tilt = d + x;
    int16_t m = (int16_t)std::lround(meas_sign * tilt * 16);
    int16_t r = (int16_t)std::lround(rate_sign * meas_sign * rate * 16);
    double corr = balance_axis_step(bal, s, true, m, 0, r) / 16.0;
    double dx = (plant_sign * corr - x) * dt / tau;
    x += dx;
    rate = dx / dt;
    if (t >= 500) {
      late = std::fmax(late, std::fabs(d + x));
      lo = std::fmin(lo, d + x);
      hi = std::fmax(hi, d + x);
    }
  }
  return {std::fabs(tilt), late, hi - lo};
}

static void test_toy_plant() {
  const double d = 10;  // a 10 deg step disturbance (a slope under the feet)
  // Defaults (Kp 0.5, Kd 30 ms): the error settles at (d + Kp*DB)/(1 + Kp).
  Loop ok = toy(d, 0.15, BAL_KP_Q8, BAL_KD_MS, 1, 1, 1);
  CHECKF(ok.final_err < 0.8 * d && std::fabs(ok.final_err - (d + 0.5 * 2) / 1.5) < 0.3 && ok.late_max < 0.8 * d,
         "converges: final %.2f, late max %.2f", ok.final_err, ok.late_max);
  // Flip the actuation (corr raises what it should lower) or the measurement
  // (sensor sign): the error grows past the disturbance, to (d - Kp*DB)/(1 - Kp)
  // = 18 deg at Kp 0.5 (Kp >= 1 would run to the clamp).
  Loop pf = toy(d, 0.15, BAL_KP_Q8, BAL_KD_MS, -1, 1, 1);
  Loop mf = toy(d, 0.15, BAL_KP_Q8, BAL_KD_MS, 1, -1, 1);
  CHECKF(pf.final_err > 1.5 * d, "plant flip: final %.2f", pf.final_err);
  CHECKF(mf.final_err > 1.5 * d, "measurement flip: final %.2f", mf.final_err);
  // Kd: with a rate gain beyond the plant lag (200 ms vs 100 ms) the correct
  // sign still converges and settles; the flipped rate sign is unstable: it
  // never settles (a sustained oscillation bounded only by slew and clamp).
  Loop kd_ok = toy(d, 0.1, 64, 200, 1, 1, 1);
  Loop kd_flip = toy(d, 0.1, 64, 200, 1, 1, -1);
  CHECKF(kd_ok.late_max < d && kd_ok.late_pp < 0.2, "Kd converges: late max %.2f pp %.2f", kd_ok.late_max,
         kd_ok.late_pp);
  CHECKF(kd_flip.late_max > 1.2 * d && kd_flip.late_pp > 5, "Kd flip: late max %.2f pp %.2f", kd_flip.late_max,
         kd_flip.late_pp);
  std::printf("balance: toy plant d=10: ok %.2f (Kd 200 ms: %.2f) | flips: plant %.2f, meas %.2f, Kd oscillates "
              "%.2f p-p deg\n", ok.final_err, kd_ok.final_err, pf.final_err, mf.final_err, kd_flip.late_pp);
}

// ---- 5. Dead band, saturation, slew --------------------------------------
static void test_shaping() {
  Balance bal;
  balance_init(bal);
  bal.kd_ms = 0;
  BalAxis s = {0, 0};
  // Inside the 2 deg dead band: nothing, ever.
  for (int t = 0; t < 100; t++) CHECK(balance_axis_step(bal, s, true, 31, 0, 0) == 0);
  for (int t = 0; t < 100; t++) CHECK(balance_axis_step(bal, s, true, -32, 0, 0) == 0);
  // Just outside: e = 1 deg past the band, corr = -Kp * 1 deg = -8 a16.
  for (int t = 0; t < 100; t++) balance_axis_step(bal, s, true, 48, 0, 0);
  CHECKF(s.corr == -8, "corr %d", s.corr);
  // Relative to a reference of 30 deg: 31 deg is in the band.
  s = {0, 0};
  for (int t = 0; t < 100; t++) balance_axis_step(bal, s, true, 31 * 16, 30 * 16, 0);
  CHECK(s.corr == 0);
  // Saturation: a huge error clamps at exactly +-15 deg; slew <= 1 deg/tick,
  // so it takes at least 15 ticks to get there.
  s = {0, 0};
  int16_t prev = 0;
  int ticks = 0;
  for (int t = 0; t < 200; t++) {
    int16_t c = balance_axis_step(bal, s, true, 60 * 16, 0, 0);
    CHECK(std::abs(c - prev) <= BAL_SLEW_A16 && c >= -240 && c <= 240);
    if (c != -240) ticks = t + 1;
    prev = c;
  }
  CHECKF(prev == -240 && ticks >= 15, "clamp %d after %d ticks", prev, ticks);
  // A large rate alone also saturates (Kd term), the other way.
  BalAxis s2 = {0, 0};
  bal.kd_ms = 255;
  for (int t = 0; t < 100; t++) balance_axis_step(bal, s2, true, 0, 0, -30000);
  CHECK(s2.corr == 240);
  // Live clamp change: max 5 deg.
  bal.max_deg = 5;
  for (int t = 0; t < 100; t++) balance_axis_step(bal, s2, true, 0, 0, -30000);
  CHECK(s2.corr == 80);
  // Inactive: decays to exactly 0 through filter and slew, never overshoots.
  prev = s2.corr;
  for (int t = 0; t < 100; t++) {
    int16_t c = balance_axis_step(bal, s2, false, 0, 0, -30000);
    CHECK(c >= 0 && c <= prev && prev - c <= BAL_SLEW_A16);
    prev = c;
  }
  CHECK(s2.corr == 0 && s2.lp == 0);
  // Negative side decays to 0 too (symmetric rounding).
  bal.kd_ms = 0;
  bal.max_deg = 15;
  BalAxis s3 = {0, 0};
  for (int t = 0; t < 50; t++) balance_axis_step(bal, s3, true, 7 * 16, 0, 0);
  CHECK(s3.corr < 0);
  for (int t = 0; t < 100; t++) balance_axis_step(bal, s3, false, 0, 0, 0);
  CHECK(s3.corr == 0 && s3.lp == 0);
}

// ---- 6. Gating and the closed loop through body_tick ---------------------
static Config cfg;
static Body b;
static uint16_t counts[16];
static TickIn in;

static void reset_body() {
  g_ee.reset();
  config_defaults(cfg.img);
  cfg.state = CFG_MIGRATED;
  params_load(cfg.img);
  body_init(b, &cfg, nullptr, true);
  in = TickIn();
  in.imu = IMU_FRESH;
  in.att.gz = 16384;
}
static void tick(int n = 1) {
  for (int i = 0; i < n; i++) {
    body_link_frame(b);  // the Pi is talking
    body_tick(b, in, counts);
  }
}
static bool at(uint8_t p) {
  for (int k = 0; k < DRIVEN_COUNT; k++)
    if (b.arb.out[k] != joint_clamp(joint_id((uint8_t)k), posture_angle(p, (uint8_t)k))) return false;
  return true;
}
static void tilt(double roll, double pitch) {
  in.att.roll = (int16_t)std::lround(roll * 16);
  in.att.pitch = (int16_t)std::lround(pitch * 16);
}

static void test_gating() {
  reset_body();
  tick();
  tilt(0, 10);  // nose down 10 deg (measured convention)
  // STAND without the flag: never balances.
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);
  tick(80);
  CHECK(!b.bal.active && at(POSTURE_STAND));
  // With the flag: active once settled; the pose moves.
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 1) == GOAL_OK);
  tick();
  CHECK(b.bal.active);
  tick(60);
  CHECK(b.bal.ax[BAL_PITCH].corr == -64 && !at(POSTURE_STAND));  // -0.5 * (10 - 2) deg: raise the nose
  // During a transition (STAND -> SIT, 25 ticks): off, corrections zeroed.
  // The mover starts from the corrected pose, so nothing jumps.
  int16_t before[DRIVEN_COUNT];
  std::memcpy(before, b.arb.out, sizeof before);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_SIT, 0, 1) == GOAL_OK);
  tick();
  for (int k = 0; k < DRIVEN_COUNT; k++) CHECK(std::abs(b.arb.out[k] - before[k]) <= 48);
  CHECK(!b.bal.active && b.bal.ax[BAL_PITCH].corr == 0);
  tick(23);
  CHECK(!b.bal.active);
  tick(2);
  CHECK(b.bal.active);  // settled in SIT; ref is -30 deg (nose up, legacy header verbatim)
  CHECK(posture_pitch(POSTURE_SIT) == -30 * 16);
  tilt(0, -30);
  tick(60);
  CHECK(b.bal.ax[BAL_PITCH].corr == 0 && at(POSTURE_SIT));
  // REST, CALIB, ZERO, LIMP: never.
  for (uint8_t p : {POSTURE_REST, POSTURE_CALIB, POSTURE_ZERO, POSTURE_LIMP}) {
    CHECK(body_goal_posture(b, b.epoch, p, 0, 1) == GOAL_OK);
    tilt(10, 10);
    tick(80);
    CHECKF(!b.bal.active && b.bal.ax[0].corr == 0 && b.bal.ax[1].corr == 0, "posture %d", p);
  }
  // SET_BALANCE enable turns it on without the posture flag; enable 0 and
  // no flag turns it off (decay).
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);
  tick(30);
  CHECK(!b.bal.active);
  const uint8_t on[5] = {1, 128, 30, 2, 15}, off[5] = {0, 128, 30, 2, 15}, bad[5] = {1, 128, 30, 2, 31};
  CHECK(body_goal_balance(b, on) == GOAL_OK);
  tick(60);
  CHECK(b.bal.active && b.bal.ax[BAL_ROLL].corr == -64);
  CHECK(body_goal_balance(b, bad) == GOAL_BAD_ARG && b.bal.max_deg == 15);
  CHECK(body_goal_balance(b, off) == GOAL_OK);
  tick(60);
  CHECK(!b.bal.active && b.bal.ax[BAL_ROLL].corr == 0 && at(POSTURE_STAND));
  // IMU_FAULT: off; the corrections decay (no jump), the pose returns.
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 1) == GOAL_OK);
  tick(60);
  CHECK(b.bal.active && b.bal.ax[BAL_PITCH].corr != 0);
  in.imu = IMU_NONE;
  int16_t prev[DRIVEN_COUNT];
  std::memcpy(prev, b.arb.out, sizeof prev);
  for (int t = 0; t < 60; t++) {
    tick();
    for (int k = 0; k < DRIVEN_COUNT; k++) CHECK(std::abs(b.arb.out[k] - prev[k]) <= 48);
    std::memcpy(prev, b.arb.out, sizeof prev);
  }
  CHECK(b.imu_fault && !b.bal.active && at(POSTURE_STAND));
  body_imu_recovered(b);
  in.imu = IMU_FRESH;
  tick(60);
  CHECK(b.bal.active);
  // REFLEX (link watchdog): off at once; the epoch bump clears the posture
  // flag, so after the link is back balance stays off until re-asserted.
  for (int t = 0; t < 200; t++) body_tick(b, in, counts);  // the Pi is silent
  CHECK(b.arb.reflex_active && !b.bal.active && b.bal.ax[BAL_PITCH].corr == 0 && b.balance_req == 0);
  tick();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 1) == GOAL_OK);
  tick(60);
  CHECK(b.bal.active);
}

// Closed loop through body_tick: the body tilt follows the joints. The
// plant is physical, independent of the mapping's sign constants: it reads
// from the arbiter's output how far the legs lower the nose (front knee
// folding, back knee extending) and raise the right side (left knees
// crouching further than the right), lagged, and reports them in the
// measured convention: nose DOWN = +pitch, RIGHT side up = +roll.
static double body_loop(bool pitch_axis, int meas_sign) {
  reset_body();
  tick();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 1) == GOAL_OK);
  tick(30);
  const double d = 10, tau = 0.15, dt = 0.02;
  double x = 0, prev_tilt = d, t_now = d;
  for (int t = 0; t < 400; t++) {
    t_now = d + x;
    double r = (t_now - prev_tilt) / dt;
    prev_tilt = t_now;
    if (pitch_axis) {
      tilt(0, meas_sign * t_now);
      in.att.rate_pitch = (int16_t)std::lround(meas_sign * r * 16);
    } else {
      tilt(meas_sign * t_now, 0);
      in.att.rate_roll = (int16_t)std::lround(meas_sign * r * 16);
    }
    tick();
    double effect;
    auto dk = [](int k) { return (b.arb.out[k] - posture_angle(POSTURE_STAND, (uint8_t)k)) / 16.0; };
    if (pitch_axis) {
      // Nose drop (= +pitch). Front knee LF (12) + = extend, back knee RB
      // (14) + = fold (SIT, nose up, has both larger than STAND), each
      // 0.785 deg per deg of correction.
      effect = -(dk(7) + dk(9)) / 2 / 0.785;
    } else {
      // Right raise (= +roll) = how much further the left legs crouched
      // than the right. Front knees fold toward negative (REST -45 vs STAND
      // 30): LF 12 / RF 13 move -1.57 per deg of crouch, 1.2x on one side.
      effect = (dk(8) - dk(7)) / (0.2 * 1.5708);
    }
    x += (effect - x) * dt / tau;
  }
  return t_now;
}

static void test_body_loop() {
  double p = body_loop(true, 1), r = body_loop(false, 1);
  double pf = body_loop(true, -1), rf = body_loop(false, -1);
  CHECKF(p < 8 && r < 8, "through the joints: pitch %.2f roll %.2f (d = 10)", p, r);
  CHECKF(std::fabs(pf) > 12 && std::fabs(rf) > 12, "flipped sensor: pitch %.2f roll %.2f", pf, rf);
  std::printf("balance: through body_tick d=10: pitch %.2f roll %.2f | sensor flipped: pitch %.2f roll %.2f deg\n", p,
              r, pf, rf);
}

int main() {
  test_legacy_mapping();
  test_directions();
  test_toy_plant();
  test_shaping();
  test_gating();
  test_body_loop();
  return report("balance");
}
