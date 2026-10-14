// The gait generator, STANCE and the body pose (core/gait.*, doc §3d), as
// wired into body_tick, the arbiter and the link (Appendix H "gait"):
// swing order and pairings per pattern, zero foot velocity at lift-off and
// touchdown, the stride ramp in and out, the stop sequence (every foot down
// before the hand-over, GAIT_STOPPED once), turning as a differential
// stride, period changes without a phase jump, head bob and tail wag,
// refusals, pre-emption by a trick, a reflex and the watchdog, STANCE with
// the pose terms and the foot nudge, the geometric balance closed through a
// physical plant, TELEMETRY through the fake link, the support state (feet
// down), the planned sway (its sign read off the stance feet), the
// stance-aware balance (the plan as its reference, the hold on two feet)
// and the early-touchdown reflex against a plant that falls on a diagonal,
// the sway's lead (the lean at every lift-off), forward placement (its
// model, sign and clamp; its shifts in normal walking; fore-aft shoves in a
// capture plant), and the per-tick cost.
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <vector>
#include "../src/core/body.h"
#include "../src/core/fx.h"
#include "../src/core/ik.h"
#include "../src/core/link.h"
#include "../src/core/params.h"
#include "../src/core/posture.h"
#include "../src/proto/crc16.h"
#include "check.h"
#include "fake_eeprom.h"
#include "fake_link_hal.h"

static Config cfg;
static Body& b = g_body;
static uint16_t counts[16];
static TickIn in;
static std::vector<Event> evlog;  // every event since reset()
static const double D2R = M_PI / 180.0;

static void reset() {
  config_defaults(cfg.img);
  cfg.state = CFG_MIGRATED;
  params_load(cfg.img);
  body_init(b, &cfg, nullptr, true);
  in = TickIn();
  in.imu = IMU_FRESH;
  in.att.gz = 16384;
  evlog.clear();
}

static void tick(int n = 1) {
  for (int i = 0; i < n; i++) {
    body_tick(b, in, counts);
    const Event* e;
    while ((e = events_peek(b.ev)) != nullptr) {
      evlog.push_back(*e);
      events_pop(b.ev);
    }
  }
}

static int count_events(uint8_t code) {
  int n = 0;
  for (const Event& e : evlog) n += e.code == code;
  return n;
}

// SET_GAIT payload. 0 for period, duty, step, body = the pattern's default.
// step -1 = §3d's default for the pattern (15 / 12 / 18 mm; 0 is a shuffle).
static std::vector<uint8_t> gp(uint8_t run, uint8_t pat, uint16_t per = 0, uint8_t duty = 0, int8_t stride = 30,
                               int8_t turn = 0, int step = -1, uint8_t body = 0, uint8_t flags = 7) {
  static const uint8_t STEP[3] = {15, 12, 18};
  if (step < 0) step = pat < 3 ? STEP[pat] : 0;
  return {b.epoch, run, pat, (uint8_t)per, (uint8_t)(per >> 8), duty, (uint8_t)stride, (uint8_t)turn,
          (uint8_t)step, body, flags};
}
static GoalResult gait(const std::vector<uint8_t>& p) { return body_goal_gait(b, p.data()); }
static GoalResult pose(int16_t pitch_a16, int16_t roll_a16, int8_t dh, int8_t xs) {
  uint8_t p[7] = {b.epoch, (uint8_t)pitch_a16, (uint8_t)(pitch_a16 >> 8), (uint8_t)roll_a16, (uint8_t)(roll_a16 >> 8),
                  (uint8_t)dh, (uint8_t)xs};
  return body_goal_pose(b, p);
}

// The model in double: OpenCat angles (a16) -> foot (mm), leg frame.
static void fk(int leg, int16_t sh, int16_t kn, double* x, double* z) {
  double l1 = PARAM(LEG_L1_MM), l2 = PARAM(LEG_L2_MM);
  double t1 = PARAM(LEG_SH_ZERO_DEG) + IK_SGN_SH * sh / 16.0;
  double t2 = ik_sigma((uint8_t)leg) * PARAM(LEG_KN_ZERO_DEG) + IK_SGN_KN * kn / 16.0;
  *x = l1 * std::sin(t1 * D2R) + l2 * std::sin((t1 + t2) * D2R);
  *z = l1 * std::cos(t1 * D2R) + l2 * std::cos((t1 + t2) * D2R);
}
// The generator's foot this tick.
static void foot(int leg, double* x, double* z) { fk(leg, b.gait.leg[leg], b.gait.leg[4 + leg], x, z); }
// The foot the joints actually command (after crossfade, limits and slew).
static void out_foot(int leg, double* x, double* z) { fk(leg, b.arb.out[3 + leg], b.arb.out[7 + leg], x, z); }
static double ground() { return b.gait.h / 4.0; }  // z of a foot on the ground (no pose, level)
static bool lifted(int leg, double thr = 0.5) {
  double x, z;
  foot(leg, &x, &z);
  return z < ground() - thr;
}

// Max |output step| over the legs this tick vs prev, in degrees.
static double max_step(const int16_t* prev) {
  double m = 0;
  for (int k = 3; k < DRIVEN_COUNT; k++) m = std::fmax(m, std::fabs((b.arb.out[k] - prev[k]) / 16.0));
  return m;
}

// STAND, then a gait goal; ticks until it runs. Returns the ticks it took.
static int start(const std::vector<uint8_t>& p) {
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 1, 0) == GOAL_OK);
  tick(30);
  std::vector<uint8_t> q = p;
  q[0] = b.epoch;  // gp() ran before reset()
  GoalResult r = gait(q);
  CHECKF(r == GOAL_OK, "start: %d", r);
  int t = 0;
  while (!body_gait_running(b) && t < 200) {
    tick();
    t++;
  }
  CHECK(body_gait_running(b));
  return t;
}

static int period_ticks() { return (int)((65536 + b.gait.dphi - 1) / b.gait.dphi); }

static const char* LEG_NAME[4] = {"LF", "RF", "RB", "LB"};

// ---- Swing order and pairings ----------------------------------------------
static void test_swing_order() {
  for (uint8_t pat = 0; pat < 3; pat++) {
    start(gp(1, pat));
    int T = period_ticks();
    tick(3 * T);  // the ramp is long over
    // Exactly two cycles of phase, not 2 * T ticks: dphi rounds, so a tick
    // window can straddle a third lift-off (seen at period 1000: 10 instead of 8).
    uint16_t prev_phi = b.gait.phi;
    while (true) { tick(); if (b.gait.phi < prev_phi) break; prev_phi = b.gait.phi; }  // just wrapped
    std::vector<std::pair<int, int>> lifts;  // (tick, leg)
    bool was[4];
    for (int l = 0; l < 4; l++) was[l] = lifted(l);
    int max_up = 0;
    int wraps = 0;
    prev_phi = b.gait.phi;
    for (int t = 0; wraps < 2; t++) {
      tick();
      if (b.gait.phi < prev_phi) wraps++;
      prev_phi = b.gait.phi;
      int up = 0;
      for (int l = 0; l < 4; l++) {
        bool now = lifted(l);
        up += now;
        if (now && !was[l]) lifts.push_back({t, l});
        was[l] = now;
      }
      if (up > max_up) max_up = up;
    }
    CHECKF(lifts.size() == 8, "pattern %d: %zu lift-offs in two cycles", pat, lifts.size());
    if (lifts.size() != 8) continue;
    std::printf("gait: pattern %d lift-offs:", pat);
    for (auto& e : lifts) std::printf(" %s@%d", LEG_NAME[e.second], e.first);
    std::printf("\n");
    if (pat == KILO_GAIT_PATTERN_TROT) {
      // Diagonal pairs lift on the same tick, half a period apart.
      for (int i = 0; i < 8; i += 2) {
        CHECK(lifts[i].first == lifts[i + 1].first);
        int a = lifts[i].second, c = lifts[i + 1].second;
        CHECK((a == LEG_LF && c == LEG_RB) || (a == LEG_RB && c == LEG_LF) || (a == LEG_RF && c == LEG_LB) ||
              (a == LEG_LB && c == LEG_RF));
        if (i) CHECK(std::abs(lifts[i].first - lifts[i - 2].first - T / 2) <= 1);
      }
      CHECK(max_up == 2);
    } else {
      // WALK and CRAWL: the lateral sequence LB -> LF -> RB -> RF, one foot
      // in the air at a time, a quarter period apart.
      static const int ORDER[4] = {LEG_LB, LEG_LF, LEG_RB, LEG_RF};
      int i0 = 0;
      while (lifts[i0].second != LEG_LB) i0++;
      for (int i = 0; i < 4; i++) CHECK(lifts[(i0 + i) % 8].second == ORDER[i]);
      for (int i = 1; i < 8; i++) CHECK(std::abs(lifts[i].first - lifts[i - 1].first - T / 4) <= 1);
      CHECK(max_up == 1);
    }
  }
}

// ---- Foot trajectory: zero velocity at lift-off and touchdown, constant
// stance speed, step height -------------------------------------------------
static void test_trajectory() {
  // A slow, long, high step makes the shape easy to see: WALK, 2 s, duty
  // 1/2 (50 swing ticks), stride 60 mm, step 40 mm.
  start(gp(1, KILO_GAIT_PATTERN_WALK, 2000, 128, 60, 0, 40));
  int T = period_ticks();
  tick(2 * T);
  std::vector<double> xs, zs;
  for (int t = 0; t < 2 * T; t++) {
    tick();
    double x, z;
    foot(LEG_LF, &x, &z);
    xs.push_back(x);
    zs.push_back(z);
  }
  // The first complete swing: the run of ticks with the foot above ground.
  size_t i0 = 1;
  while (i0 < zs.size() && !(zs[i0] < ground() - 0.15 && zs[i0 - 1] >= ground() - 0.15)) i0++;
  size_t i1 = i0;
  while (i1 + 1 < zs.size() && zs[i1 + 1] < ground() - 0.15) i1++;
  CHECKF(i1 - i0 + 1 >= 45 && i1 - i0 + 1 <= 50, "swing lasted %zu ticks", i1 - i0 + 1);
  double peak = 0, lift = 0, top = 0;
  for (size_t i = i0 + 1; i <= i1; i++) {
    double d = std::hypot(xs[i] - xs[i - 1], zs[i] - zs[i - 1]);
    peak = std::fmax(peak, d);
    top = std::fmax(top, ground() - zs[i]);
  }
  double first = std::hypot(xs[i0 + 1] - xs[i0], zs[i0 + 1] - zs[i0]);
  double last = std::hypot(xs[i1] - xs[i1 - 1], zs[i1] - zs[i1 - 1]);
  lift = std::fmax(first, last);
  std::printf("gait: swing step at lift-off %.2f, touchdown %.2f, peak %.2f mm/tick; height %.1f mm\n", first, last,
              peak, top);
  // A constant-speed swing would give first/last = peak; §3d's h sin(pi s)
  // lift alone would give 2.5 mm at lift-off.
  CHECK(lift <= 0.3 * peak);
  CHECK(std::fabs(top - 40) < 0.5);
  // The swing carries the foot from the back of the stride to the front.
  CHECK(std::fabs((xs[i1] - xs[i0]) - 60) < 2);
  // Stance: constant ground speed (Appendix H: +-2%), the stride backward
  // over beta T: 60 mm over 50 ticks. Mean speed in each half of a stance.
  size_t s0 = i1 + 2, s1 = s0;
  while (s1 + 1 < zs.size() && zs[s1 + 1] >= ground() - 0.15) s1++;
  // (The ticks next to lift-off and touchdown may be swing ticks too close
  // to the ground to tell apart: leave 3 out at each end.)
  if (s1 - s0 > 40) {
    s0 += 3;
    s1 -= 3;
    size_t mid = (s0 + s1) / 2;
    double v1 = (xs[mid] - xs[s0]) / (double)(mid - s0), v2 = (xs[s1] - xs[mid]) / (double)(s1 - mid);
    CHECKF(std::fabs(v1 / v2 - 1) < 0.02 && std::fabs(v1 + 1.2) < 0.03, "stance speeds %.3f %.3f mm/tick", v1, v2);
  } else {
    CHECKF(false, "stance of %zu ticks", s1 - s0);
  }
}

// ---- Start ramp, stop ramp, landing, hand-over --------------------------------
static void test_start_stop() {
  start(gp(1, KILO_GAIT_PATTERN_TROT, 0, 0, 35));
  // The first gait tick: phi 0, no stride yet, the legs already GAIT's
  // (with STANCE's angles).
  CHECK(b.gait.phi == 0 && b.gait.amp == 0 && b.arb.owner[3] == OWN_GAIT);
  uint16_t dphi = b.gait.dphi;
  int T = period_ticks();
  // Ramp in: amp grows by dphi per tick, full after one period.
  int16_t prev[DRIVEN_COUNT];
  double worst = 0;
  for (int t = 1; t <= T + 2; t++) {
    memcpy(prev, b.arb.out, sizeof prev);
    tick();
    uint32_t want = (uint32_t)t * dphi;
    CHECK(b.gait.amp == (want > 65535 ? 65535 : want));
    worst = std::fmax(worst, max_step(prev));
  }
  CHECK(b.arb.owner[3] == OWN_GAIT && b.arb.owner[10] == OWN_GAIT && b.arb.owner[0] == OWN_IDLE);
  tick(3 * T);
  // Stop.
  evlog.clear();
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  int t = 0, landed = -1;
  uint16_t amp_prev = b.gait.amp;
  double xr = 0;
  while (body_gait_running(b) && t < 4 * T) {
    memcpy(prev, b.arb.out, sizeof prev);
    // Before this tick: the gait's feet of the last tick.
    bool all_down = true;
    for (int l = 0; l < 4; l++) all_down = all_down && !lifted(l, 0.2);
    tick();
    t++;
    worst = std::fmax(worst, max_step(prev));
    if (b.gait.state == GS_STOP || (b.gait.state == GS_LAND && landed < 0)) {
      CHECK(b.gait.amp == (amp_prev > dphi ? amp_prev - dphi : 0));
      if (b.gait.state == GS_LAND) landed = t;
    }
    amp_prev = b.gait.amp;
    if (!body_gait_running(b)) {
      // Handed over: every foot was down, at the neutral x, on the last
      // gait tick.
      CHECK(all_down);
      for (int l = 0; l < 4; l++) {
        double x, z;
        out_foot(l, &x, &z);
        xr = std::fmax(xr, std::fabs(x));
      }
    }
  }
  std::printf("gait: stop: ramp %d ticks, landed after %d (period %d); max leg step %.2f deg/tick\n", landed, t, T,
              worst);
  CHECK(landed >= T - 1 && landed <= T + 1);
  CHECK(t <= landed + T + 1);
  CHECK(xr < 0.5);
  CHECK(count_events(EV_GAIT_STOPPED) == 1);
  CHECK(arbiter_idle_posture(b.arb) == POSTURE_STANCE && b.arb.owner[3] == OWN_IDLE);
  CHECK(body_mode(b) == KILO_MODE_IDLE);
  // IDLE STANCE holds the same pose: nothing moves after the hand-over.
  memcpy(prev, b.arb.out, sizeof prev);
  tick(20);
  CHECK(max_step(prev) < 0.2);
  CHECK(count_events(EV_GAIT_STOPPED) == 1);  // once
  // Refreshing the stop changes nothing; refreshing a run restarts at once
  // (IDLE is already in STANCE).
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  tick(5);
  CHECK(!body_gait_running(b));
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_TROT)) == GOAL_OK);
  tick();
  CHECK(body_gait_running(b) && b.gait.phi == 0 && b.gait.amp == 0);
}

// ---- Resume during the ramp-down, and a pattern change ---------------------
static void test_pattern_change() {
  start(gp(1, KILO_GAIT_PATTERN_TROT));
  int T = period_ticks();
  tick(2 * T);
  // A stop cancelled half way ramps back up without landing.
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  tick(T / 2);
  uint16_t a = b.gait.amp;
  CHECK(b.gait.state == GS_STOP && a < 65535);
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_TROT)) == GOAL_OK);
  tick();
  CHECK(b.gait.state == GS_RUN && b.gait.amp > a);
  // TROT -> WALK: stop (ramp, land) then start WALK at phi = 0, the legs
  // staying with GAIT throughout and no GAIT_STOPPED.
  tick(2 * T);
  evlog.clear();
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_WALK)) == GOAL_OK);
  int16_t prev[DRIVEN_COUNT];
  double worst = 0;
  int t = 0;
  bool restarted = false;
  while (t < 6 * T) {
    memcpy(prev, b.arb.out, sizeof prev);
    uint8_t st = b.gait.state;
    tick();
    t++;
    worst = std::fmax(worst, max_step(prev));
    CHECK(body_gait_running(b) && b.arb.owner[3] == OWN_GAIT);
    if (st == GS_LAND && b.gait.state == GS_RUN) {
      restarted = true;
      break;
    }
  }
  CHECK(restarted && b.gait.pat == KILO_GAIT_PATTERN_WALK && b.gait.phi == 0 && b.gait.amp == 0);
  CHECK(count_events(EV_GAIT_STOPPED) == 0);
  // (The default gaits themselves step up to ~9 deg/tick: see
  // test_defaults_and_cost. Nothing here may reach the safety slew.)
  CHECKF(worst < PARAM(SLEW_MAX_A16) / 16.0, "pattern change: max leg step %.2f deg/tick", worst);
}

// ---- Turning: a differential stride ----------------------------------------
static void stance_sweep(double out[4]) {
  // Median x speed of each foot while on the ground, over one period (the
  // median: swing ticks next to the ground count as "on the ground" too).
  int T = period_ticks();
  double px[4], pz[4];
  std::vector<double> d[4];
  for (int l = 0; l < 4; l++) foot(l, &px[l], &pz[l]);
  for (int t = 0; t < T; t++) {
    tick();
    for (int l = 0; l < 4; l++) {
      double x, z;
      foot(l, &x, &z);
      if (z >= ground() - 0.01 && pz[l] >= ground() - 0.01) d[l].push_back(x - px[l]);
      px[l] = x;
      pz[l] = z;
    }
  }
  for (int l = 0; l < 4; l++) {
    std::sort(d[l].begin(), d[l].end());
    out[l] = d[l].empty() ? 0 : d[l][d[l].size() / 2];
  }
}

static void test_turn() {
  // In place, + = left: the left feet sweep forward on the ground (the left
  // side backs up), the right feet backward (it advances).
  // Period 500 ms and duty 140 pinned: these checks are about geometry, and
  // the foot x is whole mm per tick, so at the slower pattern defaults
  // (1000 ms, duty 179 since 2026-10-12) the per-tick sweeps are ~0.5 mm and
  // rounding alone breaks the ratios.
  start(gp(1, KILO_GAIT_PATTERN_TROT, 500, 140, 0, 20));
  tick(3 * period_ticks());
  double v[4];
  stance_sweep(v);
  std::printf("gait: turn in place +20: stance x speed LF %.2f RF %.2f RB %.2f LB %.2f mm/tick\n", v[0], v[1], v[2],
              v[3]);
  CHECK(v[LEG_LF] > 0.5 && v[LEG_LB] > 0.5 && v[LEG_RF] < -0.5 && v[LEG_RB] < -0.5);
  CHECK(std::fabs(v[LEG_LF] + v[LEG_RF]) < 0.08 * v[LEG_LF] && std::fabs(v[LEG_LB] + v[LEG_RB]) < 0.08 * v[LEG_LB]);
  // Walking 30 and turning right 10: left stride 40, right 20.
  start(gp(1, KILO_GAIT_PATTERN_TROT, 500, 140, 30, -10));
  tick(3 * period_ticks());
  stance_sweep(v);
  CHECK(v[LEG_LF] < 0 && v[LEG_RF] < 0);
  CHECKF(std::fabs(v[LEG_LF] / v[LEG_RF] - 2) < 0.1 && std::fabs(v[LEG_LB] / v[LEG_RB] - 2) < 0.1,
         "left/right stance speeds %.2f / %.2f", v[LEG_LF], v[LEG_RF]);
  // The goal slews: a new turn takes effect at 0.5 mm per tick.
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_TROT, 500, 140, 30, 10)) == GOAL_OK);
  tick();
  CHECK(b.gait.turn == -40 + 2);
  tick(40);
  CHECK(b.gait.turn == 40);
}

// ---- Period change: the phase never jumps ----------------------------------
static void test_period_change() {
  start(gp(1, KILO_GAIT_PATTERN_WALK, 800));
  tick(2 * period_ticks() + 7);
  uint16_t phi0 = b.gait.phi, d0 = b.gait.dphi;
  CHECK(d0 == 1638);  // 65536 * 20 / 800
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_WALK, 1200)) == GOAL_OK);
  CHECK(b.gait.phi == phi0);  // the goal alone does not touch the phase
  int16_t prev[DRIVEN_COUNT];
  memcpy(prev, b.arb.out, sizeof prev);
  tick();
  CHECK(b.gait.dphi == 1092 && (uint16_t)(b.gait.phi - phi0) == 1092);
  CHECK(max_step(prev) < PARAM(SLEW_MAX_A16) / 16.0);
  // And back faster, mid-swing for some leg.
  tick(13);
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_WALK, 600)) == GOAL_OK);
  phi0 = b.gait.phi;
  memcpy(prev, b.arb.out, sizeof prev);
  tick();
  CHECK((uint16_t)(b.gait.phi - phi0) == 2184);
  CHECK(max_step(prev) < PARAM(SLEW_MAX_A16) / 16.0);
}

// ---- Head bob and tail wag --------------------------------------------------
static void test_bob_wag() {
  for (int flags = 0; flags < 4; flags++) {
    start(gp(1, KILO_GAIT_PATTERN_WALK, 2000, 0, 30, 0, -1, 0, (uint8_t)(flags | KILO_GAIT_FLAGS_BALANCE)));
    int T = period_ticks();
    tick(2 * T);
    double tilt_max = -99, tilt_min = 99, tail_max = -99, tail_min = 99;
    int tilt_peaks = 0;
    double p2 = 0, p1 = 0;
    for (int t = 0; t < T; t++) {
      tick();
      double tilt = b.arb.out[1] / 16.0, tail = b.arb.out[2] / 16.0;
      tilt_max = std::fmax(tilt_max, tilt);
      tilt_min = std::fmin(tilt_min, tilt);
      tail_max = std::fmax(tail_max, tail);
      tail_min = std::fmin(tail_min, tail);
      if (t >= 2 && p1 > p2 && p1 >= tilt && p1 > 1) tilt_peaks++;
      p2 = p1;
      p1 = tilt;
    }
    bool bob = flags & KILO_GAIT_FLAGS_BOB, wag = flags & KILO_GAIT_FLAGS_WAG;
    CHECKF(bob ? std::fabs(tilt_max - 4) < 0.15 && std::fabs(tilt_min + 4) < 0.15 : tilt_max == 0 && tilt_min == 0,
           "flags %d: tilt %.2f..%.2f", flags, tilt_min, tilt_max);
    CHECKF(wag ? std::fabs(tail_max - 15) < 0.15 && std::fabs(tail_min + 15) < 0.15 : tail_max == 0 && tail_min == 0,
           "flags %d: tail %.2f..%.2f", flags, tail_min, tail_max);
    if (bob) CHECKF(tilt_peaks == 2, "bob peaks per cycle: %d", tilt_peaks);
    CHECK(b.arb.out[0] == 0);  // pan untouched
  }
  // A SET_HEAD hold wins over the bob and wag.
  start(gp(1, KILO_GAIT_PATTERN_WALK));
  tick(100);
  uint8_t h[10] = {b.epoch, 1, 0, 0, 0x50, 0, 0x50, 0, 0, 0};  // tilt 5 deg, tail 5 deg
  CHECK(body_goal_head(b, h) == GOAL_OK);
  tick(30);
  CHECK(b.arb.out[1] == 80 && b.arb.out[2] == 80 && b.arb.owner[1] == OWN_PI_HOLD);
  // Released; at the hand-over the bob and wag have ramped out with the
  // stride.
  h[1] = 0;
  CHECK(body_goal_head(b, h) == GOAL_OK);
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  int t = 0;
  while (body_gait_running(b) && t++ < 200) tick();
  tick(PARAM(XFADE_TICKS) + 5);
  CHECK(b.arb.out[1] == 0 && b.arb.out[2] == 0);
}

// ---- Refusals and the epoch ---------------------------------------------------
static void test_refusals() {
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 1, 0) == GOAL_OK);
  tick(5);
  // Field ranges (0 = default where allowed).
  CHECK(gait(gp(2, 0)) == GOAL_BAD_ARG);
  CHECK(gait(gp(1, 3)) == GOAL_BAD_ARG);
  CHECK(gait(gp(1, 0, 299)) == GOAL_BAD_ARG && gait(gp(1, 0, 2001)) == GOAL_BAD_ARG);
  CHECK(gait(gp(1, 0, 0, 127)) == GOAL_BAD_ARG && gait(gp(1, 0, 0, 231)) == GOAL_BAD_ARG);
  CHECK(gait(gp(1, 0, 0, 0, 61)) == GOAL_BAD_ARG && gait(gp(1, 0, 0, 0, -61)) == GOAL_BAD_ARG);
  CHECK(gait(gp(1, 0, 0, 0, 0, 41)) == GOAL_BAD_ARG && gait(gp(1, 0, 0, 0, 0, -41)) == GOAL_BAD_ARG);
  CHECK(gait(gp(1, 0, 0, 0, 0, 0, 41)) == GOAL_BAD_ARG);
  CHECK(gait(gp(1, 0, 0, 0, 0, 0, -1, 255)) == GOAL_OK);  // body height: never refused (the IK clamps it)
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  CHECK(gait(gp(1, 0, 0, 0, 0, 0, -1, 0, 64)) == GOAL_BAD_ARG);  // bit 6: unknown (bit 5 is PLACE)
  CHECK(!b.gait.run);
  // STALE; a stop is never stale.
  std::vector<uint8_t> p = gp(1, 1);
  p[0] = (uint8_t)(b.epoch + 1);
  CHECK(gait(p) == GOAL_STALE);
  p[1] = 0;
  CHECK(gait(p) == GOAL_OK);
  // UNCALIBRATED (as STAND).
  cfg.state = CFG_DEFAULTED;
  CHECK(gait(gp(1, 1)) == GOAL_UNCALIBRATED);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, 0) == GOAL_UNCALIBRATED);
  cfg.state = CFG_MIGRATED;
  // SET_BODY_POSE: STALE.
  uint8_t pp[7] = {(uint8_t)(b.epoch + 1), 0, 0, 0, 0, 0, 0};
  CHECK(body_goal_pose(b, pp) == GOAL_STALE);
  // LOW_BATT for both (and a stop still passes).
  b.batt.level = BATT_LOW;
  CHECK(gait(gp(1, 1)) == GOAL_LOW_BATT && pose(0, 0, 0, 0) == GOAL_LOW_BATT && gait(gp(0, 1)) == GOAL_OK);
  b.batt.level = BATT_OK;
  // FALLEN for both.
  b.fall.state = FS_FALLEN;
  CHECK(gait(gp(1, 1)) == GOAL_FALLEN && pose(0, 0, 0, 0) == GOAL_FALLEN);
  b.fall.state = FS_UPRIGHT;
  // Accepted goals latch; the pose is clamped to +-30 deg.
  CHECK(pose(16 * 45, -16 * 45, 5, -5) == GOAL_OK);
  CHECK(b.gait.pose_g[0] == 480 && b.gait.pose_g[1] == -480 && b.gait.dh_g == 5 && b.gait.xs_g == -5);
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_CRAWL)) == GOAL_OK && b.gait.run);
  CHECK(body_mode(b) == KILO_MODE_GAIT);
  // An epoch bump clears both.
  body_epoch_bump(b);
  CHECK(!b.gait.run && b.gait.pose_g[0] == 0 && b.gait.pose_g[1] == 0 && b.gait.dh_g == 0 && b.gait.xs_g == 0);
}

// ---- SET_GAIT from another posture: IDLE moves to STANCE first ------------------
static void test_pending() {
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_SIT, 1, 0) == GOAL_OK);
  tick(30);
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_TROT)) == GOAL_OK);
  tick();
  CHECK(arbiter_idle_posture(b.arb) == POSTURE_STANCE && arbiter_idle_moving(b.arb) && !body_gait_running(b));
  CHECK(body_mode(b) == KILO_MODE_GAIT);
  int t = 1;
  while (!body_gait_running(b) && t < 100) {
    tick();
    t++;
  }
  CHECKF(t == PARAM(POSTURE_TICKS) + 1, "gait started %d ticks after the goal", t);
  // From LIMP too (the target is commanded and the slew paces it).
  reset();
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_WALK)) == GOAL_OK);
  tick(40);
  CHECK(body_gait_running(b));
  // SET_POSTURE while walking only changes IDLE underneath (the head
  // follows it); the stop hands the legs to STANCE, and the (refreshed)
  // posture goal moves on from there.
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, 0) == GOAL_OK);
  tick();
  CHECK(b.gait.state == GS_RUN);
  int16_t h0 = b.gait.h;
  CHECK(body_goal_posture(b, b.epoch, POSTURE_SIT, 0, 0) == GOAL_OK);
  tick(40);
  CHECK(b.gait.state == GS_RUN && b.gait.run && b.arb.owner[3] == OWN_GAIT && arbiter_idle_posture(b.arb) == POSTURE_SIT);
  CHECK(b.arb.out[0] == 10 * 16);  // sit's pan (no bob or wag on pan)
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, 0) == GOAL_OK);  // under the gait: keeps its height
  CHECK(b.gait.h == h0);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_SIT, 0, 0) == GOAL_OK);
  evlog.clear();
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  t = 0;
  while (body_gait_running(b) && t++ < 200) tick();
  CHECK(count_events(EV_GAIT_STOPPED) == 1 && arbiter_idle_posture(b.arb) == POSTURE_STANCE);
  tick(5);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_SIT, 0, 0) == GOAL_OK);  // the 5 Hz refresh
  tick(PARAM(POSTURE_TICKS) + 10);
  bool sit = true;
  for (int k = 0; k < DRIVEN_COUNT; k++)
    sit = sit && b.arb.out[k] == joint_clamp(joint_id((uint8_t)k), posture_angle(POSTURE_SIT, (uint8_t)k));
  CHECK(sit);
  // A stop before the gait started (IDLE still moving to STANCE) ends it at
  // once, with GAIT_STOPPED.
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_WALK)) == GOAL_OK);
  tick(3);
  CHECK(!body_gait_running(b) && body_mode(b) == KILO_MODE_GAIT);
  evlog.clear();
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  tick();
  CHECK(count_events(EV_GAIT_STOPPED) == 1 && body_mode(b) == KILO_MODE_IDLE && !b.gait.run);
  // LIMP stops it at once.
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_WALK)) == GOAL_OK);
  tick(80);
  CHECK(body_gait_running(b));
  evlog.clear();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_LIMP, 0, 0) == GOAL_OK);
  CHECK(!body_gait_running(b) && !b.gait.run && count_events(EV_GAIT_STOPPED) == 0);  // queued in b.ev
  tick();
  CHECK(count_events(EV_GAIT_STOPPED) == 1 && b.arb.out[3] == JOINT_OFF);
}

// ---- Pre-emption: a trick, a reflex, the watchdog ------------------------------
static uint8_t upload_trick(uint16_t mask) {
  kilo_trick_begin_t h;
  memset(&h, 0, sizeof h);
  h.trick_id = 9;
  h.n_frames = 1;
  h.joint_mask = mask;
  h.end_posture = POSTURE_STAND;
  uint8_t s = kf_begin(b.kf, (const uint8_t*)&h);
  uint8_t fr[2 + 14] = {0, 1, 10, 10, KF_COSINE};
  for (int k = 0; k < 11; k++) fr[5 + k] = (uint8_t)KF_UNCHANGED;
  fr[5 + 3] = 50;  // LF shoulder
  if (!s) s = kf_frames(b.kf, fr);
  uint16_t crc = KILO_CRC16_INIT;
  for (int i = 2; i < 16; i++) crc = kilo_crc16_update(crc, fr[i]);
  if (!s) s = kf_commit(b.kf, 9, crc);
  return s;
}

static void test_preempt() {
  // A trick that owns a leg takes the legs at once and pauses the gait:
  // the phase runs on, the stride and the lifting start again from nothing,
  // so the crossfade back after the trick lands on STANCE's pose and the
  // stride ramps in again. No GAIT_STOPPED.
  start(gp(1, KILO_GAIT_PATTERN_TROT));
  tick(60);
  CHECK(upload_trick(0x0100) == KILO_ACK_STATUS_OK);  // LF shoulder only
  uint8_t pl[4] = {b.epoch, 9, 64, 0};
  CHECK(body_trick_play(b, pl) == GOAL_OK);
  evlog.clear();
  uint16_t phi0 = b.gait.phi;
  tick();
  CHECK(body_gait_running(b) && b.gait.run && b.gait.amp == 0 && b.gait.armed == 0);
  CHECK((uint16_t)(b.gait.phi - phi0) == b.gait.dphi);
  CHECK(b.arb.owner[3] == OWN_TRICK && b.arb.owner[4] == OWN_GAIT);  // the other legs keep walking (in place)
  CHECK(body_mode(b) == KILO_MODE_TRICK);
  int t = 0;
  while (b.kf.run && t++ < 100) tick();
  CHECK(count_events(EV_TRICK_DONE) == 1);
  // The gait's feet as the trick lets go: all down, under the shoulders
  // (one tick of ramp: within ~1.5 mm).
  for (int l = 0; l < 4; l++) {
    double x, z;
    foot(l, &x, &z);
    CHECKF(std::fabs(x) < 1.5 && std::fabs(z - ground()) < 0.2, "leg %d at the trick's end: (%.2f, %.2f)", l, x, z);
  }
  int16_t prev[DRIVEN_COUNT];
  double worst = 0;
  for (int i = 0; i < 30; i++) {
    memcpy(prev, b.arb.out, sizeof prev);
    tick();
    worst = std::fmax(worst, max_step(prev));
  }
  CHECK(body_gait_running(b) && b.arb.owner[3] == OWN_GAIT && b.gait.amp > 0 && count_events(EV_GAIT_STOPPED) == 0);
  CHECKF(worst < PARAM(SLEW_MAX_A16) / 16.0, "after the trick: max leg step %.2f deg/tick", worst);
  // A head-only trick leaves the gait alone.
  tick(30);
  CHECK(upload_trick(0x0001) == KILO_ACK_STATUS_OK);
  pl[0] = b.epoch;
  CHECK(body_trick_play(b, pl) == GOAL_OK);
  tick(5);
  CHECK(body_gait_running(b) && b.arb.owner[0] == OWN_TRICK && b.arb.owner[3] == OWN_GAIT);
  tick(40);

  // Battery LOW while walking: the reflex takes every joint at once.
  start(gp(1, KILO_GAIT_PATTERN_WALK));
  tick(60);
  evlog.clear();
  in.batt_valid = 1;
  in.batt_counts = 640;  // ~6.46 V: below LOW
  t = 0;
  while (b.batt.level < BATT_LOW && t++ < 2000) tick();
  CHECK(b.batt.level == BATT_LOW && b.arb.reflex_active);
  CHECK(!body_gait_running(b) && !b.gait.run && count_events(EV_GAIT_STOPPED) == 1);
  CHECK(b.arb.owner[3] == OWN_REFLEX);
  in.batt_valid = 0;

  // Watchdog: the stage-1 bump stops the gait through the ramp and the
  // landing; REST waits for it.
  start(gp(1, KILO_GAIT_PATTERN_CRAWL));  // 1.2 s period: the slowest default
  int T = period_ticks();
  tick(2 * T);
  body_link_frame(b);
  evlog.clear();
  uint8_t e0 = b.epoch;
  t = 0;
  int bump = -1, stopped = -1, rest = -1;
  while (t < 400) {
    tick();
    t++;
    if (bump < 0 && b.epoch != e0) bump = t;
    if (stopped < 0 && count_events(EV_GAIT_STOPPED)) stopped = t;
    if (rest < 0 && b.arb.reflex_active) rest = t;
    if (rest > 0) break;
  }
  std::printf("gait: watchdog: bump at %d, GAIT_STOPPED at %d, REST at %d ticks\n", bump, stopped, rest);
  CHECK(bump == 25);
  CHECK(stopped > bump && stopped <= bump + 2 * T + 2);
  CHECK(rest >= 75 && rest >= stopped && rest <= stopped + 1);
  CHECK(count_events(EV_GAIT_STOPPED) == 1);
  // The old goal is stale now.
  std::vector<uint8_t> p = gp(1, 1);
  p[0] = e0;
  CHECK(gait(p) == GOAL_STALE);
}

// ---- STANCE, the pose terms and the foot nudge -------------------------------------
static void test_stance_pose() {
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 1, 0) == GOAL_OK);
  tick(20);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, 0) == GOAL_OK);
  tick(40);
  int16_t want_sh[4], want_kn[4];
  int16_t h = (int16_t)(ik_reach_q2() * KILO_GAIT_STANCE_H_256 / 256);
  for (int l = 0; l < 4; l++) ik_leg((uint8_t)l, 0, h, &want_sh[l], &want_kn[l]);
  bool ok = true;
  for (int l = 0; l < 4; l++) ok = ok && b.arb.out[3 + l] == want_sh[l] && b.arb.out[7 + l] == want_kn[l];
  CHECK(ok);
  CHECK(b.arb.out[0] == 0 && b.arb.out[1] == 0 && b.arb.out[2] == 0);  // STAND's head
  CHECK(body_mode(b) == KILO_MODE_IDLE && b.arb.owner[3] == OWN_IDLE);
  // Foot nudge: x-shift +10 mm moves every foot 10 mm forward (slewed at 1
  // mm per tick).
  double x0[4], z0[4];
  for (int l = 0; l < 4; l++) out_foot(l, &x0[l], &z0[l]);
  CHECK(pose(0, 0, 0, 10) == GOAL_OK);
  tick();
  double x, z;
  out_foot(LEG_LF, &x, &z);
  CHECK(std::fabs(x - x0[LEG_LF] - 1) < 0.2);
  tick(20);
  for (int l = 0; l < 4; l++) {
    out_foot(l, &x, &z);
    CHECKF(std::fabs(x - x0[l] - 10) < 0.3 && std::fabs(z - z0[l]) < 0.3, "nudge leg %d: dx %.2f dz %.2f", l,
           x - x0[l], z - z0[l]);
  }
  // Pitch +10 deg (nose down): the back legs lengthen by (Lb/2) sin 10, the
  // front ones shorten by as much. Roll +10 deg (right side up): the right
  // legs by (W/2) sin 10. Height delta -8 mm: all of them.
  CHECK(pose(160, 0, 0, 0) == GOAL_OK);
  tick(40);
  double dp = PARAM(LEG_LB_MM) / 2.0 * std::sin(10 * D2R), dr = PARAM(LEG_W_MM) / 2.0 * std::sin(10 * D2R);
  for (int l = 0; l < 4; l++) {
    out_foot(l, &x, &z);
    CHECKF(std::fabs(z - z0[l] - (leg_front((uint8_t)l) ? -dp : dp)) < 0.3, "pitch leg %d: dz %.2f", l, z - z0[l]);
  }
  CHECK(pose(0, 160, -8, 0) == GOAL_OK);
  tick(40);
  for (int l = 0; l < 4; l++) {
    out_foot(l, &x, &z);
    CHECKF(std::fabs(z - z0[l] - (leg_left((uint8_t)l) ? -dr : dr) + 8) < 0.3, "roll leg %d: dz %.2f", l, z - z0[l]);
  }
  // The gait starts from STANCE with the pose kept, and hands back at the
  // same height.
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_TROT, 0, 0, 30, 0, -1, 70)) == GOAL_OK);
  tick(200);
  CHECK(b.gait.h == 280);
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  tick(100);
  CHECK(!body_gait_running(b) && arbiter_idle_posture(b.arb) == POSTURE_STANCE && b.gait.h == 280);
  for (int l = 0; l < 4; l++) {
    out_foot(l, &x, &z);
    CHECKF(std::fabs(z - (70 - 8 + (leg_left((uint8_t)l) ? -dr : dr))) < 0.3, "after the gait: leg %d z %.2f", l, z);
  }
  // STANCE re-entered from another posture starts at the default height.
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);
  tick(30);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, 0) == GOAL_OK);
  CHECK(b.gait.h == h);
}

// ---- The geometric balance, closed through a physical plant -------------------------
// The body stands on a board tilted by `slope` (a16). What the IMU reads is
// the board plus the body's attitude relative to it, which the feet make:
// pitch = atan((z_back - z_front) / Lb) (back legs longer = nose down = +),
// roll = atan((z_right - z_left) / W), with the commanded feet passed
// through a first-order lag (servo response, tau 0.15 s = 7.5 ticks) and a
// gain `k` (legs that answer less than the model, or more).
static double run_plant(double slope_p, double slope_r, double k, int ticks, bool flip, double* out_r) {
  double fp = 0, fr = 0;
  for (int t = 0; t < ticks; t++) {
    double z[4], x;
    for (int l = 0; l < 4; l++) out_foot(l, &x, &z[l]);
    double cp = std::atan2((z[LEG_LB] + z[LEG_RB] - z[LEG_LF] - z[LEG_RF]) / 2, PARAM(LEG_LB_MM)) / D2R;
    double cr = std::atan2((z[LEG_RF] + z[LEG_RB] - z[LEG_LF] - z[LEG_LB]) / 2, PARAM(LEG_W_MM)) / D2R;
    fp += (cp - fp) / 7.5;
    fr += (cr - fr) / 7.5;
    double mp = slope_p + k * fp, mr = slope_r + k * fr;
    if (flip) mp = -mp;
    in.att.pitch = (int16_t)std::lround(mp * 16);
    in.att.roll = (int16_t)std::lround(mr * 16);
    tick();
  }
  *out_r = in.att.roll / 16.0;
  return in.att.pitch / 16.0;
}

static void test_geometric_balance() {
  // (At 0.5x the 10 deg slope would need 20 deg of correction, past the
  // 15 deg clamp: 2.5 deg stay.)
  static const double K[] = {1.0, 0.7, 1.6};
  for (double k : K) {
    reset();
    CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 1, 0) == GOAL_OK);
    tick(5);
    CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, KILO_POSTURE_FLAGS_BALANCE) == GOAL_OK);
    tick(30);
    uint8_t bal[5] = {1, 255, 30, 2, 15};  // Kp ~1
    CHECK(body_goal_balance(b, bal) == GOAL_OK);
    double r, p = run_plant(-10, 10, k, 50, false, &r);  // nose up 10, right up 10
    std::printf("gait: geometric balance, leg response %.1fx: 10 deg slope -> pitch %+.2f roll %+.2f after 1 s\n", k,
                p, r);
    CHECK(std::fabs(p) <= 3 && std::fabs(r) <= 3);  // H13
    CHECK(b.bal.active && b.gait.geo);
    // No joint-space corrections were added: the legs are exactly the IK's.
    bool ik_only = true;
    for (int l = 0; l < 4; l++) ik_only = ik_only && b.arb.out[3 + l] == b.gait.leg[l];
    CHECK(ik_only && b.arb.out[1] == 0);
    p = run_plant(-10, 10, k, 200, false, &r);
    // To the 2 deg dead band plus the last step's quantisation (0.5 deg; the
    // roll lever W = 65 mm makes that step coarser than with the 60 mm guess).
    CHECKF(std::fabs(p) <= 2.5 && std::fabs(r) <= 2.5, "settled %.2f %.2f", p, r);
  }
  // A flipped sensor runs away to the clamp.
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, KILO_POSTURE_FLAGS_BALANCE) == GOAL_OK);
  tick(40);
  double r, p = run_plant(-10, 0, 1.0, 200, true, &r);
  CHECKF(std::fabs(b.bal.ax[BAL_PITCH].corr) >= 15 * 16 - 1, "flipped: corr %d, pitch %.1f", b.bal.ax[BAL_PITCH].corr,
         p);
  // A commanded pose is the target attitude: on a level floor with pitch
  // +6 deg asked for, the body reads +6 deg, and balance does not fight it.
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, KILO_POSTURE_FLAGS_BALANCE) == GOAL_OK);
  tick(40);
  CHECK(pose(96, 0, 0, 0) == GOAL_OK);
  p = run_plant(0, 0, 1.0, 150, false, &r);
  CHECKF(std::fabs(p - 6) < 0.3 && b.bal.ax[BAL_PITCH].corr == 0, "pose pitch: %.2f, corr %d pose %d", p,
         b.bal.ax[BAL_PITCH].corr, b.gait.pose[0]);
  // While walking the gait's BALANCE flag gates it.
  start(gp(1, KILO_GAIT_PATTERN_TROT, 0, 0, 30, 0, -1, 0, KILO_GAIT_FLAGS_BALANCE));
  CHECK(body_goal_balance(b, (const uint8_t[]){1, 255, 30, 2, 15}) == GOAL_OK);
  tick(60);
  p = run_plant(8, 0, 1.0, 100, false, &r);
  CHECKF(b.bal.active && std::fabs(p) <= 3, "walking: active %d pitch %.2f corr %d geo %d", b.bal.active, p, b.bal.ax[BAL_PITCH].corr, b.gait.geo);
  CHECK(gait(gp(1, KILO_GAIT_PATTERN_TROT, 0, 0, 30, 0, -1, 0, 0)) == GOAL_OK);
  tick();
  CHECK(!b.bal.active);
  // Leaving the geometric balance for a joint-space posture zeroes it.
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  int t = 0;
  while (body_gait_running(b) && t++ < 200) tick();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);
  tick();
  CHECK(!b.gait.geo && b.bal.ax[BAL_PITCH].corr == 0);
  in.att.pitch = in.att.roll = 0;
}

// ---- TELEMETRY through the fake link ------------------------------------------------
static void test_telemetry() {
  g_ee.reset();
  reset();
  g_lh.reset();
  link_init(0xC0FFEE01u, 0);
  auto rx = [](uint8_t type, const std::vector<uint8_t>& p) {
    g_lh.feed(frame(type, 1, p));
    link_poll();
  };
  rx(KILO_MSG_SET_POSTURE, {b.epoch, POSTURE_STAND, 1, 0});
  tick(10);
  rx(KILO_MSG_SET_GAIT, gp(1, KILO_GAIT_PATTERN_TROT));
  rx(KILO_MSG_SET_BODY_POSE, {b.epoch, 0, 0, 0, 0, 0, 5});
  bool seen_gait = false, seen_pending = false;
  int n = 0, held = 0;
  for (int t = 0; t < 150; t++) {
    body_link_frame(b);
    tick();
    link_tick();
    for (const TxFrame& f : g_lh.take()) {
      if (f.type != KILO_MSG_TELEMETRY) continue;
      kilo_telemetry_t m;
      memcpy(&m, f.payload.data(), sizeof m);
      if (m.mode != KILO_MODE_GAIT) {
        CHECK(!(m.flags & KILO_TELEMETRY_FLAGS_GAIT) && m.gait_phase == 0);
        continue;
      }
      // From the goal on (also while IDLE moves to STANCE): flag GAIT, the
      // leg nibble GAIT, the head nibble GAIT (bob and wag are on).
      CHECK((m.flags & KILO_TELEMETRY_FLAGS_GAIT) && m.owners == (KILO_OWNER_GAIT | (KILO_OWNER_GAIT << 4)));
      if (!body_gait_running(b)) {
        seen_pending = true;
        CHECK(b.arb.owner[3] == OWN_IDLE && m.gait_phase == 0);
      } else {
        seen_gait = true;
        n++;
        CHECK(m.gait_phase == (uint8_t)(b.gait.phi >> 8));
        // The balance (flag BALANCE in gp's default) holds on the trot's
        // diagonals: flag BALANCE_HELD.
        CHECK(!(m.flags & KILO_TELEMETRY_FLAGS_BALANCE_HELD) == !body_balance_held(b));
        held += (m.flags & KILO_TELEMETRY_FLAGS_BALANCE_HELD) != 0;
      }
    }
  }
  CHECK(seen_gait && seen_pending && n > 40 && held > 5);
  CHECK(b.gait.xs == 5);
  // No GOAL_REFUSED for either goal.
  CHECK(count_events(EV_GOAL_REFUSED) == 0);
  // A stale SET_GAIT through the link is refused with GOAL_REFUSED STALE.
  std::vector<uint8_t> p = gp(1, 1);
  p[0] = (uint8_t)(b.epoch + 3);
  rx(KILO_MSG_SET_GAIT, p);
  bool refused = false;
  for (int i = 0; i < b.ev.count; i++) {
    const Event& e = b.ev.q[(b.ev.head + i) % EVENT_QUEUE_LEN];
    if (e.code == EV_GOAL_REFUSED && (uint16_t)e.arg == ((KILO_MSG_SET_GAIT << 8) | KILO_GOAL_REFUSED_REASON_STALE))
      refused = true;
  }
  CHECK(refused);
}

// ---- Support state: which feet are down -------------------------------------
// The leg's phase this tick and whether it is in its swing (the generator's
// own definition; armed in steady state).
static uint16_t leg_p(int leg) {
  static const uint8_t OFF[3][4] = {
      {KILO_GAIT_WALK_OFFSET_LF, KILO_GAIT_WALK_OFFSET_RF, KILO_GAIT_WALK_OFFSET_RB, KILO_GAIT_WALK_OFFSET_LB},
      {KILO_GAIT_TROT_OFFSET_LF, KILO_GAIT_TROT_OFFSET_RF, KILO_GAIT_TROT_OFFSET_RB, KILO_GAIT_TROT_OFFSET_LB},
      {KILO_GAIT_CRAWL_OFFSET_LF, KILO_GAIT_CRAWL_OFFSET_RF, KILO_GAIT_CRAWL_OFFSET_RB, KILO_GAIT_CRAWL_OFFSET_LB}};
  return (uint16_t)(b.gait.phi + (OFF[b.gait.pat][leg] << 8));
}
static bool in_swing(int leg) { return leg_p(leg) >= (uint16_t)(b.gait.duty << 8); }

static void test_support() {
  // STANCE: all four down.
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 1, 0) == GOAL_OK);
  tick(5);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, 0) == GOAL_OK);
  tick(40);
  CHECK(b.gait.down == 0xF && gait_feet_down(b.gait) == 4);
  for (uint8_t pat = 0; pat < 3; pat++) {
    start(gp(1, pat));
    int T = period_ticks();
    tick(2 * T);
    bool ok = true, lift_ok = true;
    int mid_seen = 0, min_down = 4, max_down = 0;
    for (int t = 0; t < T; t++) {
      tick();
      int n = gait_feet_down(b.gait);
      min_down = std::min(min_down, n);
      max_down = std::max(max_down, n);
      for (int l = 0; l < 4; l++) {
        bool down = b.gait.down & (1 << l);
        ok = ok && down == !in_swing(l);
        lift_ok = lift_ok && !(lifted(l) && down);  // a foot visibly up is never "down"
        // Mid-swing (within half a tick of phase): that leg alone is up
        // (WALK, CRAWL) or with its diagonal partner (TROT).
        uint16_t mid = (uint16_t)(32768u + (b.gait.duty << 7));
        if (std::abs((int16_t)(leg_p(l) - mid)) <= b.gait.dphi / 2) {
          mid_seen++;
          uint8_t want = (uint8_t)(0xF & ~(1 << l));
          if (pat == KILO_GAIT_PATTERN_TROT) want &= (uint8_t)~(1 << (l ^ 2));  // LF+RB, RF+LB
          CHECKF(b.gait.down == want, "pattern %d, leg %s mid-swing: down %x", pat, LEG_NAME[l], b.gait.down);
        }
      }
    }
    std::printf("gait: pattern %d feet down %d..%d over a cycle\n", pat, min_down, max_down);
    CHECKF(ok && lift_ok, "pattern %d: down bits vs swing phase", pat);
    CHECK(mid_seen >= 4);
    if (pat == KILO_GAIT_PATTERN_TROT)
      CHECK(min_down == 2 && max_down == 4);
    else
      CHECK(min_down == 3 && max_down == (pat == KILO_GAIT_PATTERN_CRAWL ? 4 : 3));
    // After a stop: all four down.
    CHECK(gait(gp(0, 0)) == GOAL_OK);
    int t = 0;
    while (body_gait_running(b) && t++ < 4 * T) tick();
    CHECK(!body_gait_running(b) && b.gait.down == 0xF && gait_feet_down(b.gait) == 4);
    tick();
    CHECK(b.gait.down == 0xF);
  }
}

// ---- Sway ----------------------------------------------------------------------------
// The planned sway in double from the leg phases (gait.h): w_i a raised
// cosine centred on mid-swing less the lead, half-width min(1.5 swings,
// half a cycle); plan_roll = A_roll sum side_i w_i, plan_pitch = A_pitch sum
// end_i w_i, times the ramp. a16.
static uint16_t sway_centre() { return (uint16_t)(32768u + (b.gait.duty << 7) - b.gait.lead); }
static void sway_ref(double* roll, double* pitch, double* w_out = nullptr) {
  double sw = (256 - b.gait.duty) / 256.0, hw = std::fmin(1.5 * sw, 0.5);
  uint16_t mid = sway_centre();
  double r = 0, p = 0;
  for (int l = 0; l < 4; l++) {
    double d = std::fabs((int16_t)(leg_p(l) - mid) / 65536.0);
    double w = d < hw ? (1 + std::cos(M_PI * d / hw)) / 2 : 0;
    if (w_out) w_out[l] = w;
    r += (leg_left((uint8_t)l) ? -1 : 1) * w;
    p += (leg_front((uint8_t)l) ? -1 : 1) * w;
  }
  double amp = b.gait.amp / 65536.0;
  *roll = GAIT_SWAY_ROLL_A16 * r * amp;
  *pitch = GAIT_SWAY_PITCH_A16 * p * amp;
}

// The body's attitude made by three stance feet with LF up (deg): the plane
// through RF, RB, LB (pitch: back legs longer = nose down = +; roll: right
// legs longer = right side up = +, §3f).
static void stance_attitude_lf_up(double* pitch, double* roll) {
  double x, z[4];
  for (int l = 0; l < 4; l++) foot(l, &x, &z[l]);
  *pitch = std::asin((z[LEG_RB] - z[LEG_RF]) / PARAM(LEG_LB_MM)) / D2R;
  *roll = std::asin((z[LEG_RB] - z[LEG_LB]) / PARAM(LEG_W_MM)) / D2R;
}

static void test_sway() {
  const uint8_t SWAY = KILO_GAIT_FLAGS_BOB | KILO_GAIT_FLAGS_WAG | KILO_GAIT_FLAGS_SWAY;  // no balance: the feet only
  // WALK and CRAWL at their defaults, and a WALK at duty 140 (the window is
  // then the whole cycle): the plan follows the model every tick, from the
  // start of the ramp on (so it fades in with amp), and the peak is the
  // amplitude.
  struct Case { uint8_t pat, duty; };
  static const Case CASES[] = {{KILO_GAIT_PATTERN_WALK, 0}, {KILO_GAIT_PATTERN_CRAWL, 0}, {KILO_GAIT_PATTERN_WALK, 140}};
  for (const Case& c : CASES) {
    start(gp(1, c.pat, 0, c.duty, 30, 0, -1, 0, SWAY));
    CHECK(b.gait.plan_roll == 0 && b.gait.plan_pitch == 0);  // the first gait tick: amp 0
    int T = period_ticks();
    double err = 0, peak_r = 0, peak_p = 0, peak_model = 0;
    int best = 70000, best_mid = 70000;
    double lean_p = 0, lean_r = 0, plan_p = 0, plan_r = 0;
    uint8_t down_mid = 0;
    for (int t = 0; t < 4 * T; t++) {
      tick();
      double r, p;
      sway_ref(&r, &p);
      err = std::fmax(err, std::fmax(std::fabs(b.gait.plan_roll - r), std::fabs(b.gait.plan_pitch - p)));
      if (t < 2 * T) continue;
      peak_model = std::fmax(peak_model, std::fabs(r));
      peak_r = std::fmax(peak_r, std::fabs(b.gait.plan_roll));
      peak_p = std::fmax(peak_p, std::fabs(b.gait.plan_pitch));
      // The centre of LF's window (the lead before its mid-swing): the
      // plane of RF, RB, LB.
      int16_t d = (int16_t)(leg_p(LEG_LF) - sway_centre());
      if (std::abs(d) < best) {
        best = std::abs(d);
        stance_attitude_lf_up(&lean_p, &lean_r);
        plan_p = b.gait.plan_pitch / 16.0;
        plan_r = b.gait.plan_roll / 16.0;
      }
      // LF's mid-swing: the body on RF, RB, LB.
      d = (int16_t)(leg_p(LEG_LF) - (uint16_t)(32768u + (b.gait.duty << 7)));
      if (std::abs(d) < best_mid) {
        best_mid = std::abs(d);
        down_mid = b.gait.down;
      }
    }
    std::printf("gait: sway pattern %d duty %d: plan vs model %.2f a16; peak roll %.2f pitch %.2f deg; at the centre of "
                "LF's window (%d ms before its mid-swing) the stance feet lean the body pitch %+.2f roll %+.2f deg (plan %+.2f %+.2f)\n",
                c.pat, b.gait.duty, err, peak_r / 16, peak_p / 16, KILO_GAIT_SWAY_LEAD_MS, lean_p, lean_r, plan_p, plan_r);
    // (The model has w_i = 0 at leg i's mid-stance and 1 at its mid-swing,
    // so following it every tick, ramp included, is the window's shape and
    // the fade-in.)
    CHECK(err <= 3);  // 0.2 deg: the fixed-point window and ramp truncate
    CHECK(down_mid == 0xE);  // LF up, the other three down
    // LF lifting: lean RIGHT (right side down: negative roll) and not
    // forward (back down or level: pitch <= 0), read off the three stance
    // feet; the roll is the full amplitude at the window's centre (the two
    // neighbours' windows are equal and their sides cancel).
    // (At duty 140 another leg is in the air there too: no plane of three.
    // WALK: the centre is LF's lift-off, the tick LB lands.)
    if (c.duty == 0) {
      CHECKF(std::fabs(lean_r + KILO_GAIT_SWAY_ROLL_DEG) < 0.5 && lean_p < 0.3, "LF up: pitch %.2f roll %.2f", lean_p,
             lean_r);
      CHECK(std::fabs(lean_r - plan_r) < 0.3 && std::fabs(lean_p - plan_p) < 0.3);
    } else {
      CHECK(c.duty == 140);  // (the default patterns have their three feet down there)
    }
    // At the defaults the back end goes down too (WALK: the neighbours'
    // windows give back half of it, -A_pitch / 2; CRAWL: -A_pitch).
    if (c.duty == 0) CHECK(lean_p < -0.4 * KILO_GAIT_SWAY_PITCH_DEG);
    // The peak: the amplitude at mid-swing, more where two same-side swings'
    // windows overlap (WALK: 1.5 A between LB's and LF's).
    CHECK(std::fabs(peak_r - peak_model) <= 3);
    if (c.pat == KILO_GAIT_PATTERN_CRAWL) CHECK(std::fabs(peak_r - GAIT_SWAY_ROLL_A16) <= 2);
    if (c.pat == KILO_GAIT_PATTERN_WALK && c.duty == 0) CHECK(std::fabs(peak_r - 1.5 * GAIT_SWAY_ROLL_A16) <= 3);
    // The stop fades it out; landed, it is 0.
    CHECK(gait(gp(0, 0)) == GOAL_OK);
    int t = 0;
    bool zero_landing = true;
    while (body_gait_running(b) && t++ < 4 * T) {
      tick();
      if (b.gait.state == GS_LAND) zero_landing = zero_landing && b.gait.plan_roll == 0 && b.gait.plan_pitch == 0;
    }
    CHECK(zero_landing && b.gait.plan_roll == 0 && b.gait.plan_pitch == 0);
  }
  // TROT: the diagonal pairs cancel exactly.
  start(gp(1, KILO_GAIT_PATTERN_TROT, 0, 0, 25, 0, -1, 0, SWAY));
  bool zero = true;
  for (int t = 0; t < 3 * period_ticks(); t++) {
    tick();
    zero = zero && b.gait.plan_roll == 0 && b.gait.plan_pitch == 0;
  }
  CHECK(zero);
  // The flag off: no sway.
  start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, KILO_GAIT_FLAGS_BOB));
  zero = true;
  for (int t = 0; t < 3 * period_ticks(); t++) {
    tick();
    zero = zero && b.gait.plan_roll == 0 && b.gait.plan_pitch == 0;
  }
  CHECK(zero);
}

// ---- Q14 sums at cos = +-1 ---------------------------------------------------------------
// The sway window (1 + cos)/2 at its centre, the swing's x profile (1 - cos
// pi s)/2 at its last tick and the lift (1 - cos 2 pi s)/2 at mid-swing all
// sum 16384 and a cos of +-16384: 32768, which wraps to -32768 in the AVR's
// 16-bit int unless the sum is int32_t (gait.cpp). The host's int is 32
// bits, so this cannot show the wrap itself; it pins the three phases where
// fx_cos is exactly +-16384 and checks the values against the double model
// (the wrap would be a -2 window, a foot 2 strides back, a lift of -h).
// The AVR side: avr-g++ refuses the 16-bit sums as constant expressions.
static void q14_at(uint16_t p_leg, int leg) {
  b.gait.phi = (uint16_t)(p_leg - (uint16_t)(leg_p(leg) - b.gait.phi));
  gait_legs(b.gait, 0, 0, nullptr);
}
static void test_q14_extremes() {
  start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, 7 | KILO_GAIT_FLAGS_SWAY));
  tick(3 * period_ticks());
  CHECK(b.gait.amp == 65535 && b.gait.state == GS_RUN);
  b.gait.geo = 0;  // (no balance terms in H_i: the model below has none)
  uint16_t b16 = (uint16_t)(b.gait.duty << 8);
  uint32_t inv_sw = (1UL << 30) / (65536UL - b16);
  auto angle = [&](uint16_t p) { return (int16_t)((((uint32_t)(p - b16) * inv_sw) >> 16) * A16_180 >> 14); };
  // 1. The window's centre: LF's w is exactly 1 (cos 0 = +16384).
  q14_at(sway_centre(), LEG_LF);
  CHECK(fx_cos(0) == 16384);
  double wr, wp, w[4];
  sway_ref(&wr, &wp, w);
  CHECKF(w[LEG_LF] == 1.0 && std::fabs(b.gait.plan_roll - wr) <= 1 && std::fabs(b.gait.plan_pitch - wp) <= 1,
         "window centre: plan roll %d pitch %d, model %.1f %.1f", b.gait.plan_roll, b.gait.plan_pitch, wr, wp);
  // 2. The swing's last tick (cos pi s = -16384): the foot at the front.
  // 3. Mid-swing (cos 2 pi s = -16384): the full step height.
  uint16_t p_last = 65535, p_mid = (uint16_t)(b16 + ((65536u - b16) >> 1));
  while (fx_cos((int16_t)(2 * angle(p_mid))) != -16384) p_mid++;
  CHECK(fx_cos(angle(p_last)) == -16384);
  double s_i = ((b.gait.stride * 65535L + 32768) >> 16) / 4.0, step = b.gait.step / 4.0;
  for (uint16_t p : {p_last, p_mid}) {
    q14_at(p, LEG_LF);
    double x, z, s = (p - b16) / (65536.0 - b16);
    foot(LEG_LF, &x, &z);
    double pitch = b.gait.plan_pitch / 16.0 * D2R, roll = b.gait.plan_roll / 16.0 * D2R;
    double h = b.gait.h / 4.0 + b.gait.dh - PARAM(LEG_LB_MM) / 2 * std::sin(pitch) - PARAM(LEG_W_MM) / 2 * std::sin(roll);
    double xm = b.gait.xs + s_i * ((1 - std::cos(M_PI * s)) / 2 - 0.5), zm = h - step * (1 - std::cos(2 * M_PI * s)) / 2;
    std::printf("gait: Q14 extremes, LF at swing s = %.4f: x %.2f z %.2f mm, model %.2f %.2f\n", s, x, z, xm, zm);
    CHECKF(std::fabs(x - xm) < 0.3 && std::fabs(z - zm) < 0.3, "s %.4f: x %.2f z %.2f, model %.2f %.2f", s, x, z, xm,
           zm);
  }
}

// ---- The stance-aware plant ----------------------------------------------------------
// run_plant tilts the body by all four commanded feet, lifted ones included:
// right for STANCE, not for a walk (a foot in the air carries nothing). This
// one follows docs/walking-and-balance.md §2. The feet are the arbiter's
// output (the last tick's command). The support is the gait's own `down`
// (bits LF RF RB LB) or, with `ground` > 0 (the lagged support), only those
// of them whose lift (how far the foot is commanded above its stance height
// H_i) has come under `ground` mm through the servo lag (tau 0.15 s): a foot
// commanded down, at a touchdown or by the reflex, carries the body once the
// leg has got there, not on the tick its down bit is set:
//  - on four feet the body lies on their plane (least squares), on three on
//    the plane through them, on the board (`slope`), reached through the
//    servo lag (tau 0.15 s = 7.5 ticks, as run_plant);
//  - on a diagonal pair, or on three feet while the body leans toward the
//    missing corner by more than `margin`, it is an inverted pendulum about
//    the diagonal: the world roll (the free axis; a rotation about the
//    123 x 65 mm diagonal is 88% roll) grows with tau = sqrt(h/g) = 90 ms,
//    integrated exactly per tick, the pitch follows the two diagonal feet;
//    (lagged support only) on one side's or one end's pair, or fewer, it
//    falls the same way with the pitch held;
//  - back on a support after that, it settles onto the feet's plane with
//    the landing lag SP_LAND ticks (the feet stop the rock: an impact, not
//    a servo move) until within 0.5 deg of it;
//  - the IMU reads the world attitude and its rates (differences x 50 Hz).
// (With the lagged support at 2 mm the WALK as tuned leans 23 deg on a
// level floor, twice its plan, and the TROT on test_two_foot_hold's slope
// falls: the walking tests keep the gait's own support; test_catch_plant
// uses both. The hold-over re-measured on both: see that test.)
static const double SP_TAU = 0.09, SP_SERVO = 7.5, SP_LAND = 2.0;
struct SPlant {
  double p, r, vp, vr, slope_p, slope_r;
  double margin;    // three feet: the lean toward the missing corner that tips the body (deg; 90 = never)
  double ground;    // lagged support: a down foot supports once its lagged lift is under this (mm); 0 = the down bits
  bool rocking;     // fell and has not settled on the feet yet
  double max_tip;   // largest roll or pitch rate toward a swinging leg (deg/s)
  double max_lean;  // largest |world roll| (deg)
  int catches;      // legs caught in their swing (armed before, disarmed now, still in swing)
  uint8_t first;    // bits of the legs of the first catch
  double lz[4];     // each foot's commanded lift through the servo lag (mm)
  uint8_t sup;      // the feet that support this tick (bits as `down`)
};
// Foot l's commanded lift this tick: its stance height H_i (the pose, the
// plan and the balance's corrections, as gait_legs adds them) less its z.
static double cmd_lift(int l) {
  double x, z;
  out_foot(l, &x, &z);
  double pitch = b.gait.pose[0] + b.gait.plan_pitch, roll = b.gait.pose[1] + b.gait.plan_roll;
  if (b.gait.geo) {
    pitch += b.bal.ax[BAL_PITCH].corr;
    roll += b.bal.ax[BAL_ROLL].corr;
  }
  double h = b.gait.h / 4.0 + b.gait.dh +
             (leg_front((uint8_t)l) ? -1 : 1) * PARAM(LEG_LB_MM) / 2 * std::sin(pitch / 16 * D2R) +
             (leg_left((uint8_t)l) ? -1 : 1) * PARAM(LEG_W_MM) / 2 * std::sin(roll / 16 * D2R);
  return std::fmax(0.0, h - z);
}
// The support this tick: the lag a (0: none, at init) applied to the lifts.
static void splant_feet(SPlant& s, double a) {
  s.sup = b.gait.down;
  for (int l = 0; l < 4; l++) {
    s.lz[l] += (cmd_lift(l) - s.lz[l]) * a;
    if (s.ground > 0 && s.lz[l] >= s.ground) s.sup &= (uint8_t)~(1 << l);
  }
}
// The attitude the down feet give (deg, on the board), and whether the body
// tips instead (then only the pitch is the feet's).
static bool splant_support(SPlant& s, double* tp, double* tr) {
  double lb = PARAM(LEG_LB_MM), w = PARAM(LEG_W_MM), x, z[4];
  for (int l = 0; l < 4; l++) out_foot(l, &x, &z[l]);
  uint8_t d = s.sup;
  // Pitch of a front/back pair (back longer = nose down = +), roll of a
  // left/right pair (right longer = right side up = +).
  auto pitch2 = [&](int f, int k) { return std::asin((z[k] - z[f]) / lb) / D2R; };
  auto roll2 = [&](int lt, int rt) { return std::asin((z[rt] - z[lt]) / w) / D2R; };
  *tr = 0;
  int n = (d & 1) + ((d >> 1) & 1) + ((d >> 2) & 1) + ((d >> 3) & 1);
  if (n < 2 || (n == 2 && d != 0x5 && d != 0xA)) {
    *tp = s.p - s.slope_p;
    return true;
  }
  switch (n) {
    case 4:
      *tp = std::asin((z[LEG_RB] + z[LEG_LB] - z[LEG_LF] - z[LEG_RF]) / 2 / lb) / D2R;
      *tr = std::asin((z[LEG_RF] + z[LEG_RB] - z[LEG_LF] - z[LEG_LB]) / 2 / w) / D2R;
      return false;
    case 3: {
      int m = 0;
      while (d & (1 << m)) m++;
      int o = m ^ 2, ps = 3 - o, rs = o ^ 1;  // the opposite corner, its same-side and same-end partners
      if (leg_left((uint8_t)m) ? s.r > s.margin : s.r < -s.margin) {
        int f = leg_front((uint8_t)(m ^ 1)) ? m ^ 1 : 3 - m;  // the diagonal: m's two neighbours
        *tp = pitch2(f, f == (m ^ 1) ? 3 - m : m ^ 1);
        return true;
      }
      *tp = leg_front((uint8_t)o) ? pitch2(o, ps) : pitch2(ps, o);
      *tr = leg_left((uint8_t)o) ? roll2(o, rs) : roll2(rs, o);
      return false;
    }
    default:  // a diagonal pair (TROT): LF+RB or RF+LB
      *tp = (d & 1) ? pitch2(LEG_LF, LEG_RB) : pitch2(LEG_RF, LEG_LB);
      return true;
  }
}
static void splant_init(SPlant& s, double slope_p, double slope_r, double margin = 90, double ground = 0) {
  s = SPlant();
  s.margin = margin;
  s.ground = ground;
  s.slope_p = slope_p;
  s.slope_r = slope_r;
  for (int l = 0; l < 4; l++) s.lz[l] = cmd_lift(l);
  splant_feet(s, 0);
  double tp, tr;
  splant_support(s, &tp, &tr);
  s.p = slope_p + tp;
  s.r = slope_r + tr;
}
// One tick: the plant answers the last tick's commands, then body_tick.
// push: deg/s added to the roll rate (a shove from the side).
static void splant_step(SPlant& s, double push = 0) {
  double tp, tr, p0 = s.p, r0 = s.r;
  splant_feet(s, 1 - std::exp(-1 / SP_SERVO));
  bool fall = splant_support(s, &tp, &tr);
  double a = 1 - std::exp(-1 / (s.rocking ? SP_LAND : SP_SERVO));
  s.p += (s.slope_p + tp - s.p) * a;
  if (fall) {
    double e = 0.02 / SP_TAU, c = std::cosh(e), sh = std::sinh(e), vr = s.vr + push;
    s.r = r0 * c + vr * SP_TAU * sh;
    s.vr = r0 * sh / SP_TAU + vr * c;
    s.rocking = true;
  } else {
    s.r += (s.slope_r + tr - s.r) * a;
    s.vr = (s.r - r0) * 50 + push;
    if (std::fabs(s.slope_r + tr - s.r) < 0.5) s.rocking = false;
  }
  s.vp = (s.p - p0) * 50;
  s.max_lean = std::fmax(s.max_lean, std::fabs(s.r));
  in.att.pitch = (int16_t)std::lround(s.p * 16);
  in.att.roll = (int16_t)std::lround(s.r * 16);
  in.att.rate_pitch = (int16_t)std::lround(s.vp * 16);
  in.att.rate_roll = (int16_t)std::lround(s.vr * 16);
  // The tipping rate toward a swinging leg, as the reflex reads it.
  uint8_t armed = b.gait.armed, d = b.gait.down;
  for (int l = 0; l < 4; l++)
    if (b.gait.state == GS_RUN && !(d & (1 << l))) {
      double tr_l = leg_left((uint8_t)l) ? s.vr : -s.vr, tp_l = leg_front((uint8_t)l) ? s.vp : -s.vp;
      s.max_tip = std::fmax(s.max_tip, std::fmax(tr_l, tp_l));
    }
  tick();
  uint8_t c = 0;
  for (int l = 0; l < 4; l++)
    if (b.gait.state == GS_RUN && in_swing(l) && (armed & ~b.gait.armed & (1 << l))) c |= (uint8_t)(1 << l);
  if (c && !s.catches++) s.first = c;
}

// STANCE on the board with the balance on (Kp ~1, Kd 30 ms, dead band 2 deg),
// levelled there first, then the gait, the plant running throughout.
static const uint8_t BAL_KP1[5] = {1, 255, 30, 2, 15};
static void walk_on(SPlant& s, uint8_t pat, uint8_t flags, double sp, double sr, const uint8_t* bal = BAL_KP1,
                    double margin = 90, double ground = 0) {
  static const int8_t STRIDE[3] = {30, 25, 25};  // spec [gait_patterns.*] stride_mm
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 1, 0) == GOAL_OK);
  tick(5);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, 0) == GOAL_OK);
  tick(30);
  CHECK(body_goal_balance(b, bal) == GOAL_OK);
  splant_init(s, sp, sr, margin, ground);
  for (int t = 0; t < 100; t++) splant_step(s);
  CHECK(gait(gp(1, pat, 0, 0, STRIDE[pat], 0, -1, 0, flags)) == GOAL_OK);
  int t = 0;
  while (!body_gait_running(b) && t++ < 100) splant_step(s);
  CHECK(body_gait_running(b));
}

// ---- The plan as the balance's reference ---------------------------------------------
// WALK and CRAWL with sway and the balance on (Kp ~1) on a level floor. The
// body follows the plan through the servo lag (the plant's 0.15 s), within
// the dead band, and the corrections stay small: what is left is that lag
// (the plan leads the body by 150 ms; the sway is fastest where the lean
// changes sides). Before the plan was the reference the balance fought the
// sway: corrections up to 10.8 deg (WALK) and 6.4 deg (CRAWL), the walk's
// 12 deg lean cut to 4.7 deg.
static void test_plan_reference() {
  for (uint8_t pat : {KILO_GAIT_PATTERN_WALK, KILO_GAIT_PATTERN_CRAWL}) {
    SPlant s;
    walk_on(s, pat, 7 | KILO_GAIT_FLAGS_SWAY, 0, 0);
    int T = period_ticks();
    for (int t = 0; t < 3 * T; t++) splant_step(s);
    double a = 1 - std::exp(-1 / SP_SERVO), lp = b.gait.plan_pitch / 16.0, lr = b.gait.plan_roll / 16.0;
    double dev = 0, corr = 0, lean = 0, plan = 0;
    for (int t = 0; t < 4 * T; t++) {
      splant_step(s);
      lp += (b.gait.plan_pitch / 16.0 - lp) * a;
      lr += (b.gait.plan_roll / 16.0 - lr) * a;
      if (t < T) continue;  // (the model's lag settles)
      dev = std::fmax(dev, std::fmax(std::fabs(s.p - lp), std::fabs(s.r - lr)));
      corr = std::fmax(corr, std::fmax(std::fabs(b.bal.ax[BAL_ROLL].corr), std::fabs(b.bal.ax[BAL_PITCH].corr)) / 16.0);
      lean = std::fmax(lean, std::fabs(s.r));
      plan = std::fmax(plan, std::fabs(b.gait.plan_roll / 16.0));
    }
    std::printf("gait: plan as reference, pattern %d, level floor, Kp ~1: body vs plan (servo-lagged) within %.2f deg, "
                "corrections at most %.2f deg; lean %.1f of the planned %.1f deg\n",
                pat, dev, corr, lean, plan);
    CHECK(b.bal.active);
    CHECKF(dev <= 2.0, "pattern %d: %.2f deg off the plan", pat, dev);  // the dead band
    CHECKF(corr <= (pat == KILO_GAIT_PATTERN_WALK ? 2.5 : 3.5), "pattern %d: corrections %.2f deg", pat, corr);
    CHECK(lean >= 0.8 * plan);  // the sway shows through
  }
}

// ---- The sway's lead -------------------------------------------------------------------
// WALK and CRAWL with sway, the balance at Kp ~1, level floor, the
// stance-aware plant. At every lift-off (the first tick a leg is commanded
// up) the body's roll, signed so that + is away from the lifting leg
// (side_i * roll: a left leg wants the right side down, roll < 0), with the
// lead set to `lead_ms` (0 = the window on mid-swing). Smallest over 4
// cycles; also the largest lean when the plant tips at `margin` deg toward
// a missing corner.
static double liftoff_lean(uint8_t pat, int lead_ms, double margin, double* max_lean) {
  SPlant s;
  walk_on(s, pat, 7 | KILO_GAIT_FLAGS_SWAY, 0, 0, BAL_KP1, margin);
  b.gait.lead = (uint16_t)(b.gait.dphi * lead_ms / 20);
  int T = period_ticks();
  for (int t = 0; t < 3 * T; t++) splant_step(s);
  double worst = 90;
  s.max_lean = 0;
  for (int t = 0; t < 4 * T; t++) {
    uint8_t d0 = b.gait.down;
    splant_step(s);
    for (int l = 0; l < 4; l++)
      if ((d0 & (1 << l)) && !(b.gait.down & (1 << l))) worst = std::fmin(worst, (leg_left((uint8_t)l) ? -1 : 1) * s.r);
  }
  if (max_lean) *max_lean = std::isfinite(s.r) ? s.max_lean : NAN;  // nan: it fell over (asin out of range)
  return worst;
}
static void test_sway_lead() {
  for (uint8_t pat : {KILO_GAIT_PATTERN_WALK, KILO_GAIT_PATTERN_CRAWL}) {
    double fell0, fell, before = liftoff_lean(pat, 0, 90, nullptr), lag = liftoff_lean(pat, 150, 90, nullptr),
                         after = liftoff_lean(pat, KILO_GAIT_SWAY_LEAD_MS, 90, nullptr);
    liftoff_lean(pat, 0, 2, &fell0);
    liftoff_lean(pat, KILO_GAIT_SWAY_LEAD_MS, 2, &fell);
    std::printf("gait: sway lead, pattern %d: at every lift-off the body leans away from the lifting leg by at least "
                "%+.2f deg with no lead, %+.2f with 150 ms (the servo lag), %+.2f with %d ms; a plant tipping at 2 deg "
                "leans at most %.1f deg with no lead (nan: fell), %.1f with %d ms\n",
                pat, before, lag, after, KILO_GAIT_SWAY_LEAD_MS, fell0, fell, KILO_GAIT_SWAY_LEAD_MS);
    CHECKF(after >= 3, "pattern %d: %.2f deg at a lift-off", pat, after);
    CHECK(after > before + 3);
    if (pat == KILO_GAIT_PATTERN_WALK) CHECK(!(fell0 < 20));  // the finding: with no lead the WALK falls there
    CHECKF(fell < 0.5 + (pat == KILO_GAIT_PATTERN_WALK ? 1.5 : 1.0) * KILO_GAIT_SWAY_ROLL_DEG, "pattern %d: lean %.1f",
           pat, fell);  // the plan's own peak (WALK 1.5 A, CRAWL A): no fall
  }
}

// ---- The two-foot hold -------------------------------------------------------------------
// TROT on a board tilted 10 deg nose up and 10 deg right side up, Kp ~1. On
// fewer than three feet (the support the balance reads: the last tick's)
// the corrections do not change, and not for BAL_HOLD_TICKS after; then they
// move again, and the body is level within the dead band at every lift-off.
// The hold-over was measured with this plant (landing lag 2 ticks): the
// largest lean per step on a 5 deg slope at Kp 0.5 is 12.9 deg with no
// hold-over, 12.0 with 1 tick, 11.65 with 2 and 11.6 with 3-6 (and with no
// hold at all the corrections swing with every step: roll -7.3..-2.4 deg on
// 5 deg instead of a steady -3.3). Re-measured 2026-10-13 on this test's
// run (largest lean, BAL_HOLD_TICKS 0 / 1 / 2 / 3 / 4 / 6): 13.1 / 12.8 /
// 12.9 / 12.8 / 14.4 / 20.8 deg with the gait's support; 14.5 / 15.8 /
// 15.9 / 16.1 / 17.9 / 19.2 with the lagged support at 3 mm (lift-off
// level within 2.9 / 3.3 / 3.6 / 3.0 / 3.1 / 3.4 deg), and at 2 mm it falls
// whatever the hold. Kept at 2 (docs/architecture.md 3f "Held on two
// feet").
static void test_two_foot_hold() {
  SPlant s;
  walk_on(s, KILO_GAIT_PATTERN_TROT, 7, -10, 10);
  int T = period_ticks();
  for (int t = 0; t < 4 * T; t++) splant_step(s);
  // The board tilts 4 deg further: the corrections follow, on 3-4 feet only.
  s.slope_r = 14;
  bool frozen_ok = true, held_ok = true;
  int moved = 0, moved_held = 0, after = -1;
  double lift = 0;
  for (int t = 0; t < 6 * T; t++) {
    uint8_t n0 = gait_feet_down(b.gait);
    int16_t c0[2] = {b.bal.ax[0].corr, b.bal.ax[1].corr};
    double p0 = s.p, r0 = s.r;
    splant_step(s);
    bool same = b.bal.ax[0].corr == c0[0] && b.bal.ax[1].corr == c0[1];
    after = n0 < 3 ? 0 : after >= 0 ? after + 1 : -1;  // ticks since the third foot landed
    bool held = n0 < 3 || (after >= 1 && after <= BAL_HOLD_TICKS);
    frozen_ok = frozen_ok && (!held || same);
    held_ok = held_ok && body_balance_held(b) == held;
    moved += !same;
    moved_held += !same && held;
    // Levelled at each lift-off (the last tick on four feet).
    if (t > 3 * T && n0 == 4 && gait_feet_down(b.gait) == 2)
      lift = std::fmax(lift, std::fmax(std::fabs(p0), std::fabs(r0)));
  }
  std::printf("gait: two-foot hold, TROT on a 10 deg slope tilting to 14: corrections moved on %d ticks, %d of them "
              "held; at lift-off level within %.2f deg; hold-over %d ticks\n",
              moved, moved_held, lift, BAL_HOLD_TICKS);
  CHECK(frozen_ok && moved_held == 0);
  CHECK(held_ok);
  CHECK(moved >= 5);  // the 4 deg are taken up on four feet
  CHECKF(lift <= 3, "lift-off at %.2f deg", lift);  // the dead band, a step and what the last rock left
  CHECKF(b.bal.ax[BAL_ROLL].corr / 16.0 < -11, "roll corr %.2f", b.bal.ax[BAL_ROLL].corr / 16.0);
  // STANCE (four feet): never held.
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, KILO_POSTURE_FLAGS_BALANCE) == GOAL_OK);
  tick(40);
  CHECK(b.bal.active && !body_balance_held(b) && b.bal.hold == 0);
}

// ---- Early touchdown (flag CATCH) -----------------------------------------------------
// A WALK (no balance: the legs alone, or `pat`) to `leg`'s mid-swing, the
// body rates `rr`, `rp` (a16/s) on the `n` ticks ending there, optionally an
// IMU fault; the legs and the support after it, and `leg`'s foot the tick
// before (the reflex wants two ticks over its threshold: one is a spike).
struct Snap {
  int16_t leg[8];
  uint8_t down, armed;
  double x[4], z[4], h[4];  // each foot's x, z and its stance height H_i (mm)
  double px, pz;            // `leg`'s foot the tick before
};
static Snap catch_at(uint8_t flags, int leg, int16_t rr, int16_t rp, bool fault = false, int n = 2,
                     uint8_t pat = KILO_GAIT_PATTERN_WALK) {
  start(gp(1, pat, 0, 0, 30, 0, -1, 0, flags));
  tick(2 * period_ticks());
  uint16_t mid = (uint16_t)(32768u + (b.gait.duty << 7) - (n - 1) * b.gait.dphi);
  int t = 0;
  while (std::abs((int16_t)(leg_p(leg) - mid)) > b.gait.dphi / 2 && t++ < 200) tick();
  in.att.rate_roll = rr;
  in.att.rate_pitch = rp;
  b.imu_fault = fault;
  Snap s;
  for (int k = 0; k < n; k++) {
    foot(leg, &s.px, &s.pz);
    tick();
  }
  in.att.rate_roll = in.att.rate_pitch = 0;
  b.imu_fault = 0;
  memcpy(s.leg, b.gait.leg, sizeof s.leg);
  s.down = b.gait.down;
  s.armed = b.gait.armed;
  double lb = PARAM(LEG_LB_MM), w = PARAM(LEG_W_MM);
  double pitch = (b.gait.pose[0] + b.gait.plan_pitch) / 16.0 * D2R, roll = (b.gait.pose[1] + b.gait.plan_roll) / 16.0 * D2R;
  for (int l = 0; l < 4; l++) {
    foot(l, &s.x[l], &s.z[l]);
    s.h[l] = b.gait.h / 4.0 + b.gait.dh + (leg_front((uint8_t)l) ? -1 : 1) * lb / 2 * std::sin(pitch) +
             (leg_left((uint8_t)l) ? -1 : 1) * w / 2 * std::sin(roll);
  }
  return s;
}
static bool same_legs(const Snap& a, const Snap& c, int except = -1) {
  for (int l = 0; l < 4; l++) {
    if (l == except) continue;
    if (a.leg[l] != c.leg[l] || a.leg[4 + l] != c.leg[4 + l] || ((a.down ^ c.down) & (1 << l))) return false;
  }
  return true;
}

static void test_catch() {
  const uint8_t F = KILO_GAIT_FLAGS_SWAY | KILO_GAIT_FLAGS_CATCH, NOCATCH = KILO_GAIT_FLAGS_SWAY;
  const int16_t TH = GAIT_TOUCHDOWN_A16S, OVER = (int16_t)(TH + 16);  // the threshold, 1 deg/s past it
  // Sign (gait.h): roll rising = right side up, left side down = falling
  // toward the LEFT legs; pitch rising = nose down = toward the FRONT legs.
  struct Case { int leg; int16_t rr, rp; bool caught; const char* what; };
  static const Case CASES[] = {
      {LEG_LF, OVER, 0, true, "LF, roll toward the left"},
      {LEG_LF, TH, 0, false, "LF, roll at the threshold"},
      {LEG_LF, (int16_t)-OVER, 0, false, "LF, roll toward the right"},
      {LEG_LF, 0, OVER, true, "LF, nose dropping"},
      {LEG_LF, 0, (int16_t)-OVER, false, "LF, nose rising"},
      {LEG_RB, (int16_t)-OVER, 0, true, "RB, roll toward the right"},
      {LEG_RB, OVER, 0, false, "RB, roll toward the left"},
      {LEG_RB, 0, (int16_t)-OVER, true, "RB, nose rising"},
      {LEG_RB, 0, OVER, false, "RB, nose dropping"},
      {LEG_LB, OVER, 0, true, "LB, roll toward the left"},
      {LEG_RF, (int16_t)-OVER, 0, true, "RF, roll toward the right"},
  };
  int T = 0;
  for (const Case& c : CASES) {
    Snap ref = catch_at(F, c.leg, 0, 0), got = catch_at(F, c.leg, c.rr, c.rp);
    T = period_ticks();
    uint8_t bit = (uint8_t)(1 << c.leg);
    CHECKF(!(ref.down & bit) && ref.z[c.leg] < ref.h[c.leg] - 5, "%s: the leg is up at mid-swing", c.what);
    if (c.caught) {
      // Caught: disarmed for the rest of the swing, its catch kept; this tick
      // the foot is where its path was (z as without the rates, x between
      // last tick's and the path's), not down yet; the others as without the
      // rates.
      bool between = std::fabs(got.x[c.leg] - ref.x[c.leg]) <= std::fabs(ref.x[c.leg] - got.px) + 0.1;
      CHECKF(!(got.armed & bit) && b.gait.caught[c.leg] && !(got.down & bit) &&
                 std::fabs(got.z[c.leg] - ref.z[c.leg]) < 0.5 && between,
             "%s: down %x armed %x x %.2f (path %.2f, last %.2f) z %.2f (path %.2f)", c.what, got.down, got.armed,
             got.x[c.leg], ref.x[c.leg], got.px, got.z[c.leg], ref.z[c.leg]);
      CHECKF(same_legs(ref, got, c.leg), "%s: the other legs moved", c.what);
    } else {
      CHECKF(same_legs(ref, got) && got.armed == ref.armed, "%s: something changed", c.what);
    }
  }
  // One tick over the threshold (an impact, a servo jerk) is not a fall:
  // nothing. Nor is every other tick over it for a whole swing.
  for (int16_t r : {OVER, (int16_t)3000}) {
    Snap ref = catch_at(F, LEG_LF, 0, 0, false, 1), got = catch_at(F, LEG_LF, r, r, false, 1);
    CHECKF(same_legs(ref, got) && got.armed == ref.armed && !b.gait.caught[LEG_LF], "a one-tick spike %d", r);
  }
  start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, F));
  tick(2 * period_ticks());
  bool none = true;
  for (int t = 0; t < 2 * period_ticks(); t++) {
    in.att.rate_roll = (int16_t)((t & 1) ? 3000 : 0);
    tick();
    for (int l = 0; l < 4; l++) none = none && !b.gait.caught[l];
  }
  in.att.rate_roll = in.att.rate_pitch = 0;
  CHECK(none);
  // Caught at mid-swing (WALK, CRAWL): the foot holds its x for the rest of
  // the swing and comes down through the rest of the lift profile from the
  // catch's height (at CATCH_DESCENT x its pace), monotonic, never faster
  // than twice the swing's own touchdown; it is down (its bit) once there,
  // stays down to the end of the swing, and the next stance starts there:
  // no x step larger than a stance step, no joint step near the safety
  // slew; then it lifts again at its next swing.
  for (uint8_t pat : {KILO_GAIT_PATTERN_WALK, KILO_GAIT_PATTERN_CRAWL}) {
    // The swing's own z steps and the stance's x step, uncaught.
    catch_at(F, LEG_LF, 0, 0, false, 2, pat);
    double dz_swing = 0, dx_stance = 0, jref = 0, x0, z0, x1, z1;
    int16_t prev[DRIVEN_COUNT];
    memcpy(prev, b.arb.out, sizeof prev);
    foot(LEG_LF, &x0, &z0);
    for (int t = 0; t < period_ticks(); t++) {
      tick();
      jref = std::fmax(jref, max_step(prev));
      memcpy(prev, b.arb.out, sizeof prev);
      foot(LEG_LF, &x1, &z1);
      if (in_swing(LEG_LF)) dz_swing = std::fmax(dz_swing, std::fabs(z1 - z0));
      else dx_stance = std::fmax(dx_stance, std::fabs(x1 - x0));
      x0 = x1, z0 = z1;
    }
    Snap got = catch_at(F, LEG_LF, OVER, 0, false, 2, pat);
    CHECK(!(got.armed & 1) && b.gait.caught[LEG_LF]);
    memcpy(prev, b.arb.out, sizeof prev);
    double dx = 0, dz = 0, jstep = 0, xc = got.x[LEG_LF], zc = got.z[LEG_LF];
    bool mono = true, stays = true, lifted_again = false, landed = false;
    int phase = 0;  // 0 the caught swing, 1 the stance after it, 2 the next swing
    for (int t = 0; t < 2 * period_ticks() && phase < 2; t++) {
      tick();
      jstep = std::fmax(jstep, max_step(prev));
      memcpy(prev, b.arb.out, sizeof prev);
      double x, z;
      foot(LEG_LF, &x, &z);
      bool sw = in_swing(LEG_LF);
      if (phase == 0 && !sw) phase = 1;
      if (phase == 1 && sw) phase = 2;
      if (phase == 2) {
        lifted_again = !(b.gait.down & 1) && (b.gait.armed & 1);
        break;
      }
      dx = std::fmax(dx, std::fabs(x - xc));
      if (phase == 0) {
        if (!landed) {  // (then it follows H_i, which the sway moves)
          dz = std::fmax(dz, std::fabs(z - zc));
          mono = mono && z >= zc - 0.1;  // (z is down: it only grows)
        }
        landed = landed || (b.gait.down & 1);
        stays = stays && (!landed || (b.gait.down & 1));
      }
      xc = x, zc = z;
    }
    std::printf("gait: catch, pattern %d, LF caught at mid-swing: foot x steps at most %.2f mm/tick (a stance step "
                "%.2f), z %.2f (the swing's own %.2f); joints %.2f deg/tick (uncaught %.2f)\n",
                pat, dx, dx_stance, dz, dz_swing, jstep, jref);
    CHECKF(dx <= dx_stance + 0.1, "pattern %d: x steps %.2f", pat, dx);
    CHECKF(mono && dz <= 2 * dz_swing + 0.1 && landed && stays, "pattern %d: z steps %.2f, landed %d", pat, dz, landed);
    CHECK(jstep <= jref + 0.5 && jstep < PARAM(SLEW_MAX_A16) / 16.0);
    CHECK(lifted_again);
  }
  // The flag off, or a faulty IMU: nothing.
  CHECK(same_legs(catch_at(NOCATCH, LEG_LF, 0, 0), catch_at(NOCATCH, LEG_LF, OVER, OVER)));
  CHECK(same_legs(catch_at(F, LEG_LF, 0, 0, true), catch_at(F, LEG_LF, OVER, OVER, true)));
  // The stop (GS_STOP, then GS_LAND): never, whatever the rates.
  std::vector<int16_t> ref;
  for (int k = 0; k < 2; k++) {
    start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, F));
    tick(2 * T);
    CHECK(gait(gp(0, 0)) == GOAL_OK);
    std::vector<int16_t> legs;
    int t = 0, stop_ticks = 0, land_ticks = 0;
    while (body_gait_running(b) && t++ < 4 * T) {
      in.att.rate_roll = k ? (int16_t)((t & 1) ? 3000 : -3000) : 0;
      in.att.rate_pitch = k ? (int16_t)((t & 2) ? 3000 : -3000) : 0;
      tick();
      stop_ticks += b.gait.state == GS_STOP;
      land_ticks += b.gait.state == GS_LAND;
      for (int l = 0; l < 8; l++) legs.push_back(b.gait.leg[l]);
    }
    in.att.rate_roll = in.att.rate_pitch = 0;
    if (!k)
      ref = legs;
    else
      CHECKF(legs == ref && stop_ticks > 0 && land_ticks > 0, "stop: %d / %d ticks", stop_ticks, land_ticks);
  }
}

// The reflex against the plant. Normal walking never triggers it: every
// pattern with sway, the balance at Kp ~1, on a level floor and on a 5 deg
// slope (nose up, right side up, both), the plant's rates fed in. A shove
// on the trot's diagonal does: the leg on the side it falls toward lands,
// and the lean stays smaller than without the reflex.
static void test_catch_plant() {
  static const double SL[4][2] = {{0, 0}, {-5, 0}, {0, 5}, {-5, 5}};
  double worst = 0;
  for (uint8_t pat = 0; pat < 3; pat++)
    for (const auto& sl : SL) {
      SPlant s;
      walk_on(s, pat, 7 | KILO_GAIT_FLAGS_SWAY | KILO_GAIT_FLAGS_CATCH, sl[0], sl[1]);
      for (int t = 0; t < 6 * period_ticks(); t++) splant_step(s);
      std::printf("gait: catch, pattern %d on %+.0f/%+.0f deg: largest tipping rate toward a swinging leg %.1f deg/s, "
                  "%d catches\n", pat, sl[0], sl[1], s.max_tip, s.catches);
      CHECKF(s.catches == 0, "pattern %d slope %.0f/%.0f: %d catches", pat, sl[0], sl[1], s.catches);
      worst = std::fmax(worst, s.max_tip);
    }
  CHECK(worst < 0.8 * KILO_GAIT_TOUCHDOWN_RATE_DPS);
  // A shove on the diagonal, toward either side (+: roll rising, falling
  // left), at the swing's start. Three feet hold the body only if the caught
  // foot is on the side it falls toward (plant margin 3 deg): which leg is
  // caught is the sign. The TROT's swing is 150 ms, as long as the servo
  // lag, and the reflex waits for a second tick over its threshold: the leg
  // is caught late in its swing, about a tick before it would land anyway.
  // With the gait's own support the lean comes down by a fifth; with the
  // lagged support (the caught foot carries the body only once the leg is
  // there, and so does every touchdown) the result turns on how close counts
  // as there: at 3 mm the reflex changes nothing, at 2 mm the body falls
  // over without it and leans 39 deg with it. (Before the catch waited for
  // a second tick and lowered the foot through its lift profile, the foot
  // was down in one tick and supported at once: 15.5 -> 7.3 deg.)
  static const double GROUND[3] = {0, 3, 2};
  for (double ground : GROUND)
    for (double push : {60.0, -60.0}) {
      double lean[2];
      int catches = 0;
      for (int k = 0; k < 2; k++) {
        SPlant s;
        walk_on(s, KILO_GAIT_PATTERN_TROT, 7 | (k ? KILO_GAIT_FLAGS_CATCH : 0), 0, 0, BAL_KP1, 3, ground);
        for (int t = 0; t < 4 * period_ticks(); t++) splant_step(s);
        int w = 0;
        while (gait_feet_down(b.gait) != 4 && w++ < 200) splant_step(s);
        while (gait_feet_down(b.gait) == 4 && w++ < 400) splant_step(s);  // now on the diagonal
        CHECK(w < 400 && s.max_lean < 3);
        s.max_lean = 0;
        splant_step(s, push);
        for (int t = 0; t < period_ticks(); t++) splant_step(s);
        lean[k] = s.max_lean;
        if (k) {
          catches = s.catches;
          uint8_t side = push > 0 ? 0x9 : 0x6;  // left LF LB, right RF RB
          // (The snap back onto three feet can then catch the other swinging
          // leg as well.)
          CHECKF(s.catches >= 1 && (s.first & side) && !(s.first & ~side), "push %+.0f: %d catches, first legs %x",
                 push, s.catches, s.first);
        }
      }
      if (ground)
        std::printf("gait: catch, a %+.0f deg/s shove on the trot's diagonal, lagged support (%.0f mm): largest lean "
                    "%.1f deg without, %.1f with (%d catches)\n", push, ground, lean[0], lean[1], catches);
      else
        std::printf("gait: catch, a %+.0f deg/s shove on the trot's diagonal, the gait's support: largest lean %.1f "
                    "deg without, %.1f with (%d catches)\n", push, lean[0], lean[1], catches);
      CHECKF(ground ? lean[1] <= lean[0] + 0.05 : lean[1] < 0.85 * lean[0], "%.0f mm: %.1f / %.1f", ground, lean[0],
             lean[1]);
    }
}

// ---- Forward placement (flag PLACE) -----------------------------------------------------
// A WALK (the legs alone, no balance) to the tick before `leg` lifts, then
// that tick with the pitch rate `rp` (a16/s; optionally an IMU fault; on
// the `n` ticks ending there):
// every leg's foot x (mm) from that tick on for one period, the places
// after the lift-off tick and the support.
struct PlaceRun {
  std::vector<double> x[4];
  std::vector<uint8_t> dn;  // the support each tick
  int8_t place[4];
  uint8_t down;
};
static PlaceRun place_run(uint8_t flags, int leg, int16_t rp, bool fault = false, int n = 1) {
  start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, flags));
  int T = period_ticks();
  tick(2 * T);
  uint16_t lift = (uint16_t)(b.gait.duty << 8);
  int t = 0;
  while (!(leg_p(leg) + (n - 1) * b.gait.dphi < lift && leg_p(leg) + n * b.gait.dphi >= lift) && t++ < 400) tick();
  in.att.rate_pitch = rp;
  b.imu_fault = fault;
  tick(n);
  in.att.rate_pitch = 0;
  b.imu_fault = 0;
  PlaceRun r;
  memcpy(r.place, b.gait.place, sizeof r.place);
  r.down = b.gait.down;
  for (int k = 0; k < T; k++) {
    for (int l = 0; l < 4; l++) {
      double x, z;
      foot(l, &x, &z);
      r.x[l].push_back(x);
    }
    r.dn.push_back(b.gait.down);
    tick();
  }
  return r;
}

static void test_place() {
  const uint8_t F = KILO_GAIT_FLAGS_SWAY | KILO_GAIT_FLAGS_PLACE;
  // The model: K mm per rad/s of pitch rate, nose down (+) = forward (+x)
  // for a front leg and a back leg alike (gait.h: the centre of mass, h
  // above the feet, moves forward at h * rate; its capture point is tau *
  // that further ahead, and the landing foot must reach it).
  for (int leg : {LEG_LF, LEG_RB})
    for (int16_t rp : {(int16_t)160, (int16_t)-160, (int16_t)300}) {  // 10, -10, 18.75 deg/s
      PlaceRun ref = place_run(F, leg, 0), got = place_run(F, leg, rp);
      double want = KILO_GAIT_CAPTURE_MM_PER_RAD_S * rp / 16.0 * D2R;
      want = std::fmax(-KILO_GAIT_CAPTURE_MAX_MM, std::fmin(KILO_GAIT_CAPTURE_MAX_MM, want));
      double pl = got.place[leg] / 4.0;
      CHECKF(std::fabs(pl - want) <= 0.25 && (pl > 0) == (rp > 0), "%s rate %d: place %.2f mm, model %.2f", LEG_NAME[leg],
             rp, pl, want);
      CHECK(!(got.down & (1 << leg)));  // it lifted
      // The foot: no change at lift-off, the whole shift at touchdown, back
      // to the path by the next lift-off, never a jump; the other feet as
      // without the rate.
      int n = (int)ref.x[leg].size(), td = 1;
      while (td < n && !(got.dn[td] & (1 << leg))) td++;  // the touchdown tick
      double d0 = got.x[leg][0] - ref.x[leg][0], peak = 0, jump = 0, end = got.x[leg][n - 1] - ref.x[leg][n - 1];
      double at_td = got.x[leg][td] - ref.x[leg][td];
      for (int k = 0; k < n; k++) {
        double d = got.x[leg][k] - ref.x[leg][k];
        peak = std::fmax(peak, std::fabs(d));
        if (k) jump = std::fmax(jump, std::fabs(d - (got.x[leg][k - 1] - ref.x[leg][k - 1])));
      }
      bool others = true;
      for (int l = 0; l < 4; l++)
        for (int k = 0; l != leg && k < n; k++) others = others && got.x[l][k] == ref.x[l][k];
      std::printf("gait: place, %s lifting at %+.2f deg/s: shift %+.2f mm (model %+.2f); foot vs no shift: %+.2f at "
                  "lift-off, %+.2f at touchdown, %+.2f a period on, largest step %.2f mm/tick\n",
                  LEG_NAME[leg], rp / 16.0, pl, want, d0, at_td, end, jump);
      CHECK(std::fabs(d0) < 0.1 && std::fabs(at_td - pl) < 0.2 && std::fabs(end) < 0.1);
      CHECK(peak <= std::fabs(pl) + 0.2 && jump < std::fabs(pl) / 2 + 0.1);  // (+ the IK's round trip)
      CHECK(others);
    }
  // The clamp, both ways.
  CHECK(place_run(F, LEG_LF, 4000).place[LEG_LF] == GAIT_CAPTURE_MAX_Q2);
  CHECK(place_run(F, LEG_LF, -4000).place[LEG_LF] == -GAIT_CAPTURE_MAX_Q2);
  // Nothing with the flag off or with a faulty IMU.
  for (bool fault : {false, true}) {
    PlaceRun ref = place_run(fault ? F : KILO_GAIT_FLAGS_SWAY, LEG_LF, 0);
    PlaceRun got = place_run(fault ? F : KILO_GAIT_FLAGS_SWAY, LEG_LF, 300, fault);
    CHECKF(got.place[LEG_LF] == 0 && got.x[LEG_LF] == ref.x[LEG_LF], "fault %d: place %d", fault, got.place[LEG_LF]);
  }
  // A leg caught at its lift-off (CATCH: tipping toward a front leg, the
  // tick before and this one) lands where it is: no shift, down within a
  // tick or two (it had lifted by a hair).
  PlaceRun c = place_run(F | KILO_GAIT_FLAGS_CATCH, LEG_LF, (int16_t)(GAIT_TOUCHDOWN_A16S + 16), false, 2);
  CHECKF(c.place[LEG_LF] == 0 && (c.dn[1] & 1), "place %d down %x %x", c.place[LEG_LF], c.down, c.dn[1]);
  // Caught later in its swing, a placed leg keeps its shift (the foot goes
  // down where its path is, no jump) until its next lift-off.
  start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, F | KILO_GAIT_FLAGS_CATCH));
  tick(2 * period_ticks());
  uint16_t lift = (uint16_t)(b.gait.duty << 8);
  int guard = 0;
  while (!(leg_p(LEG_LF) < lift && leg_p(LEG_LF) + b.gait.dphi >= lift) && guard++ < 400) tick();
  in.att.rate_pitch = 300;
  tick();
  in.att.rate_pitch = 0;
  int8_t pl0 = b.gait.place[LEG_LF];
  bool kept = true;
  double xa, za, xb, jmp = 0;
  tick(2);
  in.att.rate_pitch = (int16_t)(GAIT_TOUCHDOWN_A16S + 16);
  tick();
  foot(LEG_LF, &xa, &za);
  tick();
  in.att.rate_pitch = 0;
  CHECK(b.gait.caught[LEG_LF] && !(b.gait.armed & 1));  // caught
  while (in_swing(LEG_LF) && guard++ < 400) {
    foot(LEG_LF, &xb, &za);
    jmp = std::fmax(jmp, std::fabs(xb - xa));
    xa = xb;
    kept = kept && b.gait.place[LEG_LF] == pl0;
    tick();
  }
  CHECKF(kept && pl0 != 0 && jmp < 2.5, "caught mid-swing: place %d -> %d, step %.2f mm", pl0, b.gait.place[LEG_LF], jmp);
  // A stop before the start ramp has finished (shorter than a period, so
  // some legs never lift during it, keeping their shifts): the shifts scale
  // with the ramp, so at its end (amp 0) the feet are on STANCE's x and the
  // hand-over moves nothing; and none outlives the gait.
  start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, F));
  in.att.rate_pitch = 300;  // every lift-off takes +2.25 mm
  tick(period_ticks() / 2);
  in.att.rate_pitch = 0;
  int8_t worst = 0;
  for (int l = 0; l < 4; l++) worst = std::max(worst, (int8_t)std::abs(b.gait.place[l]));
  CHECK(b.gait.amp < 65535 && worst > 0);
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  double xg[4], xs[4], zz;
  int g2 = 0;
  while (body_gait_running(b) && g2++ < 400) {
    for (int l = 0; l < 4; l++) foot(l, &xg[l], &zz);
    tick();
  }
  for (int l = 0; l < 4; l++) foot(l, &xs[l], &zz);  // STANCE's feet, the tick the gait ended
  double hand = 0;
  bool cleared = true;
  for (int l = 0; l < 4; l++) {
    hand = std::fmax(hand, std::fabs(xs[l] - xg[l]));
    cleared = cleared && b.gait.place[l] == 0 && b.gait.caught[l] == 0;
  }
  std::printf("gait: place, a stop half a period after the start (shifts up to %.2f mm): x at the hand-over moves "
              "%.2f mm\n", worst / 4.0, hand);
  CHECKF(hand < 0.15 && cleared, "hand-over %.2f mm, cleared %d", hand, cleared);
  // The stop (GS_STOP, GS_LAND): never, whatever the rate.
  start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, F));
  tick(2 * period_ticks());
  CHECK(gait(gp(0, 0)) == GOAL_OK);
  bool zero = true;
  int t = 0, stop_ticks = 0;
  in.att.rate_pitch = 1000;
  while (body_gait_running(b) && t++ < 400) {
    tick();
    stop_ticks += b.gait.state == GS_STOP;
    for (int l = 0; l < 4; l++) zero = zero && b.gait.place[l] == 0;
  }
  in.att.rate_pitch = 0;
  CHECK(zero && stop_ticks > 0);
}

// No false action: every pattern with sway, catch and placement, the
// balance at Kp ~1, on a level floor and on 5 deg slopes, the stance-aware
// plant's rates fed in: the shifts normal walking makes stay small.
static void test_place_quiet() {
  static const double SL[4][2] = {{0, 0}, {-5, 0}, {0, 5}, {-5, 5}};
  for (uint8_t pat = 0; pat < 3; pat++) {
    int worst = 0;
    for (const auto& sl : SL) {
      SPlant s;
      walk_on(s, pat, 7 | KILO_GAIT_FLAGS_SWAY | KILO_GAIT_FLAGS_CATCH | KILO_GAIT_FLAGS_PLACE, sl[0], sl[1]);
      for (int t = 0; t < 6 * period_ticks(); t++) {
        splant_step(s);
        for (int l = 0; l < 4; l++) worst = std::max(worst, std::abs(b.gait.place[l]));
      }
      CHECK(s.catches == 0);
    }
    std::printf("gait: place, pattern %d walking on a level floor and 5 deg slopes: largest shift %.2f mm\n", pat,
                worst / 4.0);
    CHECKF(worst < 8, "pattern %d: %.2f mm", pat, worst / 4.0);  // 2 mm
  }
}

// The capture plant: the stance-aware plant's diagonal fall in two axes,
// with the feet where they are, for a fore-aft shove. The centre of mass
// (CoM) is h = 80 mm up; c = its ground projection relative to the
// shoulders' centre (mm, x forward, y left), c = h (sin pitch, sin roll)
// (nose down = +pitch moves it forward, right side up = +roll moves it
// left, §3f). The down feet (the gait's `down`) stand at (+-Lb/2 + x_i,
// +-W/2), x_i the commanded foot x. A linear inverted pendulum: c'' = (c -
// p) / tau^2, tau = 90 ms, with the centre of pressure p anywhere in the
// convex hull of the down feet (a line on a diagonal pair) and nowhere else.
// The legs aim p to bring c to the feet's attitude (as the stance-aware
// plant: their planes through the 150 ms servo lag) in SP_LAND (2) ticks,
// critically damped: a rigid stance stops a shove at once, as long as p
// stays in the hull; where it cannot (two feet; three with the CoM across
// the triangle's edge) the CoM falls about the nearest edge until a landing
// foot makes a hull under it. The trot rocks within 1.3 deg of roll here
// (the floor: 3.5) and the walk follows its sway.
struct CPlant {
  double c[2], v[2], cmd[2];  // cmd: the feet's attitude as a CoM offset, through the servo lag
  double max_p, max_r;        // largest |pitch|, |roll| (deg)
};
static const double CP_H = 80.0;
// The point of the hull of the down feet nearest q (feet in the cyclic order
// LF RF RB LB: a convex polygon).
static void hull_nearest(uint8_t down, const double q[2], double out[2]) {
  double px[4], py[4];
  int n = 0;
  for (int l = 0; l < 4; l++) {
    if (!(down & (1 << l))) continue;
    double x, z;
    out_foot(l, &x, &z);
    px[n] = (leg_front((uint8_t)l) ? 0.5 : -0.5) * PARAM(LEG_LB_MM) + x;
    py[n++] = (leg_left((uint8_t)l) ? 0.5 : -0.5) * PARAM(LEG_W_MM);
  }
  out[0] = q[0], out[1] = q[1];
  if (n == 0) return;
  if (n >= 3) {  // inside: on the same side of every edge
    int pos = 0, neg = 0;
    for (int i = 0; i < n; i++) {
      int j = (i + 1) % n;
      double cr = (px[j] - px[i]) * (q[1] - py[i]) - (py[j] - py[i]) * (q[0] - px[i]);
      pos += cr >= 0;
      neg += cr <= 0;
    }
    if (pos == n || neg == n) return;
  }
  double best = 1e18;
  for (int i = 0; i < n; i++) {
    int j = (i + 1) % n;
    double ex = px[j] - px[i], ey = py[j] - py[i], l2 = ex * ex + ey * ey;
    double t = l2 > 0 ? std::fmax(0.0, std::fmin(1.0, ((q[0] - px[i]) * ex + (q[1] - py[i]) * ey) / l2)) : 0;
    double nx = px[i] + t * ex, ny = py[i] + t * ey, d = (q[0] - nx) * (q[0] - nx) + (q[1] - ny) * (q[1] - ny);
    if (d < best) best = d, out[0] = nx, out[1] = ny;
  }
}
// One tick: the plant answers the last tick's commands, then body_tick.
// shove: deg/s of nose-down pitch rate added (h * rate to the CoM).
static void cplant_step(CPlant& s, double shove = 0) {
  SPlant sp = SPlant();
  sp.margin = 90;
  sp.sup = b.gait.down;  // (this plant's support is the gait's own: its hull below)
  double tp, tr, as = 1 - std::exp(-1 / SP_SERVO);
  splant_support(sp, &tp, &tr);  // the feet's attitude (on a diagonal: the pair's pitch, roll 0)
  s.cmd[0] += (CP_H * std::sin(tp * D2R) - s.cmd[0]) * as;
  s.cmd[1] += (CP_H * std::sin(tr * D2R) - s.cmd[1]) * as;
  const double w = 50.0 / SP_LAND, kv = 2 * SP_TAU * SP_TAU * w, kc = SP_TAU * SP_TAU * w * w;
  double p0 = std::asin(s.c[0] / CP_H) / D2R, r0 = std::asin(s.c[1] / CP_H) / D2R;
  s.v[0] += CP_H * shove * D2R;
  const int N = 20;
  const double dt = 0.02 / N;
  for (int k = 0; k < N; k++) {
    double q[2], p[2];
    for (int a = 0; a < 2; a++) q[a] = s.c[a] + kv * s.v[a] + kc * (s.c[a] - s.cmd[a]);
    hull_nearest(b.gait.down, q, p);
    for (int a = 0; a < 2; a++) {
      s.v[a] += (s.c[a] - p[a]) / (SP_TAU * SP_TAU) * dt;
      s.c[a] += s.v[a] * dt;
    }
  }
  double pp = std::asin(std::fmax(-1.0, std::fmin(1.0, s.c[0] / CP_H))) / D2R;
  double rr = std::asin(std::fmax(-1.0, std::fmin(1.0, s.c[1] / CP_H))) / D2R;
  s.max_p = std::fmax(s.max_p, std::fabs(pp));
  s.max_r = std::fmax(s.max_r, std::fabs(rr));
  in.att.pitch = (int16_t)std::lround(pp * 16);
  in.att.roll = (int16_t)std::lround(rr * 16);
  in.att.rate_pitch = (int16_t)std::lround((pp - p0) * 50 * 16);
  in.att.rate_roll = (int16_t)std::lround((rr - r0) * 50 * 16);
  tick();
}
static void walk_on_c(CPlant& s, uint8_t pat, uint8_t flags) {
  static const int8_t STRIDE[3] = {30, 25, 25};
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 1, 0) == GOAL_OK);
  tick(5);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, 0) == GOAL_OK);
  tick(30);
  CHECK(body_goal_balance(b, BAL_KP1) == GOAL_OK);
  s = CPlant();
  for (int t = 0; t < 100; t++) cplant_step(s);
  CHECK(gait(gp(1, pat, 0, 0, STRIDE[pat], 0, -1, 0, flags)) == GOAL_OK);
  int t = 0;
  while (!body_gait_running(b) && t++ < 100) cplant_step(s);
  CHECK(body_gait_running(b));
}

// Fore-aft shoves (+: nose down) on the WALK and the TROT as the mind sends
// them, at 8 phases of the cycle; for each the same run three times from the
// shove on: no placement, placement, placement with its sign flipped (the
// shift negated at each lift-off). The peak pitch and roll over two
// periods, averaged over the phases. Finding: a rigid stance on three or
// four feet stops a shove within its 2-tick settle, before any foot lifts,
// so a lift-off never sees the shove's rate and where the foot lands does
// not change the peak: pitch within 0.1 deg with placement, without and
// flipped alike, roll up to 0.25 deg more on the trot with either sign (its
// own rocking moves the feet by up to a millimetre). The capture point
// needs a stance that lets the body keep its momentum, which Kilo's
// position-controlled legs do not; so this only guards that placement does
// no harm to the pitch.
// The 120 deg/s shoves also catch a swinging leg (CATCH is on), and this
// plant puts a foot in the support on the tick its down bit is set. Since
// the catch waits for a second tick over its threshold and lowers the foot
// through its lift profile (2-4 ticks; down once there) they roll further:
// WALK +120 to 31 deg (11.6 before), TROT -120 to 15.0 (4.9): the 60 deg/s
// ones still stay up.
static void test_place_plant() {
  for (uint8_t pat : {KILO_GAIT_PATTERN_WALK, KILO_GAIT_PATTERN_TROT}) {
    uint8_t f = (uint8_t)(7 | KILO_GAIT_FLAGS_CATCH | (pat == KILO_GAIT_PATTERN_WALK ? KILO_GAIT_FLAGS_SWAY : 0));
    for (int k = 0; k < 2; k++) {
      CPlant s;
      walk_on_c(s, pat, (uint8_t)(f | (k ? KILO_GAIT_FLAGS_PLACE : 0)));
      s.max_p = s.max_r = 0;
      for (int t = 0; t < 6 * period_ticks(); t++) cplant_step(s);
      std::printf("gait: capture plant, pattern %d %s placement: rocks within pitch %.2f roll %.2f deg\n", pat,
                  k ? "with" : "without", s.max_p, s.max_r);
      CHECK(s.max_r < 15 && s.max_p < 5);  // walks (the WALK's sway leans 12 deg)
    }
    for (double shove : {60.0, -60.0, 120.0, -120.0}) {
      double pk[3] = {0, 0, 0}, rk[3] = {0, 0, 0};
      const int PH = 8;
      for (int ph = 0; ph < PH; ph++)
        for (int k = 0; k < 3; k++) {
          CPlant s;
          walk_on_c(s, pat, f);
          int T = period_ticks();
          for (int t = 0; t < 4 * T + ph * T / PH; t++) cplant_step(s);
          if (k) b.gait.flags |= KILO_GAIT_FLAGS_PLACE;  // the same run up to the shove
          s.max_p = s.max_r = 0;
          cplant_step(s, shove);
          for (int t = 0; t < 2 * T; t++) {
            uint8_t d0 = b.gait.down;
            cplant_step(s);
            if (k == 2)
              for (int l = 0; l < 4; l++)
                if ((d0 & (1 << l)) && !(b.gait.down & (1 << l))) b.gait.place[l] = (int8_t)-b.gait.place[l];
          }
          pk[k] += s.max_p / PH;
          rk[k] += s.max_r / PH;
        }
      std::printf("gait: capture plant, pattern %d, a %+.0f deg/s pitch shove: peak pitch %.2f / %.2f / %.2f deg, roll "
                  "%.2f / %.2f / %.2f deg (no placement / placement / flipped)\n",
                  pat, shove, pk[0], pk[1], pk[2], rk[0], rk[1], rk[2]);
      if (std::fabs(shove) < 100) CHECK(pk[0] < 10 && rk[0] < 15);  // it stays up
      CHECK(pk[0] < 10);
      CHECKF(std::fabs(pk[1] - pk[0]) < 0.25, "pattern %d shove %.0f: %.2f vs %.2f", pat, shove, pk[1], pk[0]);
    }
  }
}

// ---- Output steps at the defaults and the per-tick cost --------------------------
static void test_defaults_and_cost() {
  static const int8_t STRIDE[3] = {30, 25, 25};  // spec [gait_patterns.*] stride_mm (the mind sends them)
  // With and without SWAY (its lean adds leg-length motion), CATCH and PLACE
  // (the mind's defaults; neither acts on a level, still IMU).
  for (int k = 0; k < 6; k++) {
    uint8_t pat = (uint8_t)(k % 3),
            flags = (uint8_t)(k < 3 ? 7 : 7 | KILO_GAIT_FLAGS_SWAY | KILO_GAIT_FLAGS_CATCH | KILO_GAIT_FLAGS_PLACE);
    start(gp(1, pat, 0, 0, STRIDE[pat], 0, -1, 0, flags));
    int T = period_ticks();
    tick(2 * T);
    int16_t prev[DRIVEN_COUNT];
    double worst = 0;
    for (int t = 0; t < 2 * T; t++) {
      memcpy(prev, b.arb.out, sizeof prev);
      tick();
      worst = std::fmax(worst, max_step(prev));
    }
    // The gait's own angles (not the slewed output), to see the slew never
    // had to clip them.
    bool same = true;
    for (int l = 0; l < 8; l++) same = same && b.arb.out[3 + l] == b.gait.leg[l];
    std::printf("gait: pattern %d defaults, flags %d: max leg step %.2f deg/tick\n", pat, flags, worst);
    CHECK(same);
    CHECKF(worst < PARAM(SLEW_MAX_A16) / 16.0, "pattern %d flags %d: %.2f deg/tick hits the safety slew", pat, flags,
           worst);
  }
  // Placement at its clamp on every step (a pitch rate past it, held; no
  // CATCH, which such a rate would fire, and no BALANCE, whose rate term it
  // would drive): the swing's x travel grows by the clamp. The TROT, already
  // at 9.62 deg/tick, sets the clamp: +3 mm makes 9.88, +4 mm and more hit
  // the 10 deg/tick safety slew (the WALK and CRAWL would take 15 mm).
  for (int k = 0; k < 6; k++) {
    uint8_t pat = (uint8_t)(k % 3);
    start(gp(1, pat, 0, 0, STRIDE[pat], 0, -1, 0, 3 | KILO_GAIT_FLAGS_SWAY | KILO_GAIT_FLAGS_PLACE));
    int T = period_ticks();
    in.att.rate_pitch = (int16_t)(k < 3 ? 4000 : -4000);
    tick(2 * T);
    int16_t prev[DRIVEN_COUNT];
    double worst = 0;
    int pl = 0;
    for (int t = 0; t < 2 * T; t++) {
      memcpy(prev, b.arb.out, sizeof prev);
      tick();
      worst = std::fmax(worst, max_step(prev));
      for (int l = 0; l < 4; l++) pl = std::max(pl, std::abs(b.gait.place[l]));
    }
    in.att.rate_pitch = 0;
    std::printf("gait: pattern %d defaults, placement %+.1f mm on every step: max leg step %.2f deg/tick\n", pat,
                (k < 3 ? 1 : -1) * pl / 4.0, worst);
    CHECK(pl == GAIT_CAPTURE_MAX_Q2);
    CHECKF(worst < PARAM(SLEW_MAX_A16) / 16.0, "pattern %d placement: %.2f deg/tick hits the safety slew", pat, worst);
  }
  // Cost: fx calls in one walking tick (4 legs of IK + the trajectory) vs a
  // STAND tick.
  start(gp(1, KILO_GAIT_PATTERN_TROT));
  tick(30);
  FxOps a = g_fx_ops;
  tick();
  FxOps c = g_fx_ops;
  uint32_t sn = c.sin - a.sin, at = c.atan2 - a.atan2, sq = c.isqrt - a.isqrt;
  std::printf("gait: one walking tick: %u sin/cos, %u atan2, %u isqrt (+ 4 IK and 2 trajectory int32 divisions)\n",
              sn, at, sq);
  CHECK(at == 12 && sq == 4 && sn <= 16);
  // With SWAY: at most one cosine more per leg (the window; legs outside it
  // cost nothing).
  start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, 7 | KILO_GAIT_FLAGS_SWAY | KILO_GAIT_FLAGS_PLACE));
  tick(30);
  in.att.rate_pitch = 300;  // placement: no fx call (a multiply per leg)
  a = g_fx_ops;
  tick();
  c = g_fx_ops;
  in.att.rate_pitch = 0;
  uint32_t sn2 = c.sin - a.sin;
  std::printf("gait: one walking tick with sway and placement: %u sin/cos, %u atan2\n", sn2, c.atan2 - a.atan2);
  CHECK(c.atan2 - a.atan2 == 12 && c.isqrt - a.isqrt == 4 && sn2 <= 16 + 4);
}

int main() {
  test_swing_order();
  test_trajectory();
  test_start_stop();
  test_pattern_change();
  test_turn();
  test_period_change();
  test_bob_wag();
  test_refusals();
  test_pending();
  test_preempt();
  test_stance_pose();
  test_geometric_balance();
  test_telemetry();
  test_support();
  test_sway();
  test_q14_extremes();
  test_plan_reference();
  test_sway_lead();
  test_two_foot_hold();
  test_catch();
  test_catch_plant();
  test_place();
  test_place_quiet();
  test_place_plant();
  test_defaults_and_cost();
  return report("gait");
}
