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
// and the per-tick cost.
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
  CHECK(gait(gp(1, 0, 0, 0, 0, 0, -1, 0, 32)) == GOAL_BAD_ARG);  // bit 5: unknown (bit 4 is CATCH)
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
// cosine centred on mid-swing, half-width min(1.5 swings, half a cycle);
// plan_roll = A_roll sum side_i w_i, plan_pitch = A_pitch sum end_i w_i,
// times the ramp. a16.
static void sway_ref(double* roll, double* pitch, double* w_out = nullptr) {
  double sw = (256 - b.gait.duty) / 256.0, hw = std::fmin(1.5 * sw, 0.5);
  uint16_t mid = (uint16_t)(32768u + (b.gait.duty << 7));
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
    int best = 70000;
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
      // LF's mid-swing: the body on RF, RB, LB.
      int16_t d = (int16_t)(leg_p(LEG_LF) - (uint16_t)(32768u + (b.gait.duty << 7)));
      if (std::abs(d) < best) {
        best = std::abs(d);
        stance_attitude_lf_up(&lean_p, &lean_r);
        plan_p = b.gait.plan_pitch / 16.0;
        plan_r = b.gait.plan_roll / 16.0;
        down_mid = b.gait.down;
      }
    }
    std::printf("gait: sway pattern %d duty %d: plan vs model %.2f a16; peak roll %.2f pitch %.2f deg; at LF mid-swing "
                "the stance feet lean the body pitch %+.2f roll %+.2f deg (plan %+.2f %+.2f)\n",
                c.pat, b.gait.duty, err, peak_r / 16, peak_p / 16, lean_p, lean_r, plan_p, plan_r);
    // (The model has w_i = 0 at leg i's mid-stance and 1 at its mid-swing,
    // so following it every tick, ramp included, is the window's shape and
    // the fade-in.)
    CHECK(err <= 3);  // 0.2 deg: the fixed-point window and ramp truncate
    CHECK(down_mid == 0xE);  // LF up, the other three down
    // LF lifting: lean RIGHT (right side down: negative roll) and not
    // forward (back down or level: pitch <= 0), read off the three stance
    // feet; the roll is the full amplitude at mid-swing (the two
    // neighbours' windows are equal and their sides cancel).
    CHECKF(std::fabs(lean_r + KILO_GAIT_SWAY_ROLL_DEG) < 0.5 && lean_p < 0.3, "LF up: pitch %.2f roll %.2f", lean_p,
           lean_r);
    CHECK(std::fabs(lean_r - plan_r) < 0.3 && std::fabs(lean_p - plan_p) < 0.3);
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

// ---- The stance-aware plant ----------------------------------------------------------
// run_plant tilts the body by all four commanded feet, lifted ones included:
// right for STANCE, not for a walk (a foot in the air carries nothing). This
// one follows docs/walking-and-balance.md §2. The support is the gait's own
// `down` (bits LF RF RB LB), the feet the arbiter's output (this tick's
// command):
//  - on four feet the body lies on their plane (least squares), on three on
//    the plane through them, on the board (`slope`), reached through the
//    servo lag (tau 0.15 s = 7.5 ticks, as run_plant);
//  - on a diagonal pair, or on three feet while the body leans toward the
//    missing corner by more than `margin`, it is an inverted pendulum about
//    the diagonal: the world roll (the free axis; a rotation about the
//    123 x 65 mm diagonal is 88% roll) grows with tau = sqrt(h/g) = 90 ms,
//    integrated exactly per tick, the pitch follows the two diagonal feet;
//  - back on a support after that, it settles onto the feet's plane with
//    the landing lag SP_LAND ticks (the feet stop the rock: an impact, not
//    a servo move) until within 0.5 deg of it;
//  - the IMU reads the world attitude and its rates (differences x 50 Hz).
static const double SP_TAU = 0.09, SP_SERVO = 7.5, SP_LAND = 2.0;
struct SPlant {
  double p, r, vp, vr, slope_p, slope_r;
  double margin;    // three feet: the lean toward the missing corner that tips the body (deg; 90 = never)
  bool rocking;     // fell and has not settled on the feet yet
  double max_tip;   // largest roll or pitch rate toward a swinging leg (deg/s)
  double max_lean;  // largest |world roll| (deg)
  int catches;      // legs that came down in their swing (armed before, down now)
  uint8_t first;    // bits of the legs of the first catch
};
// The attitude the down feet give (deg, on the board), and whether the body
// tips instead (then only the pitch is the feet's).
static bool splant_support(const SPlant& s, double* tp, double* tr) {
  double lb = PARAM(LEG_LB_MM), w = PARAM(LEG_W_MM), x, z[4];
  for (int l = 0; l < 4; l++) out_foot(l, &x, &z[l]);
  uint8_t d = b.gait.down;
  // Pitch of a front/back pair (back longer = nose down = +), roll of a
  // left/right pair (right longer = right side up = +).
  auto pitch2 = [&](int f, int k) { return std::asin((z[k] - z[f]) / lb) / D2R; };
  auto roll2 = [&](int lt, int rt) { return std::asin((z[rt] - z[lt]) / w) / D2R; };
  *tr = 0;
  switch (gait_feet_down(b.gait)) {
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
static void splant_init(SPlant& s, double slope_p, double slope_r, double margin = 90) {
  s = SPlant();
  s.margin = margin;
  s.slope_p = slope_p;
  s.slope_r = slope_r;
  double tp, tr;
  splant_support(s, &tp, &tr);
  s.p = slope_p + tp;
  s.r = slope_r + tr;
}
// One tick: the plant answers the last tick's commands, then body_tick.
// push: deg/s added to the roll rate (a shove from the side).
static void splant_step(SPlant& s, double push = 0) {
  double tp, tr, p0 = s.p, r0 = s.r;
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
  uint8_t d = b.gait.down, armed = b.gait.armed;
  for (int l = 0; l < 4; l++)
    if (b.gait.state == GS_RUN && !(d & (1 << l))) {
      double tr_l = leg_left((uint8_t)l) ? s.vr : -s.vr, tp_l = leg_front((uint8_t)l) ? s.vp : -s.vp;
      s.max_tip = std::fmax(s.max_tip, std::fmax(tr_l, tp_l));
    }
  tick();
  uint8_t c = 0;
  for (int l = 0; l < 4; l++)
    if (b.gait.state == GS_RUN && in_swing(l) && (armed & (1 << l)) && (b.gait.down & (1 << l))) c |= (uint8_t)(1 << l);
  if (c && !s.catches++) s.first = c;
}

// STANCE on the board with the balance on (Kp ~1, Kd 30 ms, dead band 2 deg),
// levelled there first, then the gait, the plant running throughout.
static const uint8_t BAL_KP1[5] = {1, 255, 30, 2, 15};
static void walk_on(SPlant& s, uint8_t pat, uint8_t flags, double sp, double sr, const uint8_t* bal = BAL_KP1,
                    double margin = 90) {
  static const int8_t STRIDE[3] = {30, 25, 25};  // spec [gait_patterns.*] stride_mm
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 1, 0) == GOAL_OK);
  tick(5);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STANCE, 0, 0) == GOAL_OK);
  tick(30);
  CHECK(body_goal_balance(b, bal) == GOAL_OK);
  splant_init(s, sp, sr, margin);
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

// ---- The two-foot hold -------------------------------------------------------------------
// TROT on a board tilted 10 deg nose up and 10 deg right side up, Kp ~1. On
// fewer than three feet (the support the balance reads: the last tick's)
// the corrections do not change, and not for BAL_HOLD_TICKS after; then they
// move again, and the body is level within the dead band at every lift-off.
// The hold-over was measured with this plant (landing lag 2 ticks): the
// largest lean per step on a 5 deg slope at Kp 0.5 is 12.9 deg with no
// hold-over, 12.0 with 1 tick, 11.65 with 2 and 11.6 with 3-6 (and with no
// hold at all the corrections swing with every step: roll -7.3..-2.4 deg on
// 5 deg instead of a steady -3.3).
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
// A WALK (no balance: the legs alone) to `leg`'s mid-swing, then one tick
// with the body rates `rr`, `rp` (a16/s), optionally an IMU fault; the legs
// and the support after it.
struct Snap {
  int16_t leg[8];
  uint8_t down, armed;
  double z[4], h[4];  // each foot's z and its stance height H_i (mm)
};
static Snap catch_at(uint8_t flags, int leg, int16_t rr, int16_t rp, bool fault = false) {
  start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, flags));
  tick(2 * period_ticks());
  uint16_t mid = (uint16_t)(32768u + (b.gait.duty << 7));
  int t = 0;
  while (std::abs((int16_t)(leg_p(leg) - mid)) > b.gait.dphi / 2 && t++ < 200) tick();
  in.att.rate_roll = rr;
  in.att.rate_pitch = rp;
  b.imu_fault = fault;
  tick();
  in.att.rate_roll = in.att.rate_pitch = 0;
  b.imu_fault = 0;
  Snap s;
  memcpy(s.leg, b.gait.leg, sizeof s.leg);
  s.down = b.gait.down;
  s.armed = b.gait.armed;
  double lb = PARAM(LEG_LB_MM), w = PARAM(LEG_W_MM), x;
  double pitch = (b.gait.pose[0] + b.gait.plan_pitch) / 16.0 * D2R, roll = (b.gait.pose[1] + b.gait.plan_roll) / 16.0 * D2R;
  for (int l = 0; l < 4; l++) {
    foot(l, &x, &s.z[l]);
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
      // Down at once: on the ground at its stance height, its down bit set,
      // disarmed for the rest of the swing; the others as without the rates.
      CHECKF((got.down & bit) && !(got.armed & bit) && std::fabs(got.z[c.leg] - got.h[c.leg]) < 0.3,
             "%s: down %x armed %x z %.2f H %.2f", c.what, got.down, got.armed, got.z[c.leg], got.h[c.leg]);
      CHECKF(same_legs(ref, got, c.leg), "%s: the other legs moved", c.what);
    } else {
      CHECKF(same_legs(ref, got), "%s: something changed", c.what);
    }
  }
  // Caught, it stays down for the rest of the swing (the rates gone), and
  // lifts again at its next swing.
  Snap got = catch_at(F, LEG_LF, OVER, 0);
  bool down = true, lifted_again = false;
  for (int t = 0; t < T; t++) {
    tick();
    if (!in_swing(LEG_LF)) continue;
    if (b.gait.armed & 1)
      lifted_again = lifted_again || !(b.gait.down & 1);
    else
      down = down && (b.gait.down & 1) && !lifted(LEG_LF, 0.3);
  }
  CHECK((got.down & 1) && down && lifted_again);
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
  // left). Three feet hold the body only if the caught foot is on the side
  // it falls toward (plant margin 3 deg): which leg lands is the sign.
  for (double push : {60.0, -60.0}) {
    double lean[2];
    for (int k = 0; k < 2; k++) {
      SPlant s;
      walk_on(s, KILO_GAIT_PATTERN_TROT, 7 | (k ? KILO_GAIT_FLAGS_CATCH : 0), 0, 0, BAL_KP1, 3);
      for (int t = 0; t < 4 * period_ticks(); t++) splant_step(s);
      while (gait_feet_down(b.gait) != 4) splant_step(s);
      while (gait_feet_down(b.gait) == 4) splant_step(s);  // now on the diagonal
      s.max_lean = 0;
      splant_step(s, push);
      for (int t = 0; t < period_ticks(); t++) splant_step(s);
      lean[k] = s.max_lean;
      if (k) {
        uint8_t side = push > 0 ? 0x9 : 0x6;  // left LF LB, right RF RB
        // (The snap back onto three feet can then catch the other swinging
        // leg as well: the plant lands the rock in 2 ticks.)
        CHECKF(s.catches >= 1 && (s.first & side) && !(s.first & ~side), "push %+.0f: %d catches, first legs %x",
               push, s.catches, s.first);
      }
    }
    std::printf("gait: catch, a %+.0f deg/s shove on the trot's diagonal: largest lean %.1f deg without, %.1f with\n",
                push, lean[0], lean[1]);
    CHECK(lean[1] < 0.6 * lean[0]);
  }
}

// ---- Output steps at the defaults and the per-tick cost --------------------------
static void test_defaults_and_cost() {
  static const int8_t STRIDE[3] = {30, 25, 25};  // spec [gait_patterns.*] stride_mm (the mind sends them)
  // With and without SWAY (its lean adds leg-length motion) and CATCH (the
  // mind's defaults; it never fires on a level IMU).
  for (int k = 0; k < 6; k++) {
    uint8_t pat = (uint8_t)(k % 3), flags = (uint8_t)(k < 3 ? 7 : 7 | KILO_GAIT_FLAGS_SWAY | KILO_GAIT_FLAGS_CATCH);
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
  start(gp(1, KILO_GAIT_PATTERN_WALK, 0, 0, 30, 0, -1, 0, 7 | KILO_GAIT_FLAGS_SWAY));
  tick(30);
  a = g_fx_ops;
  tick();
  c = g_fx_ops;
  uint32_t sn2 = c.sin - a.sin;
  std::printf("gait: one walking tick with sway: %u sin/cos, %u atan2\n", sn2, c.atan2 - a.atan2);
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
  test_plan_reference();
  test_two_foot_hold();
  test_catch();
  test_catch_plant();
  test_defaults_and_cost();
  return report("gait");
}
