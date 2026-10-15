// The FEET player, STANCE and the body pose (core/feet.*, doc §3d), as wired
// into body_tick, the arbiter and the link, and the balance in GAIT (doc
// §3f) against a plant that tips (test/tip_plant.h). Every FEET goes
// through the real link path (COBS, CRC, dispatch) and every tick runs as
// app/main.cpp's run_tick: link_poll, body_tick, link_poll, link_tick.
//   STANCE: the IK posture, the pose terms, the foot nudge; the geometric
//     balance closed through a four-foot plant (slopes, a flipped sensor,
//     a commanded pose as the target).
//   The queue: tick matching, late and too-far ticks dropped and counted
//     (STATS feet_late), a repeat replacing its tick, no ACK, refusals.
//   Playing: the IK of the pose, a lost tick held, a wild pose slew-limited.
//   Underrun: FEET_UNDERRUN, the hold, the settle, GAIT_STOPPED 1; a FEET
//     during the hold resumes.
//   The end: SET_POSTURE, an epoch bump (a fall), a reflex: GAIT_STOPPED 0.
//   TELEMETRY at 50 Hz with FEET at 50 Hz on a 250 kbaud budget.
//   The tipping plant: its time constant; a walk plan that moves the mass
//     into each support triangle stays up with small corrections; the same
//     plan without the shift tips; a three-foot stance at Kp 0.99 shoved.
//   The per-tick cost.
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
#include "check.h"
#include "fake_eeprom.h"
#include "fake_link_hal.h"
#include "tip_plant.h"

static Config cfg;
static Body& b = g_body;
static uint16_t counts[16];
static TickIn in;
static uint8_t seq;
static bool heartbeat = true;  // the mind's HEARTBEAT every 100 ms (false: the link goes quiet)
static const double D2R = M_PI / 180.0;

struct Ev {
  uint8_t code;
  int16_t arg;
  uint16_t tick;
  uint8_t epoch;
};
static std::vector<Ev> evlog;                   // every EVENT the body sent since reset()
static std::vector<kilo_telemetry_t> tlog;      // every TELEMETRY
static std::vector<TxFrame> other;              // every other frame (ACKs, replies)

static void collect() {
  for (const TxFrame& f : g_lh.take()) {
    if (f.type == KILO_MSG_EVENT) {
      kilo_event_t e;
      memcpy(&e, f.payload.data(), sizeof e);
      evlog.push_back({e.code, e.arg, e.tick, e.epoch});
    } else if (f.type == KILO_MSG_TELEMETRY) {
      kilo_telemetry_t t;
      memcpy(&t, f.payload.data(), sizeof t);
      tlog.push_back(t);
    } else {
      other.push_back(f);
    }
  }
}

static void reset(uint8_t cfg_state = CFG_MIGRATED) {
  g_ee.reset();
  config_defaults(cfg.img);
  cfg.state = (ConfigState)cfg_state;
  params_load(cfg.img);
  body_init(b, &cfg, nullptr, true);
  g_lh.reset();
  link_init(0xFEE7FEE7u, KILO_HELLO_CAPS_DMP);
  g_lh.feed(frame(KILO_MSG_SET_STREAM, 0xFF, {1, 0, 0}));  // TELEMETRY every tick
  link_poll();
  g_lh.take();
  in = TickIn();
  in.imu = IMU_FRESH;
  in.att.gz = 16384;
  heartbeat = true;
  evlog.clear();
  tlog.clear();
  other.clear();
  seq = 0;
}

static void send(uint8_t type, const std::vector<uint8_t>& p) { g_lh.feed(frame(type, seq++, p)); }

// app/main.cpp's run_tick: the link before and after the body, then the
// streams. Frames sent before it are handled by its first link_poll.
static void tick(int n = 1) {
  for (int i = 0; i < n; i++) {
    if (heartbeat && b.tick % 5 == 0) send(KILO_MSG_HEARTBEAT, {});
    link_poll();
    body_tick(b, in, counts);
    link_poll();
    link_tick();
    collect();
  }
}

static int count_ev(uint8_t code, int arg = -1) {
  int n = 0;
  for (const Ev& e : evlog) n += e.code == code && (arg < 0 || e.arg == arg);
  return n;
}

static void posture(uint8_t p, uint8_t flags = 0, uint8_t ticks = 0) {
  send(KILO_MSG_SET_POSTURE, {b.epoch, p, ticks, flags});
}

// One FEET pose in mm and degrees.
struct Pose {
  double x[4] = {0, 0, 0, 0}, z[4] = {76, 76, 76, 76};
  double pitch = 0, roll = 0;
  uint8_t sup = 0xF;
};
static std::vector<uint8_t> feet_msg(uint16_t t, const Pose& p, int epoch = -1) {
  std::vector<uint8_t> m(KILO_FEET_LEN);
  m[0] = (uint8_t)(epoch < 0 ? b.epoch : epoch);
  m[1] = (uint8_t)t, m[2] = (uint8_t)(t >> 8);
  for (int l = 0; l < 4; l++) {
    m[3 + l] = (uint8_t)(int8_t)std::lround(p.x[l]);
    m[7 + l] = (uint8_t)std::lround(p.z[l]);
  }
  int16_t pp = (int16_t)std::lround(p.pitch * 16), rr = (int16_t)std::lround(p.roll * 16);
  m[11] = (uint8_t)pp, m[12] = (uint8_t)(pp >> 8), m[13] = (uint8_t)rr, m[14] = (uint8_t)(rr >> 8);
  m[15] = p.sup;
  return m;
}
static void feet(uint16_t t, const Pose& p, int epoch = -1) { send(KILO_MSG_FEET, feet_msg(t, p, epoch)); }

// The model in double: OpenCat angles (a16) -> foot (mm), leg frame.
static void fk(int leg, int16_t sh, int16_t kn, double* x, double* z) {
  double l1 = PARAM(LEG_L1_MM), l2 = PARAM(LEG_L2_MM);
  double t1 = PARAM(LEG_SH_ZERO_DEG) + IK_SGN_SH * sh / 16.0;
  double t2 = ik_sigma((uint8_t)leg) * PARAM(LEG_KN_ZERO_DEG) + IK_SGN_KN * kn / 16.0;
  *x = l1 * std::sin(t1 * D2R) + l2 * std::sin((t1 + t2) * D2R);
  *z = l1 * std::cos(t1 * D2R) + l2 * std::cos((t1 + t2) * D2R);
}
// The foot the joints command (after crossfade, limits and slew).
static void out_foot(int leg, double* x, double* z) { fk(leg, b.arb.out[3 + leg], b.arb.out[7 + leg], x, z); }
// The joints a pose asks for (no pose, no correction).
static bool at_pose(const Pose& p) {
  for (int l = 0; l < 4; l++) {
    int16_t sh, kn;
    ik_leg((uint8_t)l, (int16_t)(std::lround(p.x[l]) * 4), (int16_t)(std::lround(p.z[l]) * 4), &sh, &kn);
    if (b.arb.out[3 + l] != sh || b.arb.out[7 + l] != kn) return false;
  }
  return true;
}
static double max_step(const int16_t* prev) {
  double m = 0;
  for (int k = 3; k < DRIVEN_COUNT; k++) m = std::fmax(m, std::fabs((b.arb.out[k] - prev[k]) / 16.0));
  return m;
}
static GoalResult pose(int16_t pitch_a16, int16_t roll_a16, int8_t dh, int8_t xs) {
  uint8_t p[7] = {b.epoch, (uint8_t)pitch_a16, (uint8_t)(pitch_a16 >> 8), (uint8_t)roll_a16, (uint8_t)(roll_a16 >> 8),
                  (uint8_t)dh, (uint8_t)xs};
  return body_goal_pose(b, p);
}

// STANCE, settled, the link alive.
static void stance(uint8_t flags = 0) {
  reset();
  posture(POSTURE_STAND, 0, 1);
  tick(5);
  posture(POSTURE_STANCE, flags);
  tick(30);
}

// ---- STANCE, the pose terms and the foot nudge -------------------------------------
static void test_stance_pose() {
  stance();
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
  // STANCE re-entered from another posture starts at the default height.
  b.feet.h = 280;
  posture(POSTURE_STAND);
  tick(30);
  posture(POSTURE_STANCE);
  tick();
  CHECK(b.feet.h == h);
}

// ---- The geometric balance in STANCE, closed through a four-foot plant ---------------
// The body stands on a board tilted by `slope`. What the IMU reads is the
// board plus the body's attitude relative to it, which the feet make:
// pitch = atan((z_back - z_front) / Lb), roll = atan((z_right - z_left) / W),
// with the commanded feet through a first-order lag (servo, tau 0.15 s) and
// a gain `k` (legs that answer less than the model, or more).
static double run_board(double slope_p, double slope_r, double k, int ticks, bool flip, double* out_r) {
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

static const uint8_t BAL_KP99[5] = {1, 253, 30, 2, 15};  // Kp 0.99 (the daemon's default), Kd 30 ms

static void test_stance_balance() {
  // (At 0.5x the 10 deg slope would need 20 deg of correction, past the
  // 15 deg clamp.)
  for (double k : {1.0, 0.7, 1.6}) {
    stance(KILO_POSTURE_FLAGS_BALANCE);
    CHECK(body_goal_balance(b, BAL_KP99) == GOAL_OK);
    double r, p = run_board(-10, 10, k, 50, false, &r);  // nose up 10, right up 10
    std::printf("feet: STANCE balance, leg response %.1fx: 10 deg slope -> pitch %+.2f roll %+.2f after 1 s\n", k, p,
                r);
    CHECK(std::fabs(p) <= 3 && std::fabs(r) <= 3);  // H13
    CHECK(b.bal.active && b.feet.geo && !body_balance_held(b));
    // No joint-space corrections: the legs are exactly the IK's.
    int16_t legs[8];
    feet_legs(b.feet, b.bal.ax[BAL_ROLL].corr, b.bal.ax[BAL_PITCH].corr, legs);
    bool ik_only = true;
    for (int l = 0; l < 8; l++) ik_only = ik_only && b.arb.out[3 + l] == legs[l];
    CHECK(ik_only && b.arb.out[1] == 0);
    p = run_board(-10, 10, k, 200, false, &r);
    CHECKF(std::fabs(p) <= 2.5 && std::fabs(r) <= 2.5, "settled %.2f %.2f", p, r);
    // TELEMETRY: the reference (the pose, 0), the correction, all four feet.
    const kilo_telemetry_t& m = tlog.back();
    CHECK(m.ref_pitch == 0 && m.ref_roll == 0 && m.support == 0xF && m.feet_queued == 0);
    CHECKF(m.corr_pitch == b.bal.ax[BAL_PITCH].corr && m.corr_roll == b.bal.ax[BAL_ROLL].corr && m.corr_roll < -64,
           "corr %d %d / %d %d", m.corr_pitch, m.corr_roll, b.bal.ax[BAL_PITCH].corr, b.bal.ax[BAL_ROLL].corr);
  }
  // A flipped sensor runs away to the clamp.
  stance(KILO_POSTURE_FLAGS_BALANCE);
  double r, p = run_board(-10, 0, 1.0, 200, true, &r);
  CHECKF(std::fabs(b.bal.ax[BAL_PITCH].corr) >= 15 * 16 - 1, "flipped: corr %d, pitch %.1f", b.bal.ax[BAL_PITCH].corr,
         p);
  // A commanded pose is the target attitude: on a level floor with pitch
  // +6 deg asked for, the body reads +6 deg, and balance does not fight it.
  stance(KILO_POSTURE_FLAGS_BALANCE);
  CHECK(pose(96, 0, 0, 0) == GOAL_OK);
  p = run_board(0, 0, 1.0, 150, false, &r);
  CHECKF(std::fabs(p - 6) < 0.3 && b.bal.ax[BAL_PITCH].corr == 0, "pose pitch: %.2f, corr %d", p,
         b.bal.ax[BAL_PITCH].corr);
  CHECK(tlog.back().ref_pitch == 96);
  // Leaving the geometric balance for a joint-space posture zeroes it.
  posture(POSTURE_STAND);
  tick();
  CHECK(!b.feet.geo && b.bal.ax[BAL_PITCH].corr == 0);
  in.att.pitch = in.att.roll = 0;
}

// ---- The queue -------------------------------------------------------------------------
static void test_queue() {
  stance();
  Pose p;
  CHECK(body_mode(b) == KILO_MODE_IDLE && !body_gait_active(b));
  uint16_t n = b.tick;
  // Late (this tick, an earlier one) and too far ahead: dropped, counted,
  // and none of them enters GAIT.
  feet(n, p);
  feet((uint16_t)(n - 3), p);
  feet((uint16_t)(n + KILO_FEET_DEPTH + 1), p);
  link_poll();
  CHECK(b.feet.late == 3 && !body_gait_active(b) && b.feet.have == 0);
  // The window: n+1 .. n+8 all queued (8 slots, 8 ticks).
  for (int a = 1; a <= (int)KILO_FEET_DEPTH; a++) {
    Pose q = p;
    q.x[0] = a;
    feet((uint16_t)(n + a), q);
  }
  link_poll();
  CHECK(feet_queued(b.feet) == KILO_FEET_DEPTH && b.feet.late == 3);
  CHECK(body_gait_active(b) && body_mode(b) == KILO_MODE_GAIT && b.feet.state == FT_WAIT);
  // A repeat replaces its tick.
  Pose q = p;
  q.x[0] = 30;
  feet((uint16_t)(n + 2), q);
  link_poll();
  CHECK(feet_queued(b.feet) == KILO_FEET_DEPTH && b.feet.q[(n + 2) % KILO_FEET_DEPTH].x[0] == 30);
  // Each plays on its own tick: the GAIT owner takes the legs on n+1.
  tick();
  CHECK(b.tick == n + 1 && b.feet.state == FT_PLAY && b.feet.cur.x[0] == 1 && b.arb.owner[3] == OWN_GAIT);
  CHECK(feet_queued(b.feet) == KILO_FEET_DEPTH - 1 && tlog.back().feet_queued == KILO_FEET_DEPTH - 1);
  tick();
  CHECK(b.feet.cur.x[0] == 30);
  // A FEET for the tick just played, or one 9 ahead of it: dropped.
  feet(b.tick, p);
  feet((uint16_t)(b.tick + KILO_FEET_DEPTH + 1), p);
  tick();
  CHECK(b.feet.late == 5 && b.feet.cur.x[0] == 3);
  // STATS through the link.
  other.clear();
  send(KILO_MSG_STATS_REQ, {});
  tick();
  bool stats = false;
  for (const TxFrame& f : other)
    if (f.type == KILO_MSG_STATS) {
      kilo_stats_t st;
      memcpy(&st, f.payload.data(), sizeof st);
      stats = st.feet_late == 5 && st.feet_underruns == 0;
    }
  CHECK(stats);
  // A stream is never acked (nor refused while it is right).
  for (const TxFrame& f : other) CHECK(!(f.type == KILO_MSG_ACK && f.payload[1] == KILO_MSG_FEET));
  CHECK(count_ev(EV_GOAL_REFUSED) == 0);
}

// ---- Refusals ----------------------------------------------------------------------------
static void test_refusals() {
  stance();
  Pose p;
  uint16_t n = b.tick;
  // STALE through the link: one GOAL_REFUSED per second however many come.
  for (int t = 0; t < 30; t++) {
    feet((uint16_t)(n + t + 5), p, b.epoch + 1);
    tick();
  }
  // (30 ticks: within one rate-limit second, or across one boundary.)
  int nref = count_ev(EV_GOAL_REFUSED, (KILO_MSG_FEET << 8) | KILO_GOAL_REFUSED_REASON_STALE);
  CHECKF(nref >= 1 && nref <= 2, "%d refusals", nref);
  CHECK(!body_gait_active(b) && b.feet.have == 0);
  // A field out of range: BAD_ARG (and nothing queued).
  std::vector<uint8_t> m = feet_msg((uint16_t)(b.tick + 3), p);
  CHECK(body_feet(b, m.data()) == GOAL_OK);
  feet_stop(b.feet);
  m[7] = 29;
  CHECK(body_feet(b, m.data()) == GOAL_BAD_ARG);
  m[7] = 96;
  CHECK(body_feet(b, m.data()) == GOAL_BAD_ARG);
  m[7] = 76, m[4] = (uint8_t)(int8_t)-61;
  CHECK(body_feet(b, m.data()) == GOAL_BAD_ARG);
  m[4] = 0, m[15] = 0x10;
  CHECK(body_feet(b, m.data()) == GOAL_BAD_ARG && b.feet.have == 0 && !b.feet.state);
  m[15] = 0xF;
  // LOW_BATT, FALLEN, UNCALIBRATED, as SET_POSTURE STAND.
  b.batt.level = BATT_LOW;
  CHECK(body_feet(b, m.data()) == GOAL_LOW_BATT);
  b.batt.level = BATT_OK;
  b.fall.state = FS_FALLEN;
  CHECK(body_feet(b, m.data()) == GOAL_FALLEN);
  b.fall.state = FS_UPRIGHT;
  cfg.state = CFG_DEFAULTED;
  CHECK(body_feet(b, m.data()) == GOAL_UNCALIBRATED);
  cfg.state = CFG_MIGRATED;
  CHECK(b.feet.have == 0 && !b.feet.state);
  // From SIT: the first FEET makes IDLE's posture STANCE (STAND's head, the
  // default height), the legs stay IDLE's until the first tick plays.
  posture(POSTURE_SIT);
  tick(30);
  b.feet.h = 250;
  feet((uint16_t)(b.tick + 3), p);
  tick();
  CHECK(arbiter_idle_posture(b.arb) == POSTURE_STANCE && b.feet.h == 304 && b.feet.state == FT_WAIT);
  CHECK(b.arb.owner[3] == OWN_IDLE && tlog.back().mode == KILO_MODE_GAIT &&
        (tlog.back().owners & 0xF) == KILO_OWNER_GAIT && (tlog.back().flags & KILO_TELEMETRY_FLAGS_GAIT));
  tick(2);
  CHECK(b.feet.state == FT_PLAY && b.arb.owner[3] == OWN_GAIT);
}

// ---- Playing ------------------------------------------------------------------------------
// A stream whose sender is `lookahead` ticks ahead, as the mind is: before
// tick N + 1 plays it sends N + 1 + lookahead, so after each tick
// `lookahead` ticks wait in the queue.
struct Stream {
  uint16_t t0;  // the first tick of the plan
  int lookahead = KILO_FEET_LOOKAHEAD_TICKS;
};
static Stream start_stream(const Pose& p) {
  Stream st;
  st.t0 = (uint16_t)(b.tick + 1);
  for (int a = 0; a < st.lookahead; a++) feet((uint16_t)(st.t0 + a), p);
  return st;
}
// Send the pose the sender owes, then tick.
static void stream_tick(const Stream& st, const Pose& p) {
  feet((uint16_t)(b.tick + 1 + st.lookahead), p);
  tick();
  (void)st;
}

static void test_play() {
  stance();
  Pose p;
  Stream st = start_stream(p);
  for (int t = 0; t < 30; t++) stream_tick(st, p);
  CHECK(at_pose(p) && body_mode(b) == KILO_MODE_GAIT && b.arb.owner[3] == OWN_GAIT);
  CHECK(tlog.back().feet_queued == KILO_FEET_LOOKAHEAD_TICKS && tlog.back().support == 0xF);
  // A new pose: the IK of it (after the crossfade and slew).
  Pose q;
  q.x[LEG_LF] = 20, q.x[LEG_RB] = -15, q.z[LEG_RF] = 60, q.z[LEG_LB] = 90, q.sup = 0xD;  // RF lifted
  for (int t = 0; t < 30; t++) stream_tick(st, q);
  CHECK(at_pose(q) && tlog.back().support == 0xD && b.feet.cur.zs[LEG_RF] == 60);
  // A lost tick holds the pose: one FEET never sent.
  Pose r = q;
  r.x[LEG_LF] = 25;
  feet((uint16_t)(b.tick + 1 + st.lookahead), r);  // on time
  tick();
  int16_t prev[DRIVEN_COUNT];
  for (int t = 0; t < 4; t++) {
    memcpy(prev, b.arb.out, sizeof prev);
    if (t == 0)
      tick();  // (the sender skips one: the tick it should have sent is missing later)
    else
      stream_tick(st, r);
  }
  // Every tick from here on is r; the gap was held at q, no event.
  CHECK(count_ev(EV_FEET_UNDERRUN) == 0 && b.feet.state == FT_PLAY);
  for (int t = 0; t < 30; t++) stream_tick(st, r);
  CHECK(at_pose(r));
  // A wild pose: every foot 60 mm forward at once. The safety slew paces
  // it: never more than SLEW_MAX_A16 per tick.
  Pose w = r;
  for (int l = 0; l < 4; l++) w.x[l] = 60, w.z[l] = 50;
  double worst = 0;
  for (int t = 0; t < 40; t++) {
    memcpy(prev, b.arb.out, sizeof prev);
    stream_tick(st, w);
    worst = std::fmax(worst, max_step(prev));
  }
  CHECKF(worst <= PARAM(SLEW_MAX_A16) / 16.0 + 1e-9, "step %.2f deg", worst);
  CHECK(worst > 5);  // it was the slew that paced it
  std::printf("feet: a 60 mm jump of every foot: largest joint step %.2f deg/tick (the safety slew %.1f)\n", worst,
              PARAM(SLEW_MAX_A16) / 16.0);
}

// ---- Underrun: the hold, the settle, GAIT_STOPPED 1 ------------------------------------
static void test_underrun() {
  stance();
  Pose p;
  p.z[LEG_LB] = 61, p.x[LEG_LB] = -10, p.sup = 0x7;  // LB lifted 15 mm
  Stream st = start_stream(p);
  for (int t = 0; t < 40; t++) stream_tick(st, p);
  CHECK(at_pose(p));
  // The sender stops: the queued ticks play, then the queue is dry. (The
  // last FEET was handled before the last tick: at b.tick - 1.)
  uint16_t last_rx = (uint16_t)(b.tick - 1);
  int16_t held[DRIVEN_COUNT];
  int t = 0;
  while (count_ev(EV_FEET_UNDERRUN) == 0 && t++ < 20) tick();
  CHECK(t == KILO_FEET_LOOKAHEAD_TICKS + 1);  // the five queued ticks, then the first dry one
  CHECK(count_ev(EV_FEET_UNDERRUN, (int16_t)(b.tick - last_rx)) == 1);
  CHECK(b.feet.underruns == 1 && b.feet.state == FT_HOLD);
  memcpy(held, b.arb.out, sizeof held);
  for (int i = 1; i < (int)KILO_FEET_HOLD_TICKS; i++) {
    tick();
    CHECK(memcmp(held, b.arb.out, sizeof held) == 0 && b.feet.state == FT_HOLD);
  }
  // The settle: LB goes down at its x, 2 mm per tick, to the ground (the
  // other three feet's depth); then IDLE STANCE, GAIT_STOPPED 1.
  tick();
  CHECK(b.feet.state == FT_SETTLE && (b.feet.cur.zs[LEG_LB] & 0x7F) == 63 && b.feet.cur.x[LEG_LB] == -10);
  double x0, z0, x, z;
  out_foot(LEG_LB, &x0, &z0);
  int n = 1;
  while (count_ev(EV_GAIT_STOPPED) == 0 && n++ < 30) {
    tick();
    out_foot(LEG_LB, &x, &z);
    CHECK(std::fabs(x - x0) < 0.5);
  }
  std::printf("feet: underrun: FEET_UNDERRUN %d ticks after the last FEET, held %d ticks, settled in %d ticks\n",
              KILO_FEET_LOOKAHEAD_TICKS + 1, KILO_FEET_HOLD_TICKS, n);
  CHECK(n == 8);  // 61 -> 76 mm at 2 mm per tick, the last step on the 8th
  CHECK(count_ev(EV_GAIT_STOPPED, 1) == 1 && count_ev(EV_GAIT_STOPPED, 0) == 0);
  CHECK(body_mode(b) == KILO_MODE_IDLE && arbiter_idle_posture(b.arb) == POSTURE_STANCE && !body_gait_active(b));
  tick(30);
  CHECK(b.arb.owner[3] == OWN_IDLE && tlog.back().feet_queued == 0 && tlog.back().support == 0xF);
  // A FEET during the hold: the stream goes on (no GAIT_STOPPED).
  stance();
  st = start_stream(p);
  for (int i = 0; i < 20; i++) stream_tick(st, p);
  while (b.feet.state != FT_HOLD) tick();
  tick(2);
  feet((uint16_t)(b.tick + 2), p);
  tick(2);
  CHECK(b.feet.state == FT_PLAY && count_ev(EV_FEET_UNDERRUN) == 1);
  tick(KILO_FEET_HOLD_TICKS + 12);
  CHECK(count_ev(EV_FEET_UNDERRUN) == 2 && count_ev(EV_GAIT_STOPPED, 1) == 1);
}

// ---- The end: a posture, an epoch bump, a reflex ------------------------------------------
static void test_end() {
  Pose p;
  // SET_POSTURE: GAIT_STOPPED 0, IDLE takes the legs (STAND) from where they are.
  stance();
  Stream st = start_stream(p);
  for (int t = 0; t < 20; t++) stream_tick(st, p);
  posture(POSTURE_STAND);
  tick();
  CHECK(count_ev(EV_GAIT_STOPPED, 0) == 1 && !body_gait_active(b) && b.feet.have == 0);
  CHECK(body_mode(b) == KILO_MODE_IDLE && arbiter_idle_posture(b.arb) == POSTURE_STAND);
  tick(40);
  CHECK(b.arb.owner[3] == OWN_IDLE && b.arb.out[3] == posture_angle(POSTURE_STAND, 3));
  // A fall (an epoch bump): GAIT_STOPPED 0 with the new epoch, the old
  // epoch's FEET refused.
  stance();
  st = start_stream(p);
  for (int t = 0; t < 20; t++) stream_tick(st, p);
  uint8_t e0 = b.epoch;
  in.att.roll = 80 * 16;
  for (int t = 0; t < 8; t++) {  // the mind keeps sending with the epoch it knows
    feet((uint16_t)(b.tick + 1 + st.lookahead), p, e0);
    tick();
  }
  CHECK(count_ev(EV_FELL) == 1 && b.epoch == (uint8_t)(e0 + 1));
  bool ep = false;
  for (const Ev& e : evlog) ep = ep || (e.code == EV_GAIT_STOPPED && e.arg == 0 && e.epoch == e0 + 1);
  CHECK(ep && count_ev(EV_GAIT_STOPPED) == 1 && !body_gait_active(b));
  CHECK(count_ev(EV_GOAL_REFUSED, (KILO_MSG_FEET << 8) | KILO_GOAL_REFUSED_REASON_STALE) == 1);
  in.att.roll = 0;
  // The link watchdog's stop (an epoch bump) while a stream holds: no frame
  // for 500 ms; the stream underran and settled first.
  stance();
  st = start_stream(p);
  for (int t = 0; t < 20; t++) stream_tick(st, p);
  heartbeat = false;
  tick(KILO_WD_STOP_MS / 20 + 5);
  CHECK(count_ev(EV_GAIT_STOPPED, 1) == 1 && count_ev(EV_WATCHDOG) == 1 && count_ev(EV_GAIT_STOPPED) == 1);
  // A reflex (low battery: an epoch bump and REST) ends it with arg 0.
  stance();
  st = start_stream(p);
  for (int t = 0; t < 20; t++) stream_tick(st, p);
  b.arb.reflex_active = 1;
  b.arb.reflex.posture = POSTURE_HOLD;
  stream_tick(st, p);
  CHECK(count_ev(EV_GAIT_STOPPED, 0) == 1 && !body_gait_active(b));
  b.arb.reflex_active = 0;
}

// ---- TELEMETRY at 50 Hz with FEET at 50 Hz ------------------------------------------------
// The TX ring drains as the UART does at 250 kbaud: 500 B per 20 ms tick
// (the fake's ring is 95 B, as the firmware's 96 B).
static void test_budget() {
  stance(KILO_POSTURE_FLAGS_BALANCE);
  CHECK(body_goal_balance(b, BAL_KP99) == GOAL_OK);
  send(KILO_MSG_SET_STREAM, {1, 0, 0});
  tick();
  g_lh.drain = 0;
  size_t t0 = tlog.size(), rx0 = 0;
  Pose p;
  p.roll = -3;
  p.pitch = 1.5;
  Stream st = start_stream(p);
  int frames = 0;
  for (int t = 0; t < 200; t++) {
    g_lh.ring_used = std::max(0, g_lh.ring_used - 500);
    size_t before = g_lh.tx.size();
    rx0 += KILO_FEET_LEN + KILO_FRAME_OVERHEAD;
    stream_tick(st, p);
    frames += (int)(g_lh.tx.size() - before);
  }
  g_lh.drain = 1000;
  size_t n = tlog.size() - t0;
  const kilo_telemetry_t& m = tlog.back();
  std::printf("feet: 4 s of FEET and TELEMETRY at 50 Hz: %zu TELEMETRY, body->Pi %d B (%.0f B/s of 25000), Pi->body "
              "%zu B (%.0f B/s)\n",
              n, frames, frames / 4.0, rx0, rx0 / 4.0);
  CHECK(n == 200 && link_counters().tx_dropped == 0 && count_ev(EV_TX_DROPPED) == 0);
  CHECK(m.mode == KILO_MODE_GAIT && m.feet_queued == KILO_FEET_LOOKAHEAD_TICKS && m.support == 0xF);
  CHECK(m.ref_pitch == 24 && m.ref_roll == -48);  // the FEET prediction (the pose is 0)
  CHECK((m.flags & KILO_TELEMETRY_FLAGS_BALANCE_ACTIVE) && !(m.flags & KILO_TELEMETRY_FLAGS_BALANCE_HELD));
  // On two support feet: held, the reference still reported.
  p.sup = 0x5;
  for (int t = 0; t < 8; t++) stream_tick(st, p);
  CHECK((tlog.back().flags & KILO_TELEMETRY_FLAGS_BALANCE_HELD) && tlog.back().support == 0x5 &&
        tlog.back().ref_roll == -48);
  // Balance off: no reference, the correction decays to 0.
  CHECK(body_goal_balance(b, (const uint8_t[]){0, 253, 30, 2, 15}) == GOAL_OK);
  posture(POSTURE_LIMP);
  tick(60);
  CHECK(tlog.back().ref_roll == 0 && tlog.back().corr_roll == 0 && tlog.back().support == 0 &&
        tlog.back().mode == KILO_MODE_LIMP);
}

int main() {
  test_stance_pose();
  test_stance_balance();
  test_queue();
  test_refusals();
  test_play();
  test_underrun();
  test_end();
  test_budget();
  return report("feet");
}
