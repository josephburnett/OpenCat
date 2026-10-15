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
//     during the hold resumes; the settle on a leaning body (the plane of
//     the support feet; on two feet their mean).
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
  // (From SIT a dropped first FEET leaves IDLE's posture alone.)
  posture(POSTURE_SIT);
  feet(n, p);
  link_poll();
  CHECK(arbiter_idle_posture(b.arb) == POSTURE_SIT && b.feet.late == 4);
  posture(POSTURE_STANCE);
  tick(30);
  n = b.tick;
  // The window: n+1 .. n+8 all queued (8 slots, 8 ticks).
  for (int a = 1; a <= (int)KILO_FEET_DEPTH; a++) {
    Pose q = p;
    q.x[0] = a;
    feet((uint16_t)(n + a), q);
  }
  link_poll();
  CHECK(feet_queued(b.feet) == KILO_FEET_DEPTH && b.feet.late == 4);
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
  CHECK(b.feet.late == 6 && b.feet.cur.x[0] == 3);
  // STATS through the link.
  other.clear();
  send(KILO_MSG_STATS_REQ, {});
  tick();
  bool stats = false;
  for (const TxFrame& f : other)
    if (f.type == KILO_MSG_STATS) {
      kilo_stats_t st;
      memcpy(&st, f.payload.data(), sizeof st);
      stats = st.feet_late == 6 && st.feet_underruns == 0;
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

// The settle on a leaning body: the lifted foot goes to the plane through
// the three support feet at its x, not to their mean depth; on two support
// feet (no plane) to their mean.
static double plane_z(const Pose& p, const int abc[3], int leg) {
  double lb = PARAM(LEG_LB_MM), w = PARAM(LEG_W_MM), X[4], Y[4], Z[4];
  int idx[4] = {abc[0], abc[1], abc[2], leg};
  for (int k = 0; k < 4; k++) {
    int l = idx[k];
    X[k] = (leg_front((uint8_t)l) ? lb / 2 : -lb / 2) + p.x[l];
    Y[k] = leg_left((uint8_t)l) ? -w / 2 : w / 2;
    Z[k] = p.z[l];
  }
  double ux = X[1] - X[0], uy = Y[1] - Y[0], vx = X[2] - X[0], vy = Y[2] - Y[0];
  double wx = X[3] - X[0], wy = Y[3] - Y[0], d = ux * vy - uy * vx;
  return Z[0] + ((wx * vy - wy * vx) * (Z[1] - Z[0]) + (ux * wy - uy * wx) * (Z[2] - Z[0])) / d;
}
// Stream p, stop, run to GAIT_STOPPED 1 (lb: LEG_LB_MM, 0 = the default).
// The settle's ticks.
static int settle_run(const Pose& p, int16_t lb = 0) {
  stance();
  if (lb) g_param[KILO_PARAM_LEG_LB_MM] = lb;
  Stream st = start_stream(p);
  for (int t = 0; t < 40; t++) stream_tick(st, p);
  int n = 0;
  while (count_ev(EV_GAIT_STOPPED) == 0 && n++ < 60) tick();
  CHECK(count_ev(EV_GAIT_STOPPED, 1) == 1 && feet_support(b.feet) == 0xF);
  return n;
}
static void test_settle_plane() {
  // The walk's lean (6 deg pitch, 14 deg roll; doc §3d): support depths
  // z = 70 + tan(6) X - tan(14) Y (X forward, Y right of the body's centre).
  double lb = PARAM(LEG_LB_MM), w = PARAM(LEG_W_MM);
  Pose p;
  p.pitch = 6, p.roll = 14;
  for (int l = 0; l < 4; l++) {
    p.x[l] = leg_front((uint8_t)l) ? 20 : -20;
    double X = (leg_front((uint8_t)l) ? lb / 2 : -lb / 2) + p.x[l], Y = leg_left((uint8_t)l) ? -w / 2 : w / 2;
    p.z[l] = std::round(70 + std::tan(6 * D2R) * X - std::tan(14 * D2R) * Y);
  }
  Pose q0 = p;  // all four on the plane
  p.z[LEG_LF] = 66, p.sup = 0xE;  // LF lifted, 20 mm above the floor
  int abc[3] = {LEG_RF, LEG_RB, LEG_LB};
  double want = plane_z(p, abc, LEG_LF), mean = (p.z[LEG_RF] + p.z[LEG_RB] + p.z[LEG_LB]) / 3;
  settle_run(p);
  int got = b.feet.cur.zs[LEG_LF] & 0x7F;
  std::printf("feet: settle on a leaning body: LF at %d mm (the support plane %.2f, their mean %.2f)\n", got, want,
              mean);
  CHECKF(std::fabs(got - want) <= 0.5, "LF settled at %d, plane %.2f", got, want);
  CHECK(std::fabs(want - mean) > 10);  // the mean would have left it ~20 mm off the floor
  CHECK(b.feet.cur.x[LEG_LF] == 20);
  // Each leg lifted in turn: on the plane of the other three.
  for (int lift = 0; lift < 4; lift++) {
    Pose q = q0;
    q.z[lift] = 60, q.sup = (uint8_t)(0xF & ~(1u << lift));
    int sup[3], k = 0;
    for (int l = 0; l < 4; l++)
      if (l != lift) sup[k++] = l;
    double wz = plane_z(q, sup, lift);
    settle_run(q);
    int g = b.feet.cur.zs[lift] & 0x7F;
    CHECKF(std::fabs(g - wz) <= 0.5 && std::fabs(g - q0.z[lift]) <= 1, "leg %d settled at %d, plane %.2f", lift, g,
           wz);
  }
  // Two support feet (a diagonal) define no plane: their mean depth.
  Pose d = q0;
  d.z[LEG_LF] = 60, d.z[LEG_RB] = 60, d.sup = 0xA;  // RF, LB down
  settle_run(d);
  int m2 = (int)std::lround((d.z[LEG_RF] + d.z[LEG_LB]) / 2);
  CHECK((b.feet.cur.zs[LEG_LF] & 0x7F) == m2 && (b.feet.cur.zs[LEG_RB] & 0x7F) == m2);
  // Three support feet on no plane: with Lb = 80 mm, RF 40 mm back and RB
  // 40 mm forward stand on the same spot (D = 0): their mean.
  Pose c;
  c.x[LEG_RF] = -40, c.x[LEG_RB] = 40, c.x[LEG_LB] = 40;
  c.z[LEG_RF] = 70, c.z[LEG_RB] = 80, c.z[LEG_LB] = 70, c.z[LEG_LF] = 60, c.sup = 0xE;
  settle_run(c, 80);
  CHECK((b.feet.cur.zs[LEG_LF] & 0x7F) == 73);
  // A thin triangle (RB 1 mm further forward): the plane is steep, the
  // quotient is capped and the depth kept in FEET's 30..95.
  c.x[LEG_RB] = 41;
  settle_run(c, 80);
  CHECK((b.feet.cur.zs[LEG_LF] & 0x7F) == 95);
  c.z[LEG_RB] = 60;
  settle_run(c, 80);
  CHECK((b.feet.cur.zs[LEG_LF] & 0x7F) == 30);
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

// ---- The correction's shares --------------------------------------------------------------
// With the balance on and the body reading a steady 6 deg of roll, the
// correction builds on four feet. Then LB's support bit clears (the same
// pose otherwise): the correction is held, and LB lets go of its share a
// quarter per tick; the other three keep theirs. Back down: it takes it back
// the same way.
static void test_shares() {
  stance(KILO_POSTURE_FLAGS_BALANCE);
  CHECK(body_goal_balance(b, BAL_KP99) == GOAL_OK);
  in.att.roll = 6 * 16;
  Pose p;
  Stream st = start_stream(p);
  for (int t = 0; t < 60; t++) stream_tick(st, p);
  int16_t c = b.bal.ax[BAL_ROLL].corr;
  CHECKF(c < -40, "corr %d", c);  // -Kp (6 - 2) with the IK's own effect taken out: the body reads 6 whatever
  double z0[4], x;
  for (int l = 0; l < 4; l++) out_foot(l, &x, &z0[l]);
  Pose q = p;
  q.sup = 0x7;  // LB "up" (same z: only the bit)
  // The ticks already queued play first; then q.
  for (int t = 0; t < (int)KILO_FEET_LOOKAHEAD_TICKS; t++) stream_tick(st, q);
  // LB, a left leg, carried -(W/2) sin(corr) (corr < 0: longer); it gives that back.
  double term = PARAM(LEG_W_MM) / 2.0 * std::sin(c / 16.0 * D2R);
  double prev = z0[LEG_LB], steps[6];
  for (int t = 0; t < 6; t++) {
    stream_tick(st, q);
    double z;
    out_foot(LEG_LB, &x, &z);
    steps[t] = z - prev;
    prev = z;
  }
  CHECK(b.bal.ax[BAL_ROLL].corr == c && body_balance_held(b));
  // (0.3 mm: the share is applied in q2, 0.25 mm, and the IK rounds)
  for (int t = 0; t < 4; t++) CHECKF(std::fabs(steps[t] - term / 4) < 0.3, "step %d: %.2f of %.2f", t, steps[t], term);
  CHECK(std::fabs(steps[4]) < 0.05 && std::fabs(steps[5]) < 0.05);
  double z;
  for (int l = 0; l < 3; l++) {
    out_foot(l, &x, &z);
    CHECKF(std::fabs(z - z0[l]) < 0.05, "leg %d moved %.2f", l, z - z0[l]);
  }
  in.att.roll = 0;
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

// ---- The tipping plant -----------------------------------------------------------------
static TipPlant plant;
// One tick: the plant answers the last tick's outputs, the IMU reads it,
// then run_tick.
static void plant_tick() {
  plant.step(b.arb.out);
  in.att.pitch = (int16_t)std::lround(plant.pitch * 16);
  in.att.roll = (int16_t)std::lround(plant.roll * 16);
  in.att.rate_pitch = (int16_t)std::lround(plant.rate_pitch * 16);
  in.att.rate_roll = (int16_t)std::lround(plant.rate_roll * 16);
  in.att.gz = (int16_t)std::lround(16384 * std::cos(plant.pitch * D2R) * std::cos(plant.roll * D2R));
  tick();
}

// The joint angles (canonical, a16) of a pose, for the plant alone.
static void pose_out(const Pose& p, int16_t out[DRIVEN_COUNT]) {
  for (int k = 0; k < DRIVEN_COUNT; k++) out[k] = 0;
  for (int l = 0; l < 4; l++)
    ik_leg((uint8_t)l, (int16_t)std::lround(p.x[l] * 4), (int16_t)std::lround(p.z[l] * 4), &out[3 + l], &out[7 + l]);
}

// The plant's time constant: three feet with the mass 3 mm outside their
// triangle (LB lifted 41 mm, the mass moved toward it): the tilt from 2 to
// 30 deg.
static void test_plant_tip() {
  reset();
  Pose p;
  p.z[LEG_LB] = 35;
  int16_t out[DRIVEN_COUNT];
  pose_out(p, out);
  plant = TipPlant();
  // The mass toward LB, 3 mm past the LF-RB diagonal.
  double diag = std::hypot(PARAM(LEG_LB_MM), PARAM(LEG_W_MM));
  plant.com_x = -3 * PARAM(LEG_W_MM) / diag, plant.com_y = 3 * PARAM(LEG_LB_MM) / diag;
  plant.init(out);
  int t2 = -1, t30 = -1;
  for (int t = 0; t < 50 && t30 < 0; t++) {
    plant.step(out);
    double tilt = plant.tilt();
    if (t2 < 0 && tilt >= 2) t2 = t;
    if (tilt >= 30) t30 = t;
  }
  std::printf("feet: tipping plant: three feet, the mass 3 mm outside: 2 to 30 deg in %d ms (roll %+.1f pitch %+.1f; "
              "the floor 2026-10-14: about 30 deg in 0.2 s)\n",
              (t30 - t2) * 20, plant.roll, plant.pitch);
  CHECK(t2 >= 0 && t30 > t2 && (t30 - t2) * 20 >= 160 && (t30 - t2) * 20 <= 300);
  CHECK(plant.roll > 0 && plant.pitch < 0);  // toward LB: left side down (+roll: right side up), nose up
  // On four feet the same mass offset rests: level, no tip.
  plant = TipPlant();
  plant.com_x = -3, plant.com_y = 3;
  pose_out(Pose(), out);
  plant.init(out);
  for (int t = 0; t < 25; t++) plant.step(out);
  CHECK(plant.tips == 0 && std::fabs(plant.roll) < 1e-6 && std::fabs(plant.pitch) < 1e-6);
}

// A static walk planned the way the mind is to plan it (doc §3d): the legs
// swing one at a time in the lateral sequence LB, LF, RB, RF (each 19
// ticks, 0.38 s), stride S, step height 15 mm, over a 2.4 s cycle; before
// the two left legs swing the body rolls right by A (the mass ~13 mm right
// at A = 10 deg, inside the triangles of the three feet left), and before
// the two right legs left by A. The roll is made with the leg lengths (no
// sideways leg on this cat); the predicted attitude is the planned roll
// through the servo lag (tau 0.15 s), the support bits the feet not in
// their swing. A = 0: the same steps with no body shift.
struct Walk {
  double A = 10, S = 20, step = 15, H = 76;
  int T = 120, sw = 19;
};
static const int SW_START[4] = {40, 100, 80, 20};  // LF RF RB LB: in-cycle tick of each lift-off
static double walk_roll(const Walk& w, int k) {
  auto blend = [](double a, double c, double u) { return a + (c - a) * (1 - std::cos(M_PI * u)) / 2; };
  int c = k % w.T;
  if (c < 20) return blend(k < w.T ? 0 : w.A, -w.A, c / 20.0);
  if (c < 60) return -w.A;
  if (c < 80) return blend(-w.A, w.A, (c - 60) / 20.0);
  return w.A;
}
static std::vector<Pose> walk_plan(const Walk& w, int ticks) {
  std::vector<Pose> v(ticks);
  double pred = 0, lag = 1 - std::exp(-1 / 7.5);
  for (int k = 0; k < ticks; k++) {
    Pose& p = v[k];
    double roll = walk_roll(w, k), S = w.S * std::fmin(1.0, k / (double)w.T);
    p.sup = 0;
    for (int l = 0; l < 4; l++) {
      int u = ((k % w.T) - SW_START[l] + w.T) % w.T;
      double lift = 0;
      if (u <= w.sw) {
        double sv = u / (double)w.sw;
        p.x[l] = -S / 2 + S * (1 - std::cos(M_PI * sv)) / 2;
        lift = w.step * (1 - std::cos(2 * M_PI * sv)) / 2;
      } else {
        p.x[l] = S / 2 - S * (u - w.sw) / (double)(w.T - w.sw);
      }
      if (lift < 0.5) p.sup |= (uint8_t)(1 << l);
      p.z[l] = w.H + (leg_left((uint8_t)l) ? -1 : 1) * PARAM(LEG_W_MM) / 2 * std::sin(roll * D2R) - lift;
    }
    pred += (roll - pred) * lag;
    p.roll = pred;
  }
  return v;
}

struct WalkResult {
  bool fell;           // the plant passed 80 deg or the body sent FELL
  double max_tilt;     // largest |roll| or |pitch|, deg
  double max_res;      // largest |measured - reference| on the played ticks, deg
  double max_corr;     // largest |correction|, deg
  int tips;            // the plant's tips of more than 1 deg (a support lost under the mass)
  double max_turn;     // the largest such turn, deg
};
// STANCE (balance per `bal`, null = off), the plant at rest, then `plan`
// streamed through the link at 50 Hz, 5 ticks ahead.
static WalkResult walk_run(const std::vector<Pose>& plan, const uint8_t* bal, double com_up = 0) {
  stance(bal ? KILO_POSTURE_FLAGS_BALANCE : 0);
  if (bal) CHECK(body_goal_balance(b, bal) == GOAL_OK);
  plant = TipPlant();
  plant.com_up = com_up;
  plant.init(b.arb.out);
  for (int t = 0; t < 25; t++) plant_tick();
  WalkResult r = {false, 0, 0, 0, 0, 0};
  plant.reset_max();
  plant.tips = 0;
  uint16_t t0 = (uint16_t)(b.tick + 1);
  int n = (int)plan.size();
  for (int a = 0; a < (int)KILO_FEET_LOOKAHEAD_TICKS && a < n; a++) feet((uint16_t)(t0 + a), plan[a]);
  for (int k = 0; k < n; k++) {
    int ahead = k + (int)KILO_FEET_LOOKAHEAD_TICKS;
    if (ahead < n) feet((uint16_t)(t0 + ahead), plan[ahead]);
    plant_tick();
    if (b.feet.state == FT_PLAY) {
      r.max_res = std::fmax(r.max_res, std::fabs(plant.roll - plan[k].roll));
      r.max_corr = std::fmax(r.max_corr, std::fmax(std::fabs(b.bal.ax[BAL_ROLL].corr), std::fabs(b.bal.ax[BAL_PITCH].corr)) / 16.0);
    }
    if (plant.fallen || count_ev(EV_FELL)) break;
  }
  r.fell = plant.fallen || count_ev(EV_FELL) > 0;
  r.max_tilt = plant.max_tilt;
  r.tips = plant.tips;
  r.max_turn = plant.max_turn;
  return r;
}

// (a) and (b): the walk plan with the body shift stays up with small
// corrections; the same steps with no shift tip at every lift-off.
static void test_walk_plant() {
  Walk w;
  std::vector<Pose> plan = walk_plan(w, 4 * w.T);
  // The plan itself: the mass's margin inside each support (the plant's
  // static check on the planned feet, no lag).
  WalkResult on = walk_run(plan, BAL_KP99), off = walk_run(plan, nullptr);
  std::printf("feet: walk plan with the body shift (A %.0f deg), 4 cycles: balance Kp 0.99: %s, tilt %.1f, residual "
              "%.2f, corrections %.2f deg, %d tips; balance off: %s, tilt %.1f, %d tips\n",
              w.A, on.fell ? "FELL" : "up", on.max_tilt, on.max_res, on.max_corr, on.tips, off.fell ? "FELL" : "up",
              off.max_tilt, off.tips);
  CHECK(!on.fell && !off.fell && on.tips == 0 && off.tips == 0);
  CHECKF(on.max_corr <= 2.0 && on.max_res <= 2.5, "corrections %.2f, residual %.2f", on.max_corr, on.max_res);
  Walk w0 = w;
  w0.A = 0;
  std::vector<Pose> plan0 = walk_plan(w0, 4 * w.T);
  WalkResult on0 = walk_run(plan0, BAL_KP99), off0 = walk_run(plan0, nullptr);
  std::printf("feet: the same walk with no body shift: balance Kp 0.99: %s, %.1f deg off the plan, corrections %.2f deg; "
              "balance off: %s, %.1f deg off the plan\n",
              on0.fell ? "FELL" : "up", on0.max_res, on0.max_corr, off0.fell ? "FELL" : "up", off0.max_res);
  // The plant is honest: with the mass over the diagonal a lifting foot
  // takes the body down with it, loop or no loop.
  CHECK(on0.max_res > 5 && off0.max_res > 5);
}

// (f) A three-foot stance (LB lifted 15 mm, the body rolled right by A so
// the mass is inside LF RF RB), held by a stream whose prediction is that
// roll; a shove toward LB (a turn about the LF-RB diagonal at `rate`). The
// peak deviation from the plan, how many times it swings past the dead
// band, and the deviation 1 s later; with the balance off, at Kp 0.99 with
// the plan as the reference, and at Kp 0.99 with a level reference (what
// the walk on the floor had before the sway: the loop fighting the lean).
struct ShoveResult {
  bool fell;
  double peak, last, corr;  // deg: largest |dev|, |dev| after 1 s, largest |correction|
  int swings;               // sign changes of the deviation beyond 2 deg
};
static ShoveResult shove_run(double A, double rate, const uint8_t* bal, bool level_ref) {
  stance(bal ? KILO_POSTURE_FLAGS_BALANCE : 0);
  if (bal) CHECK(body_goal_balance(b, bal) == GOAL_OK);
  plant = TipPlant();
  plant.init(b.arb.out);
  for (int t = 0; t < 25; t++) plant_tick();
  // The plan: roll to -A on four feet over 0.5 s, lift LB, then hold.
  Walk w;
  w.A = A;
  auto pose_at = [&](int k, double* pred) {
    Pose p;
    double roll = -A * std::fmin(1.0, k / 25.0), lift = k < 40 ? 0 : 15 * std::fmin(1.0, (k - 40) / 10.0);
    for (int l = 0; l < 4; l++)
      p.z[l] = w.H + (leg_left((uint8_t)l) ? -1 : 1) * PARAM(LEG_W_MM) / 2 * std::sin(roll * D2R) -
               (l == LEG_LB ? lift : 0);
    p.sup = lift > 0.5 ? 0x7 : 0xF;
    *pred += (roll - *pred) * (1 - std::exp(-1 / 7.5));
    p.roll = level_ref ? 0 : *pred;
    return p;
  };
  double pred = 0;
  std::vector<Pose> plan;
  for (int k = 0; k < 220; k++) plan.push_back(pose_at(k, &pred));
  uint16_t t0 = (uint16_t)(b.tick + 1);
  for (int a = 0; a < (int)KILO_FEET_LOOKAHEAD_TICKS; a++) feet((uint16_t)(t0 + a), plan[a]);
  ShoveResult r = {false, 0, 0, 0, 0};
  int sign = 0;
  double dir_x = -PARAM(LEG_LB_MM), dir_y = PARAM(LEG_W_MM), dl = std::hypot(dir_x, dir_y);
  for (int k = 0; k < 220; k++) {
    int ahead = k + (int)KILO_FEET_LOOKAHEAD_TICKS;
    if (ahead < 220) feet((uint16_t)(t0 + ahead), plan[ahead]);
    if (k == 100) plant.shove(dir_x / dl, dir_y / dl, rate);
    plant_tick();
    if (plant.fallen || count_ev(EV_FELL)) break;
    if (k < 100) continue;
    double dev = plant.roll - (level_ref ? -A : plan[k].roll);
    r.peak = std::fmax(r.peak, std::fabs(dev));
    r.corr = std::fmax(r.corr, std::fabs(b.bal.ax[BAL_ROLL].corr) / 16.0);
    int sg = dev > 2 ? 1 : dev < -2 ? -1 : 0;
    if (sg && sign && sg != sign) r.swings++;
    if (sg) sign = sg;
    r.last = std::fabs(dev);
  }
  r.fell = plant.fallen || count_ev(EV_FELL) > 0;
  return r;
}

static void test_three_foot_shove() {
  static const uint8_t BAL_KP50[5] = {1, 128, 30, 2, 15};
  bool any_worse = false;
  for (double A : {10.0, 5.0})
    for (double rate : {30.0, 60.0}) {
      ShoveResult off = shove_run(A, rate, nullptr, false), plan = shove_run(A, rate, BAL_KP99, false),
                  half = shove_run(A, rate, BAL_KP50, false), lvl = shove_run(A, rate, BAL_KP99, true);
      std::printf("feet: three feet, mass %.0f deg of roll inside, a %.0f deg/s shove toward LB: peak / after 1 s / "
                  "swings past 2 deg: off %.1f / %.1f / %d%s; Kp 0.99 ref = plan %.1f / %.1f / %d%s (corr %.1f); "
                  "Kp 0.5 %.1f / %.1f / %d%s; Kp 0.99 ref level %.1f / %.1f / %d%s (corr %.1f)\n",
                  A, rate, off.peak, off.last, off.swings, off.fell ? " FELL" : "", plan.peak, plan.last, plan.swings,
                  plan.fell ? " FELL" : "", plan.corr, half.peak, half.last, half.swings, half.fell ? " FELL" : "",
                  lvl.peak, lvl.last, lvl.swings, lvl.fell ? " FELL" : "", lvl.corr);
      CHECK(!off.fell);
      // The finding this test guards: with the plan as the reference, and
      // held while a foot is up, the loop does not turn a shove into a
      // growing swing, a larger lean or a fall.
      CHECKF(!plan.fell && plan.swings <= off.swings && plan.peak <= off.peak + 0.5 && plan.last <= off.last + 0.5,
             "A %.0f rate %.0f: Kp 0.99 vs off: peak %.1f / %.1f, swings %d / %d, after 1 s %.1f", A, rate, plan.peak,
             off.peak, plan.swings, off.swings, plan.last);
      any_worse = any_worse || lvl.fell || lvl.last > plan.last + 1;
    }
  std::printf("feet: three-foot shove: a level reference (the loop fighting the planned lean) %s\n",
              any_worse ? "does worse than the plan as the reference" : "does no worse here");
}

// ---- The per-tick cost -------------------------------------------------------------------
// fx calls in one GAIT tick with the balance on (4 legs of IK, the pose and
// the correction terms) against one STANCE tick.
static void test_cost() {
  stance(KILO_POSTURE_FLAGS_BALANCE);
  CHECK(body_goal_balance(b, BAL_KP99) == GOAL_OK);
  FxOps a = g_fx_ops;
  tick();
  FxOps c = g_fx_ops;
  uint32_t s0 = c.sin - a.sin, t0 = c.atan2 - a.atan2, q0 = c.isqrt - a.isqrt;
  Pose p;
  Stream st = start_stream(p);
  for (int t = 0; t < 10; t++) stream_tick(st, p);
  a = g_fx_ops;
  stream_tick(st, p);
  c = g_fx_ops;
  uint32_t sn = c.sin - a.sin, at = c.atan2 - a.atan2, sq = c.isqrt - a.isqrt;
  std::printf("feet: one GAIT tick: %u sin/cos, %u atan2, %u isqrt (+ 4 IK int32 divisions); STANCE: %u, %u, %u\n", sn,
              at, sq, s0, t0, q0);
  CHECK(at == 12 && sq == 4 && sn <= s0 + 2);
}

int main() {
  test_stance_pose();
  test_stance_balance();
  test_queue();
  test_refusals();
  test_play();
  test_underrun();
  test_settle_plane();
  test_end();
  test_shares();
  test_budget();
  test_plant_tip();
  test_walk_plant();
  test_three_foot_shove();
  test_cost();
  return report("feet");
}
