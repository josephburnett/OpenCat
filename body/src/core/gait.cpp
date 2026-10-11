#include "gait.h"
#include <string.h>
#include "fx.h"
#include "ik.h"
#include "params.h"
#include "pgm.h"

// Per pattern (doc §3d, all (est.), to be tuned): defaults for the fields a
// SET_GAIT may leave 0 (outside their spec range: period, duty, body
// height), and the leg phase offsets in 1/256 cycle (LF, RF, RB, LB). A leg swings when phi + offset crosses duty, so a larger offset
// swings earlier: WALK/CRAWL give LB -> LF -> RB -> RF (the lateral
// sequence), TROT the diagonal pairs LF+RB, RF+LB. (The stride default of
// §3d, 30 / 35 / 25 mm, is the mind's: stride 0 is a real value, turning
// in place; likewise step 0, a shuffle: the mind sends §3d's 15 / 12 / 18.)
struct PatRow {
  uint16_t period_ms;
  uint8_t duty, h_256;  // h_256: body height / (L1 + L2), /256
  uint8_t off[4];
};
// Periods: WALK 1000 and CRAWL 1500 (§3d first said 800 / 1200). With
// Kilo's measured legs (45 + 50 mm) the shorter periods asked the knees for
// 9.3 and 10.0 deg per tick at the default stride and step height, i.e. the
// safety slew (SLEW_MAX_A16, 10 deg/tick) would have clipped the crawl's
// swing. Tuning on Kilo will move these again.
static const PatRow PAT[3] PROGMEM = {
    {1000, 192, 205, {128, 0, 64, 192}},  // WALK
    {500, 140, 205, {0, 128, 0, 128}},    // TROT
    {1500, 218, 179, {128, 0, 64, 192}},  // CRAWL
};
#define PAT_U8(pat, field) PGM_U8(&PAT[pat].field)

#define SLEW_Q2 2      // 0.5 mm per tick: stride, turn, step and body height
#define SLEW_POSE 8    // 0.5 deg per tick
#define POSE_MAX 480   // +-30 deg

static inline uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// v toward t by at most s.
KILO_NOINLINE static int16_t slew(int16_t v, int16_t t, int16_t s) {
  if (t > v + s) return (int16_t)(v + s);
  if (t < v - s) return (int16_t)(v - s);
  return t;
}

void gait_init(Gait& g) {
  memset(&g, 0, sizeof g);
  gait_stance_default(g);
}

void gait_stance_default(Gait& g) { g.h = (int16_t)(((int32_t)ik_reach_q2() * 205) >> 8); }

uint8_t gait_check(const uint8_t* p) {
  uint16_t per = rd16(p + 3);
  int8_t stride = (int8_t)p[6], turn = (int8_t)p[7];
  bool bad = p[1] > 1 || p[2] > 2 || (per && (per < 300 || per > 2000)) || (p[5] && (p[5] < 128 || p[5] > 230)) ||
             stride < -60 || stride > 60 || turn < -40 || turn > 40 || p[8] > 40 || (p[10] & ~7u);
  return bad ? 2 : 0;  // GOAL_BAD_ARG : GOAL_OK (core/body.h)
}

void gait_goal(Gait& g, const uint8_t* p) {
  uint8_t pat = p[2];
  g.run = 1;
  g.pattern = pat;
  uint16_t per = rd16(p + 3);
  if (!per) per = PGM_U16(&PAT[pat].period_ms);
  // 65536 * 20 ms / period: one division per goal, none per tick.
  g.dphi = (uint16_t)(1310720UL / per);
  g.duty_g = p[5];
  g.stride_g = (int8_t)p[6];
  g.turn_g = (int8_t)p[7];
  g.step_g = p[8];
  g.body_g = p[9];
  g.flags = p[10];
}

void gait_pose_goal(Gait& g, const uint8_t* p) {
  for (uint8_t i = 0; i < 2; i++) {
    int16_t v = (int16_t)rd16(p + 2 * i);
    g.pose_g[i] = v > POSE_MAX ? POSE_MAX : v < -POSE_MAX ? (int16_t)-POSE_MAX : v;
  }
  g.dh_g = (int8_t)p[4];
  g.xs_g = (int8_t)p[5];
}

// The goal's value of a "0 = default" field.
static uint8_t duty_goal(const Gait& g) { return g.duty_g ? g.duty_g : PAT_U8(g.pattern, duty); }
static int16_t step_goal(const Gait& g) { return (int16_t)(g.step_g * 4); }

void gait_start(Gait& g) {
  g.state = GS_RUN;
  g.pat = g.pattern;
  g.phi = g.amp = 0;
  g.armed = 0;
  // The stride starts from the goal: the ramp (amp) brings it in. The body
  // height slews from STANCE's to the goal's.
  g.duty = duty_goal(g);
  g.stride = (int16_t)(g.stride_g * 4);
  g.turn = (int16_t)(g.turn_g * 4);
  g.step = step_goal(g);
}

bool gait_step(Gait& g) {
  // The pose slews always (it shapes STANCE too).
  for (uint8_t i = 0; i < 2; i++) g.pose[i] = slew(g.pose[i], g.pose_g[i], SLEW_POSE);
  g.dh = (int8_t)slew(g.dh, g.dh_g, 1);
  g.xs = (int8_t)slew(g.xs, g.xs_g, 1);
  if (!g.state) return false;
  if (g.state == GS_LAND) {
    if (!g.armed) {  // every foot is down: hand back to IDLE STANCE
      g.state = GS_OFF;
      return true;
    }
  } else if (g.run && g.pattern == g.pat) {  // run (or resume a stop)
    g.state = GS_RUN;
    g.amp = (uint16_t)(g.amp > 65535u - g.dphi ? 65535u : g.amp + g.dphi);
  } else {  // stop (or change pattern): ramp the stride out over a period
    g.state = GS_STOP;
    g.amp = (uint16_t)(g.amp > g.dphi ? g.amp - g.dphi : 0);
    if (!g.amp) g.state = GS_LAND;
  }
  g.phi = (uint16_t)(g.phi + g.dphi);
  uint8_t d = duty_goal(g);
  g.duty = (uint8_t)(g.duty + (d > g.duty) - (d < g.duty));
  g.stride = slew(g.stride, (int16_t)(g.stride_g * 4), SLEW_Q2);
  g.turn = slew(g.turn, (int16_t)(g.turn_g * 4), SLEW_Q2);
  g.step = slew(g.step, step_goal(g), SLEW_Q2);
  int16_t h = g.body_g ? (int16_t)(g.body_g * 4) : (int16_t)(((int32_t)ik_reach_q2() * PAT_U8(g.pattern, h_256)) >> 8);
  g.h = slew(g.h, h, SLEW_Q2);
  return false;
}

void gait_legs(Gait& g, int16_t corr_roll, int16_t corr_pitch) {
  // Pose terms (doc §3d): the raised end or side gets the longer legs.
  int16_t pitch = g.pose[0], roll = g.pose[1];
  if (g.geo) {
    pitch = (int16_t)(pitch + corr_pitch);
    roll = (int16_t)(roll + corr_roll);
  }
  // (Lb/2) sin and (W/2) sin in q2, rounded (truncating would lose up to
  // 0.25 mm per leg, 0.2 deg of body tilt).
  int16_t dz_p = (int16_t)(((int32_t)PARAM(LEG_LB_MM) * 2 * fx_sin(pitch) + 8192) >> 14);
  int16_t dz_r = (int16_t)(((int32_t)PARAM(LEG_W_MM) * 2 * fx_sin(roll) + 8192) >> 14);
  int16_t base = (int16_t)(g.h + g.dh * 4);
  bool run = g.state != GS_OFF;
  uint16_t b16 = (uint16_t)(g.duty << 8);
  // 1/beta and 1/(1 - beta) as 2^30 / x, so each leg's s is a multiply
  // (two divisions per tick, not eight). duty is 128..230, so b16 >= 32768
  // and 65536 - b16 >= 6656: inv_st <= 2^15, inv_sw < 2^18, and p * inv <
  // 2^30 since p < b16 (stance) or p - b16 < 65536 - b16 (swing).
  uint32_t inv_st = 0, inv_sw = 0;
  if (run) {
    inv_st = (1UL << 30) / b16;
    inv_sw = (1UL << 30) / (65536UL - b16);
  }
  for (uint8_t leg = 0; leg < 4; leg++) {
    bool left = leg_left(leg);
    int16_t x = (int16_t)(g.xs * 4);
    int16_t z = (int16_t)(base + (leg_front(leg) ? -dz_p : dz_p) + (left ? -dz_r : dz_r));
    if (run) {
      uint16_t p = (uint16_t)(g.phi + (PAT_U8(g.pat, off[leg]) << 8));
      int32_t s_i = left ? g.stride - g.turn : g.stride + g.turn;
      s_i = (s_i * g.amp + 32768) >> 16;  // the start/stop ramp (rounded: left = -right)
      uint8_t bit = (uint8_t)(1u << leg);
      int32_t e;  // Q14: the foot's place in its stride, -1/2 (back) .. +1/2 (front)
      if (p < b16) {
        e = 8192 - (int32_t)(((uint32_t)p * inv_st) >> 16);  // 1/2 - s
        // A foot in stance may lift at its next swing; while landing, none
        // may (the stop waits until every foot is down).
        if (g.state == GS_LAND)
          g.armed &= (uint8_t)~bit;
        else
          g.armed |= bit;
      } else {
        int16_t a = (int16_t)((((uint32_t)(p - b16) * inv_sw) >> 16) * A16_180 >> 14);  // pi s
        e = (Q14_ONE - fx_cos(a)) / 2 - 8192;  // (1 - cos pi s)/2 - 1/2
        // Lift h (1 - cos 2 pi s)/2: zero vertical speed at lift-off and
        // touchdown, the full height at mid-swing (§3d writes h sin pi s,
        // whose vertical speed is largest at lift-off; see gait.h).
        if (g.armed & bit) z = (int16_t)(z - (((int32_t)g.step * (Q14_ONE - fx_cos((int16_t)(2 * a)))) >> 15));
      }
      x = (int16_t)(x + ((s_i * e) >> 14));
    }
    ik_leg(leg, x, z, &g.leg[leg], &g.leg[4 + leg]);
  }
}

int16_t gait_head(const Gait& g, uint8_t k) {
  if (!g.state) return 0;
  // Tilt: A_bob sin(2 * 2 pi phi); tail: A_wag sin(2 pi phi). phi * 5760 /
  // 65536 = phi * 45 / 512 a16 (at most 11520: fx_sin reduces it).
  int16_t a, amp;
  if (k == 1 && (g.flags & 1)) {  // KILO_GAIT_FLAGS_BOB
    a = (int16_t)(((uint32_t)g.phi * 45) >> 8);
    amp = GAIT_BOB_A16;
  } else if (k == 2 && (g.flags & 2)) {  // KILO_GAIT_FLAGS_WAG
    a = (int16_t)(((uint32_t)g.phi * 45) >> 9);
    amp = GAIT_WAG_A16;
  } else {
    return 0;
  }
  return (int16_t)(((((int32_t)amp * fx_sin(a)) >> 14) * g.amp) >> 16);
}
