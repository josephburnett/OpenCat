#include "gait.h"
#include <string.h>
#include "fx.h"
#include "ik.h"
#include "params.h"
#include "pgm.h"

// Per pattern (doc §3d, spec [gait_patterns.*]: the numbers and their
// rationale live in protocol/spec.toml): defaults for the fields a SET_GAIT
// may leave 0 (outside their spec range: period, duty, body height), and the
// leg phase offsets in 1/256 cycle (LF, RF, RB, LB). A leg swings when phi +
// offset crosses duty, so a larger offset swings earlier. (The spec's stride
// and step defaults are the mind's: stride 0 is a real value, turning in
// place; likewise step 0, a shuffle.) The row layout is the one
// KILO_GAIT_PATTERN_TABLE_INIT fills, in enum gait_pattern order.
struct PatRow {
  uint16_t period_ms;
  uint8_t duty, h_256;  // h_256: body height / (L1 + L2), /256
  uint8_t off[4];
};
static const PatRow PAT[KILO_GAIT_PATTERN_COUNT] PROGMEM = KILO_GAIT_PATTERN_TABLE_INIT;
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
  g.down = 0xF;
  gait_stance_default(g);
}

void gait_stance_default(Gait& g) { g.h = (int16_t)(((int32_t)ik_reach_q2() * KILO_GAIT_STANCE_H_256) >> 8); }

uint8_t gait_check(const uint8_t* p) {
  uint16_t per = rd16(p + 3);
  int8_t stride = (int8_t)p[6], turn = (int8_t)p[7];
  bool bad = p[1] > 1 || p[2] > 2 || (per && (per < 300 || per > 2000)) || (p[5] && (p[5] < 128 || p[5] > 230)) ||
             stride < -60 || stride > 60 || turn < -40 || turn > 40 || p[8] > 40 || (p[10] & ~63u);
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
  // The sway's lead: dphi * lead ticks (spec range: <= 4369 * 15 < 2^16).
  g.lead = (uint16_t)(((uint32_t)g.dphi * GAIT_SWAY_LEAD_Q6) >> 6);
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
  memset(g.place, 0, sizeof g.place);
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

// A16 * sum (w Q8) scaled by the ramp: / 256 * amp / 65536, as amp's high
// byte (16-bit shifts are byte moves on the AVR).
KILO_NOINLINE static int16_t sway_ramp(int32_t v, uint16_t amp) { return (int16_t)((v * (uint8_t)(amp >> 8)) >> 16); }

// Leg i's phase this tick.
static uint16_t leg_phase(const Gait& g, uint8_t leg) { return (uint16_t)(g.phi + (PAT_U8(g.pat, off[leg]) << 8)); }

void gait_legs(Gait& g, int16_t corr_roll, int16_t corr_pitch, const Attitude* att) {
  bool run = g.state != GS_OFF;
  // Early touchdown (gait.h): the legs on the side (end) the body tips
  // toward, bits as armed (LF RF RB LB; left = LF LB 0x9, front = LF RF 0x3).
  uint8_t tip = 0;
  if (att && g.state == GS_RUN && (g.flags & KILO_GAIT_FLAGS_CATCH)) {
    if (att->rate_roll > GAIT_TOUCHDOWN_A16S) tip = 0x9;
    if (att->rate_roll < -GAIT_TOUCHDOWN_A16S) tip = 0x6;
    if (att->rate_pitch > GAIT_TOUCHDOWN_A16S) tip |= 0x3;
    if (att->rate_pitch < -GAIT_TOUCHDOWN_A16S) tip |= 0xC;
  }
  // Forward placement (gait.h): the shift a leg lifting this tick takes.
  int16_t cap = 0;
  if (att && g.state == GS_RUN && (g.flags & KILO_GAIT_FLAGS_PLACE)) {
    int32_t v = ((int32_t)att->rate_pitch * GAIT_CAPTURE_Q16 + 32768) >> 16;  // rounded
    cap = (int16_t)(v > GAIT_CAPTURE_MAX_Q2 ? GAIT_CAPTURE_MAX_Q2 : v < -GAIT_CAPTURE_MAX_Q2 ? -GAIT_CAPTURE_MAX_Q2 : v);
  }
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
  // Sway (gait.h): w_i = (1 + cos(pi d / hw))/2 for |d| < hw, d = the leg's
  // phase from mid-swing, hw = 1.5 swings (half a cycle at most).
  int16_t sr = 0, sp = 0;  // sum side_i w_i, sum end_i w_i (w Q8: |sum| <= 512)
  if (run && (g.flags & KILO_GAIT_FLAGS_SWAY)) {
    // swing sw = 65536 - b16 <= 32768, hw <= 49152. k: a16 per unit of
    // phase, Q16: 180 deg / hw = 2880 * 2/3 / sw = inv_sw * 1920 / 2^30, i.e.
    // inv_sw * 30 >> 8 (no division; <= 30720); 2880 / 32768 when hw would
    // pass half a cycle (duty < 2/3: the window is the whole cycle).
    uint16_t sw = (uint16_t)(0u - b16), hw = (uint16_t)(sw + (sw >> 1));
    uint16_t k = (uint16_t)((inv_sw * 30) >> 8);
    if (hw > 32768) hw = 32768, k = A16_180 * 2;
    uint16_t mid = (uint16_t)(32768u + (b16 >> 1) - g.lead);  // mid-swing, less the lead
    for (uint8_t leg = 0; leg < 4; leg++) {
      int16_t d = (int16_t)(leg_phase(g, leg) - mid);
      uint16_t ad = d < 0 ? (uint16_t)(0u - (uint16_t)d) : (uint16_t)d;
      if (ad > hw) continue;
      // ad * k <= hw * k ~ 1.5 * 2^16 * 1920: < 2^28.
      int16_t w = (int16_t)((Q14_ONE + fx_cos((int16_t)(((uint32_t)ad * k) >> 16))) >> 7);
      sr = (int16_t)(sr + (leg_left(leg) ? -w : w));
      sp = (int16_t)(sp + (leg_front(leg) ? -w : w));
    }
  }
  g.plan_roll = sway_ramp((int32_t)GAIT_SWAY_ROLL_A16 * sr, g.amp);
  g.plan_pitch = sway_ramp((int32_t)GAIT_SWAY_PITCH_A16 * sp, g.amp);
  // Pose terms (doc §3d): the raised end or side gets the longer legs.
  int16_t pitch = (int16_t)(g.pose[0] + g.plan_pitch), roll = (int16_t)(g.pose[1] + g.plan_roll);
  if (g.geo) {
    pitch = (int16_t)(pitch + corr_pitch);
    roll = (int16_t)(roll + corr_roll);
  }
  // (Lb/2) sin and (W/2) sin in q2, rounded (truncating would lose up to
  // 0.25 mm per leg, 0.2 deg of body tilt).
  int16_t dz_p = (int16_t)(((int32_t)PARAM(LEG_LB_MM) * 2 * fx_sin(pitch) + 8192) >> 14);
  int16_t dz_r = (int16_t)(((int32_t)PARAM(LEG_W_MM) * 2 * fx_sin(roll) + 8192) >> 14);
  int16_t base = (int16_t)(g.h + g.dh * 4);
  uint8_t down = 0xF;
  for (uint8_t leg = 0; leg < 4; leg++) {
    bool left = leg_left(leg);
    int16_t x = (int16_t)(g.xs * 4);
    int16_t z = (int16_t)(base + (leg_front(leg) ? -dz_p : dz_p) + (left ? -dz_r : dz_r));
    if (run) {
      uint16_t p = leg_phase(g, leg);
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
        // whose vertical speed is largest at lift-off; see gait.h). Tipping
        // toward this leg: down now, for the rest of the swing.
        if (tip & bit) g.armed &= (uint8_t)~bit;
        // Lift-off (armed, on the ground last tick): this swing's placement.
        // A swing that does not lift (never armed, or caught on this tick)
        // gets none, set on its first tick; a leg caught later in its swing
        // keeps its shift (dropping it would jump the foot).
        if ((g.down & bit) && ((g.armed & bit) || (uint16_t)(p - b16) < g.dphi))
          g.place[leg] = (int8_t)((g.armed & bit) ? cap : 0);
        if (g.armed & bit) {
          z = (int16_t)(z - (((int32_t)g.step * (Q14_ONE - fx_cos((int16_t)(2 * a)))) >> 15));
          down &= (uint8_t)~bit;  // armed in swing: in the air
        }
      }
      x = (int16_t)(x + ((s_i * e) >> 14));
      // Placement: place (e + 1/2), i.e. (1 - cos pi s)/2 in the swing and
      // 1 - s in the stance after it.
      // (Rounded: a floor would put a negative shift's first and last
      // quarter-millimetre in at once.)
      if (g.place[leg]) x = (int16_t)(x + ((g.place[leg] * (e + 8192) + 8192) >> 14));
    }
    ik_leg(leg, x, z, &g.leg[leg], &g.leg[4 + leg]);
  }
  g.down = down;
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
