#include "feet.h"
#include <string.h>
#include "fx.h"
#include "ik.h"
#include "params.h"
#include "pgm.h"

#define SLEW_POSE 8    // pose pitch/roll: 0.5 deg per tick
#define POSE_MAX 480   // +-30 deg
#define SHARE_FULL 4u  // a leg's whole share of the correction, quarters

static_assert((FEET_DEPTH & (FEET_DEPTH - 1)) == 0 && FEET_DEPTH <= 8, "the queue's slots are tick bits");

static inline uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// v toward t by at most s.
KILO_NOINLINE static int16_t slew(int16_t v, int16_t t, int16_t s) {
  if (t > v + s) return (int16_t)(v + s);
  if (t < v - s) return (int16_t)(v - s);
  return t;
}

void feet_init(Feet& f) {
  memset(&f, 0, sizeof f);
  feet_stance_default(f);
}

void feet_stance_default(Feet& f) { f.h = (int16_t)(((int32_t)ik_reach_q2() * KILO_GAIT_STANCE_H_256) >> 8); }

void feet_pose_goal(Feet& f, const uint8_t* p) {
  for (uint8_t i = 0; i < 2; i++) {
    int16_t v = (int16_t)rd16(p + 2 * i);
    f.pose_g[i] = v > POSE_MAX ? POSE_MAX : v < -POSE_MAX ? (int16_t)-POSE_MAX : v;
  }
  f.dh_g = (int8_t)p[4];
  f.xs_g = (int8_t)p[5];
}

void feet_pose_step(Feet& f) {
  for (uint8_t i = 0; i < 2; i++) f.pose[i] = slew(f.pose[i], f.pose_g[i], SLEW_POSE);
  f.dh = (int8_t)slew(f.dh, f.dh_g, 1);
  f.xs = (int8_t)slew(f.xs, f.xs_g, 1);
}

// Payload offsets (kilo_feet_t; link.cpp static_asserts them): epoch 0,
// tick 1, x_mm 3, z_mm 7, pitch 11, roll 13, support 15.
uint8_t feet_check(const uint8_t* p) {
  for (uint8_t i = 0; i < 4; i++) {
    int8_t x = (int8_t)p[3 + i];
    uint8_t z = p[7 + i];
    if (x < -60 || x > 60 || z < 30 || z > 95) return 2;  // GOAL_BAD_ARG (core/body.h)
  }
  return (p[15] & 0xF0u) ? 2 : 0;
}

bool feet_push(Feet& f, const uint8_t* p, uint16_t now) {
  uint16_t ahead = (uint16_t)(rd16(p + 1) - now);
  if (ahead == 0 || ahead > FEET_DEPTH) {
    if (f.late < 0xFFFF) f.late++;
    return false;
  }
  uint8_t s = (uint8_t)(p[1] & (FEET_DEPTH - 1));
  FeetPose& e = f.q[s];
  for (uint8_t i = 0; i < 4; i++) {
    e.x[i] = (int8_t)p[3 + i];
    e.zs[i] = (uint8_t)(p[7 + i] | ((p[15] >> i) & 1 ? FEET_SUP : 0));
  }
  e.pitch = (int16_t)rd16(p + 11);
  e.roll = (int16_t)rd16(p + 13);
  f.have |= (uint8_t)(1u << s);
  f.rx_tick = now;
  if (!f.state) {
    f.state = FT_WAIT;
    f.share = 0x4444;  // from STANCE: every leg carries its whole share
  }
  return true;
}

void feet_stop(Feet& f) {
  f.state = FT_OFF;
  f.have = 0;
  f.held = 0;
}

uint8_t feet_support(const Feet& f) {
  uint8_t s = 0;
  for (uint8_t i = 0; i < 4; i++)
    if (f.cur.zs[i] & FEET_SUP) s |= (uint8_t)(1u << i);
  return s;
}

uint8_t feet_queued(const Feet& f) {
  uint8_t n = 0;
  for (uint8_t h = f.have; h; h &= (uint8_t)(h - 1)) n++;
  return n;
}

int16_t feet_ref(const Feet& f, uint8_t axis) {
  int16_t r = f.pose[axis];
  if (feet_playing(f)) r = (int16_t)(r + (axis ? f.cur.roll : f.cur.pitch));
  return r;
}

// A foot in the body's plane, doubled so that half the hip spacing stays
// whole (mm x 2): X forward from the body's centre (hips at +-Lb/2, the foot
// x ahead of its hip), Y to the right (hips at +-W/2). |X| <= 200 + 120,
// |Y| <= 150 at the parameters' limits.
KILO_NOINLINE static void foot_xy2(const Feet& f, uint8_t i, int16_t* X, int16_t* Y) {
  int16_t lb = PARAM(LEG_LB_MM), w = PARAM(LEG_W_MM);
  *X = (int16_t)((leg_front(i) ? lb : -lb) + 2 * f.cur.x[i]);
  *Y = leg_left(i) ? (int16_t)-w : w;
}

// a x b for 2-D vectors in doubled mm, int16 x int16 products.
KILO_NOINLINE static int32_t cross(int16_t ax, int16_t ay, int16_t bx, int16_t by) {
  return (int32_t)ax * by - (int32_t)ay * bx;
}

// The ground's depth (mm below the hip) at foot `i` on the plane through
// the three support feet a, b, c (the leaning body on three feet: the walk
// tilts the body up to ~14 deg, so the support depths differ by up to
// ~16 mm). Barycentric: P - A = s (B - A) + t (C - A) with s = (P-A)x(C-A) / D,
// t = (B-A)x(P-A) / D, D = (B-A)x(C-A); z = zA + s (zB - zA) + t (zC - zA).
// In doubled mm the differences are <= 640 (X) and 300 (Y), each cross
// product <= 2 x 640 x 300 = 384000 and the numerator <= 2 x 65 x 384000 =
// 49.9 M: int32. The quotient (s, t may be large for a thin triangle) is
// capped at 127 mm before it goes back to int16, then the depth is rounded
// to the nearest mm and kept in FEET's 30..95. -1 when the three are on one
// line (D = 0: no plane, the caller takes the mean).
KILO_NOINLINE static int16_t ground_plane(const Feet& f, uint8_t i, const uint8_t abc[3]) {
  int16_t X[4], Y[4], z[3];  // [0..2] the support feet, [3] foot i
  for (uint8_t k = 0; k < 4; k++) {
    uint8_t l = k < 3 ? abc[k] : i;
    foot_xy2(f, l, &X[k], &Y[k]);
    if (k < 3) z[k] = (int16_t)(f.cur.zs[l] & 0x7F);
  }
  for (uint8_t k = 1; k < 4; k++) X[k] = (int16_t)(X[k] - X[0]), Y[k] = (int16_t)(Y[k] - Y[0]);
  int32_t d = cross(X[1], Y[1], X[2], Y[2]);
  if (!d) return -1;
  int32_t num = cross(X[3], Y[3], X[2], Y[2]) * (int16_t)(z[1] - z[0]) +
                cross(X[1], Y[1], X[3], Y[3]) * (int16_t)(z[2] - z[0]);
  if (d < 0) d = -d, num = -num;
  bool neg = num < 0;
  int32_t q32 = ((neg ? -num : num) + d / 2) / d;
  int16_t q = (int16_t)(q32 > 127 ? 127 : q32);
  q = (int16_t)(z[0] + (neg ? -q : q));
  return q < 30 ? 30 : q > 95 ? 95 : q;
}

// The settle: every foot off the ground goes down (or up) toward the ground
// at its x by FEET_SETTLE_MM per tick. True when all four are down. The
// ground: the plane through the support feet when three are down (the body
// leans on them: a foot set at their mean depth would land above or into
// the floor), else the support feet's mean depth, else STANCE's default
// height when none are down.
static bool settle(Feet& f) {
  uint16_t sum = 0;
  uint8_t n = 0, abc[3];
  for (uint8_t i = 0; i < 4; i++)
    if (f.cur.zs[i] & FEET_SUP) {
      sum = (uint16_t)(sum + (f.cur.zs[i] & 0x7Fu));
      if (n < 3) abc[n] = i;
      n++;
    }
  uint8_t mean = n ? (uint8_t)((sum + n / 2) / n)
                   : (uint8_t)((((int32_t)ik_reach_q2() * KILO_GAIT_STANCE_H_256) >> 8) / 4);
  bool all = true;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t z = f.cur.zs[i];
    if (z & FEET_SUP) continue;
    int16_t p = n == 3 ? ground_plane(f, i, abc) : -1;
    uint8_t g = p < 0 ? mean : (uint8_t)p;
    if (z + FEET_SETTLE_MM < g)
      z = (uint8_t)(z + FEET_SETTLE_MM), all = false;
    else if (z > g + FEET_SETTLE_MM)
      z = (uint8_t)(z - FEET_SETTLE_MM), all = false;
    else
      z = (uint8_t)(g | FEET_SUP);
    f.cur.zs[i] = z;
  }
  return all;
}

uint8_t feet_step(Feet& f, uint16_t tick) {
  if (!f.state) return 0;
  uint8_t ev = 0;
  uint8_t bit = (uint8_t)(1u << (tick & (FEET_DEPTH - 1)));
  if (f.have & bit) {
    f.have &= (uint8_t)~bit;
    f.cur = f.q[tick & (FEET_DEPTH - 1)];
    f.state = FT_PLAY;
  } else if (f.state == FT_PLAY && !f.have) {  // ran dry: hold
    f.state = FT_HOLD;
    f.held = 0;
    if (f.underruns < 0xFFFF) f.underruns++;
    ev = FEET_EV_UNDERRUN;
  }
  // (FT_PLAY with later ticks queued: a lost frame, the pose holds.)
  if (f.state == FT_HOLD && !f.have && ++f.held > FEET_HOLD_TICKS) f.state = FT_SETTLE;
  if (f.state == FT_SETTLE && settle(f)) ev |= FEET_EV_SETTLED;
  if (f.state >= FT_PLAY) {
    // Each leg's share of the correction: a quarter per tick toward its
    // support bit (feet.h).
    uint16_t sh = 0;
    for (uint8_t i = 0; i < 4; i++) {
      uint8_t s = (uint8_t)((f.share >> (4 * i)) & 0xFu);
      if (f.cur.zs[i] & FEET_SUP) {
        if (s < SHARE_FULL) s++;
      } else if (s) {
        s--;
      }
      sh |= (uint16_t)((uint16_t)s << (4 * i));
    }
    f.share = sh;
  }
  return ev;
}

// (L/2) sin(a) in q2 for a length L in mm, rounded (truncating would lose up
// to 0.25 mm per leg, 0.2 deg of body tilt).
KILO_NOINLINE static int16_t half_sin_q2(int16_t len_mm, int16_t a) {
  return (int16_t)(((int32_t)len_mm * 2 * fx_sin(a) + 8192) >> 14);
}

void feet_legs(const Feet& f, int16_t corr_roll, int16_t corr_pitch, int16_t leg[8]) {
  bool play = feet_playing(f);
  // Pose terms (doc §3d): the raised end or side gets the longer legs.
  // STANCE: pose and correction together on all four legs. FEET: the pose
  // on all four, the correction on each leg by its share.
  int16_t pitch = f.pose[0], roll = f.pose[1];
  if (f.geo && !play) {
    pitch = (int16_t)(pitch + corr_pitch);
    roll = (int16_t)(roll + corr_roll);
  }
  int16_t lb = PARAM(LEG_LB_MM), w = PARAM(LEG_W_MM);
  int16_t dz_p = half_sin_q2(lb, pitch), dz_r = half_sin_q2(w, roll);
  int16_t dc_p = 0, dc_r = 0;
  if (f.geo && play) {
    dc_p = half_sin_q2(lb, corr_pitch);
    dc_r = half_sin_q2(w, corr_roll);
  }
  for (uint8_t i = 0; i < 4; i++) {
    int16_t ep = leg_front(i) ? -1 : 1, sr = leg_left(i) ? -1 : 1;
    int16_t x = (int16_t)(f.xs * 4);
    int16_t z = (int16_t)(f.dh * 4 + ep * dz_p + sr * dz_r);
    if (play) {
      x = (int16_t)(x + f.cur.x[i] * 4);
      z = (int16_t)(z + (f.cur.zs[i] & 0x7F) * 4);
      int16_t share = (int16_t)((f.share >> (4 * i)) & 0xFu);
      z = (int16_t)(z + ((ep * dc_p + sr * dc_r) * share) / (int16_t)SHARE_FULL);
    } else {
      z = (int16_t)(z + f.h);
    }
    ik_leg(i, x, z, &leg[i], &leg[4 + i]);
  }
}
