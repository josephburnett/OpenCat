#include "ik.h"
#include "fx.h"
#include "params.h"
#include "pgm.h"

// Lengths in q2 from the live params (mm).
static inline int32_t l1_q2(void) { return (int32_t)PARAM(LEG_L1_MM) * 4; }
static inline int32_t l2_q2(void) { return (int32_t)PARAM(LEG_L2_MM) * 4; }

int16_t ik_reach_q2(void) { return (int16_t)(l1_q2() + l2_q2()); }

KILO_NOINLINE bool ik_leg(uint8_t leg, int16_t x_q2, int16_t z_q2, int16_t* shoulder_a16, int16_t* knee_a16) {
  int32_t l1 = l1_q2(), l2 = l2_q2();
  int32_t l1l2 = l1 * l2;
  int32_t sq = l1 * l1 + l2 * l2;
  // Workspace: d <= 0.95 (L1 + L2) (243/256 = 0.949; the knee keeps >= ~36
  // deg of bend for L1 = L2, so the c/s error amplification stays small:
  // Appendix B6), and d^2 >= L1^2 + L2^2 - 2 L1 L2 cos 30 deg (bend <= 150
  // deg; 7094/4096 = 1.732 = 2 cos 30). With L <= 480 q2 these are < 2^20.
  int32_t dmax = ((l1 + l2) * 243) >> 8;
  int32_t d2max = dmax * dmax;
  int32_t d2min = sq - ((l1l2 * 7094) >> 12);
  int32_t d2 = (int32_t)x_q2 * x_q2 + (int32_t)z_q2 * z_q2;
  bool ok = true;
  if (d2 > d2max) {
    d2 = d2max;
    ok = false;
  } else if (d2 < d2min) {
    d2 = d2min;
    ok = false;
  }
  // c = (d^2 - L1^2 - L2^2) / (2 L1 L2), Q12. |num| <= 2 L1 L2 <= 460800,
  // * 4096 < 2^31. Rounded to nearest; clamped against rounding.
  int32_t den = 2 * l1l2;
  int32_t num = (d2 - sq) * 4096;
  int32_t c = (num + (num >= 0 ? den / 2 : -den / 2)) / den;
  if (c > 4096) c = 4096;
  if (c < -4096) c = -4096;
  int32_t s = fx_isqrt32((uint32_t)((int32_t)4096 * 4096 - c * c));  // Q12, >= 0
  int8_t sigma = ik_sigma(leg);
  // t2 = sigma atan2(s, c); t1 = atan2(x, z) - sigma atan2(L2 s, L1 + L2 c).
  // (atan2 normalises both arguments by a shared shift: Appendix B6.)
  int16_t t2 = fx_atan2(s, c);
  int16_t t1 = (int16_t)(fx_atan2(x_q2, z_q2) - fx_atan2(l2 * s, l1 * 4096 + l2 * c) * sigma);
  t2 = (int16_t)(t2 * sigma);
  *shoulder_a16 = (int16_t)(IK_SGN_SH * (t1 - PARAM(LEG_SH_ZERO_DEG) * A16_PER_DEG));
  *knee_a16 = (int16_t)(IK_SGN_KN * (t2 - sigma * PARAM(LEG_KN_ZERO_DEG) * A16_PER_DEG));
  return ok;
}

void ik_fk(uint8_t leg, int16_t shoulder_a16, int16_t knee_a16, int16_t* x_q2, int16_t* z_q2) {
  // Inverse of the joint mapping (the signs are +-1, so they are their own
  // inverse), then the FK above in Q14.
  int16_t t1 = (int16_t)(PARAM(LEG_SH_ZERO_DEG) * A16_PER_DEG + IK_SGN_SH * shoulder_a16);
  int16_t t12 = (int16_t)(t1 + ik_sigma(leg) * PARAM(LEG_KN_ZERO_DEG) * A16_PER_DEG + IK_SGN_KN * knee_a16);
  int32_t l1 = l1_q2(), l2 = l2_q2();
  int32_t x = l1 * fx_sin(t1) + l2 * fx_sin(t12);
  int32_t z = l1 * fx_cos(t1) + l2 * fx_cos(t12);
  *x_q2 = (int16_t)((x + 8192) >> 14);
  *z_q2 = (int16_t)((z + 8192) >> 14);
}
