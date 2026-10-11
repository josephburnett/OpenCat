#include "balance.h"
#include "joints.h"
#include "pgm.h"
#include "posture.h"

// ---- Per-joint gains -----------------------------------------------------
// The legacy adaptiveParameterArray {rollFactor, pitchFactor} (src/motion.h:
// panF 60, tiltF 60, uRF 60, uPF 30, lRF = -1.5 uRF, lPF = -1.5 uPF), which
// Kilo's legacy EEPROM holds byte for byte at ADAPT_PARAM (128). Units:
// joint degrees per 57.3 degrees of tilt (legacy divides by 57.3 through
// radPerDeg), so 60 is 1.05 deg of joint per deg of tilt. Canonical order
// [0,1,2,8,9,10,11,12,13,14,15]: pan, tilt, tail, shoulders LF RF RB LB,
// knees LF RF RB LB. Legacy joints 3-7 are never driven.
static const int8_t GAIN[DRIVEN_COUNT][2] PROGMEM = {
    {-60, 0},   {-60, -60}, {-120, 0},                        // pan, tilt, tail
    {60, 30},   {60, 30},   {-60, 30},  {-60, 30},            // shoulders
    {-90, -45}, {-90, -45}, {90, -45},  {90, -45},            // knees
};

// ---- The mapping ---------------------------------------------------------
// Legacy adjust(i) (src/motion.h), for a posture (postureOrWalkingFactor 1):
//   rollAdj = |dR| * rF * (1.2 on one side)   for i == 1 (tilt) and legs (i > 3)
//           =  dR  * rF                       for pan and tail
//       side: leftQ = (i - 1) % 4 > 1 gets 1.2 when dR > 0, the right side when dR < 0
//   adjust  = (rollAdj - pF * dP) / 57.3
// and skill.h applies it as  servo = frame + adjust  (reaction.h's
// "currentAng - currentAdjust" only undoes that to recover the frame).
//
// Here dR, dP are the legacy deviations, expressed through the corrections:
// with Kp = 1 and no Kd, corr = -e, so dR = -corr_roll and dP = -corr_pitch,
// each times LEGACY_*_SIGN (balance.h). Kilo's convention (measured): nose
// DOWN = +pitch, RIGHT side up = +roll. Physically, from the posture tables:
//   - +corr_pitch (dP < 0) moves the knees and back shoulders from STAND
//     AWAY from SIT (front knee -0.79 deg/deg, back knee -0.79, back
//     shoulder +0.52): the front folds and the back extends, so the nose
//     drops and the measured pitch rises;
//   - +corr_roll (dR < 0 after LEGACY_ROLL_SIGN) crouches every leg by
//     |corr| (toward REST) and the LEFT legs 1.2x as hard, so the left side
//     drops relative to the right: the right side rises, the measured roll
//     rises.
// So a positive correction raises the measured value on its axis, and
// corr = -Kp*e is negative feedback. Worked through: nose up 10 deg
// (pitch -10, ref 0) -> e = -8 after the 2 deg band -> corr_pitch = +4 at
// Kp 0.5 -> dP = -4 -> front knees -3.1 deg, back knees -3.1, back shoulders
// +2.1 (front folds, back extends) -> nose comes down. Right side up 10 deg
// (roll +10) -> corr_roll = -4 -> dR = -(-4) * (-1) = -4 -> all legs
// crouch 4 deg-worth, the RIGHT pair 1.2x -> the right side comes down.
// test_balance.cpp checks both against the posture tables.
//
// Fixed point: num5 = 5 * (rollAdj - pF*dP) in a16 (the 5 makes 1.2 = 6/5
// exact), then / (5 * 57.29578) = * 3660.2 / 2^20. |num5| <= 5 * (120 *
// 480 * 1.2 + 45 * 480) = 453600 at the 30 deg limit, * 3660 < 2^31.
int16_t balance_map(uint8_t k, int16_t corr_roll, int16_t corr_pitch) {
  uint8_t j = joint_id(k);
  int32_t r_f = PGM_I8(&GAIN[k][0]);
  int32_t p_f = PGM_I8(&GAIN[k][1]);
  int32_t d_r = -(int32_t)corr_roll * LEGACY_ROLL_SIGN;
  int32_t d_p = -(int32_t)corr_pitch * LEGACY_PITCH_SIGN;
  int32_t num5;
  if (j == 1 || j > 3) {
    bool left = ((uint8_t)(j - 1) & 3) > 1;
    int32_t f = ((left && d_r > 0) || (!left && d_r < 0)) ? 6 : 5;
    num5 = r_f * (d_r < 0 ? -d_r : d_r) * f;
  } else {
    num5 = r_f * d_r * 5;
  }
  num5 -= 5 * p_f * d_p;
  return (int16_t)((num5 * 3660 + ((int32_t)1 << 19)) >> 20);
}

// ---- The controller ------------------------------------------------------
// v / 2^n rounded half away from zero (symmetric, so a decaying correction
// reaches exactly 0 from either side).
KILO_NOINLINE static int32_t shr_round(int32_t v, uint8_t n) {
  int32_t h = (int32_t)1 << (n - 1);
  return v >= 0 ? (v + h) >> n : -((-v + h) >> n);
}

void balance_reset(Balance& b) {
  for (uint8_t a = 0; a < 2; a++) b.ax[a].lp = b.ax[a].corr = 0;
  b.active = 0;
}

void balance_init(Balance& b) {
  b.enable = 0;
  b.kp_q8 = BAL_KP_Q8;
  b.kd_ms = BAL_KD_MS;
  b.db_deg = BAL_DB_DEG;
  b.max_deg = BAL_MAX_DEG;
  balance_reset(b);
}

int16_t balance_axis_step(const Balance& b, BalAxis& s, bool active, int16_t measured, int16_t ref, int16_t rate) {
  int16_t max = (int16_t)(b.max_deg * 16);
  int32_t target = 0;
  if (active) {
    int32_t e = (int32_t)measured - ref;
    int32_t db = (int32_t)b.db_deg * 16;
    e = e > db ? e - db : e < -db ? e + db : 0;
    // Kp*e (Q8) + Kd*rate: rate a16/s * kd ms / 1000, as * 131 / 2^17
    // (1/1000.5; |rate * 255 * 131| < 2^31).
    int32_t u = shr_round(e * b.kp_q8, 8) + shr_round((int32_t)rate * b.kd_ms * 131, 17);
    target = -u;
    if (target > max) target = max;
    if (target < -max) target = -max;
  }
  // Low-pass, alpha = 1/4, with 4 extra fraction bits so small errors move
  // it; the last LSB is stepped too, so it settles exactly.
  int32_t diff = target * 16 - s.lp;
  int32_t step = shr_round(diff, 2);
  s.lp = (int16_t)(s.lp + (step ? step : diff));
  int16_t want = (int16_t)shr_round(s.lp, 4);
  int16_t d = (int16_t)(want - s.corr);
  if (d > BAL_SLEW_A16) d = BAL_SLEW_A16;
  if (d < -BAL_SLEW_A16) d = -BAL_SLEW_A16;
  int16_t c = (int16_t)(s.corr + d);
  if (c > max) c = max;
  if (c < -max) c = (int16_t)-max;
  s.corr = c;
  return c;
}

void balance_step(Balance& b, bool active, const Attitude& att, int16_t ref_roll, int16_t ref_pitch) {
  b.active = active;
  balance_axis_step(b, b.ax[BAL_ROLL], active, att.roll, ref_roll, att.rate_roll);
  balance_axis_step(b, b.ax[BAL_PITCH], active, att.pitch, ref_pitch, att.rate_pitch);
}
