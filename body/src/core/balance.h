// balance: the joint-space balance layer (doc §3f, §3c layer 2). P1.
//
// Per axis (roll, pitch), once per 50 Hz tick:
//   e    = deadband(measured - ref, DB)                 DB default 2 deg
//   u    = Kp*e + Kd*rate                               rate = DMP gyro on that axis
//   corr = clamp(slew(lowpass(-u, 1/4), 1 deg/tick), +-max)   max default 15 deg
// corr follows §3d: positive corr_pitch raises the nose, positive corr_roll
// raises the left side. Nose high (e > 0) gives corr < 0: negative feedback.
// (-u is also saturated at +-max before the low-pass, so the filter state
// cannot wind up past what the clamp lets out.)
//
// Joint-space owners (IDLE postures; tricks with BALANCE later) add
// balance_joint(k) to each joint's base angle, after the owner/crossfade
// stage and before limits and slew. The per-joint gains are the legacy
// adaptiveParameterArray (src/motion.h, also Kilo's EEPROM ADAPT_PARAM @128),
// applied the way legacy adjust() applied them, including its roll
// asymmetry: legs and head tilt get |roll| times their roll factor (a
// symmetric crouch) with 1.2x on one side. balance.cpp has the details and
// the sign derivation.
//
// Fixed point throughout: angles a16 (1/16 deg), Kp Q8, Kd in ms.
#ifndef KILO_CORE_BALANCE_H
#define KILO_CORE_BALANCE_H
#include <stdint.h>
#include "attitude.h"

// SET_BALANCE defaults (doc §3f, est.): Kp 0.5, Kd 0.03 s, DB 2 deg, 15 deg.
#define BAL_KP_Q8 128
#define BAL_KD_MS 30
#define BAL_DB_DEG 2
#define BAL_MAX_DEG 15
#define BAL_MAX_DEG_LIMIT 30  // SET_BALANCE max_corr_deg / deadband_deg above this: BAD_ARG
#define BAL_SLEW_A16 16       // 1 deg per tick

enum BalAxisId : uint8_t { BAL_ROLL = 0, BAL_PITCH = 1 };

struct BalAxis {
  int16_t lp;    // low-pass state, a16 * 16 (4 extra fraction bits)
  int16_t corr;  // output after slew and clamp, a16
};

struct Balance {
  // SET_BALANCE goal (live, not persisted).
  uint8_t enable;  // 1 = balance whenever eligible, even without the posture flag
  uint8_t kp_q8, kd_ms, db_deg, max_deg;
  // State.
  uint8_t active;  // the controller ran with active = true this tick (TELEMETRY flag bit 0)
  BalAxis ax[2];   // [BAL_ROLL], [BAL_PITCH]
};

// Defaults, controller at rest.
void balance_init(Balance& b);
// Zero the corrections at once (REFLEX, LIMP, CALIB, a posture change: the
// new owner or mover starts from the corrected output, so nothing jumps).
void balance_reset(Balance& b);

// One controller step for one axis. measured, ref in a16; rate in a16/s
// (d measured / dt). active = false drives the target to 0 (the correction
// decays through the filter and slew). Returns the new corr. Exposed for the
// host toy-plant test; balance_step() calls it for both axes.
int16_t balance_axis_step(const Balance& b, BalAxis& s, bool active, int16_t measured, int16_t ref, int16_t rate);

// Both axes from the attitude; sets b.active.
void balance_step(Balance& b, bool active, const Attitude& att, int16_t ref_roll, int16_t ref_pitch);

// Joint-space mapping: the a16 offset for canonical joint k (0..10) given
// the two corrections (a16). Pure; exposed for the legacy-mapping test.
int16_t balance_map(uint8_t k, int16_t corr_roll, int16_t corr_pitch);

// balance_map of the current corrections.
inline int16_t balance_joint(const Balance& b, uint8_t k) {
  return balance_map(k, b.ax[BAL_ROLL].corr, b.ax[BAL_PITCH].corr);
}

#endif
