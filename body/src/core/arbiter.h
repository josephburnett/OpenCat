// arbiter: who moves each joint, and the safety stage (doc §3c).
//
// Each tick, for each of the 11 driven joints:
//   1. owner  = the highest-priority active owner whose mask holds the joint
//               REFLEX(3) > TRICK(2) > GAIT(1) > IDLE(0)
//   2. base   = that owner's angle; if the owner just changed, crossfade from
//               the previous base so the hand-over never jumps
//   3. layers = head and balance (hooks; P1)
//   4. safety = clamp to joint limits, slew-limit, convert to PCA counts
//
// In P0 only IDLE (the built-in postures) and REFLEX (battery, link
// watchdog, warm-boot hold) exist. TRICK and GAIT are hooks that never claim
// a joint yet; see arbiter.cpp.
#ifndef KILO_CORE_ARBITER_H
#define KILO_CORE_ARBITER_H
#include <stdint.h>
#include "joints.h"

enum Owner : uint8_t { OWN_IDLE = 0, OWN_GAIT = 1, OWN_TRICK = 2, OWN_REFLEX = 3 };

#define XFADE_TICKS 10        // out of TRICK or REFLEX (param XFADE_TICKS)
#define POSTURE_TICKS 25      // default IDLE posture transition (0.5 s)
#define SLEW_MAX_A16 160      // 10 deg per tick = 500 deg/s (param SLEW_MAX_A16)

// Moves all driven joints from a snapshot to a posture with cosine easing.
// IDLE and REFLEX each own one.
struct Mover {
  int16_t from[DRIVEN_COUNT];  // a16 or JOINT_OFF
  uint8_t posture;             // PostureId (incl. LIMP, HOLD)
  uint8_t n, i;                // transition length and progress, ticks
};

struct Arbiter {
  Mover idle, reflex;
  uint8_t reflex_active;
  uint8_t owner[DRIVEN_COUNT];  // owner on the previous tick
  int16_t xsrc[DRIVEN_COUNT];   // crossfade source (previous base)
  uint8_t xn[DRIVEN_COUNT], xi[DRIVEN_COUNT];
  int16_t base[DRIVEN_COUNT];   // base pose after crossfade (a16 or OFF)
  int16_t out[DRIVEN_COUNT];    // output after the safety stage (a16 or OFF)
  int16_t last_on[DRIVEN_COUNT];  // last powered output: slew reference
};

// warm = null: cold start, everything LIMP.
// warm = the angles read back from the PCA9685: hold them with the REFLEX
// owner (JOINT_OFF entries stay limp).
void arbiter_init(Arbiter& a, const int16_t* warm);

// IDLE posture goal. Re-sending the current posture is a no-op (goals are
// refreshed at 5 Hz and must not restart the move). ticks 0 = default.
void arbiter_set_idle(Arbiter& a, uint8_t posture, uint8_t ticks);
uint8_t arbiter_idle_posture(const Arbiter& a);

// REFLEX takes every driven joint and moves it from the current output to
// `posture` over `ticks` (its first frame starts at the current pose, so no
// crossfade is needed on entry). release() hands back with a crossfade.
void arbiter_reflex(Arbiter& a, uint8_t posture, uint8_t ticks);
void arbiter_reflex_release(Arbiter& a);
// True once the active mover has reached its target.
bool arbiter_reflex_settled(const Arbiter& a);

// One tick of steps 1-4. Writes all 16 channel counts (unused channels and
// LIMP joints full-off).
void arbiter_run(Arbiter& a, const JointCal& cal, uint16_t counts[JOINT_COUNT]);

#endif
