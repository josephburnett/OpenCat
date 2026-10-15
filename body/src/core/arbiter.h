// arbiter: who moves each joint, and the safety stage (doc §3c).
//
// Each tick, for each of the 11 driven joints:
//   1. owner  = the highest-priority active owner whose mask holds the joint
//               REFLEX(3) > TRICK(2) > GAIT(1) > IDLE(0)
//   2. base   = that owner's angle; if the owner just changed, crossfade from
//               the previous base so the hand-over never jumps
//   3. layers = head (SET_HEAD hold, joints 0-2 under IDLE) and
//               balance (core/balance.h): joint-space owners (IDLE, TRICK)
//               add the balance correction, except on held head joints
//   4. safety = clamp to joint limits, slew-limit, convert to PCA counts
//
// IDLE (the built-in postures and STANCE), GAIT (the FEET player, core/
// feet.h: legs 8-15 while the mind's stream plays), TRICK (the keyframe
// player, core/keyframe.h) and REFLEX (battery, link watchdog, warm-boot
// hold, fall).
//
// Crossfades start from the joint's last OUTPUT (after balance, limits and
// slew), so a hand-over never jumps even when the previous owner's angle was
// beyond a limit or carried a balance correction. Into TRICK and REFLEX
// there is none (their first segment starts from that output); out of
// TRICK it lasts the player's xfade (TRICK_STOP transition_ticks, else
// XFADE_TICKS), out of REFLEX XFADE_TICKS.
#ifndef KILO_CORE_ARBITER_H
#define KILO_CORE_ARBITER_H
#include <stdint.h>
#include "balance.h"
#include "joints.h"
#include "keyframe.h"

// OWN_PI_HOLD is not an owner that supplies a base angle: it tags a head
// joint held by SET_HEAD (owner[] and TELEMETRY), whose real owner is IDLE
// or GAIT.
enum Owner : uint8_t { OWN_IDLE = 0, OWN_GAIT = 1, OWN_TRICK = 2, OWN_REFLEX = 3, OWN_PI_HOLD = 4 };

// Tunables are live params (core/params.h): XFADE_TICKS (out of TRICK or
// REFLEX, 10), POSTURE_TICKS (default IDLE transition, 25 = 0.5 s) and
// SLEW_MAX_A16 (160 = 10 deg per tick = 500 deg/s).

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
  uint8_t owner[DRIVEN_COUNT];  // owner on the previous tick (OWN_PI_HOLD: held head joint)
  int16_t xsrc[DRIVEN_COUNT];   // crossfade source: the output when the owner changed
                                // (also the TRICK start pose)
  uint8_t xn[DRIVEN_COUNT], xi[DRIVEN_COUNT];
  int16_t out[DRIVEN_COUNT];    // output after the safety stage (a16 or OFF)
  int16_t last_on[DRIVEN_COUNT];  // last powered output: slew reference
  // Head layer (SET_HEAD mode=1): targets for pan, tilt, tail (a16 as sent;
  // the safety stage clamps them; JOINT_OFF = that joint not held), the
  // slew in a16 per tick (1..0x3FFF), and the ticks left before the hold
  // lapses (0 = released).
  int16_t head_tgt[3];
  int16_t head_step;
  uint8_t head_ttl;
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

// Head layer (doc §3c layer 1). A hold (body_goal_head sets head_tgt,
// head_step, head_ttl): joints 0-2 follow head_tgt (JOINT_OFF = not held) at
// most head_step a16 per tick from their last output, for head_ttl ticks
// unless refreshed, whenever their owner is IDLE or GAIT (a trick that owns
// a head joint, or a reflex, wins). Held joints get no balance correction:
// the Pi is pointing them; owner[] says OWN_PI_HOLD (TELEMETRY's head owner
// nibble PI_HOLD). release(), or head_ttl running out: the joints
// crossfade back to their owner's angle over XFADE_TICKS.
// arbiter_head_held() is TELEMETRY flags bit 5 (HEAD_HELD).
// (A held joint is recorded as owned by OWN_PI_HOLD, so a release is a
// change of owner: the crossfade is the usual hand-over.)
inline void arbiter_head_release(Arbiter& a) { a.head_ttl = 0; }
inline bool arbiter_head_held(const Arbiter& a) { return a.head_ttl != 0; }

// True while the IDLE mover is still moving to its posture.
inline bool arbiter_idle_moving(const Arbiter& a) { return a.idle.i < a.idle.n; }

// One tick of steps 1-4. Writes all 16 channel counts (unused channels and
// LIMP joints full-off). bal: the balance corrections to add to joints owned
// by a joint-space owner (null = none). The caller decides when balance runs
// (body.cpp); this only applies the current corrections. kf: the keyframe
// player (null = no TRICK owner); call kf_step() first. legs: this tick's
// IK leg angles (feet_legs: shoulders LF RF RB LB, knees LF RF RB LB), or
// null (STANCE has none then). gait: the FEET stream owns the legs (GAIT
// supplies legs[]).
void arbiter_run(Arbiter& a, const JointCal& cal, uint16_t counts[JOINT_COUNT], const Balance* bal,
                 const Keyframe* kf, const int16_t* legs = 0, bool gait = false);

#endif
