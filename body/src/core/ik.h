// ik: planar two-link leg kinematics in fixed point (doc §3d "Planar IK",
// Appendix B6). P2.
//
// Leg frame: origin at the shoulder axle, x forward, z DOWN, lengths in q2
// (0.25 mm: 50 mm = 200). theta1 is the upper link's angle from straight
// down (+ = foot forward), theta2 the knee bend relative to the upper link
// (its sign is the knee direction sigma):
//   FK: x = L1 sin t1 + L2 sin(t1 + t2),  z = L1 cos t1 + L2 cos(t1 + t2)
//   IK: d^2 = x^2 + z^2, clamped to [d_min^2, (0.95 (L1 + L2))^2]
//       c  = (d^2 - L1^2 - L2^2) / (2 L1 L2)  Q12, one int32 division
//       s  = sigma sqrt(1 - c^2)             Q12, fx_isqrt32
//       t2 = atan2(s, c),  t1 = atan2(x, z) - atan2(L2 s, L1 + L2 c)
//   joints (OpenCat angles, a16, before the per-joint dir of joints.h):
//       shoulder = IK_SGN_SH (t1 - t1_0),  knee = IK_SGN_KN (t2 - sigma t2_0)
// L1, L2 are the live params LEG_L1_MM / LEG_L2_MM; t1_0 = LEG_SH_ZERO_DEG
// and t2_0 = LEG_KN_ZERO_DEG describe the calibration posture (all joints 0:
// Petoi says upper leg vertical, lower leg at 90 deg).
//
// Clamping d (rather than refusing) keeps the foot on the ray toward the
// target, so a target outside the workspace still gives the nearest
// reachable foot in the same direction; t1 only depends on the direction
// and c, so clamping d^2 before the division is exactly that. d_min keeps
// the knee bend at most 150 deg (cos 150 = -0.866).
//
// Integers only (no float on the AVR path); every intermediate fits int32
// for L1, L2 up to the params' 120 mm (Appendix B6).
#ifndef KILO_CORE_IK_H
#define KILO_CORE_IK_H
#include <stdint.h>

// Legs in the protocol order LF, RF, RB, LB (shoulders = joints 8..11,
// knees 12..15; canonical k = 3 + leg and 7 + leg).
enum LegId : uint8_t { LEG_LF = 0, LEG_RF = 1, LEG_RB = 2, LEG_LB = 3 };
inline bool leg_front(uint8_t leg) { return leg < 2; }
inline bool leg_left(uint8_t leg) { return leg == LEG_LF || leg == LEG_LB; }

// ---- Joint sign conventions: TO BE VERIFIED ON KILO -----------------------
// There is no position feedback, so these come from the legacy posture
// tables and must be checked on the robot with the foot-nudge test
// (docs/p2-hardware-test.md: STANCE, then SET_BODY_POSE x-shift +10 mm; each
// foot must move FORWARD). What the tables say: in "up" (STAND) the front
// legs are shoulder +30 / knee +30 and the back legs -30 / -30, i.e. front
// and back are mirror images (Nybble's front "elbows" point back, its back
// knees forward, as in a cat), and left and right share the same OpenCat
// angles (joints.h's dir handles the mirrored servos). Taking a positive
// OpenCat shoulder angle as "foot backward" for every leg, and a positive
// knee angle as "less bend" for the front legs, puts "up" at t1 = -30 deg,
// t2 = +60 deg in front (knee behind the foot line) and the mirror image
// behind, "rest" in a sphinx fold (front paws forward) and "sit" ~30 deg
// nose up as its header says (test_ik.cpp prints all three).
//  - IK_SGN_SH: OpenCat shoulder angle per degree of t1. If the nudge moves
//    every foot BACKWARD, flip this one.
//  - IK_SGN_KN: OpenCat knee angle per degree of t2. If STANCE crouches
//    with the knees bending the wrong way (or a leg folds), flip this one.
//  - IK_SIGMA_FRONT: knee direction of the front legs (+1: the bend t2 is
//    positive, the knee points backward); the back legs use the opposite.
//    If Kilo's front and back knees point the same way, the back legs'
//    sigma must equal the front's (change ik_sigma()).
#define IK_SGN_SH (-1)
#define IK_SGN_KN (-1)
#define IK_SIGMA_FRONT 1
inline int8_t ik_sigma(uint8_t leg) { return leg_front(leg) ? IK_SIGMA_FRONT : -IK_SIGMA_FRONT; }
// -----------------------------------------------------------------------------

// Foot (x, z) in q2 -> shoulder and knee OpenCat angles (a16). Returns false
// if the target was outside the workspace and has been clamped (the angles
// are then those of the nearest reachable foot on the same ray).
bool ik_leg(uint8_t leg, int16_t x_q2, int16_t z_q2, int16_t* shoulder_a16, int16_t* knee_a16);

// Forward kinematics of the same model (OpenCat joint angles -> foot, q2).
// Used by the host tests; not linked into the firmware unless called.
void ik_fk(uint8_t leg, int16_t shoulder_a16, int16_t knee_a16, int16_t* x_q2, int16_t* z_q2);

// L1 + L2 in q2 (the reach), for the default heights.
int16_t ik_reach_q2(void);

#endif
