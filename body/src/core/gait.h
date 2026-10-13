// gait: the gait generator, the STANCE posture and the body pose (doc §3d).
// P2. The GAIT owner of the arbiter (legs 8-15) and the IK posture of IDLE.
//
// Phase. phi is a uint16_t (65536 = one cycle); each tick adds
// dphi = 65536 * 20 / period_ms (computed when the goal changes), so a
// period change never makes the phase jump. Leg i's phase is phi +
// offset_i (wrapping), and the leg swings while its phase is >= duty.
//
// Foot (leg frame: shoulder axle origin, x forward, z down; core/ik.h),
// p = leg phase / 65536, beta = duty / 256:
//   stance (p < beta):  s = p / beta,              x = x0 + S_i (1/2 - s),            z = H_i
//   swing  (p >= beta): s = (p - beta)/(1 - beta), x = x0 + S_i ((1 - cos pi s)/2 - 1/2),
//                       z = H_i - h (1 - cos 2 pi s)/2
// zero velocity at lift-off and touchdown in both x and z (§3d's z = H_i -
// h sin pi s would leave the foot at full vertical speed at lift-off and
// touchdown: a tap on every step; this profile has the same peak h); S_left = S - turn, S_right =
// S + turn (+ turn = left). x0 = 0 (the foot under the shoulder) plus the
// pose x-shift.
//   H_i = body_h + dheight + end_i (Lb/2) sin(pitch) + side_i (W/2) sin(roll)
//   end_i = -1 front, +1 back; side_i = -1 left, +1 right;
//   pitch = pose pitch + balance correction, roll likewise (doc §3f; the
//   correction only while the geometric balance runs).
//
// Start: phi = 0, the stride ramping from 0 over one period (amp += dphi).
// A leg that is already past its lift-off at the start does not lift until
// it has been in stance once (the `armed` bits), so nothing jumps whatever
// the duty. Stop: the stride ramps to 0 over one period, then no new swing
// lifts its foot and the gait waits until every foot is down (all in
// stance or unarmed), then hands the legs to IDLE STANCE at the same
// height. A pattern change is a stop followed by a start.
//
// Goal parameters are slew-limited per tick: stride, turn, step height and
// body height 0.5 mm, duty 1/256, pose pitch/roll 0.5 deg, pose height and
// x-shift 1 mm. The period applies at once (the phase stays continuous).
//
// Support state (docs/walking-and-balance.md §4; for a stance-aware balance):
// `down` has bit i (LF RF RB LB) set when foot i is on the ground this tick:
// in stance, or in swing but not armed (it never lifted), or the gait off
// (STANCE: all four).
//
// Sway (SET_GAIT flag SWAY): the legs move only fore-aft, so the centre of
// mass moves sideways only by rolling the body and fore-aft by pitching it.
// Before a leg lifts the body leans away from it, toward the feet that stay
// down. Per leg a window w_i in [0, 1], a raised cosine centred on
// mid-swing, 1.5 swings each side (one swing of lean-in before lift-off,
// one of decay after touchdown; at most half a cycle, so it is 0 at
// mid-stance):
//   plan_roll  = +A_roll  * sum_i side_i w_i * amp   (side -1 left, +1 right)
//   plan_pitch = +A_pitch * sum_i end_i  w_i * amp   (end -1 front, +1 back)
// Sign: leaning the body right (right side down) is NEGATIVE roll (§3f:
// right side up = +roll), and it moves the centre of mass right of the
// feet (the legs tilt with the body, so the body sits on the low side); so
// a left leg (side -1) lifting needs -A: +A * side. Likewise a front leg
// lifting needs the back end down, nose up = NEGATIVE pitch: +A * end. For
// the trot the two legs of a diagonal pair have the same w and opposite
// sides and ends: the sum is exactly 0 (a line has no triangle to lean
// into). plan_* enter H_i with the pose and the balance correction.
//
// The Gait struct also holds STANCE's height (h) and the body pose
// (SET_BODY_POSE), which apply to IDLE STANCE as well as to the gait, and
// the 8 leg angles of this tick (gait or STANCE; the arbiter reads them).
#ifndef KILO_CORE_GAIT_H
#define KILO_CORE_GAIT_H
#include <stdint.h>
#include "../proto/kilo_protocol.h"

enum GaitState : uint8_t { GS_OFF = 0, GS_RUN = 1, GS_STOP = 2, GS_LAND = 3 };

// Head bob (tilt, twice per cycle) and tail wag (once per cycle), a16
// (spec [gait]).
#define GAIT_BOB_A16 (KILO_GAIT_BOB_DEG * 16)
#define GAIT_WAG_A16 (KILO_GAIT_WAG_DEG * 16)
#define GAIT_SWAY_ROLL_A16 (KILO_GAIT_SWAY_ROLL_DEG * 16)
#define GAIT_SWAY_PITCH_A16 (KILO_GAIT_SWAY_PITCH_DEG * 16)

struct Gait {
  // SET_GAIT goal as received (0 = the pattern's default for period,
  // duty and body height, which the spec's ranges leave free).
  uint8_t run, pattern, flags, duty_g, step_g, body_g;
  int8_t stride_g, turn_g;
  // SET_BODY_POSE goal and its slewed value (pitch, roll a16; mm).
  int16_t pose_g[2], pose[2];  // [0] pitch, [1] roll
  int8_t dh_g, xs_g, dh, xs;
  // Generator state.
  uint8_t state;  // GaitState
  uint8_t pat;    // the pattern running
  uint8_t armed;  // bit i: leg i may lift (it has been in stance since the start)
  uint8_t geo;    // the geometric balance ran this tick: its corrections enter H_i
  uint8_t down;   // bit i: foot i is on the ground this tick (bits as armed)
  int16_t plan_pitch, plan_roll;  // this tick's planned sway, a16 (0 without SWAY or gait)
  uint16_t phi, dphi, amp;  // phase; per tick; start/stop ramp (65535 = full stride)
  uint8_t duty;             // slewed, /256
  int16_t stride, turn, step, h;  // slewed, q2 (h is also STANCE's height)
  int16_t leg[8];  // this tick's OpenCat angles, a16: shoulders LF RF RB LB, knees LF RF RB LB
};

void gait_init(Gait& g);

// Feet on the ground this tick (0..4), from g.down.
inline uint8_t gait_feet_down(const Gait& g) {
  uint8_t d = g.down;
  return (uint8_t)((d & 1) + ((d >> 1) & 1) + ((d >> 2) & 1) + ((d >> 3) & 1));
}

// SET_GAIT field checks (spec ranges, pattern < 3, known flag bits; 0 is
// also accepted for period and duty: the pattern's default). body_h_mm has
// no range: any value is clamped by the IK workspace (0 = the default). p = the payload. Returns a
// GoalResult (OK or BAD_ARG).
uint8_t gait_check(const uint8_t* p);
// Latch a checked SET_GAIT run=1 goal (does not start anything).
void gait_goal(Gait& g, const uint8_t* p);
// Latch a SET_BODY_POSE goal (payload after the epoch: pitch, roll i16 a16,
// dheight, xshift i8 mm); pitch and roll are clamped to +-30 deg.
void gait_pose_goal(Gait& g, const uint8_t* p);
// STANCE's height when IDLE enters STANCE from another posture: 0.80 (L1 + L2)
// (spec gait.stance_h_256).
void gait_stance_default(Gait& g);

// Start running the latched goal (IDLE is in STANCE, settled).
void gait_start(Gait& g);
// One tick of phase, ramps and slews (call every tick; the phase and
// ramps only move while running). Returns true on the tick the stop
// sequence ends: all feet are down and the legs go back to IDLE STANCE.
bool gait_step(Gait& g);
// Fill leg[] for this tick: the gait's feet while it runs, else STANCE.
// corr: the balance corrections (roll, pitch, a16), used when g.geo.
void gait_legs(Gait& g, int16_t corr_roll, int16_t corr_pitch);
// Head bob (k = 1, tilt) and tail wag (k = 2), a16, scaled by the ramp.
int16_t gait_head(const Gait& g, uint8_t k);

#endif
