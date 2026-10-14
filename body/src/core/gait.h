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
// Lead: the body follows the leg lengths through the servo lag (~150 ms),
// and the windows of two legs on opposite sides that swing one after the
// other cross zero exactly at the hand-over (WALK: LF lands as RB lifts), so
// a plan centred on mid-swing leaves the body still leaning toward the
// lifting leg there. Each window is centred KILO_GAIT_SWAY_LEAD_MS before
// mid-swing instead: lead = dphi * lead ticks, computed with dphi when the
// goal arrives (no division per tick). The trot's pairs shift together and
// still cancel.
//
// Early touchdown (SET_GAIT flag CATCH; docs/walking-and-balance.md §4):
// a foot on the ground is the only thing that stops a sideways fall on this
// cat (the legs move only fore-aft), and on a diagonal the body tips with a
// 90 ms time constant, faster than the balance loop can act. So while the
// gait runs (GS_RUN only: the stop puts every foot down anyway), a leg in
// its swing comes down when the body tips toward it faster than
// KILO_GAIT_TOUCHDOWN_RATE_DPS on two ticks in a row (tip_last: one raw
// gyro sample may be a footfall or a servo jerk). Its armed bit is cleared
// and caught[i] keeps pi s at the catch (whole degrees, rounded down, >= 1)
// until its next lift-off. For the rest of the swing its x is held where it
// was (x0 + S e_c + P (e_c + 1/2), e_c the stride fraction at the catch: it
// carries the body now, so it must not travel with the swing) and z comes
// down through the rest of the lift profile from the catch's height (a
// catch before mid-swing mirrored to the same height on the way down) at
// CATCH_DESCENT (2) x the swing's pace, zero speed at the ground; its down
// bit is set once the lift is 0. The stance after it starts where the foot
// is: as the touchdown x0 + S/2 + P' with P' = P + (S + P)(e_c - 1/2) in
// the placement term below, which the stance takes back as P' (e + 1/2).
// The next stance arms it again. Sign (§3f: RIGHT side up = +roll, nose DOWN = +pitch): roll increasing
// means the right side rises and the left side drops, so the body falls
// toward the LEFT legs: a left leg lands on rate_roll > +thr, a right leg
// on rate_roll < -thr. Pitch increasing is the nose dropping: a front leg
// lands on rate_pitch > +thr, a back leg on rate_pitch < -thr. One compare
// per axis per tick, one cosine per caught leg; the test is the plant in
// test_gait.cpp ("catch").
//
// Forward placement (SET_GAIT flag PLACE; docs/walking-and-balance.md §4,
// the capture point): the legs move fore-aft, so a fall forward or back is
// caught by landing the swing foot further ahead or behind. At a leg's
// lift-off (GS_RUN only; the stop, a swing that never lifts and one caught
// at its lift-off get 0, a leg caught later keeps its shift) the pitch rate
// sets that leg's touchdown shift
//   place_i = K * rate_pitch, clamped to +-KILO_GAIT_CAPTURE_MAX_MM,
// held for the swing: x += place_i * (1 - cos pi s)/2 in the swing (0 at
// lift-off, all of it at touchdown) and place_i * (1 - s) in the following
// stance (the shift decays along the stance path, so the next lift-off is
// where it would be anyway). Both are place_i * (e + 1/2), e the stride
// fraction below: one multiply per leg, scaled by the ramp (amp) like the
// stride, so a stop that ends before a leg lifts again leaves nothing at
// amp 0 (the hand-over to STANCE does not move the foot); place and caught
// are cleared at the hand-over and at the start. K (KILO_GAIT_CAPTURE_MM_PER_RAD_S,
// 7): a body falling as an inverted pendulum of height h ~ 80 mm with time
// constant tau = sqrt(h/g) ~ 90 ms is caught where its capture point is,
// tau * v ahead of the centre of mass, v = h * pitch_rate: tau * h = 7 mm
// per rad/s (0.125 mm per deg/s; a 60 deg/s nose-dive asks for 7.5 mm).
// Sign: nose DOWN is +pitch (§3f), so +rate_pitch is the nose dropping: the
// body rotates forward about its feet and the centre of mass, h above them,
// moves FORWARD at h * rate. Its capture point is ahead of it, so the
// landing foot must go ahead: +x, for a front and a back leg alike (x is
// forward in every leg's frame; a diagonal or a triangle edge that a back
// leg is part of moves forward with it). place_i = +K * rate_pitch.
//
// The Gait struct also holds STANCE's height (h) and the body pose
// (SET_BODY_POSE), which apply to IDLE STANCE as well as to the gait, and
// the 8 leg angles of this tick (gait or STANCE; the arbiter reads them).
#ifndef KILO_CORE_GAIT_H
#define KILO_CORE_GAIT_H
#include <stdint.h>
#include "../proto/kilo_protocol.h"
#include "attitude.h"

enum GaitState : uint8_t { GS_OFF = 0, GS_RUN = 1, GS_STOP = 2, GS_LAND = 3 };

// Head bob (tilt, twice per cycle) and tail wag (once per cycle), a16
// (spec [gait]).
#define GAIT_BOB_A16 (KILO_GAIT_BOB_DEG * 16)
#define GAIT_WAG_A16 (KILO_GAIT_WAG_DEG * 16)
#define GAIT_SWAY_ROLL_A16 (KILO_GAIT_SWAY_ROLL_DEG * 16)
#define GAIT_SWAY_PITCH_A16 (KILO_GAIT_SWAY_PITCH_DEG * 16)
#define GAIT_TOUCHDOWN_A16S (KILO_GAIT_TOUCHDOWN_RATE_DPS * 16)  // a16/s, as Attitude's rates
// The sway's lead in ticks, Q6 (150 ms = 7.5 ticks = 480): lead = dphi *
// this >> 6, phase units, once per goal.
#define GAIT_SWAY_LEAD_Q6 ((KILO_GAIT_SWAY_LEAD_MS * 64 + 10) / 20)
// Placement: q2 (1/4 mm) of touchdown shift per a16/s of pitch rate, Q16:
// K mm per rad/s = K * 4 q2 / (16 * 180 / pi a16/s) = K * 0.0043633, x 65536
// = K * 285.95.
#define GAIT_CAPTURE_Q16 ((KILO_GAIT_CAPTURE_MM_PER_RAD_S * 28595L + 50) / 100)
#define GAIT_CAPTURE_MAX_Q2 (KILO_GAIT_CAPTURE_MAX_MM * 4)

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
  uint16_t lead;            // the sway's lead, phase units (dphi * GAIT_SWAY_LEAD_Q6 >> 6, per goal)
  int8_t place[4];          // PLACE: each leg's touchdown shift, q2, set at its lift-off
  uint8_t caught[4];        // CATCH: pi s at the leg's catch, deg (1..179; 0 = none), until its next lift-off
  uint8_t tip_last;         // CATCH: last tick's over-threshold bits (as armed)
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
// corr: the balance corrections (roll, pitch, a16), used when g.geo. att:
// the body rates for the early touchdown (flag CATCH), null for none.
void gait_legs(Gait& g, int16_t corr_roll, int16_t corr_pitch, const Attitude* att);
// Head bob (k = 1, tilt) and tail wag (k = 2), a16, scaled by the ramp.
int16_t gait_head(const Gait& g, uint8_t k);

#endif
