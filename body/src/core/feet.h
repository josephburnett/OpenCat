// feet: the legs' IK poses (doc §3d). Two users of the same IK:
//   - the FEET player, the GAIT owner of the arbiter (legs, canonical k >= 3):
//     the mind plans every step and streams one FEET per 20 ms tick, about
//     link.feet_lookahead_ticks (5) ahead; the body queues them and plays
//     each on its tick;
//   - STANCE, IDLE's IK posture: all four feet under the hips at STANCE's
//     height, with SET_BODY_POSE on top.
//
// The queue. link.feet_depth (8) slots, slot = tick % 8. A FEET for tick T
// arriving after tick N has been played is queued when 1 <= T - N <= 8 (the
// eight slots then hold eight different ticks), else dropped and counted
// (STATS feet_late: a tick already played or being played, or one further
// ahead than the queue). A second FEET for a queued tick replaces it.
// Entries are only ever taken on their own tick, so none goes stale.
//
// Playing (feet_step, once per tick). The entry for this tick, if any,
// becomes the pose played (`cur`). A tick with no entry while later ones
// are queued (a lost frame) holds the pose. The queue empty on a tick with
// no entry is an underrun: EVENT FEET_UNDERRUN (arg = ticks since the last
// FEET arrived), then the pose is held for link.feet_hold_ticks (5) ticks,
// then the body settles: every foot whose support bit is clear goes down
// SETTLE_MM per tick at its x to the ground (the plane through the support
// feet when three are down, so a leaning body lands its foot on the floor;
// else their mean depth, or STANCE's default height when none are down),
// its bit set when it is there; with all four down the stream ends (IDLE
// STANCE, GAIT_STOPPED arg 1, body.cpp). A FEET that arrives during the hold or the settle is
// played on its tick and the stream goes on.
//
// Every played pose goes through the IK and then the arbiter's safety stage
// (limits, the 10 deg/tick slew): the legs move from where they are, so a
// late, missing or wild tick never snaps a joint.
//
// The balance in GAIT (doc §3f; body.cpp runs the controller): the
// reference is the pose's predicted pitch and roll (plus SET_BODY_POSE),
// and the corrections lengthen or shorten only the legs whose foot is on
// the ground. Each leg's share of the correction ramps 1/4 per tick (bits
// of `share`, 0..4 per leg) toward 4 while its support bit is set and
// toward 0 while it is clear, so a lifting foot lets go of it and a landing
// foot takes it back without a step. STANCE corrects through all four.
// While any support bit is clear the corrections are held (body.cpp: on
// three feet they act through the feet a tipping body pivots on).
//
// RAM: 8 x 12 B queue + 12 B pose + 11 B player state + 15 B STANCE and the
// pose (the leg angles are body_tick's, on the stack). (The z field fits
// 7 bits, 30..95 mm: bit 7 carries the foot's support bit.)
#ifndef KILO_CORE_FEET_H
#define KILO_CORE_FEET_H
#include <stdint.h>
#include "../proto/kilo_protocol.h"

#define FEET_DEPTH KILO_FEET_DEPTH
#define FEET_HOLD_TICKS KILO_FEET_HOLD_TICKS
#define FEET_SETTLE_MM 2  // a swing foot's descent when settling, mm per tick
#define FEET_SUP 0x80u    // FeetPose::zs: the foot is on the ground

enum FeetState : uint8_t {
  FT_OFF = 0,     // no stream: the legs are IDLE's (STANCE or a posture)
  FT_WAIT = 1,    // FEET accepted, the first one's tick has not come: still IDLE's legs, mode GAIT
  FT_PLAY = 2,    // playing (a gap with later ticks queued holds the pose)
  FT_HOLD = 3,    // underrun: holding the last pose
  FT_SETTLE = 4,  // underrun: swing feet going down, then IDLE STANCE
};

// One tick of FEET as the body keeps it (12 B).
struct FeetPose {
  int8_t x[4];          // mm forward of the hip, legs LF RF RB LB
  uint8_t zs[4];        // bits 0-6: mm below the hip; FEET_SUP: on the ground
  int16_t pitch, roll;  // the attitude the plan predicts, a16
};

struct Feet {
  FeetPose q[FEET_DEPTH];  // slot = tick % FEET_DEPTH
  FeetPose cur;            // the pose being played (held through a gap and the underrun hold)
  uint8_t have;            // bit s: slot s holds a tick to come
  uint8_t state;           // FeetState
  uint8_t held;            // FT_HOLD: ticks held so far
  uint16_t rx_tick;        // the body's tick when the last FEET was queued
  uint16_t share;          // 4 bits per leg (LF in bits 0-3): its share of the correction, 0..4 quarters
  uint16_t late, underruns;  // STATS feet_late, feet_underruns
  // STANCE's height and the body pose (SET_BODY_POSE goal and its slewed
  // value: pitch, roll a16; height delta and x-shift mm), applied to STANCE
  // and on top of every FEET pose.
  int16_t h;  // STANCE's body height, q2
  int16_t pose_g[2], pose[2];  // [0] pitch, [1] roll
  int8_t dh_g, xs_g, dh, xs;
  uint8_t geo;  // the geometric balance ran this tick: its corrections enter the IK
};

void feet_init(Feet& f);
// STANCE's height when IDLE enters STANCE from another posture, and after a
// settle: 0.80 (L1 + L2) (spec gait.stance_h_256).
void feet_stance_default(Feet& f);
// Latch a SET_BODY_POSE goal (payload after the epoch: pitch, roll i16 a16,
// dheight, xshift i8 mm); pitch and roll are clamped to +-30 deg.
void feet_pose_goal(Feet& f, const uint8_t* p);

// FEET payload range check (x -60..60, z 30..95 mm, pitch and roll -30..30
// deg, support bits 0-3 only).
// p = the whole payload. Returns a GoalResult (OK or BAD_ARG).
uint8_t feet_check(const uint8_t* p);
// Queue a checked FEET (p = the whole payload) after tick `now` was played.
// False if dropped (counted in `late`).
bool feet_push(Feet& f, const uint8_t* p, uint16_t now);
// Drop the stream: queue, pose and state (the legs go back to IDLE).
void feet_stop(Feet& f);

// One tick of the player (call with the tick being played, before
// feet_legs). Returns FEET_EV_* bits.
#define FEET_EV_UNDERRUN 1  // the queue ran dry this tick: EVENT FEET_UNDERRUN
#define FEET_EV_SETTLED 2   // every foot is down: hand the legs to IDLE STANCE
uint8_t feet_step(Feet& f, uint16_t tick);
// The pose slews toward its goal (every tick, GAIT or not).
void feet_pose_step(Feet& f);

// The stream holds the legs (playing, holding or settling).
inline bool feet_playing(const Feet& f) { return f.state >= FT_PLAY; }
// The support bits (LF RF RB LB) of the pose being played.
uint8_t feet_support(const Feet& f);
// FEET ticks waiting in the queue.
uint8_t feet_queued(const Feet& f);
// The attitude the legs are told to give the body, a16 ([0] pitch, [1]
// roll): the pose, plus the FEET prediction while the stream plays. The
// geometric balance's reference.
int16_t feet_ref(const Feet& f, uint8_t axis);

// This tick's leg angles (OpenCat, a16: shoulders LF RF RB LB, then knees
// LF RF RB LB) into leg[8]: the FEET pose while the stream plays, else
// STANCE. corr: the balance corrections (roll, pitch, a16), used when f.geo.
void feet_legs(const Feet& f, int16_t corr_roll, int16_t corr_pitch, int16_t leg[8]);

#endif
