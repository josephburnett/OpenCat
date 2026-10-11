// keyframe: the trick store and player (doc §3e). P1.
//
// The Pi uploads a trick into RAM with TRICK_BEGIN (header), TRICK_FRAMES
// (4 frames per message, in order) and TRICK_COMMIT (CRC-16 of the frames
// as sent), then plays it with TRICK_PLAY (speed, mirror). One trick is held
// at a time; TRICK_COMMIT on a committed store is the cache check.
//
// Frame (14 B, the spec's keyframe struct): dur (ticks from the previous
// frame, >= 1), hold (ticks at the target), flags (bits 0-1 easing: linear,
// cosine, step; bit 2 balance in this frame), ang[11] (whole degrees,
// canonical joint order; -128 = unchanged). "Unchanged" is resolved when
// the frame arrives: it becomes the previous frame's angle (by index), so
// a -128 that survives means "never set since the start": the joint's pose
// when the trick took it.
//
// Player, once per tick (doc §3e "Timing"):
//   segment = MOVE over dur ticks (from the previous target, frame 0 from
//             the pose at the start) then HOLD for hold ticks at the target;
//   q       = from + (to - from) * e(pos / len), Q14, e per the frame's easing:
//               linear  pos/len
//               cosine  (1 - cos(pi s)) / 2   (fx_ease_cos; legacy transform())
//               step    1 from the first tick (jump, slew-limited, then wait)
//   speed   = Q6 (64 = 1x): pos advances by speed/64 tick per tick, so a
//             phase of n ticks takes ceil(n * 64 / speed) ticks;
//   loop    = after loop_end, back to loop_start until loop_start..loop_end
//             has been played `repeats` times in total (0 and 1 = once,
//             255 = until stopped); the first segment of a repeat starts
//             from loop_end's targets;
//   mirror  = L/R columns swapped (LF<->RF, RB<->LB), pan and tail negated,
//             expected roll negated, the joint mask mirrored.
// The last tick of each MOVE lands exactly on the target. The trick ends
// on the tick after its last phase; body.cpp then emits TRICK_DONE and hands
// the joints back to IDLE (end posture) with a crossfade.
//
// Fixed point only, no float. RAM: 32 x 14 B store + 24 B of header, upload
// and player state (no from[] array: see kf_angle).
#ifndef KILO_CORE_KEYFRAME_H
#define KILO_CORE_KEYFRAME_H
#include <stdint.h>
#include "../proto/kilo_protocol.h"
#include "joints.h"

#define KF_CAPACITY KILO_TRICK_CAPACITY  // 32 frames
#define KF_UNCHANGED ((int8_t)-128)
#define KF_EASE_MASK 0x03u
#define KF_BALANCE_BIT 0x04u  // keyframe flags bit 2
#define KF_FOREVER 255u       // repeats: until stopped

enum KfEasing : uint8_t { KF_LINEAR = 0, KF_COSINE = 1, KF_STEP = 2 };
enum KfStore : uint8_t { KF_EMPTY = 0, KF_LOADING = 1, KF_READY = 2 };

// TRICK_DONE reasons (EVENT arg bits 8-15; spec enum trick_done_reason).
enum KfEnd : uint8_t { KF_END_DONE = 0, KF_END_STOPPED = 1, KF_END_PREEMPTED = 2, KF_END_FELL = 3 };

struct KfFrame {
  uint8_t dur, hold, flags;
  int8_t ang[DRIVEN_COUNT];
};

struct Keyframe {
  KfFrame f[KF_CAPACITY];
  // TRICK_BEGIN header.
  uint8_t id, n, loop_start, loop_end, repeats;
  uint16_t mask;  // over the 16 joint ids
  uint8_t flags, end_posture;
  int8_t exp_roll, exp_pitch;
  // Upload.
  uint8_t store;  // KfStore
  uint8_t nrx;    // frames received since TRICK_BEGIN
  uint16_t crc;   // running CRC-16 over the frames as sent
  // Player.
  uint8_t run;     // playing
  uint8_t mirror;  // TRICK_PLAY flags bit 0
  uint8_t hold;    // in the HOLD phase of frame fi (else MOVE)
  uint8_t fi;      // current frame
  uint8_t plays;   // completed plays of the loop section
  uint8_t speed;   // Q6
  uint8_t xfade;   // crossfade out of TRICK (ticks), set when it ends
  uint16_t pos;    // progress in the phase, 1/64 tick
};

void kf_init(Keyframe& kf);

// Upload. Each returns an ACK status (KILO_ACK_STATUS_*). p is the payload,
// already length-checked by the frame decoder.
uint8_t kf_begin(Keyframe& kf, const uint8_t* p);
uint8_t kf_frames(Keyframe& kf, const uint8_t* p);
uint8_t kf_commit(Keyframe& kf, uint8_t id, uint16_t crc);

// TRICK_PLAY checks that depend only on the store (NO_TRICK, BAD_ARG, BUSY).
uint8_t kf_check_play(const Keyframe& kf, uint8_t id, uint8_t speed, uint8_t flags);
void kf_start(Keyframe& kf, uint8_t speed, uint8_t mirror);
// Stop playing (the caller emits TRICK_DONE and sets the end posture).
inline void kf_halt(Keyframe& kf, uint8_t xfade) {
  kf.run = 0;
  kf.xfade = xfade;
}

// One tick. Returns true on the tick the trick has finished (it then no
// longer claims any joint); call before arbiter_run.
bool kf_step(Keyframe& kf);

// The trick owns canonical joint k (playing and in the mask).
bool kf_claims(const Keyframe& kf, uint8_t k);
// The angle of canonical joint k this tick, a16. start: k's output when the
// trick took it (the arbiter's crossfade source).
int16_t kf_angle(const Keyframe& kf, uint8_t k, int16_t start);
// The current frame asks for balance.
inline bool kf_balance(const Keyframe& kf) { return kf.run && (kf.f[kf.fi].flags & KF_BALANCE_BIT); }
// Balance references, a16 (roll negated when mirrored).
inline int16_t kf_ref_pitch(const Keyframe& kf) { return (int16_t)(kf.exp_pitch * 16); }
inline int16_t kf_ref_roll(const Keyframe& kf) {
  return (int16_t)((kf.mirror ? -kf.exp_roll : kf.exp_roll) * 16);
}
// FALL_OK: the fall reflex must not trigger while this trick plays.
inline bool kf_fall_ok(const Keyframe& kf) { return kf.run && (kf.flags & KILO_TRICK_FLAGS_FALL_OK); }
// TELEMETRY trick_frame: the frame playing, 0xFF = none.
inline uint8_t kf_frame(const Keyframe& kf) { return kf.run ? kf.fi : 0xFF; }

// The Q14 easing of a phase at pos of len (both in the same units).
uint16_t kf_ease(uint8_t easing, uint16_t pos, uint16_t len);

#endif
