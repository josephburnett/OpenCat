#include "keyframe.h"
#include <string.h>
#include "../proto/crc16.h"
#include "fx.h"
#include "pgm.h"
#include "posture.h"

static_assert(sizeof(KfFrame) == sizeof(kilo_keyframe_t), "KfFrame != spec keyframe (14 B)");
static_assert(KF_CAPACITY == 24, "trick store is 24 frames (doc §3e, §3h shrink step 1)");

// Joint ids 0-2 and 8-15: the bits a joint_mask may hold.
#define DRIVEN_MASK 0xFF07u

void kf_init(Keyframe& kf) { memset(&kf, 0, sizeof kf); }

uint8_t kf_begin(Keyframe& kf, const uint8_t* p) {
  if (kf.run) return KILO_ACK_STATUS_BUSY;
  kilo_trick_begin_t h;
  memcpy(&h, p, sizeof h);
  if (h.n_frames < 1 || h.n_frames > KF_CAPACITY || h.loop_end >= h.n_frames || h.loop_start > h.loop_end ||
      !h.joint_mask || (h.joint_mask & ~DRIVEN_MASK) ||
      (h.flags & ~(KILO_TRICK_FLAGS_BALANCE | KILO_TRICK_FLAGS_FALL_OK)) || !posture_has_angles(h.end_posture))
    return KILO_ACK_STATUS_BAD_ARG;
  kf.id = h.trick_id;
  kf.n = h.n_frames;
  kf.loop_start = h.loop_start;
  kf.loop_end = h.loop_end;
  kf.repeats = h.repeats;
  kf.mask = h.joint_mask;
  kf.flags = h.flags;
  kf.end_posture = h.end_posture;
  kf.exp_roll = h.exp_roll;
  kf.exp_pitch = h.exp_pitch;
  kf.store = KF_LOADING;
  kf.nrx = 0;
  kf.crc = KILO_CRC16_INIT;
  return KILO_ACK_STATUS_OK;
}

uint8_t kf_frames(Keyframe& kf, const uint8_t* p) {
  if (kf.run) return KILO_ACK_STATUS_BUSY;
  if (kf.store != KF_LOADING) return KILO_ACK_STATUS_NO_TRICK;
  uint8_t first = p[0], count = p[1];
  const uint8_t* src = p + 2;
  if (first != kf.nrx || (uint8_t)(first + count) > kf.n) return KILO_ACK_STATUS_BAD_ARG;
  // Check every frame before taking any, so a refused chunk changes nothing.
  for (uint8_t i = 0; i < count; i++) {
    const uint8_t* s = src + i * sizeof(KfFrame);
    if (s[0] == 0 || (s[2] & KF_EASE_MASK) == 3) return KILO_ACK_STATUS_BAD_ARG;
  }
  for (uint8_t i = 0; i < count; i++, src += sizeof(KfFrame)) {
    for (uint8_t b = 0; b < sizeof(KfFrame); b++) kf.crc = kilo_crc16_update(kf.crc, src[b]);
    uint8_t fi = (uint8_t)(first + i);
    KfFrame& f = kf.f[fi];
    memcpy(&f, src, sizeof f);
    // "Unchanged" becomes the previous frame's (resolved) angle.
    if (fi)
      for (uint8_t k = 0; k < DRIVEN_COUNT; k++)
        if (f.ang[k] == KF_UNCHANGED) f.ang[k] = kf.f[fi - 1].ang[k];
  }
  kf.nrx = (uint8_t)(kf.nrx + count);
  return KILO_ACK_STATUS_OK;
}

uint8_t kf_commit(Keyframe& kf, uint8_t id, uint16_t crc) {
  if (kf.store == KF_EMPTY || id != kf.id) return KILO_ACK_STATUS_NO_TRICK;
  if (kf.nrx != kf.n) return KILO_ACK_STATUS_BAD_ARG;
  if (crc != kf.crc) return KILO_ACK_STATUS_CRC_MISMATCH;
  kf.store = KF_READY;
  return KILO_ACK_STATUS_OK;
}

uint8_t kf_check_play(const Keyframe& kf, uint8_t id, uint8_t speed, uint8_t flags) {
  if (kf.store != KF_READY || id != kf.id) return KILO_ACK_STATUS_NO_TRICK;
  if (speed < 16 || (flags & ~KILO_TRICK_PLAY_FLAGS_MIRROR)) return KILO_ACK_STATUS_BAD_ARG;
  if (kf.run) return KILO_ACK_STATUS_BUSY;
  return KILO_ACK_STATUS_OK;
}

void kf_start(Keyframe& kf, uint8_t speed, uint8_t mirror) {
  kf.run = 1;
  kf.mirror = mirror;
  kf.speed = speed;
  kf.fi = 0;
  kf.hold = 0;
  kf.plays = 0;
  kf.pos = 0;
}

static inline uint16_t phase_len(const Keyframe& kf) {
  const KfFrame& f = kf.f[kf.fi];
  return (uint16_t)((uint16_t)(kf.hold ? f.hold : f.dur) << 6);
}

bool kf_step(Keyframe& kf) {
  if (!kf.run) return false;
  if (kf.pos >= phase_len(kf)) {  // the previous tick finished this phase
    kf.pos = 0;
    if (!kf.hold && kf.f[kf.fi].hold) {
      kf.hold = 1;
    } else {
      kf.hold = 0;
      bool again = false;
      if (kf.fi == kf.loop_end) {
        if (kf.plays < 254) kf.plays++;
        again = kf.repeats == KF_FOREVER || kf.plays < kf.repeats;
      }
      if (again) {
        kf.fi = kf.loop_start;
      } else if (++kf.fi >= kf.n) {
        kf.run = 0;
        return true;
      }
    }
  }
  kf.pos = (uint16_t)(kf.pos + kf.speed);
  return false;
}

// Mirror: joint_mirror_k (joints.h) swaps the L/R pairs; pan (0) and tail
// (2) are negated.
#define mirror_k joint_mirror_k

bool kf_claims(const Keyframe& kf, uint8_t k) {
  if (!kf.run) return false;
  return (kf.mask >> joint_id(kf.mirror ? mirror_k(k) : k)) & 1u;
}

// Target of frame f for joint k, a16; a never-set joint holds `start`.
KILO_NOINLINE static int16_t target(const Keyframe& kf, uint8_t f, uint8_t k, int16_t start) {
  int8_t v = kf.f[f].ang[kf.mirror ? mirror_k(k) : k];
  if (v == KF_UNCHANGED) return start;
  if (kf.mirror && (k == 0 || k == 2)) v = (int8_t)-v;
  return (int16_t)(v * 16);
}

uint16_t kf_ease(uint8_t easing, uint16_t pos, uint16_t len) {
  if (pos >= len) return Q14_ONE;
  if (easing == KF_STEP) return pos ? Q14_ONE : 0;
  if (easing == KF_LINEAR) return (uint16_t)(((uint32_t)pos << 14) / len);
  return fx_ease_cos(pos, len);
}

int16_t kf_angle(const Keyframe& kf, uint8_t k, int16_t start) {
  uint8_t fi = kf.fi;
  int16_t to = target(kf, fi, k, start);
  if (kf.hold || start == JOINT_OFF) return to;
  int16_t from;
  if (fi == kf.loop_start && kf.plays)
    from = target(kf, kf.loop_end, k, start);  // a repeat starts where the loop ended
  else
    from = fi ? target(kf, (uint8_t)(fi - 1), k, start) : start;
  uint16_t e = kf_ease(kf.f[fi].flags & KF_EASE_MASK, kf.pos, phase_len(kf));
  return (int16_t)(from + (((int32_t)(to - from) * e + 8192) >> 14));
}
