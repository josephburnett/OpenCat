// The keyframe store and player (core/keyframe.*) and the TRICK owner
// (doc §3e, §3c; Appendix H "keyframe"): upload edge cases, tick-exact
// timing and easing at frame boundaries, holds, loops and repeats, speed,
// mirror, crossfades in and out, end posture, TRICK_DONE reasons, gating,
// balance per frame, FALL_OK, owner precedence, the link dispatch, and a
// replay of every first-version trick of the mind's library.
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <vector>
#include "../src/core/body.h"
#include "../src/core/fx.h"
#include "../src/core/link.h"
#include "../src/core/params.h"
#include "../src/core/posture.h"
#include "../src/proto/crc16.h"
#include "check.h"
#include "fake_eeprom.h"
#include "fake_link_hal.h"
#include "trick_library.h"

static Config cfg;
static Body& b = g_body;
static uint16_t counts[16];
static TickIn in;
static std::vector<Event> evs;  // every event since reset()

static const int8_t STAND[11] = {0, 0, 0, 30, 30, -30, -30, 30, 30, -30, -30};
static const uint16_t ALL = 0xFF07;

static void reset(uint8_t state = CFG_MIGRATED) {
  g_ee.reset();
  config_defaults(cfg.img);
  cfg.state = (ConfigState)state;
  cfg.writing = 0;
  cfg.newest = 0xFF;
  params_load(cfg.img);
  body_init(b, &cfg, nullptr, true);
  g_lh.reset();
  link_init(1, 0);
  in = TickIn();
  in.imu = IMU_FRESH;
  in.att.gz = 16384;
  evs.clear();
}

static void drain() {
  const Event* e;
  while ((e = events_peek(b.ev)) != nullptr) {
    evs.push_back(*e);
    events_pop(b.ev);
  }
}

static void tick(int n = 1) {
  for (int i = 0; i < n; i++) {
    body_tick(b, in, counts);
    drain();
  }
}

static void stand() {
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);
  tick(40);
}

// TRICK_DONE events (arg = reason << 8 | id).
static int dones(int reason = -1) {
  drain();
  int n = 0;
  for (const Event& e : evs)
    if (e.code == EV_TRICK_DONE && (reason < 0 || ((uint16_t)e.arg >> 8) == reason)) n++;
  return n;
}
static const Event* last_done() {
  for (size_t i = evs.size(); i-- > 0;)
    if (evs[i].code == EV_TRICK_DONE) return &evs[i];
  return nullptr;
}

static KfFrame fr(uint8_t dur, uint8_t hold, uint8_t flags, const int8_t a[11]) {
  KfFrame f;
  f.dur = dur;
  f.hold = hold;
  f.flags = flags;
  std::memcpy(f.ang, a, 11);
  return f;
}
static KfFrame fr(uint8_t dur, uint8_t hold, uint8_t flags, std::initializer_list<int> a) {
  int8_t v[11];
  size_t i = 0;
  for (int x : a) v[i++] = (int8_t)x;
  for (; i < 11; i++) v[i] = KF_UNCHANGED;
  return fr(dur, hold, flags, v);
}
// STAND + d on every joint.
static KfFrame fr_off(uint8_t dur, uint8_t hold, uint8_t flags, int d) {
  int8_t v[11];
  for (int k = 0; k < 11; k++) v[k] = (int8_t)(STAND[k] + d);
  return fr(dur, hold, flags, v);
}

struct T {
  uint8_t id = 9, ls = 0, le = 0, rep = 0;
  uint16_t mask = ALL;
  uint8_t flags = 0, endp = POSTURE_STAND;
  int8_t er = 0, ep = 0;
  std::vector<KfFrame> f;
};

static std::vector<uint8_t> begin_payload(const T& t) {
  return {t.id, (uint8_t)t.f.size(), t.ls, t.le, t.rep, (uint8_t)(t.mask & 0xFF), (uint8_t)(t.mask >> 8),
          t.flags, t.endp, (uint8_t)t.er, (uint8_t)t.ep};
}
static std::vector<uint8_t> chunk(const T& t, size_t first, size_t count) {
  std::vector<uint8_t> p = {(uint8_t)first, (uint8_t)count};
  for (size_t i = first; i < first + count; i++) {
    const uint8_t* s = (const uint8_t*)&t.f[i];
    p.insert(p.end(), s, s + sizeof(KfFrame));
  }
  return p;
}
static uint16_t crc_of(const T& t) {
  uint16_t c = KILO_CRC16_INIT;
  for (const KfFrame& f : t.f)
    for (size_t i = 0; i < sizeof f; i++) c = kilo_crc16_update(c, ((const uint8_t*)&f)[i]);
  return c;
}
// Begin, frames in chunks of 4, commit; the first failing status, or OK.
static uint8_t upload(const T& t) {
  uint8_t s = kf_begin(b.kf, begin_payload(t).data());
  for (size_t i = 0; s == KILO_ACK_STATUS_OK && i < t.f.size(); i += 4)
    s = kf_frames(b.kf, chunk(t, i, t.f.size() - i < 4 ? t.f.size() - i : 4).data());
  if (s == KILO_ACK_STATUS_OK) s = kf_commit(b.kf, t.id, crc_of(t));
  return s;
}
static uint8_t play(uint8_t id, uint8_t speed = 64, uint8_t flags = 0) {
  uint8_t p[4] = {b.epoch, id, speed, flags};
  return body_trick_play(b, p);
}

static bool at_posture(uint8_t p) {
  for (int k = 0; k < DRIVEN_COUNT; k++) {
    uint8_t j = joint_id((uint8_t)k);
    if (b.arb.out[k] != joint_clamp(j, posture_angle(p, (uint8_t)k))) return false;
  }
  return true;
}
static int16_t lerp_q14(int16_t a, int16_t z, uint16_t e) { return (int16_t)(a + (((int32_t)(z - a) * e + 8192) >> 14)); }

// ---------------------------------------------------------------------------
static void test_upload(void) {
  reset();
  T t;
  t.f = {fr_off(10, 0, KF_COSINE, 10), fr_off(10, 0, KF_COSINE, -10)};
  // Frames or a commit without a header.
  CHECK(kf_frames(b.kf, chunk(t, 0, 1).data()) == KILO_ACK_STATUS_NO_TRICK);
  CHECK(kf_commit(b.kf, t.id, crc_of(t)) == KILO_ACK_STATUS_NO_TRICK);
  // Bad headers.
  auto bad_begin = [&](int field, uint8_t v) {
    std::vector<uint8_t> p = begin_payload(t);
    p[(size_t)field] = v;
    return kf_begin(b.kf, p.data());
  };
  CHECK(bad_begin(1, 0) == KILO_ACK_STATUS_BAD_ARG);   // no frames
  CHECK(bad_begin(1, 33) == KILO_ACK_STATUS_BAD_ARG);  // more than the store holds
  CHECK(bad_begin(3, 2) == KILO_ACK_STATUS_BAD_ARG);   // loop_end >= n
  {
    std::vector<uint8_t> p = begin_payload(t);
    p[2] = 1;  // loop_start 1 > loop_end 0
    CHECK(kf_begin(b.kf, p.data()) == KILO_ACK_STATUS_BAD_ARG);
  }
  CHECK(bad_begin(5, 0x08) == KILO_ACK_STATUS_BAD_ARG);  // joint 3 is not driven
  {
    std::vector<uint8_t> p = begin_payload(t);
    p[5] = p[6] = 0;  // no joint at all
    CHECK(kf_begin(b.kf, p.data()) == KILO_ACK_STATUS_BAD_ARG);
  }
  CHECK(bad_begin(7, 4) == KILO_ACK_STATUS_BAD_ARG);                // unknown flag
  CHECK(bad_begin(8, POSTURE_LIMP) == KILO_ACK_STATUS_BAD_ARG);     // end posture without angles
  CHECK(bad_begin(8, POSTURE_STANCE) == KILO_ACK_STATUS_BAD_ARG);
  CHECK(bad_begin(8, 7) == KILO_ACK_STATUS_BAD_ARG);
  CHECK(b.kf.store == KF_EMPTY);  // nothing refused touched the store

  // Order: first must be the next frame; no gaps, no repeats, no overrun.
  CHECK(kf_begin(b.kf, begin_payload(t).data()) == KILO_ACK_STATUS_OK);
  CHECK(kf_frames(b.kf, chunk(t, 1, 1).data()) == KILO_ACK_STATUS_BAD_ARG);
  CHECK(kf_frames(b.kf, chunk(t, 0, 1).data()) == KILO_ACK_STATUS_OK);
  CHECK(kf_frames(b.kf, chunk(t, 0, 1).data()) == KILO_ACK_STATUS_BAD_ARG);
  CHECK(kf_commit(b.kf, t.id, crc_of(t)) == KILO_ACK_STATUS_BAD_ARG);  // a frame is missing
  {
    T u = t;
    u.f.push_back(u.f[0]);  // 3 frames into a 2-frame header
    CHECK(kf_frames(b.kf, chunk(u, 1, 2).data()) == KILO_ACK_STATUS_BAD_ARG);
  }
  // A chunk with one bad frame is refused whole.
  {
    T u = t;
    u.f[1].dur = 0;
    CHECK(kf_frames(b.kf, chunk(u, 1, 1).data()) == KILO_ACK_STATUS_BAD_ARG);
    u.f[1] = t.f[1];
    u.f[1].flags = 3;  // reserved easing
    CHECK(kf_frames(b.kf, chunk(u, 1, 1).data()) == KILO_ACK_STATUS_BAD_ARG);
    CHECK(b.kf.nrx == 1);
  }
  CHECK(kf_frames(b.kf, chunk(t, 1, 1).data()) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_NO_TRICK);  // not committed
  CHECK(kf_commit(b.kf, (uint8_t)(t.id + 1), crc_of(t)) == KILO_ACK_STATUS_NO_TRICK);
  CHECK(kf_commit(b.kf, t.id, (uint16_t)(crc_of(t) ^ 1)) == KILO_ACK_STATUS_CRC_MISMATCH);
  CHECK(b.kf.store == KF_LOADING);
  CHECK(kf_commit(b.kf, t.id, crc_of(t)) == KILO_ACK_STATUS_OK);
  // The commit is idempotent: the cache check.
  CHECK(kf_commit(b.kf, t.id, crc_of(t)) == KILO_ACK_STATUS_OK);
  CHECK(kf_commit(b.kf, t.id, 0x1234) == KILO_ACK_STATUS_CRC_MISMATCH);
  CHECK(b.kf.store == KF_READY);  // a failed probe leaves a good trick alone
  CHECK(kf_frames(b.kf, chunk(t, 0, 1).data()) == KILO_ACK_STATUS_NO_TRICK);  // committed: begin again

  // The full store: 32 frames in 8 chunks.
  T big;
  for (int i = 0; i < 32; i++) big.f.push_back(fr_off(2, 0, KF_LINEAR, (i % 5) - 2));
  CHECK(upload(big) == KILO_ACK_STATUS_OK && b.kf.n == 32);

  // While a trick plays: BEGIN and FRAMES are BUSY, COMMIT still answers.
  stand();
  CHECK(play(big.id) == KILO_ACK_STATUS_OK);
  tick();
  CHECK(kf_begin(b.kf, begin_payload(t).data()) == KILO_ACK_STATUS_BUSY);
  CHECK(kf_frames(b.kf, chunk(big, 0, 1).data()) == KILO_ACK_STATUS_BUSY);
  CHECK(kf_commit(b.kf, big.id, crc_of(big)) == KILO_ACK_STATUS_OK);
  CHECK(play(big.id) == KILO_ACK_STATUS_BUSY);

  // "Unchanged" (-128) takes the previous frame's angle; in frame 0 it is the start pose.
  reset();
  stand();
  T u;
  u.f = {fr(1, 0, KF_LINEAR, {KF_UNCHANGED, 10}), fr(1, 0, KF_LINEAR, {20, KF_UNCHANGED})};
  CHECK(upload(u) == KILO_ACK_STATUS_OK);
  CHECK(b.kf.f[1].ang[1] == 10 && b.kf.f[1].ang[0] == 20 && b.kf.f[0].ang[0] == KF_UNCHANGED);
  CHECK(b.kf.f[1].ang[5] == KF_UNCHANGED);  // never set: the start pose throughout
  // The CRC covers the frames as sent, not as resolved.
  CHECK(kf_commit(b.kf, u.id, crc_of(u)) == KILO_ACK_STATUS_OK);
}

// Easing, tick-exact: the last tick of each move is exactly the target, the
// next move starts from it, every move is monotone.
static void test_easing(void) {
  for (uint8_t easing = 0; easing < 3; easing++) {
    reset();
    stand();
    g_param[KILO_PARAM_SLEW_MAX_A16] = 800;  // out = the player's angle
    T t;
    t.f = {fr_off(10, 0, easing, 20), fr_off(20, 0, easing, -10)};
    CHECK(upload(t) == KILO_ACK_STATUS_OK);
    CHECK(play(t.id) == KILO_ACK_STATUS_OK);
    int16_t start[11], prev[11];
    for (int k = 0; k < 11; k++) start[k] = prev[k] = b.arb.out[k];
    int max_step_at_boundary = 0;
    for (int seg = 0; seg < 2; seg++) {
      uint8_t n = seg ? 20 : 10;
      for (int i = 1; i <= n; i++) {
        tick();
        CHECK(b.kf.run && kf_frame(b.kf) == seg);
        for (int k = 0; k < 11; k++) {
          int16_t from = seg ? (int16_t)((STAND[k] + 20) * 16) : start[k];
          int16_t to = (int16_t)((STAND[k] + (seg ? -10 : 20)) * 16);
          uint16_t e = easing == KF_LINEAR ? (uint16_t)(((uint32_t)i << 14) / n)
                       : easing == KF_STEP ? 16384
                                           : fx_ease_cos((uint16_t)i, n);
          int16_t want = lerp_q14(from, to, e);
          CHECKF(b.arb.out[k] == want, "easing %d seg %d tick %d joint %d: %d != %d", easing, seg, i, k,
                 b.arb.out[k], want);
          // monotone within the move
          int d = b.arb.out[k] - prev[k];
          CHECK(seg ? d <= 0 : d >= 0);
          if (seg == 1 && i == 1 && std::abs(d) > max_step_at_boundary) max_step_at_boundary = std::abs(d);
          if (i == n) CHECK(b.arb.out[k] == to);
          prev[k] = b.arb.out[k];
        }
      }
    }
    // Cosine leaves and arrives with zero velocity: the boundary step is tiny.
    if (easing == KF_COSINE) CHECKF(max_step_at_boundary <= 8, "boundary step %d a16", max_step_at_boundary);
    CHECK(dones() == 0);
    tick();  // the tick after the last move: DONE, IDLE crossfades back
    CHECK(!b.kf.run && dones(KF_END_DONE) == 1 && last_done()->arg == t.id);
  }
}

static void test_hold_speed(void) {
  reset();
  stand();
  g_param[KILO_PARAM_SLEW_MAX_A16] = 800;
  T t;
  t.f = {fr_off(5, 7, KF_COSINE, 10), fr_off(10, 0, KF_LINEAR, 0)};
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  tick(5);
  int16_t at = b.arb.out[3];
  CHECK(at == (STAND[3] + 10) * 16);
  for (int i = 0; i < 7; i++) {
    tick();
    CHECK(b.arb.out[3] == at && kf_frame(b.kf) == 0 && b.kf.hold);
  }
  tick();
  CHECK(kf_frame(b.kf) == 1 && b.arb.out[3] < at);
  // Speed: a phase of n ticks takes ceil(n * 64 / speed) ticks.
  const uint8_t speeds[] = {16, 32, 64, 96, 128, 255};
  for (uint8_t sp : speeds) {
    reset();
    stand();
    CHECK(upload(t) == KILO_ACK_STATUS_OK);
    CHECK(play(t.id, sp) == KILO_ACK_STATUS_OK);
    int want = 0;
    for (int len : {5, 7, 10}) want += (len * 64 + sp - 1) / sp;
    int n = 0;
    while (!dones() && n < 2000) {
      tick();
      n++;
    }
    CHECKF(n == want + 1, "speed %d: DONE after %d ticks, want %d", sp, n, want + 1);
  }
  // Out-of-range speed and flags.
  reset();
  stand();
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id, 15) == KILO_ACK_STATUS_BAD_ARG);
  CHECK(play(t.id, 64, 2) == KILO_ACK_STATUS_BAD_ARG);
}

// Frames of a 4-frame trick with loop 1..2 played `rep` times: the frame
// shown each tick, the first segment of a repeat starting from loop_end.
static void test_loops(void) {
  const uint8_t reps[] = {0, 1, 2, 3, 255};
  for (uint8_t rep : reps) {
    reset();
    stand();
    g_param[KILO_PARAM_SLEW_MAX_A16] = 800;
    T t;
    t.ls = 1;
    t.le = 2;
    t.rep = rep;
    t.f = {fr_off(2, 0, KF_LINEAR, 10), fr_off(3, 1, KF_LINEAR, 20), fr_off(4, 0, KF_LINEAR, -10),
           fr_off(1, 0, KF_LINEAR, 0)};
    CHECK(upload(t) == KILO_ACK_STATUS_OK);
    CHECK(play(t.id) == KILO_ACK_STATUS_OK);
    std::vector<int> want;
    auto add = [&](int f, int ticks) {
      for (int i = 0; i < ticks; i++) want.push_back(f);
    };
    add(0, 2);
    int plays = rep == 255 ? 40 : (rep < 1 ? 1 : rep);
    for (int p = 0; p < plays; p++) {
      add(1, 4);
      add(2, 4);
    }
    if (rep != 255) add(3, 1);
    std::vector<int> got;
    bool repeat_from_ok = true;
    for (size_t i = 0; i < want.size(); i++) {
      tick();
      got.push_back(kf_frame(b.kf));
      // First tick of frame 1 on a repeat: from frame 2's target (-10), 1/3 of the way to +20.
      if (i >= 2 && want[i] == 1 && want[i - 1] == 2) {
        int16_t want_a = lerp_q14((int16_t)((STAND[3] - 10) * 16), (int16_t)((STAND[3] + 20) * 16), (16384 + 1) / 3);
        if (std::abs(b.arb.out[3] - want_a) > 1) repeat_from_ok = false;
      }
    }
    CHECKF(got == want, "repeats %d: frame sequence differs", rep);
    CHECK(repeat_from_ok);
    if (rep == 255) {
      tick(2000);
      CHECK(b.kf.run && dones() == 0 && b.kf.plays == 254);  // still looping, counter saturated
      body_trick_stop(b, 0);
      CHECK(dones(KF_END_STOPPED) == 1 && !b.kf.run);
    } else {
      CHECK(dones() == 0);
      tick();
      CHECK(dones(KF_END_DONE) == 1);
    }
  }
}

static void test_mirror(void) {
  reset();
  stand();
  g_param[KILO_PARAM_SLEW_MAX_A16] = 800;
  T t;
  t.er = 5;
  t.f = {fr(1, 0, KF_LINEAR, {30, 10, -20, 50, 10, -10, -50, 40, 20, -40, -20})};
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id, 64, KILO_TRICK_PLAY_FLAGS_MIRROR) == KILO_ACK_STATUS_OK);
  tick();
  const int want[11] = {-30, 10, 20, 10, 50, -50, -10, 20, 40, -20, -40};
  for (int k = 0; k < 11; k++) CHECKF(b.arb.out[k] == want[k] * 16, "mirror joint %d: %d", k, b.arb.out[k] / 16);
  CHECK(kf_ref_roll(b.kf) == -5 * 16);
  // The mask is mirrored too: a left-front-shoulder trick moves the right front.
  reset();
  stand();
  g_param[KILO_PARAM_SLEW_MAX_A16] = 800;
  t.mask = 1u << 8;
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id, 64, KILO_TRICK_PLAY_FLAGS_MIRROR) == KILO_ACK_STATUS_OK);
  tick();
  for (int k = 0; k < 11; k++) CHECK((b.arb.owner[k] == OWN_TRICK) == (k == 4));
  CHECK(b.arb.out[4] == 50 * 16);
}

// Owner precedence: REFLEX > TRICK > IDLE per joint; a partial mask leaves
// the rest with IDLE; a reflex preempts with no jump; TRICK_DONE reasons.
static void test_owners_and_preemption(void) {
  reset();
  stand();
  T t;
  t.mask = 0x0007;  // head and tail only
  t.f = {fr(50, 0, KF_COSINE, {40, 20, 60})};
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  tick(5);
  for (int k = 0; k < 11; k++) CHECK(b.arb.owner[k] == (k < 3 ? OWN_TRICK : OWN_IDLE));
  CHECK(body_mode(b) == KILO_MODE_TRICK);
  // A reflex takes every joint on the next tick and the trick ends PREEMPTED.
  int16_t before[11];
  for (int k = 0; k < 11; k++) before[k] = b.arb.out[k];
  arbiter_reflex(b.arb, POSTURE_REST, 50);
  tick();
  CHECK(!b.kf.run && dones(KF_END_PREEMPTED) == 1);
  for (int k = 0; k < 11; k++) {
    CHECK(b.arb.owner[k] == OWN_REFLEX);
    CHECK(std::abs(b.arb.out[k] - before[k]) <= 16);  // the reflex move starts from the output
  }
  // Under a reflex a trick cannot start.
  CHECK(play(t.id) == KILO_ACK_STATUS_BUSY);

  // The link watchdog: stage 1 bumps the epoch, which stops the trick
  // (PREEMPTED, with the new epoch); the joints crossfade to the end posture.
  reset();
  stand();
  body_link_frame(b);
  t.mask = ALL;
  t.endp = POSTURE_SIT;
  t.f = {fr_off(200, 0, KF_COSINE, 15)};
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  int n = 0;
  while (!dones() && n < 100) {
    tick();
    n++;
  }
  CHECK(n == WD_STOP_TICKS && dones(KF_END_PREEMPTED) == 1 && last_done()->epoch == 1);
  CHECK(arbiter_idle_posture(b.arb) == POSTURE_SIT);
  // A play with the old epoch is STALE.
  uint8_t old[4] = {0, t.id, 64, 0};
  CHECK(body_trick_play(b, old) == KILO_ACK_STATUS_STALE);

  // FELL: the fall reflex's entry point; FALL_OK suppresses fall detection.
  reset();
  stand();
  t.flags = KILO_TRICK_FLAGS_FALL_OK;
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(!body_fall_suppressed(b));
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  tick();
  CHECK(body_fall_suppressed(b));
  body_epoch_bump(b, KF_END_FELL);  // what the fall reflex does
  CHECK(!body_fall_suppressed(b) && dones(KF_END_FELL) == 1 && last_done()->epoch == 1);

  // SET_POSTURE LIMP is an emergency stop: STOPPED, limp at once.
  reset();
  stand();
  t.flags = 0;
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  tick(3);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_LIMP, 0, 0) == GOAL_OK);
  tick();
  CHECK(dones(KF_END_STOPPED) == 1);
  bool off = true;
  for (int c = 0; c < 16; c++) off = off && (counts[c] & PCA_FULL_OFF);
  CHECK(off);
  // Another posture during a trick moves only the joints it does not own.
  reset();
  stand();
  t.mask = 0x0007;
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_SIT, 0, 0) == GOAL_OK);
  tick(30);
  CHECK(b.kf.run && b.arb.owner[0] == OWN_TRICK && b.arb.out[3] == 30 * 16 && b.arb.out[5] == -80 * 16);
}

// Gating: no trick in LIMP, CALIB, under a reflex, on a low battery,
// uncalibrated, with a stale epoch or while one plays.
static void test_gating(void) {
  T t;
  t.f = {fr_off(10, 0, KF_COSINE, 5)};
  reset();  // cold boot: LIMP
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_BUSY);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_CALIB, 0, 0) == GOAL_OK);
  tick(30);
  CHECK(play(t.id) == KILO_ACK_STATUS_BUSY);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_REST, 0, 0) == GOAL_OK);
  tick(30);
  uint8_t stale[4] = {(uint8_t)(b.epoch + 1), t.id, 64, 0};
  CHECK(body_trick_play(b, stale) == KILO_ACK_STATUS_STALE);
  CHECK(play((uint8_t)(t.id + 1)) == KILO_ACK_STATUS_NO_TRICK);
  b.batt.level = BATT_LOW;
  CHECK(play(t.id) == KILO_ACK_STATUS_REFUSED_LOW_BATT);
  b.batt.level = BATT_OK;
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);  // from REST is fine
  CHECK(play(t.id) == KILO_ACK_STATUS_BUSY);
  reset(CFG_DEFAULTED);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_SIT, 0, 0) == GOAL_OK);
  tick(30);
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_UNCALIBRATED);
}

// Start, stop and end: no jump into the trick; out of it a cosine crossfade
// of XFADE_TICKS (or TRICK_STOP's) to the end posture, which IDLE adopts.
static void test_crossfades(void) {
  reset();
  stand();
  g_param[KILO_PARAM_SLEW_MAX_A16] = 800;  // timing of the crossfade itself, not the slew
  T t;
  t.endp = POSTURE_SIT;
  t.f = {fr_off(25, 0, KF_COSINE, 15)};
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  int16_t prev[11];
  for (int k = 0; k < 11; k++) prev[k] = b.arb.out[k];
  tick();
  for (int k = 0; k < 11; k++) CHECK(std::abs(b.arb.out[k] - prev[k]) <= 2);  // 15 deg * ease(1/25) = 0.24 a16
  tick(24);
  CHECK(b.arb.out[3] == (30 + 15) * 16);
  // Done: SIT over 10 ticks, monotone, exactly there on the 10th.
  for (int k = 0; k < 11; k++) prev[k] = b.arb.out[k];
  for (int i = 1; i <= 10; i++) {
    tick();
    if (i == 1) CHECK(dones(KF_END_DONE) == 1 && arbiter_idle_posture(b.arb) == POSTURE_SIT);
    for (int k = 0; k < 11; k++) {
      int16_t to = joint_clamp(joint_id((uint8_t)k), posture_angle(POSTURE_SIT, (uint8_t)k));
      int16_t from = (int16_t)((STAND[k] + 15) * 16);
      int d = b.arb.out[k] - prev[k];
      CHECK((long)d * (to - from) >= 0);
      CHECK(std::abs(d) <= PARAM(SLEW_MAX_A16));
      prev[k] = b.arb.out[k];
    }
    CHECK(at_posture(POSTURE_SIT) == (i == 10));
  }
  for (int k = 0; k < 11; k++) CHECK(b.arb.owner[k] == OWN_IDLE);
  // TRICK_STOP mid-move with 20 ticks: STOPPED, at SIT after exactly 20.
  reset();
  stand();
  g_param[KILO_PARAM_SLEW_MAX_A16] = 800;
  t.f = {fr_off(100, 0, KF_COSINE, 15)};
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  tick(50);
  body_trick_stop(b, 20);
  CHECK(dones(KF_END_STOPPED) == 1 && !b.kf.run);
  for (int i = 1; i <= 20; i++) {
    tick();
    CHECK(at_posture(POSTURE_SIT) == (i == 20));
  }
  body_trick_stop(b, 0);  // nothing playing: no event
  CHECK(dones() == 1);
}

// Balance runs only in frames with the balance bit, against the trick's
// expected attitude; between them the corrections decay.
static void test_balance(void) {
  reset();
  stand();
  in.att.pitch = 10 * 16;  // nose down 10 deg
  T t;
  t.f = {fr_off(60, 0, KF_COSINE | KF_BALANCE_BIT, 0), fr_off(60, 0, KF_COSINE, 0)};
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  tick(40);
  CHECK(b.bal.active && b.bal.ax[BAL_PITCH].corr < 0);  // nose down: corr lifts the nose
  CHECK(b.arb.out[7] != STAND[7] * 16);                  // a knee carries the correction
  tick(20);
  int16_t c = b.bal.ax[BAL_PITCH].corr;
  bool decays = true;
  for (int i = 0; i < 30; i++) {
    tick();
    int16_t n = b.bal.ax[BAL_PITCH].corr;
    if (!(n >= c && n <= 0)) decays = false;
    c = n;
  }
  CHECK(decays && c == 0 && !b.bal.active);
  // The reference is the trick's expected pitch: at pitch == exp, nothing to do.
  reset();
  stand();
  in.att.pitch = 10 * 16;
  t.ep = 10;
  t.f = {fr_off(60, 0, KF_COSINE | KF_BALANCE_BIT, 0)};
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  tick(40);
  CHECK(b.bal.active && b.bal.ax[BAL_PITCH].corr == 0);
  // No balance bit: none, even with the IDLE balance requested.
  reset();
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 1) == GOAL_OK);
  in.att.pitch = 10 * 16;
  tick(60);
  CHECK(b.bal.active);
  t.f = {fr_off(60, 0, KF_COSINE, 0)};
  CHECK(upload(t) == KILO_ACK_STATUS_OK);
  CHECK(play(t.id) == KILO_ACK_STATUS_OK);
  tick();
  CHECK(!b.bal.active && b.bal.ax[BAL_PITCH].corr == 0);
}

// Through the link: upload, de-dup of a retried chunk, TELEMETRY.
static void test_link(void) {
  reset();
  stand();
  T t;
  t.f = {fr_off(10, 0, KF_COSINE, 5), fr_off(10, 0, KF_COSINE, 10), fr_off(10, 0, KF_COSINE, 0),
         fr_off(10, 0, KF_COSINE, 3), fr_off(10, 0, KF_COSINE, 0)};
  uint16_t done_arg = 0xFFFF;
  auto send = [&](uint8_t type, uint8_t seq, const std::vector<uint8_t>& p) {
    g_lh.feed(frame(type, seq, p));
    link_poll();
    link_send_events();
    uint8_t st = 0xFF;
    for (const TxFrame& f : g_lh.take()) {
      if (f.type == KILO_MSG_ACK && f.payload[0] == seq && f.payload[1] == type) st = f.payload[2];
      if (f.type == KILO_MSG_EVENT && f.payload[0] == KILO_EVENT_TRICK_DONE)
        done_arg = (uint16_t)(f.payload[1] | (f.payload[2] << 8));
    }
    return st;
  };
  CHECK(send(KILO_MSG_TRICK_BEGIN, 1, begin_payload(t)) == KILO_ACK_STATUS_OK);
  CHECK(send(KILO_MSG_TRICK_FRAMES, 2, chunk(t, 0, 4)) == KILO_ACK_STATUS_OK);
  CHECK(send(KILO_MSG_TRICK_FRAMES, 2, chunk(t, 0, 4)) == KILO_ACK_STATUS_OK);  // a retry: re-acked, not re-run
  CHECK(b.kf.nrx == 4);
  CHECK(send(KILO_MSG_TRICK_FRAMES, 3, chunk(t, 4, 1)) == KILO_ACK_STATUS_OK);
  uint16_t crc = crc_of(t);
  CHECK(send(KILO_MSG_TRICK_COMMIT, 4, {t.id, (uint8_t)crc, (uint8_t)(crc >> 8)}) == KILO_ACK_STATUS_OK);
  CHECK(send(KILO_MSG_TRICK_PLAY, 5, {b.epoch, t.id, 64, 0}) == KILO_ACK_STATUS_OK);
  link_init(1, 0);  // TELEMETRY every 2 ticks
  g_lh.take();
  for (int i = 0; i < 14; i++) {
    tick();
    link_tick();
  }
  std::vector<TxFrame> out = g_lh.take();
  kilo_telemetry_t m = {};
  for (const TxFrame& f : out)
    if (f.type == KILO_MSG_TELEMETRY) std::memcpy(&m, f.payload.data(), sizeof m);
  CHECK(m.mode == KILO_MODE_TRICK && (m.flags & KILO_TELEMETRY_FLAGS_TRICK) && m.trick_frame == 1);
  CHECK((m.owners & 0x0F) == KILO_OWNER_TRICK && (m.owners >> 4) == KILO_OWNER_TRICK);
  CHECK(send(KILO_MSG_TRICK_STOP, 6, {0}) == KILO_ACK_STATUS_OK);
  CHECK(done_arg == ((KF_END_STOPPED << 8) | t.id));  // EVENT TRICK_DONE(STOPPED, id) with the ACK
}

// Every first-version trick, uploaded as the mind encodes it and played
// through the whole tick: angles within the joint limits, the frame shown
// each tick and the end exactly as the JSON durations say.
static void test_library(void) {
  for (int li = 0; li < LIB_TRICK_COUNT; li++) {
    const LibTrick& L = LIB_TRICKS[li];
    CHECKF(kilo_crc16(L.frames, (uint16_t)(L.n * 14)) == L.crc, "%s: crc", L.name);
    for (int variant = 0; variant < 3; variant++) {
      // 0: 1x; 1: mirrored at 1x; 2: mirrored at 1.5x
      uint8_t speed = variant == 2 ? 96 : 64, mirror = variant ? 1 : 0;
      reset();
      stand();
      CHECK(kf_begin(b.kf, L.begin) == KILO_ACK_STATUS_OK);
      for (int i = 0; i < L.n; i += 4) {
        int c = L.n - i < 4 ? L.n - i : 4;
        std::vector<uint8_t> p = {(uint8_t)i, (uint8_t)c};
        p.insert(p.end(), L.frames + i * 14, L.frames + (i + c) * 14);
        CHECK(kf_frames(b.kf, p.data()) == KILO_ACK_STATUS_OK);
      }
      CHECK(kf_commit(b.kf, L.begin[0], L.crc) == KILO_ACK_STATUS_OK);
      CHECK(play(L.begin[0], speed, mirror) == KILO_ACK_STATUS_OK);
      // The schedule from the JSON: (frame, ticks) in play order.
      std::vector<int> want;
      auto add = [&](int f) {
        int n = (L.dur[f] * 64 + speed - 1) / speed;
        int h = (L.hold[f] * 64 + speed - 1) / speed;
        for (int i = 0; i < n + h; i++) want.push_back(f);
      };
      int plays = L.repeats < 1 ? 1 : L.repeats;
      for (int f = 0; f <= L.loop_end; f++) add(f);
      for (int p = 1; p < plays; p++)
        for (int f = L.loop_start; f <= L.loop_end; f++) add(f);
      for (int f = L.loop_end + 1; f < L.n; f++) add(f);
      if (variant == 0) CHECKF((int)want.size() == L.total_ticks, "%s: %zu ticks, model says %d", L.name, want.size(),
                               L.total_ticks);
      std::vector<int> got;
      bool limits_ok = true, targets_ok = true;
      for (size_t i = 0; i < want.size(); i++) {
        tick();
        got.push_back(kf_frame(b.kf));
        for (int k = 0; k < 11; k++) {
          uint8_t j = joint_id((uint8_t)k);
          int16_t raw = kf_angle(b.kf, (uint8_t)k, b.arb.xsrc[k]);
          if (kf_claims(b.kf, (uint8_t)k) && (raw < joint_lo_a16(j) || raw > joint_hi_a16(j))) limits_ok = false;
          if (b.arb.out[k] < joint_lo_a16(j) || b.arb.out[k] > joint_hi_a16(j)) limits_ok = false;
          // The last tick of each phase is exactly on the frame's target.
          bool last = i + 1 == want.size() || want[i + 1] != want[i];
          if (last && kf_claims(b.kf, (uint8_t)k)) {
            uint8_t mk = mirror && k >= 3 ? (uint8_t)(((k - 3) ^ 1) + 3) : (uint8_t)k;
            int v = (int8_t)L.frames[want[i] * 14 + 3 + mk];
            if (mirror && (k == 0 || k == 2)) v = -v;
            if (raw != v * 16) targets_ok = false;
          }
        }
      }
      CHECKF(got == want, "%s variant %d: frame timing differs from the JSON", L.name, variant);
      CHECKF(limits_ok, "%s variant %d: an angle beyond a joint limit", L.name, variant);
      CHECKF(targets_ok, "%s variant %d: a phase did not end on its target", L.name, variant);
      CHECK(dones() == 0);
      tick();
      CHECKF(dones(KF_END_DONE) == 1, "%s variant %d: no TRICK_DONE on time", L.name, variant);
      CHECK(arbiter_idle_posture(b.arb) == L.begin[8]);
      tick(PARAM(XFADE_TICKS) + 30);
      CHECKF(at_posture(L.begin[8]), "%s: not at the end posture", L.name);
    }
  }
}

int main() {
  test_upload();
  test_easing();
  test_hold_speed();
  test_loops();
  test_mirror();
  test_owners_and_preemption();
  test_gating();
  test_crossfades();
  test_balance();
  test_link();
  test_library();
  return report("keyframe");
}
