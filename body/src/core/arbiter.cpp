#include "arbiter.h"
#include "pgm.h"
#include "fx.h"
#include "posture.h"
#include "params.h"

// ---- Hooks for the owners of later phases --------------------------------
// GAIT (P2, gait generator + IK) plugs in here: return true from
// gait_claims(k) while it is active and its mask holds canonical joint k,
// and supply the angle from gait_angle(k). TRICK (core/keyframe.h) is wired
// the same way in arbiter_run. The crossfade, layers and safety stage then
// apply to them unchanged.
static inline bool gait_claims(uint8_t) { return false; }  // legs only: k >= 3
static inline int16_t gait_angle(uint8_t) { return 0; }
// Head layer (P1): Pi head target or gait bob/wag on k = 0..2.
static inline int16_t head_layer(uint8_t, uint8_t, int16_t base) { return base; }
// Balance layer (doc §3c layer 2, §3f): joint-space owners only. IDLE holds
// the built-in postures and TRICK the keyframes (body.cpp runs the
// controller only for frames with the balance bit while a trick plays, and
// lets the corrections decay otherwise); GAIT and STANCE get geometric
// balance through the IK in P2; REFLEX never balances.
static inline int16_t balance_layer(uint8_t k, uint8_t own, int16_t base, const Balance* bal) {
  if (!bal || own == OWN_GAIT || own == OWN_REFLEX) return base;
  return (int16_t)(base + balance_joint(*bal, k));
}
// ---------------------------------------------------------------------------

// a + (b - a) * e, e in Q14. The difference of two a16 angles fits 17 bits,
// so widen before multiplying.
KILO_NOINLINE static int16_t lerp(int16_t a, int16_t b, uint16_t e) {
  return (int16_t)(a + (((int32_t)(b - a) * e + 8192) >> 14));
}

KILO_NOINLINE static void mover_start(Mover& m, uint8_t posture, uint8_t ticks, const int16_t* cur) {
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++) m.from[k] = cur[k];
  m.posture = posture;
  m.n = ticks;
  m.i = 0;
}

KILO_NOINLINE static int16_t mover_angle(const Mover& m, uint8_t k) {
  if (m.posture == POSTURE_LIMP) return JOINT_OFF;
  int16_t from = m.from[k];
  if (m.posture == POSTURE_HOLD) return from;
  int16_t to = posture_angle(m.posture, k);
  // From limp there is nothing to interpolate from: the servo's real pose is
  // unknown. Command the target and let the slew limit pace it.
  if (from == JOINT_OFF) return to;
  return lerp(from, to, fx_ease_cos(m.i, m.n));
}

void arbiter_init(Arbiter& a, const int16_t* warm) {
  a.reflex_active = 0;
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++) {
    int16_t v = warm ? warm[k] : JOINT_OFF;
    a.out[k] = v;
    // Slew reference for a limp joint: REST, the most likely resting pose
    // of an unpowered cat (and where legacy started).
    a.last_on[k] = v == JOINT_OFF ? posture_angle(POSTURE_REST, k) : v;
    a.owner[k] = warm ? OWN_REFLEX : OWN_IDLE;
    a.xn[k] = a.xi[k] = 0;
  }
  // Cold: IDLE is LIMP. Warm: IDLE and REFLEX both hold the read-back pose,
  // REFLEX owning it until the warm-boot hold ends.
  mover_start(a.idle, warm ? POSTURE_HOLD : POSTURE_LIMP, 0, a.out);
  if (warm) arbiter_reflex(a, POSTURE_HOLD, 0);
}

void arbiter_set_idle(Arbiter& a, uint8_t posture, uint8_t ticks) {
  if (posture == a.idle.posture) return;
  mover_start(a.idle, posture, ticks ? ticks : (uint8_t)PARAM(POSTURE_TICKS), a.out);
}

uint8_t arbiter_idle_posture(const Arbiter& a) { return a.idle.posture; }

void arbiter_reflex(Arbiter& a, uint8_t posture, uint8_t ticks) {
  if (a.reflex_active && a.reflex.posture == posture) return;
  mover_start(a.reflex, posture, ticks, a.out);
  a.reflex_active = 1;
}

void arbiter_reflex_release(Arbiter& a) { a.reflex_active = 0; }

bool arbiter_reflex_settled(const Arbiter& a) { return a.reflex.i >= a.reflex.n; }

void arbiter_run(Arbiter& a, const JointCal& cal, uint16_t counts[JOINT_COUNT], const Balance* bal,
                 const Keyframe* kf) {
  // Unused channels (joints 3-7) stay full-off; driven ones are set below.
  for (uint8_t c = 0; c < JOINT_COUNT; c++) counts[c] = PCA_FULL_OFF;
  if (a.idle.i < a.idle.n) a.idle.i++;
  if (a.reflex_active && a.reflex.i < a.reflex.n) a.reflex.i++;

  for (uint8_t k = 0; k < DRIVEN_COUNT; k++) {
    uint8_t j = joint_id(k);
    // 1. Owner selection, highest priority first.
    uint8_t own;
    if (a.reflex_active)
      own = OWN_REFLEX;
    else if (kf && kf_claims(*kf, k))
      own = OWN_TRICK;
    else if (gait_claims(k))
      own = OWN_GAIT;
    else
      own = OWN_IDLE;
    // 2. Crossfade on a change of owner, from the last output. TRICK and
    //    REFLEX start from the current pose themselves (the trick reads it
    //    back from xsrc), so they get none; out of TRICK the player says how
    //    long, everything else fades over XFADE_TICKS.
    if (own != a.owner[k]) {
      uint8_t n = (uint8_t)PARAM(XFADE_TICKS);
      if (a.owner[k] == OWN_TRICK && kf) n = kf->xfade;
      if (own == OWN_TRICK || own == OWN_REFLEX) n = 0;
      a.owner[k] = own;
      a.xsrc[k] = a.out[k];
      a.xn[k] = n;
      a.xi[k] = 0;
    }
    int16_t raw;
    if (own == OWN_REFLEX)
      raw = mover_angle(a.reflex, k);
    else if (own == OWN_TRICK)
      raw = kf_angle(*kf, k, a.xsrc[k]);
    else if (own == OWN_GAIT)
      raw = gait_angle(k);
    else
      raw = mover_angle(a.idle, k);
    int16_t b = raw;
    if (a.xi[k] < a.xn[k]) {
      a.xi[k]++;
      if (raw != JOINT_OFF && a.xsrc[k] != JOINT_OFF) b = lerp(a.xsrc[k], raw, fx_ease_cos(a.xi[k], a.xn[k]));
    }
    // 3. Layers. Never applied to a limp joint.
    if (b != JOINT_OFF) b = balance_layer(k, own, head_layer(k, own, b), bal);
    // 4. Safety: limits, then slew against the last powered output, then
    //    counts. Limits come first so the slew never chases an angle the
    //    joint may not reach.
    if (b == JOINT_OFF) {
      a.out[k] = JOINT_OFF;
      continue;
    }
    int16_t t = joint_clamp(j, b);
    int16_t d = (int16_t)(t - a.last_on[k]);
    int16_t slew = PARAM(SLEW_MAX_A16);
    if (d > slew) d = slew;
    if (d < -slew) d = (int16_t)-slew;
    int16_t o = (int16_t)(a.last_on[k] + d);
    a.last_on[k] = a.out[k] = o;
    counts[joint_channel(j)] = joint_counts(cal, j, o);
  }
}
