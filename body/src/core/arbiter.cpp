#include "arbiter.h"
#include "pgm.h"
#include "fx.h"
#include "posture.h"
#include "params.h"

// ---- GAIT (P2, core/gait.h) ----------------------------------------------
// The gait owns the legs (canonical k >= 3) while it runs or stops, and
// supplies this tick's IK angles from g.leg[] (shoulders LF RF RB LB then
// knees, i.e. k - 3). TRICK (core/keyframe.h) is wired the same way in
// arbiter_run. The crossfade, layers and safety stage then apply unchanged.
static inline bool gait_claims(const Gait* g, uint8_t k) { return g && k >= 3 && g->state != GS_OFF; }
// Head layer (P1, doc §3c layer 1): a held head joint (k = 0..2) under IDLE
// or GAIT steps toward the Pi's target from its last output, at most
// head_step a16 per tick (the safety slew still applies after). The gait's
// bob and wag (P2) are added to the IDLE base when nothing is held.
static inline bool head_holds(const Arbiter& a, uint8_t k, uint8_t own) {
  return k < 3 && a.head_ttl && own <= OWN_GAIT && a.head_tgt[k] != JOINT_OFF;
}
// (The last output is within the joint limits and head_step <= 0x3FFF, so
// from +- step fits 16 bits whatever the target; the target is never
// subtracted.)
static inline int16_t head_layer(const Arbiter& a, uint8_t k) {
  int16_t from = a.last_on[k], s = a.head_step, t = a.head_tgt[k];
  if (t > from + s) return (int16_t)(from + s);
  if (t < from - s) return (int16_t)(from - s);
  return t;
}
// Balance layer (doc §3c layer 2, §3f): joint-space owners only. IDLE holds
// the built-in postures and TRICK the keyframes (body.cpp runs the
// controller only for frames with the balance bit while a trick plays, and
// lets the corrections decay otherwise); GAIT and STANCE get the geometric
// balance through the IK instead (body.cpp passes bal = null then, core/
// gait.h); REFLEX never balances.
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

// legs: STANCE's leg angles this tick (Gait::leg, from the IK), or null.
KILO_NOINLINE static int16_t mover_angle(const Mover& m, uint8_t k, const int16_t* legs) {
  if (m.posture == POSTURE_LIMP) return JOINT_OFF;
  int16_t from = m.from[k];
  if (m.posture == POSTURE_HOLD) return from;
  // STANCE: the head and tail of STAND, the legs from the IK.
  int16_t to = (m.posture == POSTURE_STANCE && k >= 3 && legs) ? legs[k - 3] : posture_angle(m.posture, k);
  // From limp there is nothing to interpolate from: the servo's real pose is
  // unknown. Command the target and let the slew limit pace it.
  if (from == JOINT_OFF) return to;
  return lerp(from, to, fx_ease_cos(m.i, m.n));
}

void arbiter_init(Arbiter& a, const int16_t* warm) {
  a.reflex_active = 0;
  a.head_ttl = 0;
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
                 const Keyframe* kf, const Gait* g) {
  const int16_t* legs = g ? g->leg : 0;
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
    else if (gait_claims(g, k))
      own = OWN_GAIT;
    else
      own = OWN_IDLE;
    // 2. Crossfade on a change of owner, from the last output. TRICK and
    //    REFLEX start from the current pose themselves (the trick reads it
    //    back from xsrc), so they get none; out of TRICK the player says how
    //    long, everything else fades over XFADE_TICKS.
    //    A head joint held by the Pi counts as owned by PI_HOLD (its base is
    //    replaced in step 3), so taking and releasing the hold are hand-overs
    //    too.
    bool held = head_holds(a, k, own);
    uint8_t tag = held ? (uint8_t)OWN_PI_HOLD : own;
    if (tag != a.owner[k]) {
      uint8_t n = (uint8_t)PARAM(XFADE_TICKS);
      if (a.owner[k] == OWN_TRICK && kf) n = kf->xfade;
      if (own == OWN_TRICK || own == OWN_REFLEX) n = 0;
      a.owner[k] = tag;
      a.xsrc[k] = a.out[k];
      a.xn[k] = n;
      a.xi[k] = 0;
    }
    int16_t raw;
    if (own == OWN_REFLEX) {
      raw = mover_angle(a.reflex, k, legs);
    } else if (own == OWN_TRICK) {
      raw = kf_angle(*kf, k, a.xsrc[k]);
    } else if (own == OWN_GAIT) {
      raw = g->leg[k - 3];
    } else {
      raw = mover_angle(a.idle, k, legs);
      // Head bob and tail wag ride on IDLE's head while the gait runs (a
      // hold replaces the base below, so it wins).
      if (g && k < 3 && raw != JOINT_OFF) raw = (int16_t)(raw + gait_head(*g, k));
    }
    int16_t b = raw;
    if (a.xi[k] < a.xn[k]) {
      a.xi[k]++;
      if (raw != JOINT_OFF && a.xsrc[k] != JOINT_OFF) b = lerp(a.xsrc[k], raw, fx_ease_cos(a.xi[k], a.xn[k]));
    }
    // 3. Layers. Never applied to a limp joint. A held head joint takes the
    //    Pi's target instead of its base, and no balance correction.
    if (b != JOINT_OFF) b = held ? head_layer(a, k) : balance_layer(k, own, b, bal);
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
  // The hold lasts head_ttl runs from the last SET_HEAD (HEAD_HOLD_MS).
  if (a.head_ttl) a.head_ttl--;
}
