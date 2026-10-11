// Planar leg IK/FK (core/ik.*, doc §3d "Planar IK", Appendix H "ik"):
// round trips on a 1 mm grid over the workspace against a double-precision
// reference (<= 0.25 mm, <= 0.25 deg), several leg geometries (the
// sanitizers catch any int32 overflow), the workspace clamp, and FK of the
// legacy postures (feet below the body, left = right).
#include <cmath>
#include <cstdio>
#include "../src/core/fx.h"
#include "../src/core/ik.h"
#include "../src/core/params.h"
#include "../src/core/posture.h"
#include "check.h"

static const double D2R = M_PI / 180.0;

// The model in double: joint angles (deg) -> foot (mm).
static void fk_ref(int leg, double sh_deg, double kn_deg, double* x, double* z) {
  double l1 = PARAM(LEG_L1_MM), l2 = PARAM(LEG_L2_MM);
  double t1 = PARAM(LEG_SH_ZERO_DEG) + IK_SGN_SH * sh_deg;
  double t2 = ik_sigma((uint8_t)leg) * PARAM(LEG_KN_ZERO_DEG) + IK_SGN_KN * kn_deg;
  *x = l1 * std::sin(t1 * D2R) + l2 * std::sin((t1 + t2) * D2R);
  *z = l1 * std::cos(t1 * D2R) + l2 * std::cos((t1 + t2) * D2R);
}

// The IK in double (mm in, joint degrees out), no clamp.
static void ik_ref(int leg, double x, double z, double* sh, double* kn) {
  double l1 = PARAM(LEG_L1_MM), l2 = PARAM(LEG_L2_MM);
  double c = (x * x + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  double s = ik_sigma((uint8_t)leg) * std::sqrt(1 - c * c);
  double t2 = std::atan2(s, c) / D2R;
  double t1 = (std::atan2(x, z) - std::atan2(l2 * s, l1 + l2 * c)) / D2R;
  *sh = IK_SGN_SH * (t1 - PARAM(LEG_SH_ZERO_DEG));
  *kn = IK_SGN_KN * (t2 - ik_sigma((uint8_t)leg) * PARAM(LEG_KN_ZERO_DEG));
}

static double wrap180(double d) { return std::fmod(d + 540.0, 360.0) - 180.0; }

struct Stats {
  double pos = 0, ang = 0, fkfix = 0;
  long n = 0;
};

// Every q2 point on a 1 mm grid strictly inside the workspace (with a 0.5 mm
// margin to the clamp), all four legs.
// bound: the position criterion. The a16 output step (1/16 deg) alone moves
// the foot up to (L1 + L2) * 1.1e-3 mm, so the 0.25 mm target of §3d holds
// for real Nybble legs (L1 + L2 ~ 100 mm) with room to spare, but not for
// the 2 x 120 mm extreme of the params' range, which only checks overflow.
static void grid(Stats& st, double bound = 0.25) {
  double l1 = PARAM(LEG_L1_MM), l2 = PARAM(LEG_L2_MM);
  double dmax = (l1 + l2) * 243.0 / 256.0 - 0.5;
  double dmin = std::sqrt(l1 * l1 + l2 * l2 - 2 * l1 * l2 * std::cos(30 * D2R)) + 0.5;
  for (int leg = 0; leg < 4; leg++)
    for (int xm = -(int)dmax; xm <= (int)dmax; xm++)
      for (int zm = -(int)dmax; zm <= (int)dmax; zm++) {
        double d = std::hypot(xm, zm);
        if (d > dmax || d < dmin) continue;
        int16_t sh, kn;
        bool ok = ik_leg((uint8_t)leg, (int16_t)(xm * 4), (int16_t)(zm * 4), &sh, &kn);
        CHECKF(ok, "leg %d (%d, %d) reported clamped", leg, xm, zm);
        double x, z, rsh, rkn;
        fk_ref(leg, sh / 16.0, kn / 16.0, &x, &z);
        double ep = std::hypot(x - xm, z - zm);
        ik_ref(leg, xm, zm, &rsh, &rkn);
        double ea = std::fmax(std::fabs(wrap180(sh / 16.0 - rsh)), std::fabs(wrap180(kn / 16.0 - rkn)));
        int16_t fx, fz;
        ik_fk((uint8_t)leg, sh, kn, &fx, &fz);
        double ef = std::hypot(fx / 4.0 - x, fz / 4.0 - z);
        if (ep > st.pos) st.pos = ep;
        if (ea > st.ang) st.ang = ea;
        if (ef > st.fkfix) st.fkfix = ef;
        st.n++;
        if (ep > bound || ea > 0.25 || ef > bound)
          CHECKF(false, "L %g/%g leg %d (%d, %d): pos err %.3f mm, angle err %.3f deg, fx fk %.3f", l1, l2, leg, xm,
                 zm, ep, ea, ef);
      }
}

static void set_geometry(int l1, int l2) {
  PARAM(LEG_L1_MM) = (int16_t)l1;
  PARAM(LEG_L2_MM) = (int16_t)l2;
}

int main() {
  // ---- Round trips on the default geometry and a spread of others.
  static const int GEOM[][2] = {{50, 50}, {45, 55}, {55, 45}, {20, 20}, {80, 80}, {120, 120}, {40, 70}, {70, 40}};
  for (const auto& g : GEOM) {
    set_geometry(g[0], g[1]);
    Stats st;
    grid(st, g[0] + g[1] <= 160 ? 0.25 : 0.35);
    std::printf("ik: L1 %3d L2 %3d: %6ld points, max |FK(IK(p)) - p| %.3f mm, max angle err %.3f deg, "
                "fixed-point FK %.3f mm\n",
                g[0], g[1], st.n, st.pos, st.ang, st.fkfix);
    CHECK(st.n > 1000);
  }
  // Non-default calibration-posture angles move the zero, nothing else.
  set_geometry(50, 50);
  PARAM(LEG_SH_ZERO_DEG) = 7;
  PARAM(LEG_KN_ZERO_DEG) = 80;
  {
    Stats st;
    grid(st);
    CHECKF(st.pos <= 0.25 && st.ang <= 0.25, "zero offsets: %.3f mm %.3f deg", st.pos, st.ang);
  }
  PARAM(LEG_SH_ZERO_DEG) = KILO_PARAM_LEG_SH_ZERO_DEG_DEFAULT;
  PARAM(LEG_KN_ZERO_DEG) = KILO_PARAM_LEG_KN_ZERO_DEG_DEFAULT;

  // ---- The clamp: beyond reach the foot lands on the same ray at
  // 0.95 (L1 + L2); too close, at d_min; both report false.
  for (int leg = 0; leg < 4; leg++) {
    static const int TGT[][2] = {{0, 400}, {300, 300}, {-250, 380}, {400, 0}, {0, 20}, {8, 8}, {-12, 30}};
    for (const auto& t : TGT) {
      int16_t sh, kn;
      bool ok = ik_leg((uint8_t)leg, (int16_t)t[0], (int16_t)t[1], &sh, &kn);
      CHECK(!ok);
      double x, z;
      fk_ref(leg, sh / 16.0, kn / 16.0, &x, &z);
      double d_tgt = std::hypot(t[0], t[1]) / 4.0, d = std::hypot(x, z);
      double want = d_tgt > 50 ? 100.0 * 243 / 256 : std::sqrt(5000 - 5000 * std::cos(30 * D2R));
      CHECKF(std::fabs(d - want) < 0.3, "clamp leg %d (%d,%d): d %.2f want %.2f", leg, t[0], t[1], d, want);
      double dir = std::fabs(wrap180(std::atan2(x, z) / D2R - std::atan2(t[0], t[1]) / D2R));
      CHECKF(dir < 0.3, "clamp leg %d keeps the direction (off by %.2f deg)", leg, dir);
    }
    // At the edge, just inside and just outside agree.
    int16_t a1, k1, a2, k2;
    CHECK(ik_leg((uint8_t)leg, 0, 379, &a1, &k1));
    CHECK(!ik_leg((uint8_t)leg, 0, 381, &a2, &k2));
    CHECK(std::abs(a1 - a2) <= 2 && std::abs(k1 - k2) <= 2);
  }

  // ---- FK of the legacy postures: every foot below the shoulder (z > 0),
  // left = right (same OpenCat angles on both sides), front and back mirror
  // images. Printed so the conventions can be eyeballed against the robot.
  static const struct {
    uint8_t id;
    const char* name;
  } POS[] = {{POSTURE_STAND, "up"}, {POSTURE_SIT, "sit"}, {POSTURE_REST, "rest"}};
  for (const auto& p : POS) {
    double fx[4], fz[4];
    for (int leg = 0; leg < 4; leg++) {
      int16_t sh = posture_angle(p.id, (uint8_t)(3 + leg)), kn = posture_angle(p.id, (uint8_t)(7 + leg));
      // The safety stage clamps sit's back shoulders to -80 deg.
      if (sh < -80 * 16 && !leg_front((uint8_t)leg)) sh = -80 * 16;
      fk_ref(leg, sh / 16.0, kn / 16.0, &fx[leg], &fz[leg]);
      CHECKF(fz[leg] > 10, "%s leg %d foot z %.1f mm: not below the body", p.name, leg, fz[leg]);
    }
    CHECK(std::fabs(fx[LEG_LF] - fx[LEG_RF]) < 1e-9 && std::fabs(fz[LEG_LF] - fz[LEG_RF]) < 1e-9);
    CHECK(std::fabs(fx[LEG_LB] - fx[LEG_RB]) < 1e-9 && std::fabs(fz[LEG_LB] - fz[LEG_RB]) < 1e-9);
    double pitch = std::atan2(fz[LEG_LF] - fz[LEG_LB], PARAM(LEG_LB_MM)) / D2R;  // front higher = nose up = -pitch
    std::printf("ik: %-4s front foot (%5.1f, %5.1f) mm, back foot (%5.1f, %5.1f) mm, body pitch %+5.1f deg\n",
                p.name, fx[LEG_LF], fz[LEG_LF], fx[LEG_LB], fz[LEG_LB], -pitch);
    if (p.id == POSTURE_STAND) {
      // up: both feet under their shoulders, front and back mirrored.
      CHECK(std::fabs(fx[LEG_LF]) < 1 && std::fabs(fx[LEG_LB]) < 1 && std::fabs(fz[LEG_LF] - fz[LEG_LB]) < 1e-6);
    }
    if (p.id == POSTURE_SIT) CHECK(-pitch < -20 && -pitch > -40);  // nose up, as its header's -30
  }

  // ---- STANCE: x = 0 at 0.8 (L1 + L2) is within every joint limit.
  for (int leg = 0; leg < 4; leg++) {
    int16_t sh, kn;
    CHECK(ik_leg((uint8_t)leg, 0, (int16_t)(ik_reach_q2() * 4 / 5), &sh, &kn));
    uint8_t js = (uint8_t)(8 + leg), jk = (uint8_t)(12 + leg);
    CHECK(joint_clamp(js, sh) == sh && joint_clamp(jk, kn) == kn);
    if (leg == 0) std::printf("ik: STANCE (default) front shoulder %.1f knee %.1f deg\n", sh / 16.0, kn / 16.0);
  }
  return report("ik");
}
