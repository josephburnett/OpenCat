// A plant that tips: Kilo's body as a point mass on four position-controlled
// legs, for the host tests of the FEET player and the balance (doc §3f
// "The tipping plant"). Double precision, host only.
//
// What it models:
//   - the servos: each leg joint follows its commanded angle (the
//     arbiter's output) through a first-order lag, tau 0.15 s;
//   - the legs: the body's own IK model run forwards (L1, L2, the zero
//     angles, the joint signs), so a foot is where the joints put it;
//     the hips at (+-Lb/2, +-W/2) around the body centre;
//   - the mass: one point at the body centre, `com_up` mm above the hips
//     (and `com_x`, `com_y` forward / left of the centre), configurable;
//   - the floor: level and rigid; the feet are points;
//   - resting: the body rests on three feet whose triangle holds the
//     mass's projection with the fourth foot not below their plane (on
//     four coplanar feet any of them); the legs move it quasi-statically
//     (the attitude is that plane's, every 1 ms sub-step);
//   - tipping: when no such triangle is within reach of the current
//     attitude (a foot lifted from under the mass, the mass carried over an
//     edge, a shove), the body turns about the edge of the feet on the
//     ground nearest the mass (or about one foot) as an inverted pendulum,
//     alpha = g d / L^2 (d the mass's horizontal distance outside the edge,
//     L its distance from the edge), until another foot reaches the floor:
//     an inelastic impact (the turn stops) and it rests again if it can;
//     with the mass ~76 mm above the feet the time constant is
//     sqrt(L/g) ~ 90 ms: from 2 deg past an edge, 30 deg in ~0.25 s (the
//     floor, 2026-10-14: about 30 deg in 0.2 s once it went);
//   - fallen: more than 80 deg from level (it stays there: past the fall
//     reflex's 75 deg);
//   - the IMU: the world pitch and roll (Kilo's convention: nose down +,
//     right side up +) and their rates (differences over the 20 ms tick).
// What it does not model: foot slip, compliance and backlash, the servo's
// torque limit, the head and tail, yaw.
#ifndef KILO_TEST_TIP_PLANT_H
#define KILO_TEST_TIP_PLANT_H
#include <cmath>
#include "../src/core/ik.h"
#include "../src/core/joints.h"
#include "../src/core/params.h"

struct TipPlant {
  // Configuration.
  double com_up = 0, com_x = 0, com_y = 0;  // mm
  double servo_tau = 0.15;                  // s
  int substeps = 20;                        // per 20 ms tick
  // State.
  double ang[8] = {};      // the joints through the servo lag, deg (shoulders LF RF RB LB, knees)
  double R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // body -> world
  bool tipping = false;
  int piv[2] = {-1, -1};   // the foot (feet) it turns about
  double out_n[2] = {0, 0};  // horizontal unit normal of the pivot edge, outward (the falling side)
  double w = 0;            // turn rate about the pivot, rad/s (+ = falling outward)
  bool fallen = false;
  int tips = 0;            // tips started (each one that went past a support)
  // Outputs (deg, deg/s).
  double pitch = 0, roll = 0, rate_pitch = 0, rate_roll = 0;
  double max_tilt = 0;     // largest |pitch| or |roll| since reset_max()

  static constexpr double D2R = M_PI / 180.0;
  static constexpr double G = 9810.0;  // mm/s^2

  // Foot l in the body frame (x forward, y left, z up), mm, from the lagged joints.
  void foot(int l, double f[3]) const {
    double l1 = PARAM(LEG_L1_MM), l2 = PARAM(LEG_L2_MM);
    double t1 = PARAM(LEG_SH_ZERO_DEG) + IK_SGN_SH * ang[l];
    double t2 = ik_sigma((uint8_t)l) * PARAM(LEG_KN_ZERO_DEG) + IK_SGN_KN * ang[4 + l];
    double x = l1 * std::sin(t1 * D2R) + l2 * std::sin((t1 + t2) * D2R);
    double z = l1 * std::cos(t1 * D2R) + l2 * std::cos((t1 + t2) * D2R);
    f[0] = (leg_front((uint8_t)l) ? 0.5 : -0.5) * PARAM(LEG_LB_MM) + x;
    f[1] = (leg_left((uint8_t)l) ? 0.5 : -0.5) * PARAM(LEG_W_MM);
    f[2] = -z;
  }
  void com(double c[3]) const { c[0] = com_x, c[1] = com_y, c[2] = com_up; }
  void world(const double v[3], double o[3]) const {
    for (int i = 0; i < 3; i++) o[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
  }
  // Pitch and roll of R (the body's up axis in the world: R^T z = (-sin p,
  // -sin r cos p, cos r cos p)).
  void attitude(double* p, double* r) const {
    *p = std::asin(std::fmax(-1.0, std::fmin(1.0, -R[2][0]))) / D2R;
    *r = std::atan2(-R[2][1], R[2][2]) / D2R;
  }
  // R = Ry(p) Rx(-r): no yaw.
  void set_attitude(double p, double r) {
    double cp = std::cos(p * D2R), sp = std::sin(p * D2R), cr = std::cos(-r * D2R), sr = std::sin(-r * D2R);
    double Ry[3][3] = {{cp, 0, sp}, {0, 1, 0}, {-sp, 0, cp}};
    double Rx[3][3] = {{1, 0, 0}, {0, cr, -sr}, {0, sr, cr}};
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) R[i][j] = Ry[i][0] * Rx[0][j] + Ry[i][1] * Rx[1][j] + Ry[i][2] * Rx[2][j];
  }
  // R = Rot(k, a) R (k a world unit vector).
  void rotate(const double k[3], double a) {
    double c = std::cos(a), s = std::sin(a), t = 1 - c;
    double M[3][3] = {{t * k[0] * k[0] + c, t * k[0] * k[1] - s * k[2], t * k[0] * k[2] + s * k[1]},
                      {t * k[0] * k[1] + s * k[2], t * k[1] * k[1] + c, t * k[1] * k[2] - s * k[0]},
                      {t * k[0] * k[2] - s * k[1], t * k[1] * k[2] + s * k[0], t * k[2] * k[2] + c}};
    double N[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) N[i][j] = M[i][0] * R[0][j] + M[i][1] * R[1][j] + M[i][2] * R[2][j];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) R[i][j] = N[i][j];
  }

  // The rest attitude nearest (p0, r0): the plane of three feet that hold
  // the mass with the fourth not below it. False if there is none.
  bool rest(double p0, double r0, double* pr, double* rr) const {
    double f[4][3], c[3];
    for (int l = 0; l < 4; l++) foot(l, f[l]);
    com(c);
    bool any = false;
    double best = 1e9;
    for (int skip = 0; skip < 4; skip++) {
      int t[3], n = 0;
      for (int l = 0; l < 4; l++)
        if (l != skip) t[n++] = l;
      double u[3], v[3], nn[3];
      for (int i = 0; i < 3; i++) u[i] = f[t[1]][i] - f[t[0]][i], v[i] = f[t[2]][i] - f[t[0]][i];
      nn[0] = u[1] * v[2] - u[2] * v[1], nn[1] = u[2] * v[0] - u[0] * v[2], nn[2] = u[0] * v[1] - u[1] * v[0];
      double len = std::sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
      if (len < 1e-9) continue;
      if (nn[2] < 0) len = -len;  // the normal toward the body (up)
      for (int i = 0; i < 3; i++) nn[i] /= len;
      // The fourth foot not below the plane.
      double d4 = 0;
      for (int i = 0; i < 3; i++) d4 += (f[skip][i] - f[t[0]][i]) * nn[i];
      if (d4 < -0.05) continue;
      // The mass's projection along the normal inside the triangle.
      bool in = true;
      for (int e = 0; e < 3 && in; e++) {
        const double* a = f[t[e]];
        const double* b = f[t[(e + 1) % 3]];
        const double* o = f[t[(e + 2) % 3]];
        double ab[3], ac[3], ao[3], x1[3], x2[3];
        for (int i = 0; i < 3; i++) ab[i] = b[i] - a[i], ac[i] = c[i] - a[i], ao[i] = o[i] - a[i];
        x1[0] = ab[1] * ac[2] - ab[2] * ac[1], x1[1] = ab[2] * ac[0] - ab[0] * ac[2], x1[2] = ab[0] * ac[1] - ab[1] * ac[0];
        x2[0] = ab[1] * ao[2] - ab[2] * ao[1], x2[1] = ab[2] * ao[0] - ab[0] * ao[2], x2[2] = ab[0] * ao[1] - ab[1] * ao[0];
        double s1 = x1[0] * nn[0] + x1[1] * nn[1] + x1[2] * nn[2];
        double s2 = x2[0] * nn[0] + x2[1] * nn[1] + x2[2] * nn[2];
        in = s1 * s2 >= 0;
      }
      if (!in) continue;
      double p = std::asin(-nn[0]) / D2R, r = std::atan2(-nn[1], nn[2]) / D2R;
      double d = std::fabs(p - p0) + std::fabs(r - r0);
      if (d < best) best = d, *pr = p, *rr = r, any = true;
    }
    return any;
  }

  // World positions of the feet relative to the body centre, and the index
  // of the lowest.
  int feet_world(double w[4][3]) const {
    int lo = 0;
    for (int l = 0; l < 4; l++) {
      double f[3];
      foot(l, f);
      world(f, w[l]);
      if (w[l][2] < w[lo][2]) lo = l;
    }
    return lo;
  }

  // The feet the body stands on now: of the triangles whose fourth foot is
  // not below their plane, the one whose plane is nearest the current
  // attitude (all four when the fourth is on it). Bits LF RF RB LB.
  uint8_t standing_on(double p0, double r0) const {
    double f[4][3];
    for (int l = 0; l < 4; l++) foot(l, f[l]);
    uint8_t on = 0xF;
    double best = 1e9;
    for (int skip = 0; skip < 4; skip++) {
      int t[3], n = 0;
      for (int l = 0; l < 4; l++)
        if (l != skip) t[n++] = l;
      double u[3], v[3], nn[3];
      for (int i = 0; i < 3; i++) u[i] = f[t[1]][i] - f[t[0]][i], v[i] = f[t[2]][i] - f[t[0]][i];
      nn[0] = u[1] * v[2] - u[2] * v[1], nn[1] = u[2] * v[0] - u[0] * v[2], nn[2] = u[0] * v[1] - u[1] * v[0];
      double len = std::sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
      if (len < 1e-9) continue;
      if (nn[2] < 0) len = -len;
      for (int i = 0; i < 3; i++) nn[i] /= len;
      double d4 = 0;
      for (int i = 0; i < 3; i++) d4 += (f[skip][i] - f[t[0]][i]) * nn[i];
      if (d4 < -0.05) continue;
      double p = std::asin(-nn[0]) / D2R, r = std::atan2(-nn[1], nn[2]) / D2R;
      double d = std::fabs(p - p0) + std::fabs(r - r0);
      if (d < best - 1e-6) best = d, on = (uint8_t)(0xF & ~(1 << skip)) | (d4 < 0.05 ? (1 << skip) : 0);
    }
    return on;
  }

  // Start a turn about the edge (or foot) of the feet it stands on nearest
  // the mass's projection (which is outside them).
  void start_tip() {
    double w4[4][3], c[3], cw[3], p, r;
    feet_world(w4);
    com(c);
    world(c, cw);
    attitude(&p, &r);
    uint8_t on = standing_on(p, r);
    int f[4], n = 0;
    for (int l = 0; l < 4; l++)  // LF RF RB LB: the cyclic order around the body
      if (on & (1 << l)) f[n++] = l;
    double best = 1e18, nx = 1, ny = 0;
    for (int i = 0; i < n; i++) {
      int a = f[i], b = f[(i + 1) % n];
      double ex = w4[b][0] - w4[a][0], ey = w4[b][1] - w4[a][1], l2 = ex * ex + ey * ey;
      double t = l2 > 1e-9 ? ((cw[0] - w4[a][0]) * ex + (cw[1] - w4[a][1]) * ey) / l2 : 0;
      bool vertex = t <= 0 || t >= 1;
      t = std::fmax(0.0, std::fmin(1.0, t));
      double px = w4[a][0] + t * ex, py = w4[a][1] + t * ey;
      double d = (cw[0] - px) * (cw[0] - px) + (cw[1] - py) * (cw[1] - py);
      if (d < best) {
        best = d, nx = cw[0] - px, ny = cw[1] - py;
        if (vertex)
          piv[0] = t <= 0 ? a : b, piv[1] = -1;
        else
          piv[0] = a, piv[1] = b;
      }
    }
    double ln = std::sqrt(nx * nx + ny * ny);
    if (ln < 1e-6) nx = 1, ny = 0, ln = 1;  // on the edge: either way
    out_n[0] = nx / ln, out_n[1] = ny / ln;
    tipping = true;
    tips++;
  }

  // A shove: start a turn about the support edge on the side `dir` (world
  // horizontal unit vector the body is pushed toward) at `rate` deg/s.
  void shove(double dir_x, double dir_y, double rate) {
    double w4[4][3];
    int lo = feet_world(w4);
    // The two feet on the floor furthest along dir.
    int a = -1, b = -1;
    for (int l = 0; l < 4; l++) {
      if (w4[l][2] > w4[lo][2] + 0.5) continue;
      double s = w4[l][0] * dir_x + w4[l][1] * dir_y;
      if (a < 0 || s > w4[a][0] * dir_x + w4[a][1] * dir_y) b = a, a = l;
      else if (b < 0 || s > w4[b][0] * dir_x + w4[b][1] * dir_y) b = l;
    }
    piv[0] = a, piv[1] = b;
    // Outward: perpendicular to the edge, toward dir.
    double ex = w4[b][0] - w4[a][0], ey = w4[b][1] - w4[a][1], l = std::sqrt(ex * ex + ey * ey);
    double nx = ey / l, ny = -ex / l;
    if (nx * dir_x + ny * dir_y < 0) nx = -nx, ny = -ny;
    out_n[0] = nx, out_n[1] = ny;
    w = rate * D2R;
    tipping = true;
  }

  // One turn sub-step of h seconds about the pivot.
  void tip_step(double h) {
    double w4[4][3], c[3], cw[3];
    feet_world(w4);
    com(c);
    world(c, cw);
    double P[3], k[3];
    if (piv[1] >= 0) {
      double ex = w4[piv[1]][0] - w4[piv[0]][0], ey = w4[piv[1]][1] - w4[piv[0]][1], l = std::sqrt(ex * ex + ey * ey);
      ex /= l, ey /= l;
      // Keep the outward normal perpendicular to the edge, on its side.
      double nx = ey, ny = -ex;
      if (nx * out_n[0] + ny * out_n[1] < 0) nx = -nx, ny = -ny;
      out_n[0] = nx, out_n[1] = ny;
      double t = (cw[0] - w4[piv[0]][0]) * ex + (cw[1] - w4[piv[0]][1]) * ey;
      P[0] = w4[piv[0]][0] + t * ex, P[1] = w4[piv[0]][1] + t * ey;
      P[2] = w4[piv[0]][2];  // (the edge is on the floor)
    } else {
      for (int i = 0; i < 3; i++) P[i] = w4[piv[0]][i];
    }
    // The axis: horizontal, perpendicular to the outward normal, oriented so
    // that a positive turn moves the top of the body outward (k = z x n).
    k[0] = -out_n[1], k[1] = out_n[0], k[2] = 0;
    double d = (cw[0] - P[0]) * out_n[0] + (cw[1] - P[1]) * out_n[1];  // + outside
    double hz = cw[2] - P[2];
    double L2 = d * d + hz * hz;
    w += G * d / L2 * h;
    rotate(k, w * h);
  }

  // One 20 ms tick: the joints move toward `out` (a16, canonical order; the
  // legs are out[3..10]), the body rests or tips.
  void step(const int16_t* out) {
    double p0 = pitch, r0 = roll;
    double h = 0.02 / substeps, a = 1 - std::exp(-h / servo_tau);
    for (int s = 0; s < substeps && !fallen; s++) {
      for (int j = 0; j < 8; j++)
        if (out[3 + j] != JOINT_OFF) ang[j] += (out[3 + j] / 16.0 - ang[j]) * a;
      double p, r;
      attitude(&p, &r);
      if (!tipping) {
        double pr, rr;
        // Within reach: the legs move the body at most 0.3 deg per 1 ms.
        if (rest(p, r, &pr, &rr) && std::fabs(pr - p) + std::fabs(rr - r) < 0.3)
          set_attitude(pr, rr);
        else
          start_tip(), w = 0;
      }
      if (tipping) {
        tip_step(h);
        // A foot other than the pivot's below the floor (the pivot's
        // height): it has reached the floor, an inelastic impact, the turn
        // stops (the next sub-step rests on it, or tips again from rest).
        double w4[4][3];
        feet_world(w4);
        double floor = w4[piv[0]][2];
        for (int l = 0; l < 4; l++)
          if (l != piv[0] && l != piv[1] && w4[l][2] < floor - 1e-6) tipping = false, w = 0;
      }
      attitude(&p, &r);
      if (std::fabs(p) > 80 || std::fabs(r) > 80) fallen = true;
    }
    attitude(&pitch, &roll);
    rate_pitch = (pitch - p0) * 50;
    rate_roll = (roll - r0) * 50;
    max_tilt = std::fmax(max_tilt, std::fmax(std::fabs(pitch), std::fabs(roll)));
  }

  // Start with the joints at `out` and the body at rest on them.
  void init(const int16_t* out) {
    for (int j = 0; j < 8; j++) ang[j] = out[3 + j] / 16.0;
    set_attitude(0, 0);
    double pr, rr;
    if (rest(0, 0, &pr, &rr)) set_attitude(pr, rr);
    attitude(&pitch, &roll);
    rate_pitch = rate_roll = 0;
    tipping = fallen = false;
    w = 0;
    tips = 0;
    max_tilt = 0;
  }
  void reset_max() { max_tilt = 0; }
};

#endif
