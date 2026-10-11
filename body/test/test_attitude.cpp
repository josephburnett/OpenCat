// Integer attitude vs the legacy float math, in double: V6_12
// dmpGetQuaternion(Quaternion*) (int16 / 16384), dmpGetGravity(VectorFloat*),
// and the active dmpGetYawPitchRoll() branch with the gz < 0 reflection.
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include "../src/core/attitude.h"
#include "check.h"

static void legacy_ypr(const int16_t qi[4], double* pitch, double* roll, double* gz_out, double* horiz) {
  double w = qi[0] / 16384.0, x = qi[1] / 16384.0, y = qi[2] / 16384.0, z = qi[3] / 16384.0;
  double gx = 2 * (x * z - w * y);
  double gy = 2 * (w * x + y * z);
  double gz = w * w - x * x - y * y + z * z;
  double p = std::atan2(gx, std::sqrt(gy * gy + gz * gz));
  double r = std::atan2(gy, gz);
  if (gz < 0) p = p > 0 ? M_PI - p : -M_PI - p;
  *pitch = p * 180 / M_PI;
  *roll = r * 180 / M_PI;
  *gz_out = gz;
  *horiz = std::sqrt(gy * gy + gz * gz);
}

// The legacy path exactly as the AVR ran it, in float32: imu.h read_IMU()
// -> dmpGetQuaternion(Quaternion*) (int16 / 16384.0f) -> dmpGetGravity(
// VectorFloat*) -> dmpGetYawPitchRoll() (active V6_12 branch) -> ypr *=
// degPerRad (float 180/M_PI). No axis swap or sign flip follows anywhere:
// skill.h uses ypr[1] as pitch and ypr[2] as roll (RollPitchDeviation[i] =
// ypr[2 - i] - expectedRollPitch[i]).
static void legacy_ypr_float(const int16_t qi[4], float* pitch, float* roll) {
  float w = (float)qi[0] / 16384.0f, x = (float)qi[1] / 16384.0f, y = (float)qi[2] / 16384.0f,
        z = (float)qi[3] / 16384.0f;
  float gx = 2 * (x * z - w * y);
  float gy = 2 * (w * x + y * z);
  float gz = w * w - x * x - y * y + z * z;
  float p = atan2f(gx, sqrtf(gy * gy + gz * gz));
  float r = atan2f(gy, gz);
  if (gz < 0) p = p > 0 ? (float)M_PI - p : (float)-M_PI - p;
  const float degPerRad = (float)(180.0 / M_PI);
  *pitch = p * degPerRad;
  *roll = r * degPerRad;
}

static std::string slurp(const char* rel) {
  std::ifstream f(std::string(REPO_ROOT) + "/" + rel);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

static double wrap(double d) { return std::fmod(d + 540.0, 360.0) - 180.0; }

static void put_be32(uint8_t* p, int32_t v) {
  p[0] = (uint8_t)((uint32_t)v >> 24);
  p[1] = (uint8_t)((uint32_t)v >> 16);
  p[2] = (uint8_t)((uint32_t)v >> 8);
  p[3] = (uint8_t)v;
}

int main() {
  std::mt19937 rng(42);
  std::normal_distribution<double> nd(0, 1);
  double max_p = 0, max_r = 0;
  int skipped = 0, skipped_roll = 0;
  const int N = 200000;
  for (int i = 0; i < N; i++) {
    double q[4], n = 0;
    for (double& v : q) {
      v = nd(rng);
      n += v * v;
    }
    n = std::sqrt(n);
    // The DMP sends Q30; the firmware (like legacy) uses the top 16 bits.
    uint8_t pkt[DMP_PACKET_SIZE] = {0};
    int16_t qi[4];
    for (int k = 0; k < 4; k++) {
      int32_t q30 = (int32_t)std::lround(q[k] / n * 1073741823.0);
      put_be32(pkt + 4 * k, q30);
      qi[k] = (int16_t)(q30 >> 16);
    }
    Attitude a;
    CHECK(attitude_from_packet(pkt, a));
    double rp, rr, gz, horiz;
    legacy_ypr(qi, &rp, &rr, &gz, &horiz);
    // Within ~1e-4 of the reflection seam (gz = 0) the two quantisations can
    // legitimately land on different sides; legacy float has the same knife
    // edge. Count those instead of comparing them.
    if (std::fabs(gz) < 2e-4) {
      skipped++;
      continue;
    }
    double ep = std::fabs(wrap(a.pitch / 16.0 - rp)), er = std::fabs(wrap(a.roll / 16.0 - rr));
    // Roll = atan2(gy, gz) is undefined at pitch +-90 deg (gy = gz = 0), and
    // within ~3 deg of it the Q14 inputs cannot resolve 0.1 deg of roll
    // (nor can legacy float vs this double reference). Count, don't compare.
    if (horiz < 0.05) {
      skipped_roll++;
      er = 0;
    }
    if (ep > max_p) max_p = ep;
    if (er > max_r) max_r = er;
    CHECKF(ep <= 0.1 && er <= 0.1, "q=(%d,%d,%d,%d) pitch %.3f vs %.3f, roll %.3f vs %.3f", qi[0], qi[1], qi[2],
           qi[3], a.pitch / 16.0, rp, a.roll / 16.0, rr);
  }
  // Exact poses.
  int16_t up[4] = {16384, 0, 0, 0};
  Attitude a;
  CHECK(attitude_from_quat(up, a) && a.pitch == 0 && a.roll == 0 && a.gz == 16384);
  // Rotation of +30 deg about y: gx = -sin 30 -> pitch -30 deg.
  int16_t ry[4] = {(int16_t)std::lround(16384 * std::cos(M_PI / 12)), 0,
                   (int16_t)std::lround(16384 * std::sin(M_PI / 12)), 0};
  CHECK(attitude_from_quat(ry, a) && std::abs(a.pitch - (-30 * 16)) <= 1 && a.roll == 0);
  // Upside down, nose slightly up: the legacy reflection (pitch near 180).
  int16_t inv[4] = {0, 16384, 0, 0};
  CHECK(attitude_from_quat(inv, a) && std::abs(std::abs(a.roll) - 2880) <= 1);
  // Non-unit quaternions (misaligned FIFO bytes) are rejected.
  int16_t bad1[4] = {30000, 0, 0, 0}, bad2[4] = {8000, 0, 0, 0}, bad3[4] = {16384, 16384, 0, 0};
  CHECK(!attitude_from_quat(bad1, a) && !attitude_from_quat(bad2, a) && !attitude_from_quat(bad3, a));
  // Gyro: bytes 22..27 big-endian, +-2000 deg/s; 16.4 LSB per deg/s.
  uint8_t pkt[DMP_PACKET_SIZE] = {0x40, 0, 0, 0};
  pkt[22] = 0x01;
  pkt[23] = 0x48;  // +328 LSB about x = +20 deg/s -> roll rate +320 a16/s
  pkt[24] = 0xFE;
  pkt[25] = 0xB8;  // -328 about y -> pitch rate +20 deg/s (pitch = -rot_y)
  pkt[26] = 0x00;
  pkt[27] = 0xA4;  // +164 about z -> +10 deg/s
  CHECK(attitude_from_packet(pkt, a));
  CHECKF(std::abs(a.rate_roll - 320) <= 1 && std::abs(a.rate_pitch - 320) <= 1 && std::abs(a.rate_yaw - 160) <= 1,
         "rates %d %d %d", a.rate_roll, a.rate_pitch, a.rate_yaw);
  // ---- P1 A/B vs the legacy ypr (doc §7: within 1 deg static). Random
  // orientations with legacy |pitch|, |roll| <= 60 deg (any yaw), float32
  // legacy maths against the integer firmware.
  {
    std::string imu = slurp("src/imu.h");
    CHECK(imu.find("mpu.dmpGetGravity(&gravity, &q);\n    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);") !=
          std::string::npos);
    CHECK(imu.find("ypr[i] *= degPerRad;") != std::string::npos);
    CHECK(imu.find("ypr[1] =") == std::string::npos && imu.find("ypr[2] =") == std::string::npos);
    std::string skill = slurp("src/skill.h");
    CHECK(skill.find("RollPitchDeviation[i] = ypr[2 - i] - expectedRollPitch[i];") != std::string::npos);
    int n = 0, tries = 0;
    double ab_p = 0, ab_r = 0;
    while (n < 5000 && tries < 200000) {
      tries++;
      double q[4], nn = 0;
      for (double& v : q) {
        v = nd(rng);
        nn += v * v;
      }
      nn = std::sqrt(nn);
      uint8_t pk[DMP_PACKET_SIZE] = {0};
      int16_t qi[4];
      for (int k = 0; k < 4; k++) {
        int32_t q30 = (int32_t)std::lround(q[k] / nn * 1073741823.0);
        put_be32(pk + 4 * k, q30);
        qi[k] = (int16_t)(q30 >> 16);
      }
      float lp, lr;
      legacy_ypr_float(qi, &lp, &lr);
      if (std::fabs(lp) > 60 || std::fabs(lr) > 60) continue;
      n++;
      Attitude a2;
      CHECK(attitude_from_packet(pk, a2));
      double ep = std::fabs(a2.pitch / 16.0 - lp), er = std::fabs(a2.roll / 16.0 - lr);
      ab_p = std::fmax(ab_p, ep);
      ab_r = std::fmax(ab_r, er);
      CHECKF(ep <= 1.0 && er <= 1.0, "A/B: pitch %.3f vs legacy %.3f, roll %.3f vs %.3f", a2.pitch / 16.0, lp,
             a2.roll / 16.0, lr);
    }
    CHECK(n == 5000);
    std::printf("attitude: A/B vs legacy float ypr, %d orientations within +-60 deg: max |pitch err| %.4f deg, "
                "max |roll err| %.4f deg\n", n, ab_p, ab_r);
  }
  // ---- Rate signs: the gyro-derived rates must be d(pitch)/dt and
  // d(roll)/dt of the pitch/roll above. The DMP quaternion maps sensor to
  // world (its gravity row is what dmpGetGravity reads), so a body-frame
  // gyro w moves it as q' = q * (1, w dt / 2). Rotate random tilted
  // orientations (|pitch|, |roll| <= 20 deg) by 1 deg about sensor x, then
  // y, and compare the change of the (double) legacy pitch/roll with the
  // firmware's rate for the same gyro reading.
  {
    std::uniform_real_distribution<double> ud(-1, 1);
    int checked = 0;
    double worst = 0;
    for (int i = 0; i < 4000 && checked < 2000; i++) {
      double yaw = ud(rng) * M_PI, p0 = ud(rng) * 20 * M_PI / 180, r0 = ud(rng) * 20 * M_PI / 180;
      // Any orientation; keep the ones the legacy ypr calls <= 20 deg.
      double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2), cp = std::cos(p0 / 2), sp = std::sin(p0 / 2),
             cr = std::cos(r0 / 2), sr = std::sin(r0 / 2);
      double q[4] = {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy};
      for (int axis = 0; axis < 2; axis++) {
        const double th = 1.0 * M_PI / 180;  // 1 deg in one 20 ms tick = 50 deg/s
        double h[4] = {std::cos(th / 2), 0, 0, 0};
        h[1 + axis] = std::sin(th / 2);
        double q2[4] = {q[0] * h[0] - q[1] * h[1] - q[2] * h[2] - q[3] * h[3],
                        q[0] * h[1] + q[1] * h[0] + q[2] * h[3] - q[3] * h[2],
                        q[0] * h[2] - q[1] * h[3] + q[2] * h[0] + q[3] * h[1],
                        q[0] * h[3] + q[1] * h[2] - q[2] * h[1] + q[3] * h[0]};
        int16_t qa[4], qb[4];
        for (int k = 0; k < 4; k++) {
          qa[k] = (int16_t)std::lround(q[k] * 16384);
          qb[k] = (int16_t)std::lround(q2[k] * 16384);
        }
        double pa, ra, pb, rb, g, hz;
        legacy_ypr(qa, &pa, &ra, &g, &hz);
        legacy_ypr(qb, &pb, &rb, &g, &hz);
        if (std::fabs(pa) > 20 || std::fabs(ra) > 20) continue;
        // The same rotation as a gyro reading: 50 deg/s * 16.4 LSB = 820.
        uint8_t pk[DMP_PACKET_SIZE] = {0x40, 0, 0, 0};
        int16_t raw = 820;
        pk[22 + 2 * axis] = (uint8_t)((uint16_t)raw >> 8);
        pk[23 + 2 * axis] = (uint8_t)raw;
        Attitude ra2;
        CHECK(attitude_from_packet(pk, ra2));
        double fw = (axis == 0 ? ra2.rate_roll : ra2.rate_pitch) / 16.0;  // deg/s
        double truth = (axis == 0 ? rb - ra : pb - pa) / 0.02;
        double other = (axis == 0 ? pb - pa : rb - ra) / 0.02;  // cross-coupling at tilt
        worst = std::fmax(worst, std::fabs(fw - truth) / 50.0);
        CHECKF(fw * truth > 0 && std::fabs(fw - truth) <= 0.1 * 50, "axis %d: fw %.2f vs d/dt %.2f deg/s", axis, fw,
               truth);
        CHECK(std::fabs(other) <= 0.4 * 50);
        checked++;
      }
    }
    CHECK(checked >= 1000);
    std::printf("attitude: rate signs: %d gyro steps within 20 deg of level, rate_roll = +gyro_x and rate_pitch = "
                "-gyro_y agree with d/dt (max err %.1f%%)\n", checked, worst * 100);
  }
  std::printf("attitude: %d random unit quaternions, max |pitch err| %.4f deg, max |roll err| %.4f deg\n"
              "          (%d on the gz=0 reflection seam and %d roll values within 3 deg of pitch +-90 not compared)\n",
              N, max_p, max_r, skipped, skipped_roll);
  return report("attitude");
}
