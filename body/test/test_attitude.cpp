// Integer attitude vs the legacy float math, in double: V6_12
// dmpGetQuaternion(Quaternion*) (int16 / 16384), dmpGetGravity(VectorFloat*),
// and the active dmpGetYawPitchRoll() branch with the gz < 0 reflection.
#include <cmath>
#include <cstdlib>
#include <random>
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
  std::printf("attitude: %d random unit quaternions, max |pitch err| %.4f deg, max |roll err| %.4f deg\n"
              "          (%d on the gz=0 reflection seam and %d roll values within 3 deg of pitch +-90 not compared)\n",
              N, max_p, max_r, skipped, skipped_roll);
  return report("attitude");
}
