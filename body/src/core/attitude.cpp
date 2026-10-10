#include "attitude.h"
#include "fx.h"

static int16_t be16(const uint8_t* p) { return (int16_t)(((uint16_t)p[0] << 8) | p[1]); }

void attitude_quat_q14(const uint8_t* pkt, int16_t q[4]) {
  for (uint8_t i = 0; i < 4; i++) q[i] = be16(pkt + 4 * i);
}

// Q14 x Q14 -> Q28. Written as int16 x int16 so avr-gcc uses its 16x16->32
// multiply (__mulhisi3) instead of a full 32x32 one.
static inline int32_t mul(int16_t a, int16_t b) { return (int32_t)a * b; }

// Q28 -> Q14 with rounding. Right-shifting a negative int32 is arithmetic on
// both avr-gcc and g++ (implementation-defined before C++20, relied on here).
static inline int16_t q14(int32_t v) { return (int16_t)((v + 8192) >> 14); }

bool attitude_from_quat(const int16_t q[4], Attitude& out) {
  int16_t w = q[0], x = q[1], y = q[2], z = q[3];
  // Reject anything that is not close to a unit quaternion (a misaligned
  // FIFO read gives random bytes). Since |q|^2 <= 1.2, every product below
  // is <= 1.2 * 2^28 and every gravity component <= 1.2: nothing overflows.
  int32_t n = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (q[i] > 16500 || q[i] < -16500) return false;
    n += mul(q[i], q[i]);
  }
  if (n < (int32_t)214748365 || n > (int32_t)322122547) return false;  // 0.8 .. 1.2 in Q28
  // Each sum is shifted back to Q14 BEFORE squaring (doc §3f), so the
  // squares below are Q28 again and their sum (<= 2^29) still fits int32.
  int16_t gx = q14(2 * (mul(x, z) - mul(w, y)));
  int16_t gy = q14(2 * (mul(w, x) + mul(y, z)));
  int16_t gz = q14(mul(w, w) - mul(x, x) - mul(y, y) + mul(z, z));
  int16_t pitch = fx_atan2(gx, fx_isqrt32((uint32_t)(mul(gy, gy) + mul(gz, gz))));
  int16_t roll = fx_atan2(gy, gz);
  // Legacy reflection (V6_12 dmpGetYawPitchRoll, active branch): when the
  // body is upside down, report pitch as if measured past the vertical.
  if (gz < 0) pitch = (int16_t)(pitch > 0 ? A16_180 - pitch : -A16_180 - pitch);
  out.pitch = pitch;
  out.roll = roll;
  out.gz = gz;
  return true;
}

// deg/s per LSB at +-2000 deg/s full scale is 1/16.4, so a16/s per LSB is
// 16/16.4 = 40/41 ~= 999/1024 (2.4e-5 relative error; a shift beats a divide).
static int16_t gyro_to_a16s(int16_t raw) { return (int16_t)(((int32_t)raw * 999) >> 10); }

bool attitude_from_packet(const uint8_t* pkt, Attitude& out) {
  int16_t q[4];
  attitude_quat_q14(pkt, q);
  if (!attitude_from_quat(q, out)) return false;
  // Body rates. Signs follow from the gravity formulas above (estimated, to
  // be confirmed on Kilo in P1): a positive rotation about the sensor's x
  // axis raises gy, so roll rate = +gx_gyro; a positive rotation about y
  // lowers gx, so pitch rate = -gy_gyro.
  out.rate_roll = gyro_to_a16s(be16(pkt + 22));
  out.rate_pitch = (int16_t)-gyro_to_a16s(be16(pkt + 24));
  out.rate_yaw = gyro_to_a16s(be16(pkt + 26));
  return true;
}
