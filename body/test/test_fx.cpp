// fx vs libm (Appendix H "fx").
#include <cmath>
#include <cstdlib>
#include "../src/core/fx.h"
#include "check.h"

static double a16_to_rad(double a) { return a / 16.0 * M_PI / 180.0; }

// Angular difference in degrees, wrapped to [-180, 180).
static double ang_diff_deg(double a, double b) {
  double d = std::fmod(a - b + 540.0, 360.0) - 180.0;
  return d;
}

static double max_atan_err = 0;

static void atan_case(int32_t y, int32_t x) {
  double ref = std::atan2((double)y, (double)x) * 180.0 / M_PI;
  double got = fx_atan2(y, x) / 16.0;
  double e = std::fabs(ang_diff_deg(got, ref));
  if (x == 0 && y == 0) e = std::fabs(got);
  if (e > max_atan_err) max_atan_err = e;
  // The output is a16, so half an LSB (1/32 = 0.031 deg) is the floor.
  // Ratio rounding (~0.002), table rounding (0.002) and interpolation
  // (0.001) add the rest. (Appendix H asks for 0.02 deg, which an a16
  // result cannot meet.)
  CHECKF(e <= 0.04, "atan2(%d, %d) = %.4f, libm %.4f", y, x, got, ref);
}

int main() {
  // sin/cos over every a16 value (+-2048 deg).
  double max_sin = 0, max_cos = 0;
  for (int32_t a = -32768; a <= 32767; a++) {
    double s = fx_sin((int16_t)a) / 16384.0, c = fx_cos((int16_t)a) / 16384.0;
    double es = std::fabs(s - std::sin(a16_to_rad(a))), ec = std::fabs(c - std::cos(a16_to_rad(a)));
    if (es > max_sin) max_sin = es;
    if (ec > max_cos) max_cos = ec;
  }
  CHECKF(max_sin <= 2e-4, "max sin error %g", max_sin);
  CHECKF(max_cos <= 2e-4, "max cos error %g", max_cos);
  CHECK(fx_sin(0) == 0 && fx_sin(1440) == 16384 && fx_sin(2880) == 0 && fx_sin(-1440) == -16384);

  // atan2 over a 1001 x 1001 grid including the axes and zero.
  for (int32_t y = -500; y <= 500; y++)
    for (int32_t x = -500; x <= 500; x++) atan_case(y, x);
  // Large magnitudes (the shared-shift normalisation) and extremes.
  srand(1);
  for (int i = 0; i < 200000; i++) {
    int32_t y = (int32_t)(((uint32_t)rand() << 16) ^ (uint32_t)rand());
    int32_t x = (int32_t)(((uint32_t)rand() << 16) ^ (uint32_t)rand());
    int sh = rand() % 31;
    atan_case(y >> sh, x >> (rand() % 31));
  }
  atan_case(INT32_MAX, INT32_MAX);
  atan_case(INT32_MIN, INT32_MIN);
  atan_case(INT32_MIN, 1);
  atan_case(1, INT32_MIN);
  atan_case(0, INT32_MIN);
  atan_case(INT32_MAX, 0);
  CHECK(fx_atan2(0, -5) == 2880);  // +180, like std::atan2(+0, -x)
  CHECK(fx_atan2(0, 0) == 0);

  // isqrt: exact on edges and samples.
  for (uint32_t n = 0; n < 200000; n++) {
    uint32_t r = fx_isqrt32(n);
    CHECK((uint64_t)r * r <= n && (uint64_t)(r + 1) * (r + 1) > n);
  }
  for (int i = 0; i < 200000; i++) {
    uint32_t n = ((uint32_t)rand() << 16) ^ (uint32_t)rand() ^ ((uint32_t)rand() << 31);
    uint32_t r = fx_isqrt32(n);
    CHECKF((uint64_t)r * r <= n && (uint64_t)(r + 1) * (r + 1) > n, "isqrt(%u) = %u", n, r);
  }
  CHECK(fx_isqrt32(0xFFFFFFFFu) == 65535);
  CHECK(fx_isqrt32(65535u * 65535u) == 65535);
  CHECK(fx_isqrt32(65535u * 65535u - 1) == 65534);

  // Cosine ease: endpoints, monotonic, symmetric.
  for (int n = 1; n <= 255; n++) {
    CHECK(fx_ease_cos(0, (uint8_t)n) == 0);
    CHECK(fx_ease_cos((uint8_t)n, (uint8_t)n) == 16384);
    uint16_t prev = 0;
    for (int i = 0; i <= n; i++) {
      uint16_t e = fx_ease_cos((uint8_t)i, (uint8_t)n);
      CHECK(e >= prev);
      prev = e;
      double ref = (1 - std::cos(M_PI * i / n)) / 2 * 16384;
      CHECKF(std::fabs(e - ref) <= 12, "ease(%d,%d)=%u ref %.1f", i, n, e, ref);
    }
  }
  std::printf("fx: max |sin err| %.2e, max |cos err| %.2e, max |atan2 err| %.4f deg\n", max_sin, max_cos,
              max_atan_err);
  return report("fx");
}
