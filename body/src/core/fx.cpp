#include "fx.h"
#include "pgm.h"

// sin(d degrees) in Q14 for d = 0..90. One entry per whole degree so the
// index is just a16 >> 4 and the interpolation fraction a16 & 15: no
// division. (The design note sketched a 65-entry table; that needs a divide
// by 22.5 a16 per lookup. 26 more entries buy the divide back.)
// Linear interpolation over 1 degree steps: error <= (pi/180)^2 / 8 = 4e-5.
static const int16_t SIN_Q14[91] PROGMEM = {
    0,     286,   572,   857,   1143,  1428,  1713,  1997,  2280,  2563,
    2845,  3126,  3406,  3686,  3964,  4240,  4516,  4790,  5063,  5334,
    5604,  5872,  6138,  6402,  6664,  6924,  7182,  7438,  7692,  7943,
    8192,  8438,  8682,  8923,  9162,  9397,  9630,  9860,  10087, 10311,
    10531, 10749, 10963, 11174, 11381, 11585, 11786, 11982, 12176, 12365,
    12551, 12733, 12911, 13085, 13255, 13421, 13583, 13741, 13894, 14044,
    14189, 14330, 14466, 14598, 14726, 14849, 14968, 15082, 15191, 15296,
    15396, 15491, 15582, 15668, 15749, 15826, 15897, 15964, 16026, 16083,
    16135, 16182, 16225, 16262, 16294, 16322, 16344, 16362, 16374, 16382,
    16384};

// atan(i/64) for i = 0..64, in 1/256 degree (so 45 deg = 11520). Storing
// finer than a16 keeps table rounding (1/512 deg) well below the a16 output
// step; the interpolation error over 1/64 steps is about 0.001 deg.
static const uint16_t ATAN_Q8DEG[65] PROGMEM = {
    0,     229,   458,   687,   916,   1144,  1371,  1598,  1824,  2049,
    2273,  2497,  2719,  2939,  3159,  3377,  3593,  3808,  4021,  4233,
    4443,  4650,  4856,  5060,  5262,  5462,  5660,  5856,  6049,  6240,
    6429,  6616,  6801,  6983,  7163,  7340,  7516,  7689,  7859,  8027,
    8193,  8357,  8518,  8677,  8834,  8989,  9141,  9291,  9439,  9584,
    9728,  9869,  10008, 10145, 10280, 10413, 10544, 10672, 10799, 10924,
    11047, 11168, 11287, 11405, 11520};

// sin of an angle already reduced to the first quadrant, 0..1440 inclusive.
static int16_t sin_q1(uint16_t x) {
  uint8_t i = (uint8_t)(x >> 4);
  uint8_t f = (uint8_t)(x & 15);
  int16_t a = PGM_I16(&SIN_Q14[i]);
  if (f == 0) return a;  // also covers x = 1440, where i + 1 would overrun
  int16_t b = PGM_I16(&SIN_Q14[i + 1]);
  return (int16_t)(a + (((int16_t)(b - a) * (int16_t)f + 8) >> 4));
}

KILO_NOINLINE int16_t fx_sin(int16_t a16) {
  // Reduce to [0, 5760). A loop beats '%' on AVR (no 16-bit divide), and
  // int16 a16 spans at most +-2048 deg, so it runs at most 6 times.
  int16_t r = a16;
  while (r < 0) r = (int16_t)(r + A16_360);
  while (r >= A16_360) r = (int16_t)(r - A16_360);
  // Fold the circle onto the first quadrant using sin's symmetries.
  bool neg = false;
  if (r >= A16_180) {  // sin(180 + t) = -sin(t)
    r = (int16_t)(r - A16_180);
    neg = true;
  }
  if (r > A16_90) r = (int16_t)(A16_180 - r);  // sin(180 - t) = sin(t)
  int16_t s = sin_q1((uint16_t)r);
  return neg ? (int16_t)-s : s;
}

int16_t fx_cos(int16_t a16) {
  // cos(t) = sin(t + 90). Widen first so +90 deg cannot overflow int16.
  int32_t t = (int32_t)a16 + A16_90;
  if (t >= 32768) t -= A16_360;
  return fx_sin((int16_t)t);
}

int16_t fx_atan2(int32_t y, int32_t x) {
  if (x == 0 && y == 0) return 0;
  // Absolute values as unsigned so that -2^31 is representable.
  uint32_t ax = x < 0 ? (uint32_t)0 - (uint32_t)x : (uint32_t)x;
  uint32_t ay = y < 0 ? (uint32_t)0 - (uint32_t)y : (uint32_t)y;
  // Shared shift: the ratio is unchanged, and afterwards both fit 15 bits,
  // so the rest is 16-bit arithmetic (min << 14 < 2^29 for the one divide).
  while ((ax | ay) >= 32768u) {
    ax >>= 1;
    ay >>= 1;
  }
  // First octant: r = min/max in Q14, 0..16384.
  bool swap = ay > ax;
  uint16_t mn = (uint16_t)(swap ? ax : ay);
  uint16_t mx = (uint16_t)(swap ? ay : ax);
  uint16_t r = (uint16_t)((((uint32_t)mn << 14) + (mx >> 1)) / mx);  // rounded
  uint8_t i = (uint8_t)(r >> 8);  // 0..64
  uint8_t f = (uint8_t)r;
  // Angle in 1/256 deg; 180 deg = 46080 still fits uint16.
  uint16_t a = PGM_U16(&ATAN_Q8DEG[i]);
  if (f) a = (uint16_t)(a + (((uint16_t)(PGM_U16(&ATAN_Q8DEG[i + 1]) - a) * f + 128) >> 8));
  // Rebuild the full angle: octant, then quadrant, then sign.
  if (swap) a = (uint16_t)(23040u - a);  // 90 deg - a
  if (x < 0) a = (uint16_t)(46080u - a);  // 180 deg - a
  int16_t out = (int16_t)((a + 8) >> 4);  // 1/256 deg -> a16, rounded
  return y < 0 ? (int16_t)-out : out;
}

uint16_t fx_isqrt32(uint32_t n) {
  // Digit-by-digit square root in base 4: each step decides one result bit.
  uint32_t res = 0;
  uint32_t one = 1UL << 30;
  while (one > n) one >>= 2;
  while (one) {
    if (n >= res + one) {
      n -= res + one;
      res = (res >> 1) + one;
    } else {
      res >>= 1;
    }
    one >>= 2;
  }
  return (uint16_t)res;
}

uint16_t fx_ease_cos(uint8_t i, uint8_t n) {
  if (i >= n) return Q14_ONE;
  int16_t s = fx_sin((int16_t)(((uint32_t)A16_90 * i + n / 2) / n));  // rounded
  return (uint16_t)(((uint32_t)((int32_t)s * s) + 8192) >> 14);
}
