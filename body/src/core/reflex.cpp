#include "reflex.h"
#include "fx.h"
#include "pgm.h"

// -lim < v < lim (one shared copy: cheaper on the AVR than inlined abs()).
KILO_NOINLINE static bool under(int16_t v, int16_t lim) { return v < lim && v > -lim; }

int8_t fall_detect(uint8_t& over, const Attitude& a) {
  const int16_t lim = FALL_DEG * A16_PER_DEG + 1;  // over = |angle| > 75 deg
  if (under(a.roll, lim) && under(a.pitch, lim)) {
    over = 0;
    return 0;
  }
  if (++over < FALL_SAMPLES) return 0;
  over = 0;
  // Pitch fall: 75 < |pitch| < 105 deg (|pitch| folded about 90 deg).
  if (!under(a.pitch, lim) && under(a.pitch, (int16_t)(A16_180 - lim + 1))) return a.pitch < 0 ? -2 : 2;
  return a.roll < 0 ? -1 : 1;
}

bool fall_level(const Attitude& a) {
  const int16_t lim = FALL_LEVEL_DEG * A16_PER_DEG;
  return under(a.roll, lim) && under(a.pitch, lim);
}

bool fall_still(const Attitude& a) {
  const int16_t lim = FALL_STILL_DPS * A16_PER_DEG;
  return under(a.rate_roll, lim) && under(a.rate_pitch, lim) && under(a.rate_yaw, lim);
}
