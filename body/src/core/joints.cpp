#include "joints.h"
#include "pgm.h"

// One row per legacy joint id, straight from src/OpenCat.h (pwm_pin[],
// rotationDirection[], middleShift[], angleLimit[]). Limits are whole degrees
// and fit int8. Rows 3-7 are the unused 12-DoF slots: kept so the legacy
// EEPROM arrays can be cross-checked in full, never driven.
//
// Exception: the tail (row 2) is dir +1, the Nybble value up to
// N_240702. Upstream commit b152455 ("support robotic arm", N_240907) flipped
// the shared rotationDirection[2] to -1 for Bittle's arm clip, and Nybble
// inherited it. Kilo's EEPROM (test/test_kilo_eeprom.cpp) says +1, and the
// legacy firmware drives with the EEPROM value, so +1 is what the hardware
// was calibrated and run with.
struct JointRow {
  uint8_t ch;
  int8_t dir;
  int8_t mid;
  int8_t lo;
  int8_t hi;
};

static const JointRow JOINTS[JOINT_COUNT] PROGMEM = {
    {12, 1, 0, -120, 120},    // 0 head pan
    {11, -1, 15, -80, 45},    // 1 head tilt
    {4, 1, 0, -120, 120},     // 2 tail (+1: see note above)
    {3, 1, 0, -120, 120},     // 3 unused
    {13, 1, -45, -90, 60},    // 4 unused
    {10, -1, -45, -90, 60},   // 5 unused
    {5, 1, -45, -90, 90},     // 6 unused
    {2, -1, -45, -90, 90},    // 7 unused
    {14, 1, 10, -100, 80},    // 8 LF shoulder
    {9, -1, 10, -100, 80},    // 9 RF shoulder
    {6, -1, -10, -80, 100},   // 10 RB shoulder
    {1, 1, -10, -80, 100},    // 11 LB shoulder
    {15, -1, -30, -60, 120},  // 12 LF knee
    {8, 1, -30, -60, 120},    // 13 RF knee
    {7, 1, 30, -120, 60},     // 14 RB knee
    {0, -1, 30, -120, 60},    // 15 LB knee
};

static const uint8_t CANONICAL[DRIVEN_COUNT] PROGMEM = {0,  1,  2,  8,  9, 10,
                                                        11, 12, 13, 14, 15};

uint8_t joint_id(uint8_t k) { return PGM_U8(&CANONICAL[k]); }
uint8_t joint_channel(uint8_t j) { return PGM_U8(&JOINTS[j].ch); }
int8_t joint_dir(uint8_t j) { return PGM_I8(&JOINTS[j].dir); }
int8_t joint_mid_shift(uint8_t j) { return PGM_I8(&JOINTS[j].mid); }
int16_t joint_lo_a16(uint8_t j) { return (int16_t)(PGM_I8(&JOINTS[j].lo) * 16); }
int16_t joint_hi_a16(uint8_t j) { return (int16_t)(PGM_I8(&JOINTS[j].hi) * 16); }

// The legacy formula works in degrees with a float slope a2p/1000. Here the
// servo angle is a16 (x16) and the slope stays x1000, so everything is
// scaled by 16000 and one integer division does the rest:
//   counts = floor((a2p * servo_a16 + b * 16000) / 16000)
// floor, not round: legacy assigns the float to an int, which truncates, and
// the value is positive wherever it is not clamped to 500 anyway.
#define SCALE ((int32_t)16000)

KILO_NOINLINE uint16_t joint_counts(const JointCal& cal, uint8_t j, int16_t angle_a16) {
  // The servo angle fits int16 for any configuration that passed
  // config_slot_valid() (|zero| <= 360 deg) and any clamped joint angle.
  int16_t servo = (int16_t)(cal.zero_a16[j] + joint_dir(j) * angle_a16);
  // int16 x int16 -> int32: avr-gcc's cheap 16x16 multiply.
  int32_t num = (int32_t)cal.a2p[j] * servo + (int32_t)cal.b_offset_counts * SCALE;
  if (num < 500 * SCALE) return 500;
  int32_t c = num / SCALE;
  return c > 2500 ? 2500 : (uint16_t)c;
}

int16_t joint_angle_from_counts(const JointCal& cal, uint8_t j, uint16_t counts) {
  if (counts & PCA_FULL_OFF) return JOINT_OFF;
  // The count rises monotonically with the servo angle (zero + dir*angle),
  // so bisect on the servo angle for the smallest one that reaches `counts`
  // using the forward function itself. Boot-time only, so 16 forward
  // evaluations are fine, and the round trip is exact by construction.
  int8_t dir = joint_dir(j);
  int16_t lo = -5760, hi = 5760;  // servo a16: +-360 deg, the valid range
  while (lo < hi) {
    int16_t mid = (int16_t)(lo + ((hi - lo) >> 1));
    // joint_counts takes a joint angle: servo = zero + dir * angle, so
    // angle = dir * (servo - zero) since dir is +-1.
    if (joint_counts(cal, j, (int16_t)(dir * (mid - cal.zero_a16[j]))) >= counts) hi = mid;
    else lo = (int16_t)(mid + 1);
  }
  return (int16_t)(dir * (lo - cal.zero_a16[j]));
}

int16_t joint_clamp(uint8_t j, int16_t a) {
  if (a == JOINT_OFF) return a;
  int16_t lo = joint_lo_a16(j), hi = joint_hi_a16(j);
  return a < lo ? lo : (a > hi ? hi : a);
}

uint8_t pca_prescale(uint32_t osc_hz, uint16_t freq_hz) {
  // floor(x + 0.5) - 1 with x = osc / (freq * 4096)
  uint32_t div = (uint32_t)freq_hz * 4096;
  uint32_t p = (osc_hz + div / 2) / div;
  p = p ? p - 1 : 0;
  return (uint8_t)(p < 3 ? 3 : (p > 255 ? 255 : p));
}
