// Differential test: the integer transfer function against a transcription
// of the legacy float path (src/motion.h calibratedPWM() ->
// src/PCA9685servo.h writeAngle()), for every driven joint and every whole
// degree within its limits, over several plausible per-robot EEPROM sets.
#include <cstdlib>
#include "../src/core/joints.h"
#include "../src/proto/kilo_protocol.h"
#include "check.h"

// ---- legacy, transcribed. avr-gcc's double is 32-bit, so every "double"
// in the original is float here, and the host build uses -ffp-contract=off
// so no fused multiply-add changes the rounding.
static int legacy_writeAngle(int a2p, int b_offset_byte, float angle) {
  // int clipped = min(max(EEPROMReadInt(A2P + i*2) / 1000.0 * angle
  //                       + (byte)eeprom(B_OFFSET) * 10, 500), 2500);
  float v = (float)a2p / 1000.0f * angle + (float)(b_offset_byte * 10);
  float m = v > 500 ? v : 500;  // Arduino max(): float vs int -> float
  float c = m < 2500 ? m : 2500;
  return (int)c;  // assignment to int truncates
}
static int legacy_calibratedPWM(int czp, int dir, int lo, int hi, int a2p, int b, float angle) {
  angle = (float)lo > (angle < (float)hi ? angle : (float)hi) ? (float)lo : (angle < (float)hi ? angle : (float)hi);
  int duty = (int)(czp + angle * dir);  // int duty = CZP + angle * dir
  return legacy_writeAngle(a2p, b, (float)duty);
}
// -------------------------------------------------------------------------

int main() {
  // The robot model's limits are the spec's ([[joints]], by id): each row of
  // JOINTS[] names its joint's KILO_JOINT_<NAME>_LO/HI by hand, so check the
  // rows against the id-ordered table.
  static const int8_t spec_limits[JOINT_COUNT][2] = KILO_JOINT_LIMITS_INIT;
  for (int j = 0; j < JOINT_COUNT; j++)
    CHECKF(joint_lo_a16((uint8_t)j) == spec_limits[j][0] * 16 && joint_hi_a16((uint8_t)j) == spec_limits[j][1] * 16,
           "joint %d limits != spec", j);
  srand(7);
  const uint16_t oscs[] = {25000, 23100, 23500, 24700, 25300, 26500, 27400};
  const uint8_t bytes_b[] = {60, 55, 65, 40, 80};
  long compared = 0, mismatched = 0;
  for (int set = 0; set < 60; set++) {
    JointCal cal;
    uint16_t osc = oscs[set % 7];
    uint8_t bb = bytes_b[set % 5];
    for (int j = 0; j < 16; j++) {
      int calib = set == 0 ? 0 : rand() % 51 - 25;
      int czp = 90 + joint_mid_shift(j) * joint_dir(j) + calib * joint_dir(j);
      cal.zero_a16[j] = (int16_t)(czp * 16);
      // As the setup sketch computes it (see fake_eeprom.h), plus some
      // arbitrary values in the accepted 9000..13000 window.
      int a2p = set < 30 ? (int)(11.0f * (float)(osc * 1000.0 / 25000000.0) * 1000) : 9000 + rand() % 4001;
      cal.a2p[j] = (int16_t)a2p;
    }
    cal.b_offset_counts = (uint16_t)(bb * 10);
    for (int k = 0; k < DRIVEN_COUNT; k++) {
      int j = joint_id((uint8_t)k);
      int lo = joint_lo_a16((uint8_t)j) / 16, hi = joint_hi_a16((uint8_t)j) / 16;
      for (int deg = lo; deg <= hi; deg++) {
        int ref = legacy_calibratedPWM(cal.zero_a16[j] / 16, joint_dir((uint8_t)j), lo, hi, cal.a2p[j], bb, (float)deg);
        int got = joint_counts(cal, (uint8_t)j, (int16_t)(deg * 16));
        compared++;
        if (ref != got) {
          mismatched++;
          CHECKF(ref == got, "set %d joint %d deg %d: legacy %d new %d (a2p %d zero %d)", set, j, deg, ref, got,
                 cal.a2p[j], cal.zero_a16[j] / 16);
        }
      }
      // Warm-boot inverse: every count strictly inside the clamp range maps
      // back to an angle that reproduces it exactly.
      for (int c = 501; c < 2500; c++) {
        int16_t a = joint_angle_from_counts(cal, (uint8_t)j, (uint16_t)c);
        CHECKF(joint_counts(cal, (uint8_t)j, a) == c, "inverse joint %d count %d -> %d -> %d", j, c, a,
               joint_counts(cal, (uint8_t)j, a));
      }
      CHECK(joint_angle_from_counts(cal, (uint8_t)j, 0x1000 | 1500) == JOINT_OFF);
    }
  }
  // Limits and the clamp.
  CHECK(joint_clamp(10, -90 * 16) == -80 * 16);  // legacy "sit" RB shoulder
  CHECK(joint_clamp(1, 60 * 16) == 45 * 16);
  CHECK(joint_clamp(0, JOINT_OFF) == JOINT_OFF);
  // Channel map spot checks (OpenCat.h pwm_pin).
  CHECK(joint_channel(0) == 12 && joint_channel(15) == 0 && joint_channel(8) == 14);
  // The legacy prescaler: 25 MHz nominal -> 24; a calibrated 26.5 MHz
  // oscillator would give 26 (why hal/pca9685 does NOT use the stored value).
  CHECK(pca_prescale(25000000, 240) == 24);
  CHECK(pca_prescale(26500000, 240) == 26);
  CHECK(pca_prescale(0, 1000) == 3);
  std::printf("joints: %ld legacy comparisons, %ld mismatches\n", compared, mismatched);
  return report("joints");
}
