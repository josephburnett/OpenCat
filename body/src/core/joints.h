// joints: the robot model (PROGMEM) and the angle -> PCA9685 count transfer
// function that reproduces the legacy firmware exactly.
//
// Joint ids 0..15 are the legacy OpenCat indices (doc §3b): 0 head pan,
// 1 head tilt, 2 tail, 3-7 unused 12-DoF slots, 8-11 shoulders, 12-15 knees,
// legs in the order LF, RF, RB, LB. Only 11 joints are driven; the protocol
// and most of the body use the "canonical" order of those 11 (k = 0..10).
#ifndef KILO_CORE_JOINTS_H
#define KILO_CORE_JOINTS_H
#include <stdint.h>

#define JOINT_COUNT 16   // legacy joint ids / PCA9685 channels
#define DRIVEN_COUNT 11  // canonical order [0,1,2,8,9,10,11,12,13,14,15]

// Left/right mirror in canonical order: 3..10 are the LF RF RB LB shoulders
// then knees, so the L/R partner of k is ((k - 3) ^ 1) + 3; 0-2 (pan, tilt,
// tail) map to themselves (mirroring negates pan and tail: the caller's job).
inline uint8_t joint_mirror_k(uint8_t k) { return k < 3 ? k : (uint8_t)(((k - 3) ^ 1) + 3); }

// An angle of "no output": the channel is written full-off (LIMP).
#define JOINT_OFF ((int16_t)-32768)

// PCA9685 OFF-count bit 12 means "full off" (datasheet 7.3.3). Counts carry it
// through the pipeline so a single uint16 says both where and whether.
#define PCA_FULL_OFF 0x1000u

// Per-robot calibration, imported from the legacy EEPROM (core/config).
struct JointCal {
  int16_t zero_a16[JOINT_COUNT];  // CALIBRATED_ZERO_POSITIONS x 16 (servo a16)
  int16_t a2p[JOINT_COUNT];       // ANGLE2PULSE_FACTOR, counts/deg x 1000
  uint16_t b_offset_counts;       // B_OFFSET x 10
};

// Robot-model constants (src/OpenCat.h), read from flash.
uint8_t joint_id(uint8_t k);  // canonical index -> joint id
uint8_t joint_channel(uint8_t j);
int8_t joint_dir(uint8_t j);
int8_t joint_mid_shift(uint8_t j);
int16_t joint_lo_a16(uint8_t j);
int16_t joint_hi_a16(uint8_t j);

// Legacy-exact transfer function. angle_a16 is the joint angle (OpenCat
// convention), the result a PCA9685 OFF count in 500..2500.
//   servo  = zero[j] + dir[j] * angle
//   counts = clamp(floor(a2p/1000 * servo + b), 500, 2500)
// For whole-degree angles this is bit-identical to legacy writeAngle(), which
// truncates the float result to int (test_joints.cpp checks every degree).
uint16_t joint_counts(const JointCal& cal, uint8_t j, int16_t angle_a16);

// Inverse, for the warm-boot read-back: the smallest servo angle whose count
// is `counts`, so joint_counts(joint_angle_from_counts(c)) == c for every c
// strictly inside the clamp range. Full-off counts give JOINT_OFF.
int16_t joint_angle_from_counts(const JointCal& cal, uint8_t j, uint16_t counts);

// Clamp to the joint's limits (doc §3b table). JOINT_OFF passes through.
int16_t joint_clamp(uint8_t j, int16_t angle_a16);

// Legacy setPWMFreq() prescaler: (uint8_t)(osc / (freq * 4096.0) + 0.5 - 1),
// clamped to 3..255, in integers. See hal/pca9685.h for why the firmware
// always uses the 25 MHz nominal oscillator here (prescale 24).
uint8_t pca_prescale(uint32_t osc_hz, uint16_t freq_hz);

#endif
