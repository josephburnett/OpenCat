// pca9685: the 16-channel servo PWM chip at I2C 0x40.
//
// The PCA9685 generates the servo pulses on its own: once a count is
// written it keeps outputting it, even while the AVR resets. That is what
// makes the warm boot possible: the new firmware can read the counts back
// and continue from the pose the robot is actually in.
#ifndef KILO_HAL_PCA9685_H
#define KILO_HAL_PCA9685_H
#include <stdint.h>

#define PCA_ADDR 0x40

// The legacy prescaler: setPWMFreq(240) with the nominal 25 MHz oscillator
// gives 24, i.e. a 244 Hz frame and ~1 us per count (Appendix C2).
//
// Why not compute it from the stored PCA9685_FREQ (the calibrated
// oscillator)? Because legacy never did: Petoi_PWMServoDriver::setup() calls
// setPWMFreq(240) BEFORE setOscillatorFrequency(stored), so the prescaler is
// always computed from 25 MHz (begin() set that) and is always 24. The stored
// oscillator only went into ANGLE2PULSE_FACTOR, which assumes prescale 24.
// Recomputing the prescaler from it (e.g. 26.5 MHz -> 26) would change the
// count period and silently break every imported calibration.
// core/joints.h pca_prescale(25000000, 240) == 24 (checked by the host tests).
#define PCA_PRESCALE 24

// Boot probe. True (= warm start) if the chip is awake, auto-incrementing,
// set to our prescaler, and at least one channel is driving a pulse; then
// counts[ch] holds each channel's OFF count (bit 12 = full-off).
bool pca_probe_warm(uint16_t counts[16]);

// Cold start: configure (sleep, prescale, totem-pole outputs, auto-
// increment) with every channel full-off, then wake. Blocks ~1 ms (boot only).
uint8_t pca_init_cold(void);

// One 66 B burst: address, LED0_ON_L, then ON_L ON_H OFF_L OFF_H for all 16
// channels with auto-increment. ON is always 0 (as legacy). The outputs all
// change together at the STOP (MODE2.OCH = 0), so a pose is never half-new.
uint8_t pca_write_all(const uint16_t counts[16]);

// Read the 16 OFF counts back (PCA_COUNTS, warm boot).
uint8_t pca_read_all(uint16_t counts[16]);

#endif
