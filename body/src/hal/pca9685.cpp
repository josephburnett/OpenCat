#include "pca9685.h"
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "../core/joints.h"
#include "i2c.h"

// Registers and bits (PCA9685 datasheet; same names as the Adafruit driver).
#define MODE1 0x00
#define MODE2 0x01
#define LED0_ON_L 0x06
#define ALL_LED_ON_L 0xFA
#define PRESCALE 0xFE
#define MODE1_RESTART 0x80
#define MODE1_AI 0x20
#define MODE1_SLEEP 0x10
#define MODE2_OUTDRV 0x04

bool pca_probe_warm(uint16_t counts[16]) {
  uint8_t m1, pre;
  if (i2c_read(PCA_ADDR, MODE1, &m1, 1) || i2c_read(PCA_ADDR, PRESCALE, &pre, 1)) return false;
  // Power-on state is SLEEP set and prescale 30: that is a cold chip.
  if ((m1 & MODE1_SLEEP) || !(m1 & MODE1_AI) || pre != PCA_PRESCALE) return false;
  if (pca_read_all(counts)) return false;
  // Awake but every servo channel off (legacy shutServos) means the pose is
  // unknown again: treat as cold.
  for (uint8_t c = 0; c < 16; c++)
    if (!(counts[c] & PCA_FULL_OFF)) return true;
  return false;
}

uint8_t pca_init_cold(void) {
  static const uint8_t all_off[4] PROGMEM = {0, 0, 0, 0x10};  // ON = 0, OFF = full-off
  uint8_t e = 0;
  // The prescaler can only be written while asleep. Auto-increment on so the
  // multi-byte writes below land in consecutive registers.
  e |= i2c_write_byte(PCA_ADDR, MODE1, MODE1_SLEEP | MODE1_AI);
  e |= i2c_write_P(PCA_ADDR, ALL_LED_ON_L, all_off, 4);  // LIMP before waking
  e |= i2c_write_byte(PCA_ADDR, PRESCALE, PCA_PRESCALE);
  e |= i2c_write_byte(PCA_ADDR, MODE2, MODE2_OUTDRV);  // totem-pole (default)
  e |= i2c_write_byte(PCA_ADDR, MODE1, MODE1_AI);      // wake
  _delay_us(500);  // oscillator start-up (datasheet: 500 us max)
  // As legacy setPWMFreq() does: set RESTART, which also clears it.
  e |= i2c_write_byte(PCA_ADDR, MODE1, MODE1_RESTART | MODE1_AI);
  return e;
}

uint8_t pca_write_all(const uint16_t counts[16]) {
  i2c_begin_write(PCA_ADDR, LED0_ON_L);
  for (uint8_t c = 0; c < 16; c++) {
    uint16_t v = counts[c];
    i2c_put(0);  // ON_L
    i2c_put(0);  // ON_H
    i2c_put((uint8_t)v);  // OFF_L
    i2c_put((uint8_t)(v >> 8));  // OFF_H (bit 4 = full-off)
  }
  return i2c_end();
}

uint8_t pca_read_all(uint16_t counts[16]) {
  i2c_begin_read(PCA_ADDR, LED0_ON_L);
  for (uint8_t c = 0; c < 16; c++) {
    i2c_get(false);  // ON_L
    i2c_get(false);  // ON_H
    uint8_t lo = i2c_get(false);
    uint8_t hi = i2c_get(c == 15);
    counts[c] = (uint16_t)(((uint16_t)(hi & 0x1F) << 8) | lo);
  }
  return i2c_end();
}
