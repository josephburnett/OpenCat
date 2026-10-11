// A 1 KB fake EEPROM implementing the ee_* seam of core/config.h.
// Writes are "in progress" for `busy_calls` ee_ready() polls, like the
// 3.4 ms hardware write, and every write is logged so tests can prove the
// legacy region is never touched. A write can be made to "lose power":
// after `cut_after` more writes, writes are silently dropped.
#ifndef KILO_TEST_FAKE_EEPROM_H
#define KILO_TEST_FAKE_EEPROM_H
#include <cstdint>
#include <cstring>
#include <vector>
#include "../src/core/config.h"

struct FakeEeprom {
  uint8_t mem[1024];
  int busy = 0;
  int busy_calls = 2;
  long cut_after = -1;  // -1 = never
  std::vector<uint16_t> writes;
  bool read_while_busy = false;
  void reset() {
    std::memset(mem, 0xFF, sizeof mem);
    busy = 0;
    cut_after = -1;
    writes.clear();
    read_while_busy = false;
  }
  void put16(uint16_t a, int16_t v) {
    mem[a] = (uint8_t)v;
    mem[a + 1] = (uint8_t)((uint16_t)v >> 8);
  }
};

extern FakeEeprom g_ee;


// Legacy EEPROM content the setup sketch would leave for a Nybble with the
// given per-joint calibration offsets, oscillator and IMU offsets.
inline void legacy_fill(FakeEeprom& e, const int8_t calib[16], uint16_t osc_khz, int16_t a2p_override = 0) {
  static const uint8_t pin[16] = {12, 11, 4, 3, 13, 10, 5, 2, 14, 9, 6, 1, 15, 8, 7, 0};
  static const int8_t mid[16] = {0, 15, 0, 0, -45, -45, -45, -45, 10, 10, -10, -10, -30, -30, 30, 30};
  static const int8_t dir[16] = {1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1};
  for (int j = 0; j < 16; j++) {
    e.mem[0 + j] = pin[j];
    e.mem[16 + j] = (uint8_t)calib[j];
    e.mem[32 + j] = (uint8_t)mid[j];
    e.mem[48 + j] = (uint8_t)dir[j];
    int zero = 90 + mid[j] * dir[j];
    e.put16((uint16_t)(64 + 2 * j), (int16_t)zero);
    e.put16((uint16_t)(96 + 2 * j), (int16_t)(zero + calib[j] * dir[j]));
    // setup sketch: int((2000/180 = 11) * k_us2pulse * 1000), with
    // k_us2pulse = osc / 25 MHz (prescale 24 + 1 = 25).
    int16_t a2p = a2p_override ? a2p_override : (int16_t)(int)(11.0f * (float)(osc_khz * 1000.0 / 25000000.0) * 1000);
    e.put16((uint16_t)(160 + 2 * j), a2p);
  }
  for (int i = 0; i < 6; i++) e.put16((uint16_t)(256 + 2 * i), (int16_t)(100 * i - 250));
  e.mem[274] = 60;  // B_OFFSET 600 / 10
  e.put16(275, (int16_t)osc_khz);
}

#endif
