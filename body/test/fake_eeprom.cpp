// The fake EEPROM, linked into every host test (core/config.cpp needs the
// ee_* functions even when a test does not use them).
#include "fake_eeprom.h"

FakeEeprom g_ee;
uint8_t ee_read(uint16_t a) {
  if (g_ee.busy) g_ee.read_while_busy = true;
  return g_ee.mem[a & 1023];
}
bool ee_ready(void) {
  if (g_ee.busy) {
    g_ee.busy--;
    return false;
  }
  return true;
}
void ee_write(uint16_t a, uint8_t v) {
  g_ee.writes.push_back(a);
  g_ee.busy = g_ee.busy_calls;
  if (g_ee.cut_after == 0) return;
  if (g_ee.cut_after > 0) g_ee.cut_after--;
  g_ee.mem[a & 1023] = v;
}
