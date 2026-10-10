#include "eeprom.h"
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>

static uint16_t refused;

bool ee_ready(void) { return !(EECR & _BV(EEPE)); }

uint8_t ee_read(uint16_t addr) { return eeprom_read_byte((const uint8_t*)addr); }

void ee_write(uint16_t addr, uint8_t v) {
  if (addr < CFG_REGION_START || addr >= CFG_REGION_END) {
    refused++;
    return;
  }
  if (!ee_ready()) return;
  EEARH = (uint8_t)(addr >> 8);
  EEARL = (uint8_t)addr;
  EEDR = v;
  EECR = 0;  // erase + write (atomic) mode
  // EEPE must be set within 4 cycles of EEMPE, so no interrupt may come in
  // between. EECR is in the low I/O space: each line is one `sbi`.
  uint8_t sreg = SREG;
  cli();
  EECR |= _BV(EEMPE);
  EECR |= _BV(EEPE);
  SREG = sreg;
}

uint16_t ee_refused(void) { return refused; }
