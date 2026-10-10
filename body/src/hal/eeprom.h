// eeprom: the AVR's internal 1 KB EEPROM, non-blocking writes only.
//
// One EEPROM byte write takes ~3.4 ms in hardware. avr-libc's
// eeprom_write_byte() waits for the previous write first, which could stall
// the tick; here a write is only started when the hardware is idle
// (ee_ready()), and the caller (core/config's background writer) comes back
// later for the next byte.
//
// Writes outside the config slots (704..959) are refused: the legacy layout
// is never written, so the old firmware finds its EEPROM intact.
#ifndef KILO_HAL_EEPROM_H
#define KILO_HAL_EEPROM_H
#include <stdint.h>
#include "../core/config.h"

// ee_ready(), ee_read() and ee_write() are declared in core/config.h.
// ee_read() waits if a write is in progress: check ee_ready() first.
uint16_t ee_refused(void);  // count of out-of-region write attempts

#endif
