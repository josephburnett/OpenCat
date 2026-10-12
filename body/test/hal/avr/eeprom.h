// Host shim for <avr/eeprom.h>: see ../fake_avr.h.
#ifndef KILO_TEST_FAKE_AVR_EEPROM_H
#define KILO_TEST_FAKE_AVR_EEPROM_H
#include "../fake_avr.h"
// hal/eeprom.cpp passes the EEPROM address as a pointer, cast from a
// uint16_t: exact on the AVR (16-bit pointers), a size-changing cast here.
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
// As avr-libc: wait for a write in progress (polling EECR), then read.
uint8_t eeprom_read_byte(const uint8_t* addr);
#endif
