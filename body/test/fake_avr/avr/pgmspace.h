// Host fake of <avr/pgmspace.h>: flash is ordinary memory.
#ifndef KILO_TEST_FAKE_AVR_PGMSPACE_H
#define KILO_TEST_FAKE_AVR_PGMSPACE_H
#include <cstdint>
#define PROGMEM
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#endif
