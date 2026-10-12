// Host shim for <avr/pgmspace.h>: one address space, flash reads are plain reads.
#ifndef KILO_TEST_FAKE_PGMSPACE_H
#define KILO_TEST_FAKE_PGMSPACE_H
#include <stdint.h>
#ifndef PROGMEM
#define PROGMEM
#endif
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#define pgm_read_word(p) (*(const uint16_t*)(p))
#endif
