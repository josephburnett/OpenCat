// Flash-resident constant tables, portable between AVR and the host.
//
// On the ATmega328P, `const` data is copied into RAM at boot unless it is
// marked PROGMEM, and PROGMEM data must then be read with pgm_read_*() because
// flash is a separate address space (Harvard architecture). On the host there
// is one address space, so the same macros collapse to plain reads. This is
// the only place core/ is allowed to look at the target: avr-libc's
// <avr/pgmspace.h> is a compiler-library header, not an Arduino one.
#ifndef KILO_CORE_PGM_H
#define KILO_CORE_PGM_H
#include <stdint.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#define PGM_I8(p) ((int8_t)pgm_read_byte(p))
#define PGM_U8(p) ((uint8_t)pgm_read_byte(p))
#define PGM_I16(p) ((int16_t)pgm_read_word(p))
#define PGM_U16(p) ((uint16_t)pgm_read_word(p))
#else
#define PROGMEM
#define PGM_I8(p) (*(const int8_t*)(p))
#define PGM_U8(p) (*(const uint8_t*)(p))
#define PGM_I16(p) (*(const int16_t*)(p))
#define PGM_U16(p) (*(const uint16_t*)(p))
#endif

// Link-time optimisation inlines functions into every caller. On the AVR,
// where flash is what we count, a function with several callers and more
// than a few instructions is cheaper as one shared copy. Marking those
// KILO_NOINLINE changes nothing but size (each saves roughly 50-200 B).
#define KILO_NOINLINE __attribute__((noinline))

#endif
