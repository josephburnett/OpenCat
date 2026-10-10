// clock: microseconds since boot from Timer0, replacing Arduino's micros().
//
// Timer0 counts at 16 MHz / 64 = 250 kHz (4 us per count) and overflows
// every 256 counts (1.024 ms); the overflow ISR extends it to 32 bits. This
// is the same scheme as the Arduino core, written out (about 150 B) so the
// sketch does not need the core's init() and its other side effects (it sets
// up all three timers for analogWrite and pulls in wiring.c).
#ifndef KILO_HAL_CLOCK_H
#define KILO_HAL_CLOCK_H
#include <stdint.h>

void clock_init(void);
// Wraps every 71.6 minutes; compare times with signed differences.
// Safe to call from an ISR.
uint32_t clock_micros(void);

#endif
