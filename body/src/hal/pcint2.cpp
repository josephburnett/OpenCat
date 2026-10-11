#include "pcint2.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../features.h"
#include "clock.h"
#include "ir.h"
#include "ultrasonic.h"

#if KILO_ULTRASONIC || KILO_IR
void pcint2_init(void) { PCICR |= _BV(PCIE2); }

#if KILO_IR
static uint8_t prev_pind = 0xFF;  // D4 idles high
#endif

// PIND is read once. The ultrasonic is dispatched on every interrupt with
// the level of D6: its state machine only acts on a high level while waiting
// for the rise and a low level while waiting for the fall, so an interrupt
// caused by D4 is harmless to it (and it cannot use the previous PIND: D6 is
// driven as an output for the trigger pulse while PCINT22 is off, so the
// remembered D6 level may be stale). D4 is an input with PCINT20 always on,
// so the previous PIND finds its falling edges.
ISR(PCINT2_vect) {
  uint32_t t = clock_micros();
  uint8_t pin = PIND;
#if KILO_ULTRASONIC
  us_pcint(pin, t);
#endif
#if KILO_IR
  uint8_t fell = prev_pind & (uint8_t)~pin;
  prev_pind = pin;
  if (fell & _BV(4)) ir_edge(t);
#endif
}
#else
void pcint2_init(void) {}
#endif
