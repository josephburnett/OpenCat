#include "clock.h"
#include <avr/interrupt.h>
#include <avr/io.h>

static volatile uint32_t ovf;  // Timer0 overflows since boot

ISR(TIMER0_OVF_vect) { ovf++; }

void clock_init(void) {
  TCCR0A = 0;                        // normal mode, counts 0..255
  TCCR0B = _BV(CS01) | _BV(CS00);    // clk/64
  TIMSK0 = _BV(TOIE0);
}

uint32_t clock_micros(void) {
  uint8_t sreg = SREG;
  cli();
  uint32_t o = ovf;
  uint8_t t = TCNT0;
  // The counter may have wrapped after interrupts were disabled, with the
  // overflow ISR still pending: count that overflow here. (t < 255 rules out
  // reading TCNT0 just before the wrap and TOV0 just after.)
  if ((TIFR0 & _BV(TOV0)) && t < 255) o++;
  SREG = sreg;
  return ((o << 8) + t) * 4;
}
