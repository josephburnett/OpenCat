#include "ultrasonic.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// D6 = PD6 = PCINT22, in pin-change group 2 (PCINT16..23 = PORTD).
enum : uint8_t { US_IDLE, US_WAIT_RISE, US_WAIT_FALL, US_DONE };

static volatile uint8_t state;
static volatile uint32_t t_rise, t_fall;
static uint32_t t_trig;

// Called from the shared PCINT2 ISR (hal/pcint2.cpp) on every pin change of
// PORTD, with PIND as read there; acts only on the edge it is waiting for.
void us_pcint(uint8_t pind, uint32_t t) {
  if (pind & _BV(6)) {
    if (state == US_WAIT_RISE) {
      t_rise = t;
      state = US_WAIT_FALL;
    }
  } else if (state == US_WAIT_FALL) {
    t_fall = t;
    state = US_DONE;
    PCMSK2 &= (uint8_t)~_BV(PCINT22);  // done; D4 (IR) keeps its PCINT20
  }
}

void us_init(void) {
  PCMSK2 &= (uint8_t)~_BV(PCINT22);
  state = US_IDLE;
}

// PCMSK2 is not sbi/cbi-reachable, so these are read-modify-writes from the
// main loop. The ISR only ever clears PCINT22 (in US_WAIT_FALL), which these
// never race with: us_trigger runs with PCINT22 off, and the timeout path
// clears the same bit.
void us_trigger(uint32_t now_us) {
  PCMSK2 &= (uint8_t)~_BV(PCINT22);
  // Single-instruction sbi/cbi only: PORTD is shared with the buzzer ISR.
  PORTD &= (uint8_t)~_BV(6);
  DDRD |= _BV(6);
  _delay_us(2);
  PORTD |= _BV(6);
  _delay_us(20);
  PORTD &= (uint8_t)~_BV(6);
  DDRD &= (uint8_t)~_BV(6);  // input, no pull-up (PORTD6 is 0)
  t_trig = now_us;
  state = US_WAIT_RISE;
  // No PCIFR clear: PCINT22 was masked during the pulse, so the pulse set no
  // flag, and a pending flag now is an IR edge on D4 that must not be lost.
  PCMSK2 |= _BV(PCINT22);
}

bool us_poll(uint32_t now_us, uint16_t* mm) {
  uint8_t s = state;
  if (s == US_IDLE) return false;
  if (s == US_DONE) {
    uint8_t sreg = SREG;
    cli();
    uint32_t echo = t_fall - t_rise;
    SREG = sreg;
    state = US_IDLE;
    // Sound travels 343 m/s and the echo goes there and back:
    // mm = us * 0.1715 = us * 1715 / 10000.
    *mm = (uint16_t)(echo * 1715 / 10000);
    return true;
  }
  if (now_us - t_trig < US_TIMEOUT_US) return false;
  PCMSK2 &= (uint8_t)~_BV(PCINT22);
  state = US_IDLE;
  *mm = 0;
  return true;
}

bool us_busy(void) { return state == US_WAIT_RISE || state == US_WAIT_FALL; }
