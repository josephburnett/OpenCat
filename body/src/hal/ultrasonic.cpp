#include "ultrasonic.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "clock.h"

// D6 = PD6 = PCINT22, in pin-change group 2 (PCINT16..23 = PORTD).
enum : uint8_t { US_IDLE, US_WAIT_RISE, US_WAIT_FALL, US_DONE };

static volatile uint8_t state;
static volatile uint32_t t_rise, t_fall;
static uint32_t t_trig;

ISR(PCINT2_vect) {
  uint32_t t = clock_micros();
  if (PIND & _BV(6)) {
    if (state == US_WAIT_RISE) {
      t_rise = t;
      state = US_WAIT_FALL;
    }
  } else if (state == US_WAIT_FALL) {
    t_fall = t;
    state = US_DONE;
    PCMSK2 = 0;  // done: no more interrupts from this pin
  }
}

void us_init(void) {
  PCMSK2 = 0;
  PCICR |= _BV(PCIE2);  // group 2 enabled; individual pins via PCMSK2
  state = US_IDLE;
}

void us_trigger(uint32_t now_us) {
  PCMSK2 = 0;
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
  PCIFR = _BV(PCIF2);   // drop any edge from our own pulse
  PCMSK2 = _BV(PCINT22);
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
  PCMSK2 = 0;
  state = US_IDLE;
  *mm = 0;
  return true;
}

bool us_busy(void) { return state == US_WAIT_RISE || state == US_WAIT_FALL; }
