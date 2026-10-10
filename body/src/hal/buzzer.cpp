#include "buzzer.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

ISR(TIMER2_COMPA_vect) { PIND = _BV(5); }  // toggle D5

void buzzer_init(void) {
  PORTD &= (uint8_t)~_BV(5);
  DDRD |= _BV(5);
  TCCR2A = _BV(WGM21);  // CTC: count to OCR2A, then restart
  TCCR2B = 0;           // stopped
}

void buzzer_set(uint16_t hz) {
  TIMSK2 = 0;
  TCCR2B = 0;
  PORTD &= (uint8_t)~_BV(5);
  if (!hz) return;
  // Toggle rate 2*hz: the period in CPU cycles is F_CPU / (2 hz). Pick the
  // smallest Timer2 prescaler that brings it under 256 counts (finest pitch).
  // Prescalers 1, 8, 32, 64, 128, 256, 1024 as shifts (flash, not RAM).
  static const uint8_t SHIFT[7] PROGMEM = {0, 3, 5, 6, 7, 8, 10};
  uint32_t cycles = (F_CPU / 2) / hz;
  for (uint8_t cs = 0; cs < 7; cs++) {
    uint32_t n = cycles >> pgm_read_byte(&SHIFT[cs]);
    if (n <= 256) {
      OCR2A = (uint8_t)(n ? n - 1 : 0);
      TCNT2 = 0;
      TCCR2B = (uint8_t)(cs + 1);  // CS22:0 = 1..7 for those prescalers
      TIMSK2 = _BV(OCIE2A);
      return;
    }
  }
  // Below ~31 Hz: out of Timer2's range, stay silent.
}
