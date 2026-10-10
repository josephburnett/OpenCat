#include "adc.h"
#include <avr/io.h>

static uint8_t pending;

void adc_init(void) {
  ADMUX = _BV(REFS0) | 7;  // AVCC (5 V) reference, channel ADC7 = A7
  // Enable, clock 16 MHz / 128 = 125 kHz (the 50-200 kHz full-accuracy band).
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
}

void adc_start(void) {
  ADCSRA |= _BV(ADSC);  // no ISR touches ADCSRA, so the RMW is safe
  pending = 1;
}

bool adc_take(uint16_t* counts) {
  if (!pending || (ADCSRA & _BV(ADSC))) return false;
  pending = 0;
  *counts = ADC;  // reads ADCL then ADCH
  return true;
}
