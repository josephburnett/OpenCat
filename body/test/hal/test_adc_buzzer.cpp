// hal/adc.cpp (the non-blocking battery sample) and hal/buzzer.cpp (Timer2
// tone generation) on fake registers: the ADC start/collect contract, and
// for every audible pitch the Timer2 prescaler and compare value, checked
// against the pitch they actually produce.
#include <cstdio>
#include <cstdlib>
#include "../check.h"
#include "fake_avr.h"
#include "../../src/hal/adc.cpp"
#include "../../src/hal/buzzer.cpp"

int main() {
  // ---- ADC ----
  fake_avr_reset();
  adc_init();
  CHECK(ADMUX == (_BV(REFS0) | 7));  // AVCC reference, channel 7 (A7)
  CHECK(ADCSRA == (_BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0)));  // /128 = 125 kHz
  uint16_t v = 0;
  CHECK(!adc_take(&v));  // nothing started
  adc_start();
  CHECK(ADCSRA & _BV(ADSC));
  CHECK(!adc_take(&v));  // converting
  ADC = 612;
  ADCSRA = (uint8_t)(ADCSRA & ~_BV(ADSC));  // the hardware finishes
  CHECK(adc_take(&v) && v == 612);
  CHECK(!adc_take(&v));  // once per conversion
  CHECK(ADCSRA == (_BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0)));  // start touched only ADSC

  // ---- buzzer ----
  fake_avr_reset();
  PORTD = _BV(3);  // the LED is on; the buzzer must not disturb it
  buzzer_init();
  CHECK((DDRD & _BV(5)) && !(PORTD & _BV(5)) && (PORTD & _BV(3)));
  CHECK(TCCR2A == _BV(WGM21) && TCCR2B == 0);  // CTC, stopped
  // The ISR toggles D5 by writing PIND (a single-instruction toggle).
  PIND = 0;
  isr_TIMER2_COMPA();
  CHECK(PIND == _BV(5));
  static const uint16_t PRESC[8] = {0, 1, 8, 32, 64, 128, 256, 1024};
  bool all = true;
  int a4_err_1e4 = 0;
  double worst = 0;
  uint32_t worst_hz = 0;
  for (uint32_t hz = 31; hz <= 20000; hz++) {
    buzzer_set((uint16_t)hz);
    uint8_t cs = TCCR2B;
    if (hz < 31 || cs < 1 || cs > 7) {
      all = false;
      continue;
    }
    // Toggle rate F_CPU / (presc * (OCR2A + 1)); the pitch is half of it.
    double f = (double)F_CPU / (2.0 * PRESC[cs] * (OCR2A + 1));
    double err = (f - hz) / hz;
    // Never more than one count off (the count is truncated, so the pitch
    // is a little sharp), and at least 32 counts per half period.
    double one = 1.0 / (OCR2A + 1) + 1.0 / (double)((F_CPU / 2) / hz);
    if (err < 0 || err > one || OCR2A + 1 < 32) all = false;
    if (err > worst) worst = err, worst_hz = hz;
    if (cs > 1 && ((F_CPU / 2) / hz) / PRESC[cs - 1] <= 256) all = false;  // a smaller prescaler fitted
    if (TIMSK2 != _BV(OCIE2A) || TCNT2 != 0) all = false;
    if (hz == 440) a4_err_1e4 = (int)(err * 1e4);
  }
  CHECK(all);
  CHECK(std::abs(a4_err_1e4) < 50);  // A4 within 0.5 %
  std::printf("buzzer: worst pitch error %.2f %% at %u Hz\n", worst * 100, (unsigned)worst_hz);
  // 1 kHz: 8 MHz / 1000 = 8000 cycles per toggle -> /32, 250 counts.
  buzzer_set(1000);
  CHECK(TCCR2B == 3 && OCR2A == 249);
  // Silence: timer and interrupt off, D5 low, LED untouched.
  PORTD = (uint8_t)(PORTD | _BV(5));
  buzzer_set(0);
  CHECK(TCCR2B == 0 && TIMSK2 == 0 && !(PORTD & _BV(5)) && (PORTD & _BV(3)));
  // Below Timer2's range (~31 Hz at /1024): silent rather than wrong.
  buzzer_set(30);
  CHECK(TCCR2B == 0 && TIMSK2 == 0);
  buzzer_set(1);
  CHECK(TCCR2B == 0);

  return report("hal_adc_buzzer");
}
