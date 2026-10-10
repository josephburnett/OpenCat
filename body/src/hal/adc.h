// adc: battery voltage on A7 (doc §3g), non-blocking.
//
// A conversion takes 13 ADC clocks at 125 kHz = 104 us. Rather than wait,
// the tick starts one (adc_start) and collects it on a later tick
// (adc_take): the result is 20 ms old, which does not matter for a battery
// that changes over minutes.
#ifndef KILO_HAL_ADC_H
#define KILO_HAL_ADC_H
#include <stdint.h>

void adc_init(void);
void adc_start(void);
// True once, with the counts (0..1023), after a started conversion is done.
bool adc_take(uint16_t* counts);

#endif
