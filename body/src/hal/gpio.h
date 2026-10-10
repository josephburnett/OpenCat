// gpio: the on-board LED (D3) and the PORTD rule.
//
// D3 (LED), D5 (buzzer), D6 (ultrasonic) and D7 (WS2812 eyes) share PORTD,
// and the buzzer ISR toggles D5. A read-modify-write of PORTD from the main
// loop (in, or, out) could be interrupted half way and undo a toggle. So
// PORTD/DDRD bits are only changed with single-instruction sbi/cbi, which is
// what avr-gcc emits for `PORTD |= _BV(n)` / `PORTD &= ~_BV(n)` with a
// constant n, and the buzzer toggles with a write to PIND (doc §3a).
#ifndef KILO_HAL_GPIO_H
#define KILO_HAL_GPIO_H
#include <avr/io.h>

static inline void led_init(void) { DDRD |= _BV(3); }
static inline void led_on(void) { PORTD |= _BV(3); }
static inline void led_off(void) { PORTD &= (uint8_t)~_BV(3); }

#endif
