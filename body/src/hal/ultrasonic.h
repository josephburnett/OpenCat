// ultrasonic: the RGB ultrasonic module on D6, timed by a pin-change ISR
// (doc §3a, Appendix B3).
//
// The module has one signal pin for both trigger and echo (legacy
// RgbUltrasonic(6, 7): signal on D6, the RGB LEDs on D7). Legacy used
// pulseIn(), which blocks for the whole echo (up to 38 ms with nothing in
// range, or 1 s if no echo starts). Here the tick pulses D6 for 20 us, turns
// it into an input and enables PCINT22; the PCINT2 ISR (hal/pcint2.cpp, shared
// with the IR receiver on D4) stamps both edges with clock_micros(), and
// background() collects the result.
#ifndef KILO_HAL_ULTRASONIC_H
#define KILO_HAL_ULTRASONIC_H
#include <stdint.h>

#define US_PERIOD_TICKS 4      // 80 ms: longer than the module's ~60 ms ring-down
#define US_TIMEOUT_US 25000UL  // no echo end within 25 ms (~4.3 m) = no echo

void us_init(void);
// Start a measurement (step 9 of the tick). ~22 us busy-wait for the pulse.
void us_trigger(uint32_t now_us);
// True once per measurement with the distance in mm (0 = no echo).
bool us_poll(uint32_t now_us, uint16_t* mm);
// An echo is being timed: the WS2812 update (interrupts off) must wait.
bool us_busy(void);
// The shared PCINT2 ISR's hook: PIND as read there and its time stamp.
void us_pcint(uint8_t pind, uint32_t t_us);

#endif
