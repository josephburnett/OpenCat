// Host shim for <util/delay.h>: see ../fake_avr.h.
#ifndef KILO_TEST_FAKE_DELAY_H
#define KILO_TEST_FAKE_DELAY_H
#include "../fake_avr.h"
#define _delay_us(us) fake_delay_us((uint32_t)(us))
#define _delay_ms(ms) fake_delay_us((uint32_t)(ms) * 1000u)
#endif
