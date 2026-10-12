// Host shim for <avr/wdt.h>: see ../fake_avr.h.
#ifndef KILO_TEST_FAKE_WDT_H
#define KILO_TEST_FAKE_WDT_H
#include "../fake_avr.h"
#define WDTO_15MS 0
#define WDTO_250MS 4
void wdt_enable(int timeout);
void wdt_disable(void);
void wdt_reset(void);
#endif
