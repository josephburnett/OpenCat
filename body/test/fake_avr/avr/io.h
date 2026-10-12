// Host fake of <avr/io.h>: only what hal/i2c.cpp touches (../fake_twi.h).
#ifndef KILO_TEST_FAKE_AVR_IO_H
#define KILO_TEST_FAKE_AVR_IO_H
#include "../fake_twi.h"
#define _BV(b) (1u << (b))
#define TWINT 7
#define TWEA 6
#define TWSTA 5
#define TWSTO 4
#define TWEN 2
#define TWCR g_twcr_reg
#define TWDR g_twi.twdr
#define TWSR g_twi.twsr
#define TWBR g_twi.twbr
#define PORTC g_twi.portc
#define DDRC g_twi.ddrc
#define PINC g_twi.pinc
#endif
