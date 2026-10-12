// Host fake of <util/twi.h>: the status codes, and TW_STATUS from the fake.
#ifndef KILO_TEST_FAKE_UTIL_TWI_H
#define KILO_TEST_FAKE_UTIL_TWI_H
#include "../fake_twi.h"
#define TW_START 0x08
#define TW_REP_START 0x10
#define TW_MT_SLA_ACK 0x18
#define TW_MT_DATA_ACK 0x28
#define TW_MT_ARB_LOST 0x38
#define TW_MR_SLA_ACK 0x40
#define TW_MR_DATA_ACK 0x50
#define TW_MR_DATA_NACK 0x58
#define TW_WRITE 0
#define TW_READ 1
#define TW_STATUS (g_twi.status)
#endif
