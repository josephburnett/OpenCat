// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
// check("123456789") = 0x29B1. Used here for the EEPROM config slots; the
// protocol layer uses the same CRC for frames (the integration round may
// replace this file with the protocol package's implementation).
#ifndef KILO_CORE_CRC16_H
#define KILO_CORE_CRC16_H
#include <stdint.h>

#define CRC16_INIT 0xFFFFu

uint16_t crc16_update(uint16_t crc, uint8_t byte);
uint16_t crc16(const uint8_t* data, uint16_t len);

#endif
