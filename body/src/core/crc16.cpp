#include "crc16.h"

#ifdef __AVR__
#include <util/crc16.h>
#endif

uint16_t crc16_update(uint16_t crc, uint8_t byte) {
#ifdef __AVR__
  // avr-libc's xmodem update is the same polynomial, unreflected; with init
  // 0xFFFF it is CCITT-FALSE. It is a few tens of cycles of inline asm.
  return _crc_xmodem_update(crc, byte);
#else
  crc ^= (uint16_t)((uint16_t)byte << 8);
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
  return crc;
#endif
}

uint16_t crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = CRC16_INIT;
  while (len--) crc = crc16_update(crc, *data++);
  return crc;
}
