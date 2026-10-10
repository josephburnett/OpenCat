/* CRC-16/CCITT-FALSE for the Kilo link: poly 0x1021, init 0xFFFF, no
 * reflection, no final xor. check("123456789") = 0x29B1.
 *
 * kilo_crc16_update() has exactly the semantics of avr-libc's
 * _crc_xmodem_update(crc, byte) (<util/crc16.h>), so AVR code may substitute
 * it: define KILO_CRC16_USE_AVR_LIBC when compiling crc16.c for AVR.
 * C99, no allocation, no tables. */
#ifndef KILO_CRC16_H
#define KILO_CRC16_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KILO_CRC16_INIT 0xFFFFu

/* Fold one byte into a running CRC. */
uint16_t kilo_crc16_update(uint16_t crc, uint8_t byte);

/* CRC of len bytes, starting from KILO_CRC16_INIT. */
uint16_t kilo_crc16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* KILO_CRC16_H */
