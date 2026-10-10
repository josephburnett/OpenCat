#include "crc16.h"

#if defined(KILO_CRC16_USE_AVR_LIBC) && defined(__AVR__)
#include <util/crc16.h>
uint16_t kilo_crc16_update(uint16_t crc, uint8_t byte)
{
    return _crc_xmodem_update(crc, byte);
}
#else
uint16_t kilo_crc16_update(uint16_t crc, uint8_t byte)
{
    uint8_t i;
    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (i = 0; i < 8u; i++) {
        if (crc & 0x8000u) {
            crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}
#endif

uint16_t kilo_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = KILO_CRC16_INIT;
    uint16_t i;
    for (i = 0; i < len; i++) {
        crc = kilo_crc16_update(crc, data[i]);
    }
    return crc;
}
