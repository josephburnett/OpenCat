#include "cobs.h"

uint16_t kilo_cobs_encode(const uint8_t *src, uint16_t len, uint8_t *dst)
{
    uint16_t out = 1;      /* next write position */
    uint16_t code_pos = 0; /* where the current block's code byte goes */
    uint8_t code = 1;
    uint8_t open = 1;      /* a code byte is reserved at code_pos */
    uint16_t i;

    for (i = 0; i < len; i++) {
        uint8_t b = src[i];
        if (b != 0u) {
            dst[out++] = b;
            code++;
        }
        if (b == 0u || code == 0xFFu) {
            dst[code_pos] = code;
            code = 1;
            if (b == 0u || (uint16_t)(i + 1u) < len) {
                code_pos = out++;
                open = 1;
            } else {
                open = 0;
            }
        }
    }
    if (open) {
        dst[code_pos] = code;
    }
    return out;
}

int16_t kilo_cobs_decode(const uint8_t *src, uint16_t len, uint8_t *dst)
{
    uint16_t in = 0;
    uint16_t out = 0;

    if (len == 0u) {
        return -1;
    }
    while (in < len) {
        uint8_t code = src[in];
        uint16_t end;
        if (code == 0u) {
            return -1;
        }
        end = (uint16_t)(in + code);
        if (end > len) {
            return -1;
        }
        in++;
        while (in < end) {
            uint8_t b = src[in++];
            if (b == 0u) {
                return -1;
            }
            dst[out++] = b;
        }
        if (code != 0xFFu && in < len) {
            dst[out++] = 0;
        }
    }
    return (int16_t)out;
}
