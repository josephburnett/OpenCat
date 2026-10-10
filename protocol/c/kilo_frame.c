#include "kilo_frame.h"
#include "crc16.h"

/* Frames are at most 64 decoded bytes, so no COBS block can reach 254 bytes
 * and the frame-specific COBS below is much smaller than the general one in
 * cobs.c (it matters on the AVR): encode lays the decoded bytes out at
 * wire[1..] and then fills in the code bytes back to front; decode works in
 * place, front to back. Both are exactly equivalent to cobs.c for these
 * lengths (protocol/test.sh checks them against the vectors and fuzzes). */
uint8_t kilo_frame_encode(uint8_t type, uint8_t seq, const uint8_t *payload,
                          uint8_t len, uint8_t *wire)
{
    uint16_t crc;
    uint8_t n, i, next;

    if (len > KILO_MAX_PAYLOAD) {
        return 0;
    }
    wire[1] = type;
    wire[2] = seq;
    for (i = 0; i < len; i++) {
        wire[3u + i] = payload[i];
    }
    crc = kilo_crc16(wire + 1, (uint16_t)(len + 2u));
    n = (uint8_t)(len + 4u); /* decoded bytes, at wire[1..n] */
    wire[n - 1u] = (uint8_t)(crc & 0xFFu);
    wire[n] = (uint8_t)(crc >> 8);
    /* Each zero becomes the distance to the next zero (or to the end), and
     * wire[0] the distance to the first one. */
    next = (uint8_t)(n + 1u);
    for (i = n; i >= 1u; i--) {
        if (wire[i] == 0u) {
            wire[i] = (uint8_t)(next - i);
            next = i;
        }
    }
    wire[0] = next;
    wire[n + 1u] = 0u;
    return (uint8_t)(n + 2u);
}

uint8_t kilo_frame_decode(uint8_t *buf, uint8_t n, kilo_frame_t *out)
{
    uint8_t plen, st, i, code;
    uint16_t crc;

    out->type = 0;
    out->seq = 0;
    out->len = 0;
    out->payload = 0;
    if (n > KILO_MAX_ENCODED) {
        return KILO_FRAME_BAD_LENGTH;
    }
    if (n == 0u) {
        return KILO_FRAME_BAD_COBS;
    }
    /* In place: decoded[i - 1] = buf[i], except that each code position
     * after the first decodes to 0. The codes must chain exactly to n. */
    code = buf[0];
    for (i = 1; i < n; i++) {
        uint8_t b = buf[i];
        if (b == 0u || code == 0u) {
            return KILO_FRAME_BAD_COBS;
        }
        if (i == code) {
            code = (uint8_t)(i + b);
            if (code < i) { /* wrapped past 255: runs past the end */
                return KILO_FRAME_BAD_COBS;
            }
            b = 0u;
        }
        buf[i - 1u] = b;
    }
    if (code != n) {
        return KILO_FRAME_BAD_COBS;
    }
    if (n < KILO_MIN_DECODED + 1u) {
        return KILO_FRAME_BAD_LENGTH;
    }
    plen = (uint8_t)(n - 5u); /* decoded n - 1 = type, seq, payload, crc */
    crc = kilo_crc16(buf, (uint16_t)(plen + 2u));
    if ((uint8_t)(crc & 0xFFu) != buf[plen + 2u] || (uint8_t)(crc >> 8) != buf[plen + 3u]) {
        return KILO_FRAME_BAD_CRC;
    }
    out->type = buf[0];
    out->seq = buf[1];
    out->len = plen;
    out->payload = buf + 2;
    st = kilo_payload_len_check(buf[0], buf + 2, plen);
    if (st == KILO_LEN_UNKNOWN_TYPE) {
        return KILO_FRAME_UNKNOWN_TYPE;
    }
    if (st != KILO_LEN_OK) {
        return KILO_FRAME_BAD_PAYLOAD_LEN;
    }
    return KILO_FRAME_OK;
}

void kilo_rx_init(kilo_rx_t *rx)
{
    rx->n = 0;
    rx->overflow = 0;
}

uint8_t kilo_rx_push(kilo_rx_t *rx, uint8_t byte, kilo_frame_t *out)
{
    uint8_t n;
    if (byte != 0u) {
        if (rx->overflow) {
            return KILO_FRAME_NONE;
        }
        if (rx->n >= KILO_MAX_ENCODED) {
            rx->overflow = 1;
            rx->n = 0;
            return KILO_FRAME_OVERFLOW;
        }
        rx->buf[rx->n++] = byte;
        return KILO_FRAME_NONE;
    }
    if (rx->overflow) {
        rx->overflow = 0;
        return KILO_FRAME_NONE;
    }
    if (rx->n == 0u) {
        return KILO_FRAME_NONE;
    }
    n = rx->n;
    rx->n = 0;
    return kilo_frame_decode(rx->buf, n, out);
}
