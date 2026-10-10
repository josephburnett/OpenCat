#include "kilo_frame.h"
#include "cobs.h"
#include "crc16.h"

uint8_t kilo_frame_encode(uint8_t type, uint8_t seq, const uint8_t *payload,
                          uint8_t len, uint8_t *wire)
{
    /* Encode straight into wire without a staging buffer: COBS over the
     * logical sequence type, seq, payload..., crc_lo, crc_hi. */
    uint16_t crc;
    uint8_t out = 1, code_pos = 0, code = 1;
    uint8_t total, i;

    if (len > KILO_MAX_PAYLOAD) {
        return 0;
    }
    crc = kilo_crc16_update(KILO_CRC16_INIT, type);
    crc = kilo_crc16_update(crc, seq);
    for (i = 0; i < len; i++) {
        crc = kilo_crc16_update(crc, payload[i]);
    }
    total = (uint8_t)(len + 4u); /* <= 64, so no 254-byte block can occur */
    for (i = 0; i < total; i++) {
        uint8_t b;
        if (i == 0u) {
            b = type;
        } else if (i == 1u) {
            b = seq;
        } else if (i < (uint8_t)(len + 2u)) {
            b = payload[i - 2u];
        } else if (i == (uint8_t)(len + 2u)) {
            b = (uint8_t)(crc & 0xFFu);
        } else {
            b = (uint8_t)(crc >> 8);
        }
        if (b != 0u) {
            wire[out++] = b;
            code++;
        } else {
            wire[code_pos] = code;
            code = 1;
            code_pos = out++;
        }
    }
    wire[code_pos] = code;
    wire[out++] = 0u;
    return out;
}

uint8_t kilo_frame_decode(uint8_t *buf, uint8_t n, kilo_frame_t *out)
{
    int16_t dn;
    uint8_t plen, st;
    uint16_t crc;

    out->type = 0;
    out->seq = 0;
    out->len = 0;
    out->payload = 0;
    if (n > KILO_MAX_ENCODED) {
        return KILO_FRAME_BAD_LENGTH;
    }
    dn = kilo_cobs_decode(buf, n, buf);
    if (dn < 0) {
        return KILO_FRAME_BAD_COBS;
    }
    if (dn < (int16_t)KILO_MIN_DECODED || dn > (int16_t)KILO_MAX_DECODED) {
        return KILO_FRAME_BAD_LENGTH;
    }
    plen = (uint8_t)(dn - 4);
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
