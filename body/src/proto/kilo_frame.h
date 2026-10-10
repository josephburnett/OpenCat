/* Kilo link frames (docs/architecture.md §4). C99, no allocation, no floats.
 *
 *   decoded: type u8 | seq u8 | payload (0..60) | crc16 u16 LE   (4..64 B)
 *   wire:    COBS(decoded) | 0x00                                (<= 66 B)
 *   crc16 = CRC-16/CCITT-FALSE over type, seq, payload.
 *
 * Needs kilo_protocol.h (protocol/gen/) for the per-type length rules. */
#ifndef KILO_FRAME_H
#define KILO_FRAME_H

#include <stdint.h>
#include "kilo_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KILO_MAX_ENCODED (KILO_MAX_WIRE - 1u) /* 65: COBS bytes before the delimiter */

enum kilo_frame_status {
    KILO_FRAME_OK = 0,
    KILO_FRAME_BAD_COBS = 1,        /* invalid COBS: header untrusted */
    KILO_FRAME_BAD_CRC = 2,         /* CRC mismatch: header untrusted, never ACK */
    KILO_FRAME_BAD_LENGTH = 3,      /* decoded < 4 or > 64 B: header untrusted */
    KILO_FRAME_BAD_PAYLOAD_LEN = 4, /* CRC ok, known type, wrong payload length: header trusted */
    KILO_FRAME_UNKNOWN_TYPE = 5,    /* CRC ok, type not in the spec: header trusted */
    KILO_FRAME_OVERFLOW = 6,        /* rx stream: > 65 B before a delimiter */
    KILO_FRAME_NONE = 255           /* rx stream: no complete frame yet */
};

/* A decoded frame. payload points into the buffer that was decoded in place
 * (the caller's buffer, or the kilo_rx_t buffer); it stays valid until that
 * buffer is reused. type/seq/payload/len are set for OK, BAD_PAYLOAD_LEN and
 * UNKNOWN_TYPE; otherwise len = 0 and payload = 0. */
typedef struct kilo_frame {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    const uint8_t *payload;
} kilo_frame_t;

/* Build a wire frame including the trailing 0x00. wire must hold
 * KILO_MAX_WIRE (66) bytes; payload and wire must not overlap. Returns the
 * number of bytes written, or 0 if len > KILO_MAX_PAYLOAD. Does not apply the
 * per-type length rule. */
uint8_t kilo_frame_encode(uint8_t type, uint8_t seq, const uint8_t *payload,
                          uint8_t len, uint8_t *wire);

/* Decode one frame in place. buf holds n COBS bytes without the delimiter.
 * Checks, in order: COBS, decoded length, CRC, type, payload length.
 * Returns a kilo_frame_status (never OVERFLOW or NONE). */
uint8_t kilo_frame_decode(uint8_t *buf, uint8_t n, kilo_frame_t *out);

/* Byte-at-a-time receiver following §4's receiver rules. */
typedef struct kilo_rx {
    uint8_t buf[KILO_MAX_ENCODED];
    uint8_t n;
    uint8_t overflow;
} kilo_rx_t;

void kilo_rx_init(kilo_rx_t *rx);

/* Feed one received byte. Returns KILO_FRAME_NONE until a delimiter completes
 * a non-empty segment, then that segment's kilo_frame_status (out filled as
 * for kilo_frame_decode; out->payload points into rx->buf and is valid only
 * until the next kilo_rx_push call, so handle the frame first). Empty segments (repeated 0x00) are ignored. The
 * 66th non-zero byte before a delimiter returns KILO_FRAME_OVERFLOW once and
 * everything up to the next delimiter is discarded. */
uint8_t kilo_rx_push(kilo_rx_t *rx, uint8_t byte, kilo_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* KILO_FRAME_H */
