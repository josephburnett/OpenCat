/* Consistent Overhead Byte Stuffing for the Kilo link. C99, no allocation.
 *
 * Encoded output never contains 0x00 and excludes the frame delimiter.
 * Data that ends exactly on a 254-byte non-zero run gets no trailing 0x01
 * code (the common encoder; the Python codec matches). */
#ifndef KILO_COBS_H
#define KILO_COBS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Worst-case encoded size for n input bytes. */
#define KILO_COBS_MAX_ENCODED(n) ((uint16_t)((n) + 1u + (n) / 254u))

/* Encode len bytes of src into dst (capacity >= KILO_COBS_MAX_ENCODED(len)).
 * src and dst must not overlap. Returns the encoded length (>= 1). */
uint16_t kilo_cobs_encode(const uint8_t *src, uint16_t len, uint8_t *dst);

/* Decode len encoded bytes (no delimiter) into dst (capacity >= len - 1; the
 * output is always shorter than the input). dst may equal src (in place).
 * Returns the decoded length, or -1 if the input is empty, contains 0x00,
 * or a code byte runs past the end. */
int16_t kilo_cobs_decode(const uint8_t *src, uint16_t len, uint8_t *dst);

#ifdef __cplusplus
}
#endif

#endif /* KILO_COBS_H */
