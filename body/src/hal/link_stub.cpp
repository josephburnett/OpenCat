// Placeholder implementation of core/link_iface.h until the protocol package
// (protocol/spec.toml -> COBS, CRC, codec, dispatch) is integrated. It keeps
// the UART drained so the RX ring never overflows, and throws events away.
//
// With this stub no frame is ever "valid", so the link watchdog is never
// armed: a cold boot stays LIMP, a warm boot holds, lies down, then limps.
// That is the safe behaviour for a body nobody is talking to.
#include "../core/body.h"
#include "../core/link_iface.h"
#include "uart.h"

static Body* body;
static link_frame_cb on_frame;

void link_init(Body* b, link_frame_cb cb) {
  body = b;
  on_frame = cb;
}

void link_poll(void) {
  uint8_t c;
  while (uart_read(&c)) {
    // INTEGRATION: feed c to the COBS decoder; on a CRC-valid frame call
    // body_link_frame(*body) and then on_frame(type, payload, len).
  }
}

bool link_send_bytes(const uint8_t* data, uint8_t len) { return uart_write(data, len, 0); }

void link_tick(void) {
  // INTEGRATION: queue TELEMETRY/JOINTS/IMU_RAW per the stream divisors,
  // and send queued events (pop only what fits in the TX ring).
  while (events_peek(body->ev)) events_pop(body->ev);
}

void link_boot(uint8_t, uint16_t, uint8_t, uint8_t) {
  // INTEGRATION: send HELLO (reset cause, last alive tick, config state...).
}
