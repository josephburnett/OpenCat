// link_iface: the narrow interface between the tick and the protocol layer.
//
// The protocol (message types, COBS framing, CRC, dispatch, telemetry) is
// generated from protocol/spec.toml in a separate work package. Until it is
// integrated, hal/link_stub.cpp implements this interface: it drains and
// discards received bytes and sends nothing. The integration round replaces
// link_stub.cpp; nothing else should need to change.
//
// What the real implementation must do (doc §3a, §4):
//   link_poll()        decode complete frames from the UART RX ring; for each
//                      CRC-valid frame call body_link_frame() (link watchdog),
//                      then the frame callback / dispatcher. Called at least
//                      every 2 ms of busy work (ticks steps 3 and 7, and in
//                      every loop pass). Must not block.
//   link_tick(...)     once per tick (step 10): queue TELEMETRY/JOINTS/IMU_RAW
//                      per the stream divisors and drain the event queue,
//                      in priority order ACK/EVENT > TELEMETRY > JOINTS >
//                      IMU_RAW, dropping what does not fit in the TX ring.
//   link_boot(...)     after config load: send HELLO and EVENT BOOT.
#ifndef KILO_CORE_LINK_IFACE_H
#define KILO_CORE_LINK_IFACE_H
#include <stdint.h>

struct Body;

// A decoded, CRC-checked frame: message type and payload (no seq, no CRC).
typedef void (*link_frame_cb)(uint8_t type, const uint8_t* payload, uint8_t len);

void link_init(Body* body, link_frame_cb on_frame);
void link_poll(void);
// Queue raw bytes for transmission (an already-encoded frame). All or
// nothing: returns false, and queues nothing, if it does not fit.
bool link_send_bytes(const uint8_t* data, uint8_t len);
void link_tick(void);
// reset_cause: core/reset.h ResetCause; last_alive: the previous run's
// canary; config_state/reason: core/config.h.
void link_boot(uint8_t reset_cause, uint16_t last_alive, uint8_t config_state, uint8_t config_reason);

#endif
