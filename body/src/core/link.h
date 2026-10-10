// link: the body's end of the protocol (doc §4, Appendix E): COBS/CRC
// receive, dispatch of every P0 message, ACK with de-duplication, HELLO,
// the TELEMETRY / JOINTS / IMU_RAW streams and the EVENT queue, with the
// TX drop priority.
//
// Pure logic like the rest of core/: the hardware is reached through the
// link_hal_* functions below, a link-time seam implemented by
// app/link_hal.cpp on the AVR and by test/fake_link_hal.cpp on the host. So
// the exact dispatch that runs on Kilo also runs in the host tests and in the
// cross-check against the mind's virtual body.
//
// Dispatch contract (per CRC-valid frame from the Pi; doc §4 "Acks and
// errors"):
//   * every CRC-valid frame feeds the link watchdog (body_link_frame);
//   * unknown type -> ACK UNKNOWN_TYPE; wrong payload length -> ACK BAD_LEN
//     for a command, EVENT GOAL_REFUSED(type, BAD_LEN) for a goal;
//   * goals are never acked; a rejection is EVENT GOAL_REFUSED(type << 8 |
//     reason), at most one per goal type per second (reason = ACK code);
//   * queries answer with their data reply only, an ACK only on failure,
//     and are never de-duplicated;
//   * other commands get exactly one ACK; a repeat of the last executed one
//     (same seq and type, within 500 ms) is re-acked with the cached status
//     and not executed again.
#ifndef KILO_CORE_LINK_H
#define KILO_CORE_LINK_H
#include <stdint.h>
#include "../proto/kilo_protocol.h"

struct Body;

// ---- API for app/main.cpp ----
// The link serves g_body (defined by app/main.cpp, or by the host test).
extern Body g_body;
// build: HELLO.fw_build. caps: HELLO.caps (KILO_HELLO_CAPS_*).
void link_init(uint32_t build, uint16_t caps);
// After config load: the unsolicited HELLO. (EVENT BOOT goes through the
// event queue like every other event.)
void link_boot(uint8_t reset_cause, uint16_t last_alive);
// Drain the RX ring and handle every complete frame. Never blocks, except
// that a reply waits (at most one TX-ring drain, < 4 ms) for room.
void link_poll(void);
// Once per tick, after body_tick: queued events, then the streams due this
// tick in priority order TELEMETRY > JOINTS > IMU_RAW, dropping what does
// not fit. Events are never dropped: they wait in the queue.
void link_tick(void);
// Send the queued events that fit (link_tick calls this first).
void link_send_events(void);

// Counters kept by the link (STATS, TELEMETRY rx_err).
struct LinkCounters {
  uint16_t rx_crc_err;   // bad CRC, bad COBS or bad length: header untrusted
  uint16_t rx_overflow;  // > 65 B before a delimiter
  uint16_t tx_dropped;   // stream frames that did not fit in the TX ring
  uint32_t uptime_s;
};
const LinkCounters& link_counters(void);

// ---- Seams: implemented by app/link_hal.cpp (AVR) or the host fake ----
// Values for TELEMETRY that the app measures.
struct LinkSample {
  uint16_t dist_mm;   // last ultrasonic range, 0 = no echo
  uint8_t dist_fresh; // a new range since the last sample
  uint8_t tick_load;  // max tick time since the last sample, 100 us units
  uint8_t overrun;    // a tick overran since the last sample
};
bool link_hal_read(uint8_t* c);                        // next RX byte
uint8_t link_hal_tx_free(void);                        // free TX ring bytes
void link_hal_write(const uint8_t* data, uint8_t len); // only called when it fits
void link_hal_sample(LinkSample* s);                   // and reset the "since" values
// STATS fields the hardware owns: overruns, max_tick_us, stack_free_min,
// rx_overflow (UART ring), i2c_err. The link adds its own counters.
void link_hal_stats(kilo_stats_t* s);
uint16_t link_hal_imu_packets(void);                   // DMP packets since boot (wraps)
void link_hal_imu_raw(int16_t raw[6]);                 // last packet: accel xyz, gyro xyz
bool link_hal_pca_read(uint16_t counts[16]);           // false on an I2C error
void link_hal_imu_offsets(const int16_t offsets[6]);   // write the MPU offset registers
void link_hal_reboot(void);                            // after the ACK is queued

#endif
