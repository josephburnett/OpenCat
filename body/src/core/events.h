// events: a small queue of body events waiting to be sent as EVENT frames.
//
// The reflexes push here; the link layer (integration round) pops and sends
// them, and only pops once the frame fits in the TX ring, so events are never
// dropped (doc §4 "TX priority"): they wait here instead. If more than 8
// pile up, the oldest is kept and the newest is counted as lost.
//
// The codes mirror Appendix E. The integration round should static_assert
// them against the generated protocol header rather than keep two lists.
#ifndef KILO_CORE_EVENTS_H
#define KILO_CORE_EVENTS_H
#include <stdint.h>

enum EventCode : uint8_t {
  EV_BOOT = 1,
  EV_FELL = 2,
  EV_RECOVERED = 3,
  EV_RECOVERY_FAILED = 4,
  EV_LOW_BATTERY = 5,
  EV_BATTERY_OK = 6,
  EV_WATCHDOG = 7,
  EV_LINK_RESTORED = 8,
  EV_IMU_FAULT = 9,
  EV_IMU_OK = 10,
  EV_TRICK_DONE = 11,
  EV_GAIT_STOPPED = 12,
  EV_GOAL_REFUSED = 13,
  EV_CONFIG_MIGRATED = 14,
  EV_CONFIG_DEFAULTED = 15,
  EV_EEPROM_SAVED = 16,
  EV_TX_DROPPED = 17,
};

#define EVENT_QUEUE_LEN 8

struct Event {
  uint8_t code;
  int16_t arg;
  uint16_t tick;
  uint8_t epoch;
};

struct EventQueue {
  Event q[EVENT_QUEUE_LEN];
  uint8_t head, count;
  uint8_t lost;
};

void events_init(EventQueue& e);
void events_push(EventQueue& e, uint8_t code, int16_t arg, uint16_t tick, uint8_t epoch);
// Oldest event, or null. Pop only after it has been sent.
const Event* events_peek(const EventQueue& e);
void events_pop(EventQueue& e);

#endif
