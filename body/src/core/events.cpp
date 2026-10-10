#include "events.h"

void events_init(EventQueue& e) { e.head = e.count = e.lost = 0; }

void events_push(EventQueue& e, uint8_t code, int16_t arg, uint16_t tick, uint8_t epoch) {
  if (e.count >= EVENT_QUEUE_LEN) {
    if (e.lost < 255) e.lost++;
    return;
  }
  Event& v = e.q[(uint8_t)((e.head + e.count) % EVENT_QUEUE_LEN)];
  v.code = code;
  v.arg = arg;
  v.tick = tick;
  v.epoch = epoch;
  e.count++;
}

const Event* events_peek(const EventQueue& e) { return e.count ? &e.q[e.head] : 0; }

void events_pop(EventQueue& e) {
  if (!e.count) return;
  e.head = (uint8_t)((e.head + 1) % EVENT_QUEUE_LEN);
  e.count--;
}
