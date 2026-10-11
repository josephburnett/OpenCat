#include "reset.h"

uint8_t reset_boot_epoch(uint8_t last_epoch) {
  uint8_t e = (uint8_t)(last_epoch + 1);
  return e ? e : 1;
}

ResetCause reset_classify(ResetRecord& r, uint32_t build, uint16_t& last_alive) {
  ResetCause c;
  if (r.magic != RESET_MAGIC) {
    c = RESET_COLD;
    last_alive = 0;
  } else {
    last_alive = r.alive_tick;
    if (r.build != build) c = RESET_FLASHED;
    else if (r.reason == RESET_REASON_REQUESTED) c = RESET_REQUESTED;
    else c = RESET_WARM_UNEXPECTED;
  }
  r.magic = RESET_MAGIC;
  r.build = build;
  r.reason = RESET_REASON_NONE;
  r.alive_tick = 0;
  r.epoch = reset_boot_epoch(r.epoch);  // garbage on a cold boot: still a start
  return c;
}
