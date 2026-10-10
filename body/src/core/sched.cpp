#include "sched.h"

void sched_start(Sched& s, uint32_t now_us) {
  s.next = now_us + TICK_US;
  s.overruns = 0;
}

bool sched_due(Sched& s, uint32_t now_us) {
  // Signed difference: correct across the 71-minute micros() wrap.
  int32_t lag = (int32_t)(now_us - s.next);
  if (lag < 0) return false;
  s.next += TICK_US;
  if (lag > (int32_t)(2 * TICK_US)) {
    s.next = now_us + TICK_US;
    if (s.overruns < 0xFFFF) s.overruns++;
  }
  return true;
}
