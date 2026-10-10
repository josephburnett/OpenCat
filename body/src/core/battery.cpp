#include "battery.h"
#include "pgm.h"

// Thresholds (doc §3g table). Become params later (BATT_*_MV).
#define WARN_TRIP 6900
#define WARN_CLEAR 7100
#define LOW_TRIP 6600
#define LOW_CLEAR 7000
#define CRIT_TRIP 6300
#define NOBATT_BELOW 3000
#define NOBATT_ABOVE 5000
// Dwell times in 10 Hz samples.
#define TRIP_5S 50
#define CLEAR_10S 100
#define CRIT_1S 10

void battery_init(Battery& b) {
  b.filt_q1 = 0;
  b.seeded = 0;
  b.no_battery = 0;
  b.warn.active = b.warn.count = 0;
  b.low.active = b.low.count = 0;
  b.crit.active = b.crit.count = 0;
  b.level = BATT_OK;
}

uint16_t battery_counts_to_mv(uint16_t counts, uint16_t mv_per_64) {
  return (uint16_t)(((uint32_t)counts * mv_per_64 + 32) >> 6);
}

// Count consecutive samples on the "other side" of the threshold that would
// change the state, and flip once the dwell time is reached. A sample on the
// wrong side restarts the count, so a brief sag (or spike) never accumulates.
KILO_NOINLINE static void hyst_step(Hyst& h, uint16_t mv, uint16_t trip, uint16_t clear, uint8_t trip_n,
                      uint8_t clear_n, bool latch) {
  bool toward = h.active ? (!latch && mv > clear) : (mv < trip);
  if (!toward) {
    h.count = 0;
    return;
  }
  if (++h.count >= (h.active ? clear_n : trip_n)) {
    h.active = !h.active;
    h.count = 0;
  }
}

BattLevel battery_sample(Battery& b, uint16_t mv) {
  if (!b.seeded) {
    // Start the filter at the first reading, not at 0 V, so boot does not
    // look like a dead battery for the first few seconds.
    b.filt_q1 = (uint16_t)(mv << 1);
    b.seeded = 1;
  } else {
    // alpha = 1/16: f += (x - f) / 16. mv <= 10.4 V, so x2 values and their
    // differences fit 16 bits. The arithmetic shift rounds toward -inf: the
    // steady-state bias is under 8 mV, far below the thresholds' spacing.
    int16_t d = (int16_t)((int16_t)(mv << 1) - (int16_t)b.filt_q1);
    b.filt_q1 = (uint16_t)(b.filt_q1 + (d >> 4));
  }
  // USB bench power reads near 0 V: switch the battery reflexes off rather
  // than declare a critical battery. Decided on the raw sample, with its own
  // wide hysteresis; when a pack appears, restart the filter at its voltage
  // so the climb from 0 V does not look like a discharged pack.
  if (b.no_battery ? mv > NOBATT_ABOVE : mv < NOBATT_BELOW) {
    b.no_battery = !b.no_battery;
    b.filt_q1 = (uint16_t)(mv << 1);
  }
  if (b.no_battery) {
    b.warn.count = b.low.count = b.crit.count = 0;
    b.level = b.crit.active ? BATT_CRITICAL : BATT_OK;
    return b.level;
  }
  uint16_t f = battery_mv(b);
  hyst_step(b.warn, f, WARN_TRIP, WARN_CLEAR, TRIP_5S, CLEAR_10S, false);
  hyst_step(b.low, f, LOW_TRIP, LOW_CLEAR, TRIP_5S, CLEAR_10S, false);
  hyst_step(b.crit, f, CRIT_TRIP, 0, CRIT_1S, 0, true);  // latched until reboot
  b.level = b.crit.active ? BATT_CRITICAL
            : b.low.active ? BATT_LOW
            : b.warn.active ? BATT_WARN
                            : BATT_OK;
  return b.level;
}
