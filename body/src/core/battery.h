// battery: filtered pack voltage and the WARN / LOW / CRITICAL levels with
// hysteresis (doc §3g). Pure logic: one call per 10 Hz ADC sample.
//
// Why filter: servos pull amps when they move, and the pack sags by several
// hundred mV for a moment. Thresholds on raw samples would trip on every
// stride. An IIR low-pass (alpha = 1/16 at 10 Hz, tau ~ 1.6 s) plus "must
// stay below for 5 s" means only a real discharge trips a level.
//
// Why hysteresis: once tripped, a level clears only well ABOVE its trip
// voltage, for longer. A resting pack recovers a little; without the gap,
// Kilo would lie down, recover, stand, sag and lie down again forever.
#ifndef KILO_CORE_BATTERY_H
#define KILO_CORE_BATTERY_H
#include <stdint.h>

enum BattLevel : uint8_t { BATT_OK = 0, BATT_WARN = 1, BATT_LOW = 2, BATT_CRITICAL = 3 };

// One threshold with its own trip/clear voltages and dwell times (samples).
struct Hyst {
  uint8_t active;
  uint8_t count;  // consecutive samples beyond the threshold in play
};

struct Battery {
  uint16_t filt_q1;  // filtered mV x 2 (a fraction bit; x2 keeps differences in int16)
  uint8_t seeded;
  uint8_t no_battery;  // < 3 V: running from USB; battery reflexes off
  Hyst warn, low, crit;
  BattLevel level;
};

#define BATT_SAMPLE_TICKS 5  // 10 Hz in 20 ms ticks

void battery_init(Battery& b);

// ADC counts (0..1023, A7 through the divider) -> mV with the stored scale.
uint16_t battery_counts_to_mv(uint16_t counts, uint16_t mv_per_64);

// Feed one sample. Returns the new level (BATT_OK when no_battery).
BattLevel battery_sample(Battery& b, uint16_t mv);

inline uint16_t battery_mv(const Battery& b) { return (uint16_t)((b.filt_q1 + 1) >> 1); }

#endif
