// app: boot sequence and the main loop, the glue between core/ and hal/.
#ifndef KILO_APP_MAIN_H
#define KILO_APP_MAIN_H
#include <stdint.h>
#include "../core/body.h"
#include "../core/config.h"

// Counters for STATS (§4); filled by the loop, read by the link layer.
struct BodyStats {
  uint16_t max_tick_us;   // longest tick since last read (link resets it)
  uint16_t overruns;
  uint16_t dist_mm;       // last ultrasonic range, 0 = no echo
  uint8_t dist_fresh;     // set on a new range (TELEMETRY flag 11)
  uint16_t batt_counts;   // last raw ADC sample
  uint8_t reset_cause;
  uint16_t last_alive_tick;
  uint32_t build;
};

extern Body g_body;
extern Config g_config;
extern BodyStats g_stats;
extern uint16_t g_counts[16];  // the last counts written to the PCA9685

#endif
