// Battery filter and hysteresis (doc §3g table, Appendix H "reflex").
#include "../src/core/battery.h"
#include "check.h"

static Battery b;

static BattLevel feed(uint16_t mv, int samples) {
  BattLevel l = b.level;
  for (int i = 0; i < samples; i++) l = battery_sample(b, mv);
  return l;
}

int main() {
  // Scale: legacy 99 counts per volt; 650 counts ~ 6.57 V.
  CHECK(battery_counts_to_mv(650, 646) == 6561);
  CHECK(battery_counts_to_mv(832, 646) == 8398);

  battery_init(b);
  CHECK(feed(7400, 1) == BATT_OK && battery_mv(b) == 7400);  // seeded, not ramped
  // Motion sag: 1 s dips to 6.0 V every 3 s for a minute never trip.
  for (int cycle = 0; cycle < 20; cycle++) {
    feed(6000, 10);
    feed(7400, 20);
  }
  CHECK(b.level == BATT_OK);
  // Steady 6.8 V: WARN after ~5 s (plus the filter's settling time).
  int n = 0;
  while (battery_sample(b, 6800) == BATT_OK && n < 1000) n++;
  CHECKF(b.level == BATT_WARN && n >= 50 && n <= 120, "warn after %d samples", n);
  // Recovery to 7.05 V is inside the hysteresis band: stays WARN.
  feed(7050, 300);
  CHECK(b.level == BATT_WARN);
  // Above 7.1 V for 10 s clears.
  n = 0;
  while (battery_sample(b, 7200) != BATT_OK && n < 1000) n++;
  CHECKF(n >= 100 && n <= 200, "warn cleared after %d samples", n);
  // LOW at 6.5 V; clears only above 7.0 V for 10 s (back to WARN band first).
  feed(6500, 200);
  CHECK(b.level == BATT_LOW);
  feed(6950, 300);
  CHECK(b.level == BATT_LOW);
  feed(7050, 300);
  CHECK(b.level == BATT_WARN);  // LOW cleared, WARN (< 7.1) still on
  // CRITICAL: < 6.3 V for 1 s, latched even if the voltage recovers.
  feed(6000, 100);
  CHECK(b.level == BATT_CRITICAL);
  feed(8400, 1000);
  CHECK(b.level == BATT_CRITICAL);
  // USB bench power: no battery, reflexes off.
  battery_init(b);
  CHECK(feed(100, 50) == BATT_OK && b.no_battery);
  feed(4000, 100);
  CHECK(b.no_battery);  // still inside the 3..5 V hysteresis
  feed(7400, 200);
  CHECK(!b.no_battery && b.level == BATT_OK);
  return report("battery");
}
