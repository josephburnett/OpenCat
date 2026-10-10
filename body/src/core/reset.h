// reset: why did we boot? (doc §3a "Reset cause")
//
// The stock Uno bootloader (optiboot) clears MCUSR, the register that says
// whether the last reset was power-on, watchdog or the reset pin. So the
// firmware keeps its own record in RAM that the C runtime does not zero
// (the .noinit section, placed by hal/noinit.cpp). Power loss scrambles RAM,
// so a valid magic number means "RAM survived": some kind of warm reset.
#ifndef KILO_CORE_RESET_H
#define KILO_CORE_RESET_H
#include <stdint.h>

#define RESET_MAGIC 0x4B4Cu  // "KL"
#define RESET_REASON_NONE 0
#define RESET_REASON_REQUESTED 0x5A  // set just before a REBOOT

enum ResetCause : uint8_t {
  RESET_COLD = 0,             // magic invalid: power-on or RAM-losing brown-out
  RESET_FLASHED = 1,          // magic ok, different firmware build
  RESET_REQUESTED = 2,        // REBOOT message
  RESET_WARM_UNEXPECTED = 3,  // watchdog hang, reset button, stray GPIO4 pulse
};

struct ResetRecord {
  uint16_t magic;
  uint32_t build;       // hash of the running firmware
  uint8_t reason;
  uint16_t alive_tick;  // tick-alive canary, written every tick
};

// Classify the record left by the previous run, report its last alive tick,
// then rewrite the record for this boot.
ResetCause reset_classify(ResetRecord& r, uint32_t build, uint16_t& last_alive);

#endif
