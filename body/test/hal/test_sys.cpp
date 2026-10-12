// hal/sys.cpp on the fake AVR: the stack high-water scan behind STATS
// stack_free_min (over a fake free-RAM region painted as the .init1 code
// paints the real one: 0xC5 from _end up to and including __stack), the
// watchdog calls, and REBOOT marking the .noinit reset record so that the
// next boot classifies itself as RESET_REQUESTED. (reset_classify's own
// cases are covered by test/test_body.cpp.)
#include "../check.h"
#include "fake_avr.h"
#include "../../src/hal/sys.h"
// _end and __stack are zero-length arrays at the region's labels (see
// fake_avr.h), so UBSan's object-size check would flag every access past
// the first byte: switch that one check off for the functions that walk the
// region (ASan still checks every access).
#define WALKS_RAM __attribute__((no_sanitize("object-size")))
WALKS_RAM uint16_t sys_stack_free_min(void);
#include "../../src/hal/sys.cpp"
#include "../../src/core/reset.cpp"

static uint8_t* lo() { return &_end; }
static uint8_t* hi() { return &__stack; }

// What sys_paint_stack does at reset (it is AVR assembly, not built here).
WALKS_RAM static void paint() {
  for (uint8_t* p = lo(); p <= hi(); p++) *p = 0xC5;
}
// The stack reaching `depth` bytes below the top, with these bytes.
WALKS_RAM static void use_stack(uint16_t depth, uint8_t fill) {
  for (uint16_t i = 0; i < depth; i++) hi()[-(int)i] = fill;
}

WALKS_RAM static void poke(int off, uint8_t v) { hi()[off] = v; }

int main() {
  CHECK(hi() - lo() + 1 == FAKE_RAM_FREE);

  // ---- stack_free_min ----
  paint();
  CHECK(sys_stack_free_min() == FAKE_RAM_FREE);  // never used: all of it
  use_stack(100, 0x00);
  CHECK(sys_stack_free_min() == FAKE_RAM_FREE - 100);
  // The stack returning upwards does not repaint: the minimum stays.
  use_stack(300, 0x12);
  use_stack(50, 0xC5);  // even paint-coloured bytes left by later frames
  CHECK(sys_stack_free_min() == FAKE_RAM_FREE - 300);
  // Paint-coloured bytes inside the used part do not hide the deepest point:
  // the scan runs up from _end and stops at the first overwritten byte.
  paint();
  use_stack(200, 0x00);
  poke(-150, 0xC5);
  poke(-199, 0xC5);  // except the very deepest byte itself (a 1 in 256 chance, off by one)
  CHECK(sys_stack_free_min() == FAKE_RAM_FREE - 199);
  // Edges: everything used, one byte left.
  paint();
  use_stack(FAKE_RAM_FREE, 0x00);
  CHECK(sys_stack_free_min() == 0);
  paint();
  use_stack(FAKE_RAM_FREE - 1, 0x00);
  CHECK(sys_stack_free_min() == 1);

  // ---- watchdog ----
  fake_avr_reset();
  sys_wdt_arm();
  CHECK(fake_wdt_timeout == WDTO_250MS);
  sys_wdt_kick();
  sys_wdt_kick();
  CHECK(fake_wdt_resets == 2);

  // ---- REBOOT: the record says "requested", the next boot agrees ----
  {
    const uint32_t BUILD = 0x12345678u;
    uint16_t last = 0;
    sys_reset_record.magic = 0;  // power-on garbage
    CHECK(reset_classify(sys_reset_record, BUILD, last) == RESET_COLD);
    sys_reset_record.alive_tick = 4321;  // the loop ran a while
    sys_reset_record.epoch = 9;
    fake_wdt_throw = true;
    bool fired = false;
    try {
      sys_reboot();
    } catch (const FakeWdtFired& f) {
      fired = f.timeout == WDTO_15MS;
    }
    CHECK(fired);
    CHECK(sys_reset_record.reason == RESET_REASON_REQUESTED);
    CHECK(reset_classify(sys_reset_record, BUILD, last) == RESET_REQUESTED);
    CHECK(last == 4321 && sys_reset_record.epoch == 10);
    // The boot after that, if the watchdog bites without a REBOOT: unexpected.
    CHECK(reset_classify(sys_reset_record, BUILD, last) == RESET_WARM_UNEXPECTED);
  }

  return report("hal_sys");
}
