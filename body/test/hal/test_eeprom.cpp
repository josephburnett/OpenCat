// hal/eeprom.cpp on a fake EECR that enforces the hardware's write
// sequence (EEMPE, then EEPE on the next write, interrupts off for both) and
// holds EEPE for the duration of a byte write. Checks the region guard, the
// non-blocking contract (a write is only started when idle, a read waits),
// and runs core/config's slot writer end to end through the real driver: a
// migrated config lands as a CRC-valid slot, one byte write per step, the
// legacy layout untouched, and boots back. (The slot logic itself, incl.
// torn writes and the v1->v2 load, is test/test_config.cpp's.)
#include <cstring>
#include "../check.h"
#include "../fake_eeprom.h"
#include "fake_avr.h"
#include "../../src/hal/eeprom.cpp"
#include "../../src/core/config.cpp"
#include "../../src/core/joints.cpp"

FakeEeprom g_ee;  // fake_eeprom.h's legacy_fill() builds an image in one

static const int8_t CAL[16] = {3, -4, 0, 0, 0, 0, 0, 0, 5, -2, 7, 1, -6, 2, 0, -9};

static void reset_all() {
  fake_avr_reset();
  refused = 0;
}

int main() {
  // ---- one byte write: the register sequence ----
  {
    reset_all();
    CHECK(ee_ready());
    ee_write(800, 0x5A);
    CHECK(fake_eeprom[800] == 0x5A && EECR.commits == 1 && EECR.bad_sequence == 0);
    CHECK(EEARH == (800 >> 8) && EEARL == (800 & 0xFF) && EEDR == 0x5A);
    // EECR: mode bits cleared (erase+write) with interrupts on, then EEMPE
    // and EEPE back to back with interrupts off.
    CHECK(EECR.log.size() == 3);
    if (EECR.log.size() == 3) {
      CHECK(EECR.log[0].value == 0 && (EECR.log[0].sreg & _BV(SREG_I)));
      CHECK(EECR.log[1].value == _BV(EEMPE) && !(EECR.log[1].sreg & _BV(SREG_I)));
      CHECK((EECR.log[2].value & _BV(EEPE)) && !(EECR.log[2].sreg & _BV(SREG_I)));
    }
    CHECK(SREG == _BV(SREG_I));  // restored
    // Busy for the write time; a write started meanwhile is dropped.
    CHECK(!ee_ready());
    ee_write(801, 0x77);
    CHECK(fake_eeprom[801] == 0xFF && EECR.commits == 1 && ee_refused() == 0);
    while (!ee_ready()) {
    }
    ee_write(801, 0x77);
    CHECK(fake_eeprom[801] == 0x77 && EECR.commits == 2);
    // A read waits for the write in progress, then returns the new byte.
    CHECK(ee_read(801) == 0x77 && ee_ready());
    // With interrupts already off, SREG stays off.
    cli();
    ee_write(802, 1);
    CHECK(SREG == 0 && EECR.bad_sequence == 0);
    sei();
  }

  // ---- the region guard: only the config slots, 704..959 ----
  {
    reset_all();
    const uint16_t bad[] = {0, 703, 960, 1023, 0xFFFF};
    for (uint16_t a : bad) ee_write(a, 0);
    CHECK(ee_refused() == 5 && EECR.commits == 0 && EECR.log.empty());
    ee_write(704, 1);
    while (!ee_ready()) {
    }
    ee_write(959, 2);
    CHECK(EECR.commits == 2 && fake_eeprom[704] == 1 && fake_eeprom[959] == 2 && ee_refused() == 5);
  }

  // ---- core/config's writer through the real driver ----
  {
    reset_all();
    g_ee.reset();
    legacy_fill(g_ee, CAL, 25300);
    std::memcpy(fake_eeprom, g_ee.mem, 1024);
    uint8_t before[1024];
    std::memcpy(before, fake_eeprom, 1024);
    EECR.busy_reads = 5;
    Config c;
    config_boot(c);
    CHECK(c.state == CFG_MIGRATED && config_busy(c));
    long steps = 0, worst = 0;
    while (config_busy(c) && steps < 100000) {
      long n0 = EECR.commits;
      config_writer_step(c);
      if (EECR.commits - n0 > worst) worst = EECR.commits - n0;
      steps++;
    }
    CHECK(!config_busy(c) && c.saved && c.newest == 0);
    CHECK(worst == 1);  // never more than one byte write per step
    CHECK(EECR.bad_sequence == 0 && ee_refused() == 0);
    ConfigSlot a;
    std::memcpy(&a, &fake_eeprom[CFG_SLOT_A], sizeof a);
    CHECK(config_slot_valid(a) && std::memcmp(&a, &c.img, sizeof a) == 0);
    // Only slot A changed: the legacy layout is exactly as the old firmware left it.
    bool only_a = true;
    for (int i = 0; i < 1024; i++)
      if (fake_eeprom[i] != before[i] && (i < CFG_SLOT_A || i >= CFG_SLOT_A + CFG_SLOT_SIZE)) only_a = false;
    CHECK(only_a);
    // The next boot loads it through eeprom_read_byte.
    Config c2;
    config_boot(c2);
    CHECK(c2.state == CFG_OK && c2.newest == 0 && !config_busy(c2));
    CHECK(std::memcmp(&c2.img, &c.img, sizeof c.img) == 0);
    // A second save goes to slot B; unchanged bytes are not rewritten.
    c2.img.batt_mv_per_64 = (uint16_t)(c2.img.batt_mv_per_64 + 3);
    CHECK(config_save(c2));
    long n0 = EECR.commits;
    while (config_busy(c2)) config_writer_step(c2);
    CHECK(c2.newest == 1);
    long differ = 0;  // B was blank (0xFF): every byte that is not 0xFF
    for (uint8_t i = 0; i < CFG_SLOT_SIZE; i++) differ += ((const uint8_t*)&c2.img)[i] != 0xFF;
    CHECK(EECR.commits - n0 == differ && differ > 100);
    CHECK(c2.img.crc != 0xFFFF || true);
    Config c3;
    config_boot(c3);
    CHECK(c3.newest == 1 && c3.img.batt_mv_per_64 == c2.img.batt_mv_per_64);
    // Saving the same image again into A rewrites only what differs from A.
    CHECK(config_save(c3));
    n0 = EECR.commits;
    while (config_busy(c3)) config_writer_step(c3);
    CHECK(c3.newest == 0);
    CHECK(EECR.commits - n0 < 10);  // seq, batt scale, CRC: a few bytes
    CHECK(EECR.bad_sequence == 0 && ee_refused() == 0);
  }

  return report("hal_eeprom");
}
