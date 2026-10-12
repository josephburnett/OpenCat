// hal/pca9685.cpp through the real hal/i2c.cpp against a register-level
// PCA9685 model: the cold configuration (prescaler written while asleep,
// every channel full-off before waking, the 500 us oscillator wait,
// RESTART), the per-tick 66 B burst for a set of joint counts (one
// transaction, outputs latched together at the STOP), the read-back, and
// the warm-boot probe's decision.
#include <vector>
#include "../check.h"
#include "fake_avr.h"
#include "fake_i2c_devices.h"
#include "../../src/hal/i2c.cpp"
#include "../../src/hal/pca9685.cpp"
#include "../../src/core/joints.cpp"

static PcaModel pca;

static void reset_all() {
  fake_avr_reset();
  err = 0;
  err_count = 0;
  pca.power_on();
  g_twi.attach(PCA_ADDR, &pca);
  i2c_init();
}

// The counts a pose produces: the 11 driven joints on their channels,
// every other channel full-off (LIMP).
static void pose_counts(uint16_t counts[16], uint16_t base) {
  for (uint8_t c = 0; c < 16; c++) counts[c] = PCA_FULL_OFF;
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++)
    counts[joint_channel(joint_id(k))] = (uint16_t)(base + 37 * k);
}

static size_t writes_at_delay;
static void note_delay(uint32_t us) {
  if (us == 500) writes_at_delay = pca.writes.size();
}

int main() {
  // ---- cold start ----
  {
    reset_all();
    fake_delay_hook = note_delay;
    writes_at_delay = 0;
    CHECK(pca_init_cold() == I2C_OK);
    // MODE1 = SLEEP|AI, ALL_LED = full-off, PRESCALE, MODE2, wake, RESTART.
    const uint8_t expect[][2] = {{0x00, 0x30}, {0xFA, 0}, {0xFB, 0}, {0xFC, 0}, {0xFD, 0x10},
                                 {0xFE, 24},   {0x01, 0x04}, {0x00, 0x20}, {0x00, 0xA0}};
    size_t n = sizeof expect / sizeof expect[0];
    bool same = pca.writes.size() == n;
    for (size_t i = 0; same && i < n; i++)
      same = pca.writes[i].reg == expect[i][0] && pca.writes[i].val == expect[i][1];
    CHECK(same);
    CHECK(pca.prescale_ignored == 0 && pca.regs[0xFE] == PCA_PRESCALE);  // written while asleep
    CHECK(pca.regs[0x00] == 0x20);  // awake, auto-increment, RESTART self-cleared
    CHECK(pca.regs[0x01] == 0x04);
    bool limp = true;
    for (int c = 0; c < 16; c++) limp = limp && pca.out_off[c] == PCA_FULL_OFF;
    CHECK(limp);  // every channel was full-off before the chip woke
    CHECK(writes_at_delay == 8);  // the 500 us wait is between wake and RESTART
    // No chip: an error, not a hang.
    reset_all();
    g_twi.attach(PCA_ADDR, nullptr);
    CHECK(pca_init_cold() != I2C_OK);
  }

  // ---- the tick's burst for a pose ----
  {
    reset_all();
    pca_init_cold();
    uint16_t counts[16];
    pose_counts(counts, 1100);
    pca.writes.clear();
    long tr = pca.transactions, latches = pca.latches;
    size_t log0 = g_twi.log.size();
    CHECK(pca_write_all(counts) == I2C_OK);
    CHECK(pca.transactions - tr == 1);
    // On the wire: SLA, LED0_ON_L, then ON_L ON_H OFF_L OFF_H x 16.
    std::vector<uint8_t> w;
    for (size_t i = log0; i < g_twi.log.size(); i++)
      if (g_twi.log[i].kind == 'W') w.push_back(g_twi.log[i].v);
    std::vector<uint8_t> expect = {0x06};
    for (int c = 0; c < 16; c++) {
      expect.push_back(0);
      expect.push_back(0);
      expect.push_back((uint8_t)counts[c]);
      expect.push_back((uint8_t)(counts[c] >> 8));
    }
    CHECK(w == expect && w.size() == 65);
    // One output update for all 16 channels, at the STOP.
    CHECK(pca.latches - latches == 1);
    bool all = true;
    for (int c = 0; c < 16; c++) all = all && pca.out_off[c] == counts[c];
    CHECK(all);
    // Read back (what PCA_READ and the warm boot use).
    uint16_t back[16] = {};
    long r0 = pca.reads;
    CHECK(pca_read_all(back) == I2C_OK);
    CHECK(pca.reads - r0 == 64 && !pca.last_ack);  // 64 bytes, the last one NACKed
    all = true;
    for (int c = 0; c < 16; c++) all = all && back[c] == counts[c];
    CHECK(all);
    // The read masks OFF_H to its 5 count bits (bits 7..5 are reserved).
    pca.regs[0x06 + 3] = 0xE5;
    CHECK(pca_read_all(back) == I2C_OK && back[0] == (0x0500 | (uint8_t)counts[0]));
    // A burst that fails part way returns the error (the next tick tries
    // again). Note: the model does not see the STOP that i2c_recover()
    // bit-bangs, but a real PCA9685 (MODE2.OCH = 0) latches on any STOP, so
    // the channels written before the failure take effect for that tick
    // while the rest keep the previous pose (a half-new pose for <= 20 ms).
    // The same holds for a NACK_DATA, which ends with a normal STOP.
    pose_counts(counts, 1500);
    g_twi.arb_lost_at = g_twi.actions + 30;
    CHECK(pca_write_all(counts) == I2C_ARB_LOST);
    CHECK(pca.regs[0x06 + 2] == (uint8_t)counts[0]);  // channel 0's registers already hold the new count
    // Absent chip.
    g_twi.attach(PCA_ADDR, nullptr);
    CHECK(pca_write_all(counts) == I2C_NACK_ADDR && pca_read_all(back) == I2C_NACK_ADDR);
  }

  // ---- the warm-boot probe ----
  {
    uint16_t counts[16], got[16];
    pose_counts(counts, 1200);
    // Power-on chip: SLEEP set, prescale 30: cold.
    reset_all();
    CHECK(!pca_probe_warm(got));
    // A chip that kept driving our pose across an AVR reset: warm, counts back.
    reset_all();
    pca.driving(counts);
    CHECK(pca_probe_warm(got));
    bool all = true;
    for (int c = 0; c < 16; c++) all = all && got[c] == counts[c];
    CHECK(all);
    // Our own cold init followed by one burst, then a reset: warm.
    reset_all();
    pca_init_cold();
    pca_write_all(counts);
    CHECK(pca_probe_warm(got) && got[joint_channel(joint_id(0))] == 1200);
    // Awake but every channel off (legacy shutServos, or LIMP): the pose is unknown.
    reset_all();
    pca_init_cold();
    CHECK(!pca_probe_warm(got));
    // Another prescaler (some other firmware ran): cold.
    reset_all();
    pca.driving(counts, 30);
    CHECK(!pca_probe_warm(got));
    // No auto-increment: cold (our read-back would not work either).
    reset_all();
    pca.driving(counts);
    pca.regs[0x00] = 0x00;
    CHECK(!pca_probe_warm(got));
    // No chip: cold, and the boot carries on.
    reset_all();
    g_twi.attach(PCA_ADDR, nullptr);
    CHECK(!pca_probe_warm(got));
    // An I2C error during the read-back: cold.
    reset_all();
    pca.driving(counts);
    g_twi.arb_lost_at = 30;
    CHECK(!pca_probe_warm(got));
  }

  return report("hal_pca9685");
}
