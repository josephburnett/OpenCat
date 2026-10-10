// Config slots: migration accept/reject, never writing the legacy layout,
// the non-blocking writer, torn writes at every byte, sequence wrap.
#include "fake_eeprom.h"
#include "check.h"

static const int8_t CAL[16] = {3, -4, 0, 0, 0, 0, 0, 0, 5, -2, 7, 1, -6, 2, 0, -9};

static void run_writer(Config& c, long max_steps = 100000) {
  while (config_busy(c) && max_steps--) config_writer_step(c);
}

static bool all_writes_in_slots() {
  for (uint16_t a : g_ee.writes)
    if (a < CFG_REGION_START || a >= CFG_REGION_END) return false;
  return true;
}

int main() {
  // 1. Fresh robot with legacy data: migrates, queues slot A, writes only
  //    the slots, and the next boot loads slot A.
  g_ee.reset();
  legacy_fill(g_ee, CAL, 25300);
  uint8_t legacy_before[1024];
  std::memcpy(legacy_before, g_ee.mem, 1024);
  Config c;
  config_boot(c);
  CHECK(c.state == CFG_MIGRATED && c.reason == CFG_REASON_NONE);
  CHECK(config_busy(c));
  for (int j = 0; j < 16; j++) {
    int dir = joint_dir((uint8_t)j);
    int czp = 90 + joint_mid_shift((uint8_t)j) * dir + CAL[j] * dir;
    bool driven = j < 3 || j > 7;
    if (driven) {
      CHECK(c.img.cal.zero_a16[j] == czp * 16);
      CHECK(c.img.cal.a2p[j] == (int16_t)(int)(11.0f * (float)(25300 * 1000.0 / 25000000.0) * 1000));
      CHECK(config_calib_a16(c.img, (uint8_t)j) == CAL[j] * 16);  // protocol "calib" = legacy CALIB
    }
  }
  CHECK(c.img.cal.b_offset_counts == 600 && c.img.pca9685_freq_khz == 25300);
  CHECK(c.img.imu_offset[0] == -250 && c.img.imu_offset[5] == 250);
  // One step = at most one byte written, and never while the EEPROM is busy.
  size_t before = g_ee.writes.size();
  config_writer_step(c);
  CHECK(g_ee.writes.size() - before <= 1);
  run_writer(c);
  CHECK(!config_busy(c) && c.saved && c.newest == 0);
  CHECK(!g_ee.read_while_busy);
  CHECK(all_writes_in_slots());
  CHECK(std::memcmp(legacy_before, g_ee.mem, 704) == 0);           // legacy 0..703 untouched
  CHECK(std::memcmp(legacy_before + 960, g_ee.mem + 960, 64) == 0);  // melodies untouched
  Config c2;
  config_boot(c2);
  CHECK(c2.state == CFG_OK && c2.newest == 0 && std::memcmp(&c2.img, &c.img, sizeof c.img) == 0);

  // 2. A save goes to the OTHER slot with seq + 1; the next boot picks it.
  c2.img.cal.zero_a16[8] = (int16_t)(c2.img.cal.zero_a16[8] + 16);
  CHECK(config_save(c2));
  CHECK(!config_save(c2));  // busy: the caller answers EEPROM_BUSY
  run_writer(c2);
  CHECK(c2.newest == 1);
  Config c3;
  config_boot(c3);
  CHECK(c3.newest == 1 && c3.img.seq == 2 && c3.img.cal.zero_a16[8] == c2.img.cal.zero_a16[8]);

  // 3. Torn write at every byte: power fails after n bytes of a save. Boot
  //    must load either the old slot or the complete new one, never a mix.
  uint8_t snapshot[1024];
  std::memcpy(snapshot, g_ee.mem, 1024);
  for (int n = 0; n <= 130; n++) {
    std::memcpy(g_ee.mem, snapshot, 1024);
    Config a;
    config_boot(a);  // newest = B (seq 2)
    ConfigSlot old = a.img;
    for (int j = 0; j < 16; j++) a.img.cal.a2p[j] = (int16_t)(a.img.cal.a2p[j] + 7);  // many bytes differ
    a.img.params[3] = 1234;
    config_save(a);
    ConfigSlot fresh = a.img;
    g_ee.cut_after = n;
    run_writer(a);
    g_ee.cut_after = -1;
    Config b;
    config_boot(b);
    bool is_old = std::memcmp(&b.img, &old, sizeof old) == 0;
    bool is_new = std::memcmp(&b.img, &fresh, sizeof fresh) == 0;
    CHECKF(b.state == CFG_OK && (is_old || is_new), "torn after %d bytes: old %d new %d", n, is_old, is_new);
  }

  // 4. Sequence wrap: 255 -> 0 is newer.
  std::memcpy(g_ee.mem, snapshot, 1024);
  Config w;
  config_boot(w);
  for (int i = 0; i < 300; i++) {
    w.img.params[0] = (int16_t)i;
    config_save(w);
    run_writer(w);
    Config r;
    config_boot(r);
    CHECKF(r.img.params[0] == i, "after save %d (seq %d) boot loaded params[0]=%d", i, w.img.seq, r.img.params[0]);
  }
  CHECK(all_writes_in_slots());

  // 5. Migration rejections -> defaults, nothing written, STAND gated by the
  //    caller (core/body.cpp).
  struct Case {
    uint16_t addr;
    uint8_t val;
    ConfigReason why;
  } cases[] = {
      {0, 13, CFG_REASON_PWM_PIN},       // PWM_PIN[0] wrong
      {33, 0, CFG_REASON_MID_SHIFT},     // MID_SHIFT[1] wrong
      {48 + 9, 1, CFG_REASON_DIRECTION}, // ROTATION_DIRECTION[9] wrong
      {16 + 8, 9, CFG_REASON_CALIB},     // CALIB[8] disagrees with CZP by 4
      {274, 30, CFG_REASON_B_OFFSET},    // 300 counts
  };
  for (const Case& k : cases) {
    g_ee.reset();
    legacy_fill(g_ee, CAL, 25000);
    g_ee.mem[k.addr] = k.val;
    Config d;
    config_boot(d);
    CHECKF(d.state == CFG_DEFAULTED && d.reason == k.why, "case addr %d: state %d reason %d", k.addr, d.state, d.reason);
    CHECK(!config_busy(d) && g_ee.writes.empty());
    CHECK(d.img.cal.a2p[8] == 11000 && d.img.cal.b_offset_counts == 600);
    CHECK(d.img.cal.zero_a16[9] == (90 + 10 * -1) * 16);  // 90 + dir*midShift
  }
  // A2P out of range.
  g_ee.reset();
  legacy_fill(g_ee, CAL, 25000, 14000);
  Config d;
  config_boot(d);
  CHECK(d.state == CFG_DEFAULTED && d.reason == CFG_REASON_A2P);
  // CALIB off by exactly 1 degree is accepted (rounding in old tools).
  g_ee.reset();
  legacy_fill(g_ee, CAL, 25000);
  g_ee.mem[16 + 12] = (uint8_t)(CAL[12] + 1);
  config_boot(d);
  CHECK(d.state == CFG_MIGRATED);
  // A blank (erased) EEPROM fails the first check.
  g_ee.reset();
  config_boot(d);
  CHECK(d.state == CFG_DEFAULTED && d.reason == CFG_REASON_PWM_PIN);
  // A corrupted slot falls back to the other one.
  g_ee.reset();
  legacy_fill(g_ee, CAL, 25000);
  config_boot(d);
  run_writer(d);
  config_save(d);
  run_writer(d);  // A (seq 1) and B (seq 2) valid
  g_ee.mem[CFG_SLOT_B + 40] ^= 0x01;
  Config e;
  config_boot(e);
  CHECK(e.state == CFG_OK && e.newest == 0 && e.img.seq == 1);
  return report("config");
}
