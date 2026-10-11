#include "config.h"
#include "../proto/crc16.h"
#include "pgm.h"

// Legacy addresses (src/OpenCat.h; Appendix C4).
#define L_PWM_PIN 0
#define L_CALIB 16
#define L_MID_SHIFT 32
#define L_ROTATION_DIRECTION 48
#define L_CALIBRATED_ZERO_POSITIONS 96
#define L_ANGLE2PULSE_FACTOR 160
#define L_MPUCALIB 256
#define L_B_OFFSET 274
#define L_PCA9685_FREQ 275

static_assert(sizeof(ConfigSlot) == CFG_SLOT_SIZE, "slot layout");
static_assert(sizeof(JointCal) == 66, "JointCal layout");

static uint8_t rd(uint16_t a) { return ee_read(a); }
static int16_t rd16(uint16_t a) {
  // Legacy EEPROMWriteInt stores low byte first.
  return (int16_t)(ee_read(a) | ((uint16_t)ee_read((uint16_t)(a + 1)) << 8));
}

void config_defaults(ConfigSlot& s) {
  uint8_t* p = (uint8_t*)&s;
  for (uint8_t i = 0; i < CFG_SLOT_SIZE; i++) p[i] = 0;
  for (uint8_t j = 0; j < JOINT_COUNT; j++) {
    // Servo neutral (90 deg) shifted by the model's midShift, the same as
    // the setup sketch with zero calibration offsets (Appendix C2).
    s.cal.zero_a16[j] = (int16_t)((90 + joint_dir(j) * joint_mid_shift(j)) * 16);
    s.cal.a2p[j] = 11000;  // (2500-500)/180 in integer maths = 11, x1000
  }
  s.cal.b_offset_counts = 600;  // 240/50 (= 4, integer) x 150
  s.pca9685_freq_khz = 25000;
  s.batt_mv_per_64 = 646;  // legacy: 99 ADC counts per volt -> 64000/99
  for (uint8_t i = 0; i < CFG_PARAM_COUNT; i++) s.params[i] = CFG_PARAM_DEFAULT;
  config_seal(s);
}

// Checks only; nothing is written to the image until everything passed.
static ConfigReason check_legacy(void) {
  // 1. The robot-model arrays must be exactly what this firmware believes,
  //    otherwise the stored per-joint numbers belong to a different wiring.
  for (uint8_t j = 0; j < JOINT_COUNT; j++) {
    if (rd((uint16_t)(L_PWM_PIN + j)) != joint_channel(j)) return CFG_REASON_PWM_PIN;
    if ((int8_t)rd((uint16_t)(L_MID_SHIFT + j)) != joint_mid_shift(j)) return CFG_REASON_MID_SHIFT;
    if ((int8_t)rd((uint16_t)(L_ROTATION_DIRECTION + j)) != joint_dir(j)) return CFG_REASON_DIRECTION;
  }
  uint8_t b = rd(L_B_OFFSET);
  if (b < 40 || b > 80) return CFG_REASON_B_OFFSET;
  // 2. Per driven joint: the two copies of the calibration must agree, and
  //    the slope must be plausible. (Unused joints 3-7 are not checked: the
  //    firmware never drives them, so their numbers cannot hurt.)
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++) {
    uint8_t j = joint_id(k);
    int8_t dir = joint_dir(j);
    int16_t czp = rd16((uint16_t)(L_CALIBRATED_ZERO_POSITIONS + 2 * j));
    // calib = dir * (CZP - 90 - dir * midShift): the setup sketch's
    // CZP = 90 + dir * midShift + dir * calib, solved for calib.
    int16_t diff = (int16_t)(dir * (czp - 90 - dir * joint_mid_shift(j)) - (int8_t)rd((uint16_t)(L_CALIB + j)));
    if (diff > 1 || diff < -1 || czp < -60 || czp > 240) return CFG_REASON_CALIB;
    int16_t a2p = rd16((uint16_t)(L_ANGLE2PULSE_FACTOR + 2 * j));
    if (a2p < 9000 || a2p > 13000) return CFG_REASON_A2P;
  }
  return CFG_REASON_NONE;
}

ConfigReason config_migrate(ConfigSlot& s) {
  config_defaults(s);
  ConfigReason r = check_legacy();
  if (r != CFG_REASON_NONE) return r;
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++) {
    uint8_t j = joint_id(k);
    s.cal.zero_a16[j] = (int16_t)(rd16((uint16_t)(L_CALIBRATED_ZERO_POSITIONS + 2 * j)) * 16);
    s.cal.a2p[j] = rd16((uint16_t)(L_ANGLE2PULSE_FACTOR + 2 * j));
  }
  s.cal.b_offset_counts = (uint16_t)(rd(L_B_OFFSET) * 10);
  // The PCA9685 oscillator value is only stored. Same plausibility window
  // as legacy servoSetup() (F_MIN..F_MAX), same fallback.
  uint16_t f = (uint16_t)rd16(L_PCA9685_FREQ);
  if (f >= 23000 && f <= 27500) s.pca9685_freq_khz = f;
  for (uint8_t i = 0; i < 6; i++) s.imu_offset[i] = rd16((uint16_t)(L_MPUCALIB + 2 * i));
  config_seal(s);
  return CFG_REASON_NONE;
}

uint16_t config_slot_crc(const ConfigSlot& s) {
  return kilo_crc16((const uint8_t*)&s, CFG_SLOT_SIZE - 2);
}

void config_seal(ConfigSlot& s) {
  s.magic[0] = 'K';
  s.magic[1] = 'L';
  s.version = CFG_VERSION;
  s.crc = config_slot_crc(s);
}

KILO_NOINLINE bool config_slot_valid(const ConfigSlot& s) {
  if (s.magic[0] != 'K' || s.magic[1] != 'L') return false;
  if (s.version < CFG_VERSION_OLDEST || s.version > CFG_VERSION) return false;
  if (s.crc != config_slot_crc(s)) return false;
  // A CRC-valid slot is trusted, but only within the ranges the transfer
  // function's integer maths is written for (joints.cpp).
  if (s.cal.b_offset_counts > 2550) return false;
  for (uint8_t j = 0; j < JOINT_COUNT; j++) {
    if (s.cal.zero_a16[j] > 3840 || s.cal.zero_a16[j] < -960) return false;  // -60..240 deg
    if (s.cal.a2p[j] < 1000 || s.cal.a2p[j] > 20000) return false;
  }
  return true;
}

// Read a slot into img; true if it is valid.
static bool load_slot(ConfigSlot& img, uint8_t slot) {
  uint16_t base = config_slot_addr(slot);
  uint8_t* p = (uint8_t*)&img;
  for (uint8_t i = 0; i < CFG_SLOT_SIZE; i++) p[i] = ee_read((uint16_t)(base + i));
  if (!config_slot_valid(img)) return false;
  // A v1 slot: its reserved bytes (now params 16..19) mean "default".
  if (img.version < CFG_VERSION)
    for (uint8_t i = CFG_PARAM_COUNT_V1; i < CFG_PARAM_COUNT; i++) img.params[i] = CFG_PARAM_DEFAULT;
  return true;
}

void config_boot(Config& c) {
  c.writing = 0;
  c.saved = 0;
  c.reason = CFG_REASON_NONE;
  c.state = CFG_OK;
  // Both slots valid: keep the newer (wrap-safe sequence comparison).
  // One valid: keep it. The image doubles as the scratch buffer, so slot A
  // may be read twice; that costs ~0.1 ms once per boot and saves 128 B.
  bool a = load_slot(c.img, 0);
  uint8_t seq_a = c.img.seq;
  bool b = load_slot(c.img, 1);
  if (a && (!b || (int8_t)(uint8_t)(c.img.seq - seq_a) <= 0)) {
    load_slot(c.img, 0);
    c.newest = 0;
    return;
  }
  if (b) {
    c.newest = 1;
    return;
  }
  c.newest = 0xFF;
  c.reason = config_migrate(c.img);
  if (c.reason == CFG_REASON_NONE) {
    c.state = CFG_MIGRATED;
    c.img.seq = 0;  // config_save makes it 1
    config_save(c);
  } else {
    // Defaults are not persisted: the next boot retries the migration, and
    // the Pi's calibration (CAL_SAVE) is what writes the first slot.
    c.state = CFG_DEFAULTED;
  }
}

bool config_save(Config& c) {
  if (c.writing) return false;
  c.img.seq++;
  config_seal(c.img);
  // Target the older slot so the newest valid copy survives a torn write.
  c.target = (c.newest == 0) ? 1 : 0;
  c.idx = 0;
  c.writing = 1;
  return true;
}

void config_writer_step(Config& c) {
  if (!c.writing || !ee_ready()) return;
  uint16_t base = config_slot_addr(c.target);
  const uint8_t* p = (const uint8_t*)&c.img;
  // Skip bytes that already hold the right value (reading is ~1 us each and
  // saves EEPROM wear); write the first one that differs, then return. The
  // CRC is the last two bytes, so it is necessarily written last.
  while (c.idx < CFG_SLOT_SIZE) {
    uint16_t a = (uint16_t)(base + c.idx);
    uint8_t v = p[c.idx++];
    if (ee_read(a) != v) {
      ee_write(a, v);
      return;
    }
  }
  // Every byte has been compared and the last write has been started. The
  // slot is complete once that write finishes; wait for ready() once more.
  c.writing = 0;
  c.newest = c.target;
  c.saved = 1;
  // A calibration written by the Pi replaces the model defaults: STAND is
  // no longer refused as UNCALIBRATED.
  if (c.state == CFG_DEFAULTED) c.state = CFG_OK;
}

void config_set_calib_a16(ConfigSlot& s, uint8_t j, int16_t calib_a16) {
  int8_t dir = joint_dir(j);
  s.cal.zero_a16[j] = (int16_t)(90 * 16 + dir * joint_mid_shift(j) * 16 + dir * calib_a16);
}

void config_revert(Config& c) {
  if (c.newest != 0xFF && load_slot(c.img, c.newest)) return;
  config_migrate(c.img);
}

int16_t config_calib_a16(const ConfigSlot& s, uint8_t j) {
  int8_t dir = joint_dir(j);
  return (int16_t)(dir * (s.cal.zero_a16[j] - 90 * 16 - dir * joint_mid_shift(j) * 16));
}
