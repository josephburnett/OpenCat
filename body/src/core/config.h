// config: per-robot configuration in two EEPROM slots, with migration from
// the legacy OpenCat layout (doc §3b, Appendix C4).
//
// Why two slots: an EEPROM byte takes 3.4 ms to write, so a 128 B save takes
// up to half a second and power can fail half way. Each save goes into the
// OLDER slot, data first and CRC last, so a torn write leaves that slot
// invalid and boot falls back to the other one, which is untouched.
//
// Why never write the legacy layout: so `kilo flash --legacy` finds the old
// firmware's EEPROM exactly as it left it. The writer can only address the
// two slots (704..959), and hal/eeprom refuses anything else as well.
#ifndef KILO_CORE_CONFIG_H
#define KILO_CORE_CONFIG_H
#include <stdint.h>
#include "joints.h"

#define CFG_SLOT_A 704
#define CFG_SLOT_B 832
#define CFG_SLOT_SIZE 128
#define CFG_REGION_START CFG_SLOT_A
#define CFG_REGION_END (CFG_SLOT_B + CFG_SLOT_SIZE)  // exclusive: 960
// Version 2 (proto 1.4, P2): the 8 reserved bytes became params 16..19. A
// version 1 slot is still accepted (those params read as default) and is
// rewritten as version 2 on the next save, so a saved calibration survives
// the upgrade and `kilo flash --legacy` still finds its EEPROM untouched.
#define CFG_VERSION 2
#define CFG_VERSION_OLDEST 1
#define CFG_PARAM_COUNT 20
#define CFG_PARAM_COUNT_V1 16
#define CFG_PARAM_DEFAULT 0x7FFF

// The RAM image is the slot byte for byte (little-endian on both AVR and
// x86, every field naturally aligned), so saving is a plain byte copy.
struct ConfigSlot {
  uint8_t magic[2];  // 'K','L'
  uint8_t version;
  uint8_t seq;  // serial-number arithmetic: newer if (int8_t)(a - b) > 0
  JointCal cal;  // offsets 4..69: zero_a16[16], a2p[16], b_offset_counts
  uint16_t pca9685_freq_khz;  // legacy PCA9685_FREQ, stored, unused (see hal/pca9685)
  int16_t imu_offset[6];      // accel xyz, gyro xyz (MPU offset registers)
  uint16_t batt_mv_per_64;    // battery scale: mV per 64 ADC counts
  int16_t params[CFG_PARAM_COUNT];  // persisted params, 0x7FFF = default (16..19 were reserved bytes in v1)
  uint16_t crc;  // CRC-16/CCITT-FALSE over bytes 0..125
};

enum ConfigState : uint8_t { CFG_OK = 0, CFG_MIGRATED = 1, CFG_DEFAULTED = 2 };

// Why migration fell back to defaults (CONFIG_DEFAULTED's arg).
enum ConfigReason : uint8_t {
  CFG_REASON_NONE = 0,
  CFG_REASON_PWM_PIN = 1,     // PWM_PIN[] != this robot model's channels
  CFG_REASON_MID_SHIFT = 2,   // MID_SHIFT[] != model
  CFG_REASON_DIRECTION = 3,   // ROTATION_DIRECTION[] != model
  CFG_REASON_CALIB = 4,       // CALIB vs CALIBRATED_ZERO_POSITIONS disagree > 1 deg
  CFG_REASON_A2P = 5,         // ANGLE2PULSE_FACTOR outside 9000..13000
  CFG_REASON_B_OFFSET = 6,    // B_OFFSET x 10 outside 400..800
};

// EEPROM access. Implemented by hal/eeprom.cpp on the AVR and by a fake
// EEPROM in the host tests (a link-time seam: plain calls, no pointers).
uint8_t ee_read(uint16_t addr);           // only when ee_ready()
bool ee_ready(void);                      // no write in progress
void ee_write(uint16_t addr, uint8_t v);  // starts a write, returns at once

struct Config {
  ConfigSlot img;      // the live configuration (RAM image of a slot)
  ConfigState state;   // reported in HELLO.config_state
  ConfigReason reason; // when DEFAULTED
  uint8_t newest;      // slot holding img: 0 = A, 1 = B, 0xFF = none
  // background writer
  uint8_t writing;     // 1 while a save is in progress
  uint8_t target;      // slot being written
  uint8_t idx;         // next byte to compare/write
  uint8_t saved;       // set when a save completes (-> EVENT EEPROM_SAVED)
};

// Boot: load the newer valid slot. If neither is valid, migrate from the
// legacy layout and, if that passes, queue a write of slot A; otherwise fill
// model defaults and leave both slots alone (state DEFAULTED). Only reads
// (blocking reads are fine at boot, before the tick runs).
void config_boot(Config& c);

// Migration alone (exposed for tests). Fills img from the legacy EEPROM and
// returns CFG_REASON_NONE, or fills model defaults and returns the reason.
ConfigReason config_migrate(ConfigSlot& img);
void config_defaults(ConfigSlot& img);

bool config_slot_valid(const ConfigSlot& s);
uint16_t config_slot_crc(const ConfigSlot& s);  // CRC-16/CCITT-FALSE over bytes 0..125
void config_seal(ConfigSlot& s);  // magic, version, CRC

// Queue a save of img into the older slot (CAL_SAVE, PARAM_SET persist).
// Returns false if a save is already running: the caller answers
// EEPROM_BUSY. Do not modify img while config_busy().
bool config_save(Config& c);
inline bool config_busy(const Config& c) { return c.writing != 0; }

// Background step: compares bytes and starts at most ONE EEPROM byte write
// per call, so it never blocks the tick. Call from background().
void config_writer_step(Config& c);

// "calib" as the protocol exposes it: dir*(zero - 90 - dir*midShift), a16.
int16_t config_calib_a16(const ConfigSlot& s, uint8_t j);
// CAL_SET: the inverse, zero = 90 + dir*midShift + dir*calib.
void config_set_calib_a16(ConfigSlot& s, uint8_t j, int16_t calib_a16);

// CAL_REVERT: reload the image from the newest valid slot, or, if no slot
// has been written yet, redo the legacy migration (model defaults if that
// fails). Not while config_busy().
void config_revert(Config& c);

inline uint16_t config_slot_addr(uint8_t slot) { return slot ? CFG_SLOT_B : CFG_SLOT_A; }

#endif
