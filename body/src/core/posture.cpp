#include "posture.h"
#include "pgm.h"

// Whole degrees, canonical joint order [0,1,2,8,9,10,11,12,13,14,15]
// (pan, tilt, tail, 4 shoulders LF RF RB LB, 4 knees LF RF RB LB), then the
// expected roll and pitch: the InstinctNybble.h header bytes 1 and 2, kept
// verbatim in the legacy convention (posture_pitch/roll convert them).
// Joints 3-7 of the legacy rows are dropped (never driven).
// Note: legacy "sit" asks RB/LB shoulder for -90 deg, below their -80 deg
// limit; the safety stage clamps it to -80, exactly as legacy calibratedPWM()
// clamped it against the EEPROM ANGLE_LIMIT table.
struct PostureRow {
  int8_t ang[11];
  int8_t roll, pitch;
};

static const PostureRow POSTURES[] PROGMEM = {
    // rest
    {{-30, -80, -45, 60, 60, -60, -60, -45, -45, 45, 45}, 0, 0},
    // sit
    {{10, -20, -60, 30, 30, -90, -90, 60, 60, 45, 45}, 0, -30},
    // up (STAND)
    {{0, 0, 0, 30, 30, -30, -30, 30, 30, -30, -30}, 0, 0},
    // calib / zero (identical in InstinctNybble.h)
    {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0},
};

// Posture id -> table row, or -1 (LIMP and STANCE have no angles). A
// table in flash rather than a switch, which avr-gcc turns into a table in RAM.
static const int8_t ROW[POSTURE_COUNT] PROGMEM = {-1, 0, 1, 2, -1, 3, 3};

static int8_t row_of(uint8_t id) { return id < POSTURE_COUNT ? PGM_I8(&ROW[id]) : -1; }

bool posture_has_angles(uint8_t id) { return row_of(id) >= 0; }

KILO_NOINLINE int16_t posture_angle(uint8_t id, uint8_t k) {
  int8_t r = row_of(id);
  return r < 0 ? 0 : (int16_t)(PGM_I8(&POSTURES[r].ang[k]) * 16);
}

int16_t posture_pitch(uint8_t id) {
  int8_t r = row_of(id);
  return r < 0 ? 0 : (int16_t)(PGM_I8(&POSTURES[r].pitch) * 16);
}

int16_t posture_roll(uint8_t id) {
  int8_t r = row_of(id);
  return r < 0 ? 0 : (int16_t)(PGM_I8(&POSTURES[r].roll) * 16);
}
