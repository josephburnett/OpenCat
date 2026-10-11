#include "posture.h"
#include "joints.h"
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
  int8_t roll, pitch;  // rc rows (below): dur and hold, ticks
};

// The legacy recovery behaviour `rc` (src/InstinctNybble.h; legacy
// reaction.h plays it whenever |roll| > 90 deg, mirrored at random) is rows
// RC_ROW.. of the same table, converted the way the mind's importer converts
// behaviours (mind/kilo/tricks/legacy.py: dur = ceil(round(maxDelta /
// (speed/4)) * 5.5 ms / 20 ms) from `up`, hold = |delay| * 50 ms / 20 ms):
// legacy rows {30,-74,34, 30,30,-30,-30, 30,30,-59,-31} speed 8 delay 12,
// {62,0,-20, -3,-99,115,0, 81,9,-9,-71} speed 32 delay 10, {0,17,0, up}
// speed 8. Angles verbatim in canonical order (RB shoulder 115 is clamped to
// its 100 deg limit by the safety stage, as legacy clamped it).
#define RC_ROW 4
static const PostureRow POSTURES[] PROGMEM = {
    // rest
    {{-30, -80, -45, 60, 60, -60, -60, -45, -45, 45, 45}, 0, 0},
    // sit
    {{10, -20, -60, 30, 30, -90, -90, 60, 60, 45, 45}, 0, -30},
    // up (STAND)
    {{0, 0, 0, 30, 30, -30, -30, 30, 30, -30, -30}, 0, 0},
    // calib / zero (identical in InstinctNybble.h)
    {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0},
    // rc frames 0-2: angles, dur, hold
    {{30, -74, 34, 30, 30, -30, -30, 30, 30, -59, -31}, 11, 30},
    {{62, 0, -20, -3, -99, 115, 0, 81, 9, -9, -71}, 5, 25},
    {{0, 17, 0, 30, 30, -30, -30, 30, 30, -30, -30}, 18, 0},
};

uint8_t rc_dur(uint8_t i) { return PGM_U8(&POSTURES[RC_ROW + i].roll); }
uint8_t rc_hold(uint8_t i) { return PGM_U8(&POSTURES[RC_ROW + i].pitch); }

// Posture id -> table row, or -1 (LIMP and STANCE have no angles). A
// table in flash rather than a switch, which avr-gcc turns into a table in RAM.
static const int8_t ROW[POSTURE_COUNT] PROGMEM = {-1, 0, 1, 2, -1, 3, 3};

static int8_t row_of(uint8_t id) { return id < POSTURE_COUNT ? PGM_I8(&ROW[id]) : -1; }

bool posture_has_angles(uint8_t id) { return row_of(id) >= 0; }

KILO_NOINLINE int16_t posture_angle(uint8_t id, uint8_t k) {
  int8_t r = row_of(id);
  if ((id & 0xF0u) == POSTURE_RC) {
    r = (int8_t)(RC_ROW + (id & 7u));
    if (id & POSTURE_RC_MIRROR) {  // L/R legs swapped, pan and tail negated
      int8_t v = PGM_I8(&POSTURES[r].ang[joint_mirror_k(k)]);
      return (int16_t)((k == 0 || k == 2) ? -v * 16 : v * 16);
    }
  }
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
