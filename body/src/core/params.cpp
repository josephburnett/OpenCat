#include "params.h"
#include "pgm.h"

int16_t g_param[KILO_PARAM_COUNT] = KILO_PARAM_DEFAULTS_INIT;

static const int16_t PARAM_TABLE[KILO_PARAM_COUNT][3] PROGMEM = KILO_PARAM_TABLE_INIT;

void param_info(uint8_t id, int16_t out[3]) {
  for (uint8_t i = 0; i < 3; i++) out[i] = PGM_I16(&PARAM_TABLE[id][i]);
}

bool param_ok(uint8_t id, int16_t v) {
  if (id >= KILO_PARAM_COUNT) return false;
  int16_t t[3];
  param_info(id, t);
  return v >= t[0] && v <= t[1];
}

void params_load(const ConfigSlot& s) {
  for (uint8_t id = 0; id < KILO_PARAM_COUNT; id++) {
    int16_t t[3];
    param_info(id, t);
    int16_t v = s.params[id];
    g_param[id] = (v != CFG_PARAM_DEFAULT && v >= t[0] && v <= t[1]) ? v : t[2];
  }
}
