// params: the live tuning parameters (PARAM_GET / PARAM_SET, doc §4).
//
// Ids, defaults and bounds come from protocol/spec.toml [[params]] through
// the generated header (KILO_PARAM_*), so the body, the mind and the doc
// share one table. The live values are plain RAM (g_param); a persisted
// value lives in the config slot's params[] (0x7FFF = "use the default")
// and is applied at boot. Setting a value without persisting it changes
// only RAM, so a later CAL_SAVE never persists it by accident.
#ifndef KILO_CORE_PARAMS_H
#define KILO_CORE_PARAMS_H
#include <stdint.h>
#include "../proto/kilo_protocol.h"
#include "config.h"

static_assert(KILO_PARAM_COUNT <= CFG_PARAM_COUNT, "every param needs a persisted slot");
static_assert(KILO_PARAM_SLOTS == CFG_PARAM_COUNT, "spec param_slots = config slot params[]");

extern int16_t g_param[KILO_PARAM_COUNT];

#define PARAM(name) (g_param[KILO_PARAM_##name])

// Defaults, then every persisted value that is within bounds.
void params_load(const ConfigSlot& s);
// {min, max, default} of a param (id < KILO_PARAM_COUNT).
void param_info(uint8_t id, int16_t out[3]);
// True if id exists and min <= v <= max.
bool param_ok(uint8_t id, int16_t v);

// Derived values in ticks, for the reflexes.
inline uint16_t param_ms_ticks(uint8_t id) { return (uint16_t)((uint16_t)g_param[id] / 20u); }

#endif
