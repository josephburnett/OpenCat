// fx: the small fixed-point maths library (no floats anywhere on the body).
//
// Angles are a16 (1/16 degree, int16_t): 90 deg = 1440, 180 deg = 2880,
// 360 deg = 5760. Unit-scale values (sin, cos, ease) are Q14: 1.0 = 16384.
//
// Why fixed point: the ATmega328P has no FPU. avr-gcc's soft float costs
// about 1.6 KB of flash and tens of microseconds per operation, and float
// rounding differs between the AVR and the host, which would make the host
// tests less trustworthy. Integer code gives identical results on both.
#ifndef KILO_CORE_FX_H
#define KILO_CORE_FX_H
#include <stdint.h>

#define A16_PER_DEG 16
#define A16_90 1440
#define A16_180 2880
#define A16_360 5760
#define Q14_ONE 16384

// sin and cos of any a16 angle, Q14. Max error vs libm about 4e-5.
int16_t fx_sin(int16_t a16);
int16_t fx_cos(int16_t a16);

// atan2(y, x) in a16, range (-2880, 2880]. Accepts any int32 inputs: both
// are first shifted right by the same amount until they fit in 15 bits
// (atan2 only depends on the ratio, so a shared shift changes nothing),
// then a 65-entry table is interpolated. atan2(0, 0) = 0.
int16_t fx_atan2(int32_t y, int32_t x);

// floor(sqrt(n)), exact, bitwise (no division), 16 iterations.
uint16_t fx_isqrt32(uint32_t n);

// Cosine easing (1 - cos(pi*i/n)) / 2 = sin^2(90deg * i/n), Q14.
// 0 at i = 0, 16384 at i >= n: zero velocity at both ends of a move.
// (u16 so the keyframe player can ease speed-scaled segments.)
uint16_t fx_ease_cos(uint16_t i, uint16_t n);

// Host-only operation counters (test_feet.cpp's per-tick cost estimate,
// doc §3a). Nothing on the AVR.
#ifndef __AVR__
struct FxOps {
  uint32_t sin, atan2, isqrt;
};
extern FxOps g_fx_ops;
#define FX_OP(f) (g_fx_ops.f++)
#else
#define FX_OP(f) ((void)0)
#endif

#endif
