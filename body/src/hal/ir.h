// ir: the NyBoard's on-board IR receiver on D4 (PD4 = PCINT20), NEC remote.
//
// No IRremote library (it took Timer2 and ~2 KB): D4 shares the PCINT2
// vector with the ultrasonic (hal/pcint2.cpp). Every falling edge on D4 is
// stamped with clock_micros() and fed to the pure decoder core/ir_nec in the
// ISR itself (a few 32-bit compares and a shift, no loops), so nothing is
// buffered; background() collects a decoded key with ir_poll() and pushes
// EVENT IR_KEY. The mind maps keys to actions (doc §0, §3a).
#ifndef KILO_HAL_IR_H
#define KILO_HAL_IR_H
#include <stdint.h>

void ir_init(void);
// Called by the shared PCINT2 ISR on a falling edge of D4.
void ir_edge(uint32_t t_us);
// True once per decoded key: *key = NEC command | 0x100 for a held repeat.
bool ir_poll(uint16_t* key);

#endif
