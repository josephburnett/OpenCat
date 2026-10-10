// buzzer: square wave on D5 from a Timer2 compare ISR (doc §3a).
//
// Why not tone(): it blocks callers in places, pulls in the Arduino core,
// and is the only user of some timer code. Here Timer2 runs in CTC mode at
// twice the note frequency and its ISR toggles D5 by writing PIND (one
// instruction, atomic with respect to everything else on PORTD).
#ifndef KILO_HAL_BUZZER_H
#define KILO_HAL_BUZZER_H
#include <stdint.h>

void buzzer_init(void);
void buzzer_set(uint16_t hz);  // 0 = silent

#endif
