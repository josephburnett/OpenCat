// pcint2: the one PCINT2 vector (PORTD pin changes), shared by the
// ultrasonic echo (D6 = PCINT22, enabled only while a ping is timed) and the
// IR receiver (D4 = PCINT20, always enabled). Each user sets and clears only
// its own PCMSK2 bit (doc §3a).
#ifndef KILO_HAL_PCINT2_H
#define KILO_HAL_PCINT2_H

// Enable the group (boot code; the pins are chosen with PCMSK2).
void pcint2_init(void);

#endif
