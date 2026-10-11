// Build-time feature switches shared by app/ and hal/ (the shared PCINT2 ISR
// needs to know which of its users are built). Side peripherals that §7
// schedules for P1 are on by default; -DKILO_ULTRASONIC=0 / -DKILO_BUZZER=0 /
// -DKILO_IR=0 drop them from the build if flash gets tight.
#ifndef KILO_FEATURES_H
#define KILO_FEATURES_H

#ifndef KILO_ULTRASONIC
#define KILO_ULTRASONIC 1
#endif
#ifndef KILO_BUZZER
#define KILO_BUZZER 1
#endif
// The IR remote receiver on D4 (NEC; decoded on the body, relayed to the
// mind as EVENT IR_KEY). Owner's request 2026-10-11.
#ifndef KILO_IR
#define KILO_IR 1
#endif

#endif
