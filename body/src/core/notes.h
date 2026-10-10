// notes: the buzzer's note queue (BEEP goal, reflex chirps). The tick
// advances it; hal/buzzer only toggles the pin at the current frequency.
//
// Note numbers follow legacy beep(): 0 is a rest, each step is a semitone,
// note 12 is 2092 Hz. Notes above 23 (3950 Hz) are clamped: the buzzer ISR
// fires at twice the note frequency, and above ~4 kHz its CPU share and the
// jitter it adds to UART and ultrasonic timing stop being negligible.
#ifndef KILO_CORE_NOTES_H
#define KILO_CORE_NOTES_H
#include <stdint.h>

#define NOTES_LEN 8
#define NOTE_MIN (-60)
#define NOTE_MAX_BODY 23

struct Notes {
  int8_t note[NOTES_LEN];
  uint8_t dur10[NOTES_LEN];  // duration in 10 ms units
  uint8_t head, count;
  uint16_t left_ms;  // time left on the note at the head (0 = not started)
  uint16_t hz;       // what the buzzer is playing now
};

void notes_init(Notes& n);
bool notes_push(Notes& n, int8_t note, uint8_t dur10);  // false if full
void notes_clear(Notes& n);
uint16_t note_hz(int8_t note);  // 0 for a rest

// Advance by `ms`. Returns true when the buzzer frequency must change; the
// new frequency (0 = silent) is in n.hz.
bool notes_tick(Notes& n, uint8_t ms);

#endif
