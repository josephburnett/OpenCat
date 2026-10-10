#include "notes.h"
#include "pgm.h"

// Notes 12..23, the legacy notePitch[] values (int(1046.5 * 1.05946^n)).
// Lower octaves are these halved: within 1 Hz of the legacy table, and 24 B
// instead of 264 B of flash.
static const uint16_t TOP_OCTAVE[12] PROGMEM = {2092, 2217, 2349, 2488, 2636, 2793,
                                                2959, 3135, 3322, 3519, 3729, 3950};

uint16_t note_hz(int8_t note) {
  if (note == 0) return 0;
  int16_t m = note < NOTE_MIN ? NOTE_MIN : (note > NOTE_MAX_BODY ? NOTE_MAX_BODY : note);
  uint8_t shift = 0;
  while (m < 12) {
    m = (int16_t)(m + 12);
    shift++;
  }
  return (uint16_t)(PGM_U16(&TOP_OCTAVE[m - 12]) >> shift);
}

void notes_init(Notes& n) {
  n.head = n.count = 0;
  n.left_ms = 0;
  n.hz = 0;
}

void notes_clear(Notes& n) {
  n.head = n.count = 0;
  n.left_ms = 0;
}

KILO_NOINLINE bool notes_push(Notes& n, int8_t note, uint8_t dur10) {
  if (n.count >= NOTES_LEN || dur10 == 0) return false;
  uint8_t i = (uint8_t)((n.head + n.count) % NOTES_LEN);
  n.note[i] = note;
  n.dur10[i] = dur10;
  n.count++;
  return true;
}

bool notes_tick(Notes& n, uint8_t ms) {
  uint16_t want = 0;
  if (n.count) {
    if (n.left_ms == 0) n.left_ms = (uint16_t)(n.dur10[n.head] * 10);  // start it
    want = note_hz(n.note[n.head]);
    if (n.left_ms > ms) {
      n.left_ms = (uint16_t)(n.left_ms - ms);
    } else {
      // It ends within this tick: it still sounds for the whole tick (20 ms
      // is the time resolution of everything the tick does), then the next
      // note starts on the next tick.
      n.head = (uint8_t)((n.head + 1) % NOTES_LEN);
      n.count--;
      n.left_ms = 0;
    }
  }
  if (want == n.hz) return false;
  n.hz = want;
  return true;
}
