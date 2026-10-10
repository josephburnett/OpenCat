#define LENGTH_FACTOR 2
#define BASE_PITCH 1046.50
// tone: pause,1,  2,  3,  4,  5,  6,  7,  1,  2, 3, 4
// code: 0,    8, 10, 12, 13, 15, 17, 19, 20, 22,24,25

//the frequency of note n used to be computed as (unsigned int)(BASE_PITCH * pow(1.05946, n)),
//which dragged pow() and the floating point library into the sketch. the table below holds exactly
//the same integers that tone() received before, i.e. int(1046.50 * 1.05946 ** n) for n = 0..NOTE_MAX.
//NOTE_MAX covers every note the firmware itself can ask for: the melodies use at most 15,
//the other internal beeps at most 20, and the documented note range of the 'b'/'B' tokens ends at 25.
//note 0 is a pause (unchanged). a note above NOTE_MAX (only reachable by user input through
//'b'/'B') is clamped to NOTE_MAX = 6644 Hz, and a negative note is treated as a pause.
#define NOTE_MAX 32
const uint16_t notePitch[NOTE_MAX + 1] PROGMEM = {
  1046, 1108, 1174, 1244, 1318, 1396, 1479, 1567, 1661,
  1759, 1864, 1975, 2092, 2217, 2349, 2488, 2636, 2793,
  2959, 3135, 3322, 3519, 3729, 3950, 4185, 4434, 4698,
  4977, 5273, 5587, 5919, 6271, 6644
};

byte melodyNormalBoot[] = {8, 13, 10, 13, 8, 5, 8, 3, 5, 8,  //tone
                           4,  4,  8,  8, 4, 8, 8, 8, 8, 4,  //relative duration, 8 means 1/8 note length
                          };
byte melodyInit[] = {15, 12, 15, 12, 15, 12, 8, 10, 13, 12, 10, 15,//tone
                     4,  4,  4,  4,  4,  4,  2,  4, 4,  4,  4,  2,//relative duration, 8 means 1/8 note length
                    };
byte melodyLowBattery[] = {15, 11, 13, 10, 11, //tone
                           4,  4,  4,  4,  4, //relative duration, 8 means 1/8 note length
                          };

void beep(int8_t note, float duration = 50, int pause = 0, byte repeat = 1 ) {
  for (byte r = 0; r < repeat; r++) {
    if (note > 0)
      tone(BUZZER, pgm_read_word(&notePitch[note > NOTE_MAX ? NOTE_MAX : note]));//tone(pin, frequency, duration) the duration doesn't work
    else
      delay(duration);
    delay(duration);
    noTone(BUZZER);
    delay(pause);
  }
}

void playMelody(int start) {
  byte len = (byte)EEPROM.read(start) / 2;
  for (int i = 0; i < len; i++)
    beep(EEPROM.read(start - 1 - i), 1000 / EEPROM.read(start - 1 - len - i));
}

void playTone(uint16_t tone1, uint16_t duration) {
  if (tone1 < 50 || tone1 > 15000) return; // these do not play on a piezo
  for (long i = 0; i < duration * 1000L; i += tone1 * 2) {
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(tone1);
    digitalWrite(BUZZER, LOW);
    delayMicroseconds(tone1);
  }
}

void meow(byte repeat = 2, byte duration = 10, byte startF = 210 + random() % 10,  byte endF = 220 + random() % 10) { // Bird chirp
  for (byte r = 0; r < repeat; r++)
    for (byte i = startF; i < endF; i++) {
      playTone(i, duration);
    }
}
