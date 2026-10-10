#define LENGTH_FACTOR 2
#define BASE_PITCH 1046.50
// tone: pause,1,  2,  3,  4,  5,  6,  7,  1,  2, 3, 4
// code: 0,    8, 10, 12, 13, 15, 17, 19, 20, 22,24,25

//the frequency of note n used to be computed as (unsigned int)(BASE_PITCH * pow(1.05946, n)),
//which dragged pow() and the floating point library into the sketch. the table below holds exactly
//the same integers that tone() received before, i.e. int(1046.50f * 1.05946f ** n) in 32-bit float
//(avr-gcc's double is 32-bit, so the old expression was float math), one row per octave.
//the range covers every note the old code could actually sound:
// - NOTE_MIN = -60 (32 Hz) is the lowest note tone() can still render on this board. the ATmega328P
//   core only drives tone() from the 8-bit timer2, so ocr = F_CPU/f/2/1024 - 1 must fit in OCR2A;
//   that fails below 31 Hz, i.e. from note -61 (30 Hz) downwards, where the old code wrote a
//   truncated ocr and sounded an unrelated pitch.
// - NOTE_MAX = 71 (63204 Hz) is the highest note the old code could express: note 72 overflowed
//   tone()'s 16-bit unsigned int and wrapped to 1426 Hz.
//notes beyond either end (only reachable by user input through 'b'/'B') are clamped to the bound.
//note 0 is a pause, and it is the only silent note - negative notes sound below BASE_PITCH, as
//they did before, e.g. note -5 is 784 Hz.
#define NOTE_MIN (-60)
#define NOTE_MAX 71
const uint16_t notePitch[NOTE_MAX - NOTE_MIN + 1] PROGMEM = {
     32,    34,    36,    38,    41,    43,    46,    49,    51,    55,    58,    61,
     65,    69,    73,    77,    82,    87,    92,    98,   103,   110,   116,   123,
    130,   138,   146,   155,   164,   174,   185,   196,   207,   220,   233,   246,
    261,   277,   293,   311,   329,   349,   370,   392,   415,   440,   466,   493,
    523,   554,   587,   622,   659,   698,   740,   784,   830,   880,   932,   987,
   1046,  1108,  1174,  1244,  1318,  1396,  1479,  1567,  1661,  1759,  1864,  1975,
   2092,  2217,  2349,  2488,  2636,  2793,  2959,  3135,  3322,  3519,  3729,  3950,
   4185,  4434,  4698,  4977,  5273,  5587,  5919,  6271,  6644,  7039,  7457,  7901,
   8371,  8868,  9396,  9954, 10546, 11173, 11838, 12542, 13288, 14078, 14915, 15802,
  16741, 17737, 18791, 19909, 21092, 22347, 23675, 25083, 26575, 28155, 29829, 31603,
  33482, 35473, 37582, 39816, 42184, 44692, 47350, 50165, 53148, 56308, 59656, 63204,
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
    if (note) {
      int8_t clamped = note < NOTE_MIN ? NOTE_MIN : (note > NOTE_MAX ? NOTE_MAX : note);
      tone(BUZZER, pgm_read_word(&notePitch[clamped - NOTE_MIN]));//tone(pin, frequency, duration) the duration doesn't work
    } else
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
