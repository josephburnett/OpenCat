#include "ir/IRremote.h"

//#include <IRremote.h>
//The included library is identical to the IRremote library by shirriff, version 2.6.1
//Source: https://github.com/Arduino-IRremote/Arduino-IRremote
//Here, we include the decoding functions in our folder only to make it more convenient for newbie users
//All rights belong to the original author, and we follow the MIT license.
//You no longer need to modify ~/Documents/Arduino/libraries/src/IRremote/IRremote.h as mentioned in our old manual.

IRrecv irrecv(IR_PIN);
decode_results results;
//abbreviation   //gait/posture/function names
#define K00 "d"  //rest and shutdown all servos
#define K01 "F"  //forward
#define K02 "g"  //turn off gyro feedback to boost speed

#define K10 "L"   //left
#define K11 "up"  //neutral stand posture
#define K12 "R"   //right

#define K20 "p"  //pause motion and shut off all servos
#define K21 "B"  //backward
#define K22 "c"  //calibration mode with IMU turned off

#define K30 "vt"  //stepping
#define K31 "cr"  //crawl
#define K32 "wk"  //walk

#define K40 "tr"   //trot
#define K41 "sit"  //sit
#define K42 "str"  //stretch

#define K50 "hi"   //greeting
#define K51 "pu"   //push up
#define K52 "pee"  //standng with three legs

#define K60 "wsf"     //wash face
#define K61 "buttUp"  //butt up

//#define K62 "z"  //turn on/off the random behaviors
// #define K62 "zero"  //call your customized Newbility saved to PROGMEM

#define K62 "T"    //call the last skill data received from the serial port

#define SHORT_ENCODING  // activating this line will use a shorter encoding of the HEX values

// takes action based on IR code received, describing Remote IR codes.
// returns a pointer to a PROGMEM string, so it has to be read with strcpy_P().
// Arduino String was used here before. it pulled malloc/realloc/free into the sketch.
const char *translateIR() {
#ifndef SHORT_ENCODING
  switch (results.value) {
    //IR signal    key on IR remote           //key mapping
    case 0xFFA25D: /*PTLF(" CH-");   */ return (PSTR(K00));
    case 0xFF629D: /*PTLF(" CH");  */ return (PSTR(K01));
    case 0xFFE21D: /*PTLF(" CH+"); */ return (PSTR(K02));

    case 0xFF22DD: /*PTLF(" |<<"); */ return (PSTR(K10));
    case 0xFF02FD: /*PTLF(" >>|"); */ return (PSTR(K11));
    case 0xFFC23D: /*PTLF(" >||"); */ return (PSTR(K12));

    case 0xFFE01F: /*PTLF(" -");   */ return (PSTR(K20));
    case 0xFFA857: /*PTLF(" +");  */ return (PSTR(K21));
    case 0xFF906F: /*PTLF(" EQ"); */ return (PSTR(K22));

    case 0xFF6897: /*PTLF(" 0");  */ return (PSTR(K30));
    case 0xFF9867: /*PTLF(" 100+"); */ return (PSTR(K31));
    case 0xFFB04F: /*PTLF(" 200+"); */ return (PSTR(K32));

    case 0xFF30CF: /*PTLF(" 1");  */ return (PSTR(K40));
    case 0xFF18E7: /*PTLF(" 2");  */ return (PSTR(K41));
    case 0xFF7A85: /*PTLF(" 3");  */ return (PSTR(K42));

    case 0xFF10EF: /*PTLF(" 4");  */ return (PSTR(K50));
    case 0xFF38C7: /*PTLF(" 5");  */ return (PSTR(K51));
    case 0xFF5AA5: /*PTLF(" 6");  */ return (PSTR(K52));

    case 0xFF42BD: /*PTLF(" 7");  */ return (PSTR(K60));
    case 0xFF4AB5: /*PTLF(" 8");  */ return (PSTR(K61));
    case 0xFF52AD: /*PTLF(" 9");  */ return (PSTR(K62));

    case 0xFFFFFFFF: return PSTR("");  //PTL(" REPEAT");
#else
  uint8_t trimmed = (results.value >> 8);
  switch (trimmed) {
    //IR signal    key on IR remote           //key mapping
    case 0xA2: /*PTLF(" CH-");   */ return (PSTR(K00));
    case 0x62: /*PTLF(" CH");  */ return (PSTR(K01));
    case 0xE2: /*PTLF(" CH+"); */ return (PSTR(K02));

    case 0x22: /*PTLF(" |<<"); */ return (PSTR(K10));
    case 0x02: /*PTLF(" >>|"); */ return (PSTR(K11));
    case 0xC2: /*PTLF(" >||"); */ return (PSTR(K12));

    case 0xE0: /*PTLF(" -");   */ return (PSTR(K20));
    case 0xA8: /*PTLF(" +");  */ return (PSTR(K21));
    case 0x90: /*PTLF(" EQ"); */ return (PSTR(K22));

    case 0x68: /*PTLF(" 0");  */ return (PSTR(K30));
    case 0x98: /*PTLF(" 100+"); */ return (PSTR(K31));
    case 0xB0: /*PTLF(" 200+"); */ return (PSTR(K32));

    case 0x30: /*PTLF(" 1");  */ return (PSTR(K40));
    case 0x18: /*PTLF(" 2");  */ return (PSTR(K41));
    case 0x7A: /*PTLF(" 3");  */ return (PSTR(K42));

    case 0x10: /*PTLF(" 4");  */ return (PSTR(K50));
    case 0x38: /*PTLF(" 5");  */ return (PSTR(K51));
    case 0x5A: /*PTLF(" 6");  */ return (PSTR(K52));

    case 0x42: /*PTLF(" 7");  */ return (PSTR(K60));
    case 0x4A: /*PTLF(" 8");  */ return (PSTR(K61));
    case 0x52: /*PTLF(" 9");  */ return (PSTR(K62));

    case 0x12: return (PSTR("ts"));  //for factory use

    case 0xFF: return PSTR("");  //PTL(" REPEAT");
#endif
    default:
      {
      }
      return PSTR("");  //PTL("null");
  }               // End Case

  // The control could be organized in another way, such as:
  // forward/backward to change the gaits corresponding to different speeds.
  // left/right key for turning left and right
  // number keys for different postures or behaviors
}

#define IR_GAIT_LEN 3  //the gait name is at most 2 characters + '\0'
#define IR_CMD_LEN 8   //the longest key string is "buttUp" (6 characters + '\0')

//read_infrared() strcpy_P()s whatever translateIR() returns into a char[IR_CMD_LEN] on the stack,
//so a key string that outgrows IR_CMD_LEN has to fail the build rather than overflow that buffer.
#define IR_KEY_FITS(k) static_assert(sizeof(k) <= IR_CMD_LEN, "IR key " k " is longer than IR_CMD_LEN")
IR_KEY_FITS(K00); IR_KEY_FITS(K01); IR_KEY_FITS(K02);
IR_KEY_FITS(K10); IR_KEY_FITS(K11); IR_KEY_FITS(K12);
IR_KEY_FITS(K20); IR_KEY_FITS(K21); IR_KEY_FITS(K22);
IR_KEY_FITS(K30); IR_KEY_FITS(K31); IR_KEY_FITS(K32);
IR_KEY_FITS(K40); IR_KEY_FITS(K41); IR_KEY_FITS(K42);
IR_KEY_FITS(K50); IR_KEY_FITS(K51); IR_KEY_FITS(K52);
IR_KEY_FITS(K60); IR_KEY_FITS(K61); IR_KEY_FITS(K62);
IR_KEY_FITS("ts");  //the factory-use key is spelled out in translateIR()
//irParser() composes gait + direction + '\0' into irCmd[IR_CMD_LEN]
static_assert(IR_GAIT_LEN + 1 <= IR_CMD_LEN, "IR_GAIT_LEN + a direction does not fit IR_CMD_LEN");

char gait[IR_GAIT_LEN] = "tr";
char direct = 'F';
char irCmd[IR_CMD_LEN];  //holds the gait + direction composition, e.g. "trF"

// returns either raw itself (pass-through keys) or the composed gait + direction command
const char *irParser(const char *raw) {
  if (!strcmp(raw, "B")) {
    strcpy(gait, "bk");
    return gait;
  }
  if (!strcmp(raw, "F") || !strcmp(raw, "L") || !strcmp(raw, "R")) {
    direct = raw[0];
    if (direct == 'F' && !strcmp(gait, "bk")) {
      strcpy(gait, "tr");
    }
  } else if (!strcmp(raw, "vt") || !strcmp(raw, "cr") || !strcmp(raw, "wk") || !strcmp(raw, "tr") || !strcmp(raw, "ph")) {
    strcpy(gait, raw);
  } else {
    return raw;
  }
  byte len = strlen(gait);  //gait + direct
  strcpy(irCmd, gait);
  irCmd[len] = direct;
  irCmd[len + 1] = '\0';
  return irCmd;
}

void read_infrared() {
  if (irrecv.decode(&results)) {
    char raw[IR_CMD_LEN];
    strcpy_P(raw, translateIR());
    const char *IRsig = irParser(raw);
    if (IRsig[0] != '\0') {
      if (IRsig[1] == '\0')  //a single character is used as a token
        token = IRsig[0];
      else {
        token = T_SKILL;
        strcpy(newCmd, IRsig);
        if (!strcmp(newCmd, "up"))
          direct = 'F';
      }
      newCmdIdx = 1;
    }
    irrecv.resume();  // receive the next value
  }
}
