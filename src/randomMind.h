const char mind2[] PROGMEM = "ksit";
const char mind3[] PROGMEM = "kstr";
const char mind4[] PROGMEM = "khi";
const char mind5[] PROGMEM = "kck";
const char mind6[] PROGMEM = "kchr";
const char mind7[] PROGMEM = "ksnf";
const char mind8[] PROGMEM = "kscrh";
const char mind9[] PROGMEM = "kpee";
const char mind10[] PROGMEM = "kpu";
const char mind11[] PROGMEM = "kcmh";
const char mind12[] PROGMEM = "kvtF";

const char *const mindList[] PROGMEM = {
  mind2,
  mind3,
  mind4,
  mind5,
  mind6,
  mind7,
  mind8,
  mind9,
  mind10,
  mind11,
  mind12,
};
byte choiceWeight[] = {
  10,
  5,
  3,
  3,
  3,
  3,
  2,
  2,
  2,
  2,
  2,
};

byte randomMindListLength = sizeof(mindList) / 2;
int randomBase = 0;
void randomMind() {
  long current = millis();
  if (current - idleTimer > randomInterval) {  //every randomInterval throw a dice
    idleTimer = current;
    int randomNum = random() % randomBase;
    byte randomChoice = -1;
    while (randomNum >= 0) {
      randomChoice++;
      randomNum -= choiceWeight[randomChoice];
    }
    if (randomChoice == 0) {
    } else {
      strcpy_P((char *)newCmd, (char *)pgm_read_word(&mindList[randomChoice]));
      token = newCmd[0];

      if (token == T_SKILL)
        strcpy(newCmd, (char *)newCmd + 1);  // this is duable only because newCmd+1 is after newCmd!
      else
        strcpy((char *)newCmd, (char *)newCmd + 1);  // this is duable only because newCmd+1 is after newCmd!
    }
    newCmdIdx = 100;
  }
}
