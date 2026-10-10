//code provided by the manufacturer
//modified by Rongzhong Li for better demonstration.
//Feb.16, 2021

#include "rgbUltrasonic/RgbUltrasonic.h"
RgbUltrasonic ultrasonic(6, 7);  //(signal, RGB)
//The RGB LED ultrasonic module should be plugged in the fourth grove socket with D6, D7

long colors[] = { RGB_RED, RGB_PURPLE, RGB_GREEN, RGB_BLUE, RGB_YELLOW, RGB_WHITE };
long ultraTimer;
int ultraInterval = 1000;
float distance;
int feedbackDirection = 1;
void readRGBultrasonic() {
  if (millis() - ultraTimer > ultraInterval) {
    ultraTimer = millis();
    ultraInterval = 0;
    randomInterval = 1000;
    distance = ultrasonic.GetUltrasonicDistance();
    if (distance > 120) {
      return;
    }

    if (distance > 90) {
      if (!manualEyeColorQ)
        ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_WHITE, E_EFFECT_BREATHING);
      ultraInterval = 1000;
      randomInterval = 1000;
    } else if (distance > 70) {
      if (!manualEyeColorQ)
        ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_YELLOW, E_EFFECT_ROTATE);
    } else if (distance > 50) {
      if (!manualEyeColorQ)
        ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_BLUE, E_EFFECT_FLASH);
    } else if (distance < 3) {
      ultraInterval = 2000;
      randomInterval = 5000;
      tQueue->addTask('k', "str", 1000);
      tQueue->addTask('k', "vtF", 1500);
      tQueue->addTask('k', "up");
    } else if (distance < 6) {
      if (!manualEyeColorQ)
        ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_RED, E_EFFECT_FLASH);
      meow(random() % 3 + 1, distance * 2);
      int8_t allRand[] = { 0, int8_t(currentAng[0] + random() % 20 - 10), 1, int8_t(currentAng[1] + random() % 20 - 10), 2, int8_t(currentAng[2] + random() % 20 - 10) };
      arrayNCPY(newCmd, allRand, cmdLen);
      newCmd[cmdLen] = '~';
      tQueue->addTask('I', newCmd);
      tQueue->addTask('i', "");
    } else if (distance < 10) {
      if (!manualEyeColorQ)
        ultrasonic.SetRgbColor(E_RGB_ALL, RGB_RED);
      tQueue->addTask('k', "sit", 2000);
      tQueue->addTask('k', "up");
    }

    else {  //10~50
      distance -= 9;
      if (!manualEyeColorQ)
        ultrasonic.SetRgbColor(E_RGB_ALL, colors[int(max(min(distance / 10, 5), 0))]);
      int mid[] = {
        15,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        30,
        30,
        -30,
        -30,
        30,
        30,
        -30,
        -30,
      };
      int allParameter[] = { mid[0] - distance / 2, -10 + distance / 2, distance * (random() % 50 < 1 ? int(random() % 2 - 1) : 1), 0,
                             0, 0, 0, 0,
                             mid[8] - 15 + distance / 2, mid[9] - 15 + distance / 2, mid[10] - 30 + distance * feedbackDirection, mid[11] - 30 + distance * feedbackDirection,
                             mid[12] + 35 - distance, mid[13] + 35 - distance, mid[14] + 40 - distance * feedbackDirection, mid[15] + 40 - distance * feedbackDirection };
      cmdLen = 16;
      for (byte i = 0; i < cmdLen; i++)
        newCmd[i] = (int8_t)min(max(allParameter[i], -128), 127);
      newCmd[cmdLen] = '~';
      randomInterval = 5000;
      tQueue->addTask('L', newCmd);
    }
  }
}
