void dealWithExceptions() {
  if (gyroBalanceQ && exceptions) {  //the gyro reaction switch can be toggled on/off by the 'g' token
    beep(20, 50, 25, 5);
    token = T_SKILL;
    strcpy(newCmd, "rc");
    newCmdIdx = -1;
  }
}

#ifdef VOLTAGE_DETECTION_PIN
bool lowBattery() {
  long currentTime = millis() / CHECK_BATTERY_PERIOD;
  if (currentTime > uptime) {
    uptime = currentTime;
    int voltage = analogRead(VOLTAGE_DETECTION_PIN);
    if (voltage < LOW_VOLTAGE && (voltage == lastVoltage || uptime < 2)) {  // if the battery still holds charges but fluctuates during moving, it will only alarm for the first two times
                                                                            //if battery voltage < threshold, it needs to be recharged
      //give the robot a break when voltage drops after sprint
      //adjust the thresholds according to your batteries' voltage
      //if set too high, the robot will stop working when the battery still has power.
      //If too low, the robot may not alarm before the battery shuts off


      PTF("Low power:");
      PT(voltage / 99);
      PTL('V');

      if (eeprom(BOOTUP_SOUND_STATE))
        playMelody(MELODY_LOW_BATTERY);
      lastVoltage = voltage;
      return true;
    }
    lastVoltage = voltage;
  }
  return false;
}
#endif

void reaction() {
  if (newCmdIdx) {
    lowerToken = tolower(token);
#ifdef MAIN_SKETCH
    if (newCmdIdx < 5 && lowerToken != T_BEEP
#ifdef BINARY_COMMAND
        && token != T_LISTED_BIN && token != T_INDEXED_SIMULTANEOUS_BIN
#endif
    )
      beep(15 + newCmdIdx, 5);                    //ToDo: check the muted sound when newCmdIdx = -1
    if (hardServoQ && (lowerToken == T_SKILL)) {
      setServoP(P_SOFT);
      hardServoQ = false;
    }
    if ((lastToken == T_CALIBRATE || lastToken == T_REST) && token != T_CALIBRATE) {
      gyroBalanceQ = true;
      PTL('G');
    }
    if (token != T_PAUSE && !tStep) {
      tStep = 1;
      PTL('p');
    }
#endif
    switch (token) {
#ifdef MAIN_SKETCH
      case T_QUERY:
        {
          PTLF(MODEL);
          PTLF(SOFTWARE_VERSION);
          break;
        }

      case T_GYRO_FINENESS:
      case T_GYRO_BALANCE:
#ifdef GYRO_PIN
#ifdef T_PRINT_GYRO
      case T_PRINT_GYRO:
#endif
#ifdef T_VERBOSELY_PRINT_GYRO
      case T_VERBOSELY_PRINT_GYRO:
#endif
#endif

#ifdef RANDOM_MIND
      case T_RANDOM_MIND:
#endif
        {
          if (token == T_GYRO_FINENESS) {
#ifdef GYRO_PIN
            fineAdjust = !fineAdjust;
            imuSkip = fineAdjust ? IMU_SKIP : IMU_SKIP_MORE;
            token = fineAdjust ? 'G' : 'g';  //G for fine adjust
#else
            token = 'g';
#endif
          } else if (token == T_GYRO_BALANCE) {

#ifdef GYRO_PIN
            gyroBalanceQ = !gyroBalanceQ;
            token = gyroBalanceQ ? 'G' : 'g';  //G for activated balance
#else
            token = 'g';
#endif
          }
#ifdef GYRO_PIN
#ifdef T_PRINT_GYRO
          else if (token == T_PRINT_GYRO) {
            print6Axis();
          }
#endif
#ifdef T_VERBOSELY_PRINT_GYRO
          else if (token == T_VERBOSELY_PRINT_GYRO) {
            printGyro = !printGyro;
            token = printGyro ? 'V' : 'v';  //V for verbosely print gyro data
          }
#endif
          else
#endif

#ifdef RANDOM_MIND
            if (token == T_RANDOM_MIND) {
            autoSwitch = !autoSwitch;
            token = autoSwitch ? 'Z' : 'z';  //Z for active random mind
          } else
#endif
          {}  //must have in cases the else combines with break
          break;
        }
      case T_PAUSE:
        {
          tStep = !tStep;             //tStep can be -1
          token = tStep ? 'p' : 'P';  //P for pause activated
          if (tStep)
            token = T_SKILL;
          else
            pwm.shutServos();
          break;
        }
      case T_JOINTS:
        {  //show the list of current joint anles
          PT('=');
          if (cmdLen)
            PTL(currentAng[atoi(newCmd)]);
          else
            printTable(currentAng);
          break;
        }
#ifdef ULTRASONIC
      case T_COLOR:
        {
          if (cmdLen < 2)  //a single 'C' will turn off the manual color mode
                           //the terminator of upper-case tokens is not '\n'. it may cause error when entered in the Arduino serial monitor
            manualEyeColorQ = false;
          else {  // turn on the manual color mode
            manualEyeColorQ = true;
            long color = ((long)(uint8_t(newCmd[0])) << 16) + ((long)(uint8_t(newCmd[1])) << 8) + (long)(uint8_t(newCmd[2]));
            ultrasonic.SetRgbEffect(E_RGB_INDEX(uint8_t(newCmd[3])), color, uint8_t(newCmd[4]));
          }
          break;
        }
#endif
      case T_SAVE:
        {
          PTLF("saved");
          saveCalib(servoCalib);
          break;
        }
      case T_ABORT:
        {
          PTLF("aborted");
          for (byte i = 0; i < DOF; i++) {
            servoCalib[i] = eeprom(CALIB, i);
          }
          break;
        }
      case T_CALIBRATE:                 //calibration
      case T_INDEXED_SEQUENTIAL_ASC:    //move multiple indexed joints to angles once at a time (ASCII format entered in the serial monitor)
      case T_INDEXED_SIMULTANEOUS_ASC:  //move multiple indexed joints to angles simultaneously (ASCII format entered in the serial monitor)
      case T_BEEP:  //beep(tone, duration): tone 0 is pause. the duration corresponds to 1/duration second
        {
          if (token == T_INDEXED_SIMULTANEOUS_ASC && cmdLen == 0)
            manualHeadQ = false;
          else {
            int targetFrame[DOF];
            nonHeadJointQ = false;
            for (int i = 0; i < DOF; i++) {
              targetFrame[i] = currentAng[i] - (gyroBalanceQ ? currentAdjust[i] : 0);
            }
            char *pch;
            pch = strtok(newCmd, " ,");
            do {  //it supports combining multiple commands at one time
              //for example: "m8 40 m8 -35 m 0 50" can be written as "m8 40 8 -35 0 50"
              //the combined commands should be less than four. string len <=30 to be exact.
              int target[2] = {};
              int inLen = 0;
              for (byte b = 0; b < 2 && pch != NULL; b++) {
                target[b] = atoi(pch);
                pch = strtok(NULL, " ,\t");
                inLen++;
              }
              if ((token == T_INDEXED_SEQUENTIAL_ASC || token == T_INDEXED_SIMULTANEOUS_ASC) && target[0] >= 0 && target[0] < DOF) {
                targetFrame[target[0]] = target[1];
                if (target[0] < 4) {
                  targetHead[target[0]] = target[1];
                  manualHeadQ = true;
                } else
                  nonHeadJointQ = true;
              }
              if (token == T_CALIBRATE) {
                gyroBalanceQ = false;
                if (lastToken != T_CALIBRATE) {
                  setServoP(P_HARD);
                  hardServoQ = true;
                  strcpy(newCmd, "calib");
                  skill.loadFrame(newCmd);  //ensure it's correctly compared in load skill for protective shifts.
                }
                if (inLen == 2) {
                  servoCalib[target[0]] = target[1];
                }
                int duty = EEPROMReadInt(ZERO_POSITIONS + target[0] * 2) + float(servoCalib[target[0]]) * eeprom(ROTATION_DIRECTION, target[0]);
                pwm.writeAngle(target[0], duty);
                printTable(servoCalib);
              } else if (token == T_INDEXED_SEQUENTIAL_ASC) {
                transform(targetFrame, 1, 2);
              }
              else if (token == T_BEEP) {
                if (inLen == 0)  //toggle the melody on/off
                                 //the terminator of upper-case tokens is not '\n'. it may cause error when entered in the Arduino serial monitor
                  EEPROM.update(BOOTUP_SOUND_STATE, !eeprom(BOOTUP_SOUND_STATE));
                else if (target[1])
                  beep(target[0], 1000 / target[1]);
              }

            } while (pch != NULL);
            if ((token == T_INDEXED_SEQUENTIAL_ASC || token == T_INDEXED_SIMULTANEOUS_ASC) && (nonHeadJointQ || lastToken != T_SKILL)) {
              transform(targetFrame, 1, 4);  // if (token == T_INDEXED_SEQUENTIAL_ASC) it will be useless
              skill.convertTargetToPosture(targetFrame);
            }
            delete[] pch;
          }
          break;
        }
      // this block handles array like arguments
      case T_INDEXED_SEQUENTIAL_BIN:
      case T_INDEXED_SIMULTANEOUS_BIN:
        {                  //indexed joint motions: joint0, angle0, joint1, angle1, ... (binary encoding)
          if (cmdLen < 2)  //the terminator of upper-case tokens is not '\n'. it may cause error when entered in the Arduino serial monitor
            manualHeadQ = false;
          else {
            int targetFrame[DOF];
            nonHeadJointQ = false;
            for (int i = 0; i < DOF; i++) {
              targetFrame[i] = currentAng[i] - (gyroBalanceQ ? currentAdjust[i] : 0);
            }
            byte group = 2;
            for (int i = 0; i < cmdLen; i += group) {
              if (newCmd[i] >= 0 && newCmd[i] < DOF) {
                targetFrame[newCmd[i]] = (int8_t)newCmd[i + 1];
                if (newCmd[i] < 4) {
                  targetHead[newCmd[i]] = (int8_t)newCmd[i + 1];
                  manualHeadQ = true;
                } else
                  nonHeadJointQ = true;
              }
              if (token == T_INDEXED_SEQUENTIAL_BIN) {
                transform(targetFrame, 1, 2);
              }
            }
            if (nonHeadJointQ || lastToken != T_SKILL) {
              transform(targetFrame, 1, 4);  // if (token == T_INDEXED_SEQUENTIAL_BIN) it will be useless
              skill.convertTargetToPosture(targetFrame);
            }
          }
          break;
        }
#ifdef BINARY_COMMAND
      case T_LISTED_BIN:
        {
          //list of all 16 joint: angle0, angle2,... angle15 (binary encoding)
          transform((int8_t *)newCmd, 1, transformSpeed);  //need to add angleDataRatio if the angles are large
          break;
        }
      case T_BEEP_BIN:
        {
          for (byte b = 0; b < cmdLen / 2; b++)
            beep(newCmd[2 * b], 1000 / newCmd[2 * b + 1]);
          break;
        }
#ifdef T_TEMP
      case T_TEMP:
        {  //call the last skill data received from the serial port
          skill.loadDataFromI2cEeprom((unsigned int)EEPROMReadInt(SERIAL_BUFF));
          skill.loadFrameByDataBuffer();
          PTL(token);
          token = T_SKILL;
          strcpy(newCmd, "tmp");
          break;
        }
#endif
#ifdef T_SKILL_DATA
      case T_SKILL_DATA:
        {  //takes in the skill array from the serial port, load it as a regular skill object and run it locally without continuous communication with the master
          unsigned int i2cEepromAddress = EEPROMReadInt(SERIAL_BUFF);
          skill.copyDataFromBufferToI2cEeprom(i2cEepromAddress, (int8_t *)newCmd);  //must be before the loading to set the period
          skill.loadFrameByDataBuffer();
          token = T_SKILL;
          strcpy(newCmd, "tmp");
          break;
        }
#endif
#endif

      case T_SKILL:
        {
          if (!strcmp("x", newCmd)        // x for random skill
              || strcmp(lastCmd, newCmd)  // won't transform for the same gait. it's better to compare skill->skillName and newCmd. but need more logics for non skill cmd in between
              || skill.period <= 1) {     // for repeating behaviors. if it's set < 1, won't repeat the last behavior

            skill.loadFrame(newCmd);
          }
          break;
        }

#endif
      case T_REST:
        {
          strcpy(newCmd, "rest");
          skill.loadFrame(newCmd);
          pwm.shutServos();
          gyroBalanceQ = false;
          manualHeadQ = false;
          PTL('g');
          break;
        }
      default:
        break;
    }
    if (token != T_SKILL || skill.period > 0) {
      PTL(token);  //postures, gaits, and other tokens can confirm completion by sending the token back
      if (lastToken == T_SKILL
          && (token == T_JOINTS || token == T_PAUSE || lowerToken == T_GYRO_FINENESS || lowerToken == T_INDEXED_SIMULTANEOUS_ASC || lowerToken == T_INDEXED_SEQUENTIAL_ASC
#ifdef T_PRINT_GYRO
              || lowerToken == T_PRINT_GYRO
#endif
              ))
        token = T_SKILL;
    }
    resetCmd();
  }

#ifdef MAIN_SKETCH
  if (token == T_SKILL) {
    skill.perform();

#ifndef GYRO_PIN
    delayMicroseconds(850);  //Slow the robot down to smooth out motion; adjust delay PRN
#endif
    if (skill.period < 0) {  //behaviors
      {                      //use the last frame as a posture
        skill.convertTargetToPosture(currentAng);
      }
      for (int i = 0; i < DOF; i++)
        currentAdjust[i] = 0;
      PTL(token);  //behavior can confirm completion by sending the token back
    }
    if (exceptions && lastCmd[strlen(lastCmd) - 1] < 'L' && skill.lookupAddressByName(lastCmd) > 0) {  // if the last command is not a behavior and not a turning gait. case wkF, wkL, wkR, rlL, rlR
      if (lastCmd[0] != '\0')
        skill.loadFrame(lastCmd);
    }
  }
#endif
}
