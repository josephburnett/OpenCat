#define SOFTWARE_VERSION "N_240907"  //NyBoard + YYMMDD
//board configuration
// -- comment out these blocks to save program space for your own codes --
#define BUZZER 5

//#define SERVO_SLOW_BOOT

#define I2C_EEPROM  //comment this line out if you don't have an I2C EEPROM in your DIY board.
#define SERIAL_TIMEOUT 5
#define SERIAL_TIMEOUT_LONG 200
//Tutorial: https://bittle.petoi.com/11-tutorial-on-creating-new-skills

#define DOF 16

int8_t middleShift[] = { 0, 15, 0, 0,
                         -45, -45, -45, -45,
                         10, 10, -10, -10,
                         -30, -30, 30, 30 };
int angleLimit[][2] = {
  { -120, 120 },
  { -80, 45 },
  { -120, 120 },
  { -120, 120 },
  { -90, 60 },
  { -90, 60 },
  { -90, 90 },
  { -90, 90 },
  { -100, 80 },
  { -100, 80 },
  { -80, 100 },
  { -80, 100 },
  { -60, 120 },
  { -60, 120 },
  { -120, 60 },
  { -120, 60 },
};

// #define INVERSE_SERVO_DIRECTION
int8_t rotationDirection[] = { 1, -1, -1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1 };


byte pwm_pin[] = { 12, 11, 4, 3,
                   13, 10, 5, 2,
                   14, 9, 6, 1,
                   15, 8, 7, 0 };
#define VOLTAGE_DETECTION_PIN A7
#define LOW_VOLTAGE 650
#define DEVICE_ADDRESS 0x54
#define BAUD_RATE 115200
#define LED_PIN 3

#define MODEL "Nybble"
#define WALKING_DOF 8
#define REGULAR G41
#define KNEE G41
#include "InstinctNybble.h"


//on-board EEPROM addresses
#define MELODY_NORMAL 1023  //melody will be saved at the end of the 1KB EEPROM, and is read reversely. That allows some flexibility on the melody length.
#define MELODY_INIT 1002
#define MELODY_LOW_BATTERY 977
#define MELODY_1 966
#define PWM_PIN 0                     // 16 byte array
#define CALIB 16                      // 16 byte array
#define MID_SHIFT 32                  // 16 byte array
#define ROTATION_DIRECTION 48         // 16 byte array
#define ZERO_POSITIONS 64             // 16*2 = 32   64+32 = 96
#define CALIBRATED_ZERO_POSITIONS 96  // 16*2 = 32   96+32 = 128
#define ADAPT_PARAM 128               // 16 x 2 byte array 128+32 = 160
#define ANGLE2PULSE_FACTOR 160        // 16*2 = 32  160+32 = 192
#define ANGLE_LIMIT 192               // 16*2*2 = 64   192+64 = 256
#define MPUCALIB 256                  // 9 int byte array 9x2 =18  256+18=274
#define B_OFFSET 274                  // 1 bytes
#define PCA9685_FREQ 275              // 2 bytes
#define NUM_SKILLS 277                // 1 bytes
#define SERIAL_BUFF 278               // 2 bytes
#define SERIAL_BUFF_RAND 280          // 2 bytes
#define BOOTUP_SOUND_STATE 282        // 1 byte
#define SKILLS 320                    // 1 byte for skill name length, followed by the char array for skill name
// then followed by i(nstinct) on progmem, or n(ewbility) on progmem

//the actual data is stored on the I2C EEPROM.
#define INITIAL_SKILL_DATA_ADDRESS 32
//The first several bytes are reserved for testing
//the above constants from onboard EEPROM to I2C EEPROM

//token list
#define T_ABORT 'a'      //abort the calibration values
#define T_BEEP 'b'       //b note1 duration1 note2 duration2 ... e.g. b12 8 14 8 16 8 17 8 19 4 \
                         //a single 'b' will toggle the melody on/off
#define T_CALIBRATE 'c'  //send the robot to calibration posture for attaching legs and fine-tuning the joint offsets. \
                         //c jointIndex1 offset1 jointIndex2 offset2 ... e.g. c0 7 1 -4 2 3 8 5
#define T_REST 'd'
#define T_GYRO_FINENESS 'g'             //adjust the finess of gyroscope adjustment to accelerate motion
#define T_GYRO_BALANCE 'G'              //toggle on/off the gyro adjustment
#define T_INDEXED_SIMULTANEOUS_ASC 'i'  //i jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. i0 70 8 -20 9 -20 \
                                        //a single 'i' will free the head joints if it were previously manually controlled.
#define T_JOINTS 'j'                    //A single "j" returns all angles. "j Index" prints the joint's angle. e.g. "j 8" or "j11".
#define T_SKILL 'k'
#define T_SKILL_DATA 'K'
#define T_INDEXED_SEQUENTIAL_ASC 'm'  //m jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. m0 70 0 -70 8 -20 9 -20
#define T_PAUSE 'p'
#define T_SAVE 's'  //save the calibration values
#define T_PRINT_GYRO 'v'            //print the Gyro data once
#define T_VERBOSELY_PRINT_GYRO 'V'  //toggle verbosely print Gyro data
#define T_RANDOM_MIND 'z'  //toggle random behaviors in the RANDOM_MIND mode
#define T_QUERY '?'

#define T_COLOR 'C'                     //change the eye colors of the RGB ultrasonic sensor
#define T_INDEXED_SIMULTANEOUS_BIN 'I'  //I jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. I0 70 8 -20 9 -20
#define T_INDEXED_SEQUENTIAL_BIN 'M'    //M jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. M0 70 0 -70 8 -20 9 -20

#define BINARY_COMMAND  //disable the binary commands to save space for the simple random demo

#ifdef BINARY_COMMAND
#define T_BEEP_BIN 'B'    //B note1 duration1 note2 duration2 ... e.g. B12 8 14 8 16 8 17 8 19 4
#define T_LISTED_BIN 'L'  //a list of the DOFx joint angles: angle0 angle1 angle2 ... angle15
#define T_TEMP 'T'  //call the last 'K' skill data received from the serial port
#endif


float degPerRad = 180.0 / M_PI;
float radPerDeg = M_PI / 180.0;

int8_t servoCalib[DOF] = { 0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0 };
#ifndef MAIN_SKETCH
enum ServoModel_t {
  G41 = 0,
  P1S
};

ServoModel_t servoModelList[] = {
  REGULAR, REGULAR, REGULAR, REGULAR,
  REGULAR, REGULAR, REGULAR, REGULAR,
  REGULAR, REGULAR, REGULAR, REGULAR,
  KNEE, KNEE, KNEE, KNEE
};
#endif

int zeroPosition[DOF] = {};
int calibratedZeroPosition[DOF] = {};

int currentAng[DOF] = { -30, -80, -45, 0,
                        0, 0, 0, 0,
                        75, 75, -75, -75,
                        -55, -55, 55, 55 };
float currentAdjust[DOF] = {};

//control related variables
#define IDLE_TIME 3000
long idleTimer = 0;
int randomInterval = 2000;
#define CHECK_BATTERY_PERIOD 10000  //every 10 seconds. 60 mins -> 3600 seconds
int uptime = -1;
int frame = 0;
byte tStep = 1;

char token;
char lowerToken;
char lastToken;
byte newCmdIdx = 0;
int cmdLen = 0;
#define CMD_LEN 10    //the last char will be '\0' so only CMD_LEN-1 elements are allowed
#define BUFF_LEN 467  //452
char newCmd[BUFF_LEN + 1];
char lastCmd[CMD_LEN + 1];
int spaceAfterStoringData = BUFF_LEN;
//22*20+7=447, +1 for '\0'.
//The max behavior allowed has 22 frames. The max gait (8 DoF) allowed has (448-4)/8=55.5 frames.
//so 56*8 + 4 = 452 is the most efficient

//468: 23*20+7=467, +1 for '\0'.
//The max behavior allowed has 23 frames. The max gait (8 DoF) allowed has (468-4)/8=58 frames.
//so 468 is the most efficient

int8_t yprTilt[3];
int lastVoltage;
bool servoOff = true;

#ifdef RANDOM_MIND
bool autoSwitch = true;
#endif
bool fineAdjust = true;
bool gyroBalanceQ = true;
bool printGyro = false;
bool manualHeadQ = false;
bool nonHeadJointQ = false;
bool hardServoQ = true;
bool manualEyeColorQ = false;
#define HEAD_GROUP_LEN 4  //used for controlling head pan, tilt, tail, and other joints independent from walking
// #define DAMPED_MOTION
int targetHead[HEAD_GROUP_LEN];


int exceptions = 0;
byte transformSpeed = 2;
float protectiveShift;  //reduce the wearing of the potentiometer

#include "MemoryFree/MemoryFree.h"  //http://playground.arduino.cc/Code/AvailableMemory
#include <EEPROM.h>

#include "tools.h"
#include <avr/wdt.h>  // https://create.arduino.cc/projecthub/rafitc/what-is-watchdog-timer-fffe20
#include "eeprom.h"
#include "sound.h"

#if defined IR_PIN
#include "infrared.h"
#endif

#ifdef RANDOM_MIND
#include "randomMind.h"
#undef BINARY_COMMAND
#endif

#include "PCA9685servo.h"
#include "motion.h"

#ifndef MAIN_SKETCH
#define GYRO_PIN 0  //the setup sketch always needs the IMU

#else

#ifdef GYRO  //the gyro is an explicit option in OpenCat.ino and is independent from the modules below
#define GYRO_PIN 0
#endif

#define TASK_QUEUE  //allow executing a sequence of tasks, if you enabled the other modules, the task queue will be automatically enabled. \
  // because it takes up memory, it is disabled when both the GYRO and the IR remote are enabled and no module needs the queue. \
  // ULTRASONIC always needs it, because readRGBultrasonic() calls tQueue->addTask() unconditionally.
#if defined GYRO_PIN && defined IR_PIN && !defined ULTRASONIC
#undef TASK_QUEUE
#endif

#ifdef TASK_QUEUE
#include "taskQueue.h"
#define T_TASK_QUEUE 'q'
#endif

#ifdef ULTRASONIC
#include "ultrasonic.h"
#endif

#endif

#include "io.h"

#ifdef GYRO_PIN
#include "imu.h"
#endif

#include "skill.h"
#include "reaction.h"

void initRobot() {
  //----------------------------------
  beep(20);
#ifdef MAIN_SKETCH  // **
  PTL('k');
  PTLF("\n* Start *");
  PTLF(MODEL);
  PTLF(SOFTWARE_VERSION);
  if (eeprom(BOOTUP_SOUND_STATE))
    playMelody(MELODY_NORMAL);

#ifdef GYRO_PIN
  imuSetup();
#endif
  servoSetup();
  skill.assignSkillAddressToOnboardEeprom();
#ifdef RANDOM_MIND
  for (byte i = 0; i < randomMindListLength; i++) {
    randomBase += choiceWeight[i];
  }
#endif

#ifdef IR_PIN
  irrecv.enableIRIn();  // Start the receiver
#endif

#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
#endif

#ifdef TASK_QUEUE
  PTLF("TaskQ");  //tQueue is a statically allocated object, so there's no constructor-time print
#endif

#ifdef ULTRASONIC
  skill.loadFrame("sit");
  delay(500);
#endif

#ifdef GYRO_PIN
  for (byte r = 0; r < 50; r++) {  //ypr is slow when starting up. leave enough time between IMU initialization and this reading
    read_IMU();
    delay(10);
  }
  token = (fabs(ypr[1]) > 30 || fabs(ypr[2]) > 30) ? T_CALIBRATE : T_REST;  //put the robot's side on the table to enter calibration posture for attaching legs
  newCmdIdx = 2;
  int rndSeed = ypr[2] * 10000;
#else
  int rndSeed = analogRead(A2);
#endif
  randomSeed(rndSeed);  //use the fluctuation of voltage caused by servos as entropy pool

  allCalibratedPWM(currentAng);  //soft boot for servos
  delay(500);
  lastCmd[0] = '\0';
//----------------------------------
#else  // ** save parameters to device's static memory
  configureEEPROM();
  servoSetup();  //servo needs to be after configureEEPROM and before imuSetup
#ifdef GYRO_PIN
  imuSetup();
#endif
#endif  // **
  PTLF("Ready!");
#ifndef MAIN_SKETCH
  PCA9685CalibrationPrompt();
#endif
}
