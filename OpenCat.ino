/*
   The driver for OpenCat, runs on ATmega328P-based NyBoard (as Arduino Uno).
   Compatible with Petoi Nybble, Bittle, and many other 8 or 12 DoF quadruped robots.
   Drives up to 16 PWM servos.

   Rongzhong Li
   Mar 15rd, 2022
   Copyright (c) 2022 Petoi LLC.

   This sketch may also include others' codes under MIT or other open-source licenses.
   Check those licenses in the corresponding module test folders.
   Feel free to contact us if you find any missing references.

  The MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/*
  To configure the board:
  1. Comment out #define MAIN_SKETCH and upload. Upload and follow the serial prompts to proceed.
  2. Uncomment #define MAIN_SKETCH to make it active. Then upload the program for main functions.
*/

#define MAIN_SKETCH  //the Petoi App only works when this mode is on
// #define AUTO_INIT //automatically select 'Y' for the reset joint and IMU prompts
// #define DEVELOPER //to print out some verbose debugging data
// it may increase the code size and crash the bootloader.
// make sure you know ISP and how to reset the bootloader!!!

#define GYRO  //read the IMU (gyro + accelerometer) for self-balancing and fall detection. \
              // comment out this line to save programming space when you enable the modules below

// you can also activate the following modes
// allowed combinations: RANDOM_MIND + ULTRASONIC, RANDOM_MIND, ULTRASONIC
// #define RANDOM_MIND  //advanced random behaviors. use token 'z' to activate/deactivate
// #define TASK_QUEUE  //allow executing a sequence of tasks, if you enabled the other modules, the task queue will be automatically enabled. \
                    // because it takes up memory, it will be disabled when both the GYRO and the IR remote are enabled \
                    // and no module needs it. See "#undef TASK_QUEUE" in src/OpenCat.h
// #define ULTRASONIC      //for Nybble's ultrasonic sensor

#define IR_PIN 4  // Signal Pin of IR receiver to Arduino Digital Pin 4
#include "src/OpenCat.h"

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(500000L);
  initRobot();
}

void loop() {
#ifdef MAIN_SKETCH
#ifdef GYRO_PIN
  readEnvironment();     //reads the IMU (Inertia Measurement Unit, i.e. acceleration and angles).
                         //May read more sensors in the future
  dealWithExceptions();  //fall over, lifted, etc.
#endif

#ifdef TASK_QUEUE
  if (!tQueue->empty()) {
    tQueue->popTask();
  } else
#endif
  {
    readSignal();  //commands sent by user interfaces and sensors
  }
  reaction();  //handle different commands
#ifdef VOLTAGE_DETECTION_PIN
  lowBattery();  //  block the loop if battery is low
  //  can be disabled to save programming space and reduce the low voltage interruptions
#endif
#else
  calibratePCA9685();
#endif
}
