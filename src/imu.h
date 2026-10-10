// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v6.12)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-10 - Uses the new version of the DMP Firmware V6.12
//                 - Note: I believe the Teapot demo is broken with this versin as
//                 - the fifo buffer structure has changed
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "mpu6050/I2Cdev.h"

#include "mpu6050/MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
   ========================================================================= */

// MPU control/status vars
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#define ARX aaReal.x
#define ARY aaReal.y
#define ARZ aaReal.z
#define AWX aaWorld.x
#define AWY aaWorld.y
#define AWZ aaWorld.z
#define IMU_SKIP 3
#define IMU_SKIP_MORE 23  //use prime number to avoid repeatly skipping the same joint
byte imuSkip = IMU_SKIP;
int16_t imuOffset[9] = { 0, 0, 0,
                         0, 0, 0,
                         0, 0, 0 };


// The REALACCEL numbers are calculated with respect to the orientation of the sensor itself, so that if it is flat and you move it straight up, the "Z" accel will change, but if you flip it up on one side and move it in the new relative "up" direction (along the sensor's Z axis), it will still register acceleration on the Z axis. Essentially, it is sensor-oriented acceleration which removes the effects of gravity and any non-flat/level orientation.

// The WORLDACCEL numbers are calculated to ignore orientation. Moving it straight up while flat will look the same as the REALACCEL numbers, but if you then flip it upside-down and do the exact same movement ("up" with respect to you), you'll get exactly the same numbers as before, even though the sensor itself is upside-down.

// #define READ_ACCELERATION
void print6Axis() {
  PT_FMT(ypr[1], 5);
  PT('\t');
  PT_FMT(ypr[2], 5);
#ifdef READ_ACCELERATION
  PT("\t");
  PT(aaReal.x);  //x is along the longer direction of the robot
  PT("\t");
  PT(aaReal.y);
  PT("\t");
  PT(aaWorld.z);
#endif
  PTL();
}
bool read_IMU() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
                                                  // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
#ifdef READ_ACCELERATION
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
#endif
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    for (int i = 0; i < 3; i++)
      ypr[i] *= degPerRad;
    if (printGyro)
      print6Axis();
    exceptions = fabs(ypr[2]) > 90;  // && aaReal.z < 0; //the second condition is used to filter out some noise

    // Acceleration Real
    //        ^ x+
    //        |
    //  y+ <----- y-
    //        |
    //        | x-
    return true;
  }
  return false;
}

void readEnvironment() {
  if (gyroBalanceQ && !(frame % imuSkip))
    read_IMU();
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

//This function will write a 2-byte integer to the EEPROM at the specified address and address + 1

void imuSetup() {
  wdt_enable(WDTO_4S);
  do {
    delay(100);
    // initialize device
    PTLF("IMU");
    mpu.initialize();
    // verify connection
  } while (!mpu.testConnection());

  // load and configure the DMP
  do {
    devStatus = mpu.dmpInitialize();
    delay(100);
  } while (devStatus);
  wdt_disable();

  // supply the gyro offsets here, scaled for min sensitivity
  for (byte axis = 0; axis < 6; axis++) {
    imuOffset[axis] = EEPROMReadInt(MPUCALIB + axis * 2);
    if (axis < 3)
      mpu.setAccelOffset(axis, imuOffset[axis]);
    else
      mpu.setGyroOffset(axis, imuOffset[axis]);
  }

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
#ifndef MAIN_SKETCH
    beep(10);
#ifndef AUTO_INIT
    PTLF("\nLay the robot/board FLAT on a table. Calibrate IMU?(Y/n):");
    char choice = getUserInputChar();
    PTL(choice);
    if (choice == 'Y' || choice == 'y') {
#endif
      beep(15, 400, 600);
      PTLF("Calibrating...");
      mpu.CalibrateAccel(10);
      mpu.CalibrateGyro(10);
      for (byte axis = 0; axis < 6; axis++) {
        if (axis < 3)
          EEPROMWriteInt(MPUCALIB + axis * 2, mpu.getAccelOffset(axis));
        else
          EEPROMWriteInt(MPUCALIB + axis * 2, mpu.getGyroOffset(axis));
      }

#ifndef AUTO_INIT
    }
#endif
    beep(18, 50, 70, 6);
#endif
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    PTF("DMP failed (");
    PT(devStatus);
    PTL(')');
  }
}
