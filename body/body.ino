// Kilo body firmware (NyBoard V1_2, ATmega328P). See docs/architecture.md.
//
// This file is intentionally empty of code. arduino-cli needs a .ino named
// after the folder, and it compiles everything under src/ recursively:
//   src/core/  plain C++ logic, no Arduino or AVR headers, host-tested
//   src/hal/   register-level drivers (UART, I2C, PCA9685, MPU6050, ...)
//   src/app/   main(): boot sequence and the 50 Hz loop
// main() is ours (src/app/main.cpp), so the Arduino core's setup()/loop()
// machinery, HardwareSerial and Wire are never linked.
