// i2c: polled TWI master at 400 kHz with bounded waits and bus recovery.
//
// Why not Wire: Wire buffers 32 B per transaction (the PCA9685 wants one
// 66 B burst), costs ~165 B of RAM, and its waits are unbounded in the
// versions that matter. Here every wait for the hardware is bounded (about
// 1.7 ms per action, and about 9.6 ms in total per tick period: see
// i2c_budget_reset), errors come back as codes, and a hung bus is recovered
// by clocking SCL by hand until the stuck slave lets go of SDA.
//
// No interrupts: the tick has nothing else to do while it waits, and polling
// keeps the code small and the order of events obvious.
#ifndef KILO_HAL_I2C_H
#define KILO_HAL_I2C_H
#include <stdint.h>

enum I2cErr : uint8_t {
  I2C_OK = 0,
  I2C_TIMEOUT = 1,    // TWINT never came: bus hung (recovered)
  I2C_NACK_ADDR = 2,  // nobody answered at that address
  I2C_NACK_DATA = 3,  // the slave refused a byte
  I2C_ARB_LOST = 4,   // another master, or noise (recovered)
  I2C_BUS_STUCK = 5,  // SDA still low after recovery
  I2C_BUDGET = 6,     // this tick's I2C time is spent: not started
};

void i2c_init(void);

// Refill the per-tick I2C time budget (about 9.6 ms of polling). Called at
// the start of every tick, by i2c_init and by each MPU init step; between
// two refills all transfers together wait at most that long, and once it is
// spent every transfer fails (I2C_BUDGET, counted) until the next refill.
void i2c_budget_reset(void);

// Register write: START, addr+W, reg, len bytes, STOP. data may be in RAM or,
// with the _P variant, in flash.
uint8_t i2c_write(uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len);
uint8_t i2c_write_P(uint8_t addr, uint8_t reg, const uint8_t* pgm, uint8_t len);
uint8_t i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t v);

// Register read: START, addr+W, reg, REPEATED START, addr+R, len bytes
// (ACK all but the last, NACK the last), STOP.
uint8_t i2c_read(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);

// Streaming form, for transfers produced or consumed on the fly (the
// PCA9685 burst writes 64 data bytes without a 64 B buffer). Every call
// after an error is a no-op returning that error; i2c_end() sends STOP (and
// recovers the bus if needed) and returns the first error of the transfer.
void i2c_begin_write(uint8_t addr, uint8_t reg);
void i2c_begin_read(uint8_t addr, uint8_t reg);
void i2c_put(uint8_t b);
uint8_t i2c_get(bool last);
uint8_t i2c_end(void);

uint16_t i2c_error_count(void);
void i2c_recover(void);  // 9 SCL pulses and a STOP; also run at init

#endif
