#include "i2c.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>

// SDA = PC4 (A4), SCL = PC5 (A5).
#define SDA_BIT 4
#define SCL_BIT 5

// Per action: about 1.7 ms of polling at 16 MHz (a pass of act()'s loop is
// 9 cycles, 0.56 us; ISRs stretch it). A healthy byte takes 23 us at
// 400 kHz (about 40 passes), so this only trips on a hung bus. Counting
// passes avoids depending on the clock module.
#define SPIN_LIMIT 3000
// Per tick period: every wait (act() and the STOP) is charged to a budget
// that app/main.cpp refills at the start of each tick (and i2c_init, and
// each MPU init step). 17000 passes is about 9.6 ms, twice the I2C of a
// healthy busy tick (IMU FIFO count and up to two packets, the 16-channel
// PCA burst and a PCA_READ: about 210 byte times, 4.8 ms). A slow (not dead)
// bus therefore costs at most ~10 ms per tick instead of overrunning it: an
// action that runs out mid-transfer ends as I2C_TIMEOUT (bus recovered), and
// a transfer begun with the budget spent is refused as I2C_BUDGET without
// touching the bus. Both are counted errors, so the callers' existing paths
// apply (IMU_ERR -> IMU_FAULT after a run; the PCA keeps its last pose).
#define TICK_SPIN_BUDGET 17000

static uint8_t err;       // first error of the current transfer
static uint16_t err_count;
static uint16_t budget = TICK_SPIN_BUDGET;  // passes left this tick period

void i2c_budget_reset(void) { budget = TICK_SPIN_BUDGET; }

// This action's spin limit: SPIN_LIMIT, or what is left of the budget.
static uint16_t spin_limit(void) { return budget < SPIN_LIMIT ? budget : SPIN_LIMIT; }

static void twi_enable(void) {
  TWSR = 0;   // prescaler 1
  TWBR = 12;  // SCL = 16 MHz / (16 + 2 * 12) = 400 kHz
  TWCR = _BV(TWEN);
}

// Open-drain by hand: "low" drives 0, "release" lets the pull-up win.
// Order matters so the pin is never actively driven high.
static void line_low(uint8_t bit) {
  PORTC &= (uint8_t)~_BV(bit);
  DDRC |= _BV(bit);
}
static void line_release(uint8_t bit) {
  DDRC &= (uint8_t)~_BV(bit);
  PORTC |= _BV(bit);  // internal pull-up, as Wire enables too
}

void i2c_recover(void) {
  // A slave that was mid-byte when we reset (or lost a clock edge) can hold
  // SDA low forever, waiting for clocks. Give it up to 9 (one byte + ACK),
  // then a STOP to put every slave back to idle.
  TWCR = 0;  // take the pins back from the TWI unit
  line_release(SDA_BIT);
  line_release(SCL_BIT);
  _delay_us(5);
  for (uint8_t i = 0; i < 9 && !(PINC & _BV(SDA_BIT)); i++) {
    line_low(SCL_BIT);
    _delay_us(5);
    line_release(SCL_BIT);
    _delay_us(5);
  }
  line_low(SDA_BIT);  // STOP = SDA rising while SCL is high
  _delay_us(5);
  line_release(SDA_BIT);
  _delay_us(5);
  twi_enable();
}

void i2c_init(void) {
  i2c_budget_reset();
  line_release(SDA_BIT);
  line_release(SCL_BIT);
  i2c_recover();
}

uint16_t i2c_error_count(void) { return err_count; }

static void fail(uint8_t e) {
  if (!err) err = e;
}

// Start an action (TWCR bits) and wait for TWINT. Returns the status code,
// or 0 after a timeout (per action, or the tick budget ran out).
static uint8_t act(uint8_t twcr) {
  TWCR = (uint8_t)(twcr | _BV(TWINT) | _BV(TWEN));
  uint16_t lim = spin_limit(), n = lim;  // computed while the hardware works
  // A do-while counting down keeps the pass at 9 cycles, as before the budget.
  if (n) do {
      if (TWCR & _BV(TWINT)) {
        budget -= (uint16_t)(lim - n);
        return TW_STATUS;
      }
    } while (--n);
  budget -= lim;
  fail(I2C_TIMEOUT);
  return 0;
}

static void start(uint8_t sla) {
  if (err) return;
  uint8_t s = act(_BV(TWSTA));
  if (err) return;
  if (s != TW_START && s != TW_REP_START) {
    fail(s == TW_MT_ARB_LOST ? I2C_ARB_LOST : I2C_TIMEOUT);
    return;
  }
  TWDR = sla;
  s = act(0);
  if (err) return;
  if (s == TW_MT_SLA_ACK || s == TW_MR_SLA_ACK) return;
  fail(s == TW_MT_ARB_LOST ? I2C_ARB_LOST : I2C_NACK_ADDR);
}

void i2c_put(uint8_t b) {
  if (err) return;
  TWDR = b;
  uint8_t s = act(0);
  if (err) return;
  if (s != TW_MT_DATA_ACK) fail(s == TW_MT_ARB_LOST ? I2C_ARB_LOST : I2C_NACK_DATA);
}

uint8_t i2c_get(bool last) {
  if (err) return 0;
  uint8_t s = act(last ? 0 : _BV(TWEA));  // ACK = "send more", NACK = "done"
  if (err) return 0;
  if (s != (last ? TW_MR_DATA_NACK : TW_MR_DATA_ACK)) fail(I2C_ARB_LOST);
  return TWDR;
}

void i2c_begin_write(uint8_t addr, uint8_t reg) {
  err = budget ? I2C_OK : I2C_BUDGET;  // spent: refuse without touching the bus
  start((uint8_t)(addr << 1) | TW_WRITE);
  i2c_put(reg);
}

void i2c_begin_read(uint8_t addr, uint8_t reg) {
  i2c_begin_write(addr, reg);
  start((uint8_t)(addr << 1) | TW_READ);  // repeated start
}

uint8_t i2c_end(void) {
  if (err == I2C_TIMEOUT || err == I2C_ARB_LOST) {
    i2c_recover();
    if (!(PINC & _BV(SDA_BIT))) err = I2C_BUS_STUCK;
  } else if (err != I2C_BUDGET) {
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
    // The STOP bit clears itself once the condition is on the bus (~3 us).
    uint16_t lim = spin_limit(), n = 0;
    while (n < lim && (TWCR & _BV(TWSTO))) n++;
    budget -= n;
  }
  if (err && err_count < 0xFFFF) err_count++;
  return err;
}

uint8_t i2c_write(uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len) {
  i2c_begin_write(addr, reg);
  for (uint8_t i = 0; i < len; i++) i2c_put(data[i]);
  return i2c_end();
}

uint8_t i2c_write_P(uint8_t addr, uint8_t reg, const uint8_t* pgm, uint8_t len) {
  i2c_begin_write(addr, reg);
  for (uint8_t i = 0; i < len; i++) i2c_put(pgm_read_byte(pgm + i));
  return i2c_end();
}

uint8_t i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t v) { return i2c_write(addr, reg, &v, 1); }

uint8_t i2c_read(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  i2c_begin_read(addr, reg);
  for (uint8_t i = 0; i < len; i++) buf[i] = i2c_get(i + 1 == len);
  return i2c_end();
}
