// The TWI driver (hal/i2c.cpp), compiled unchanged against a register fake
// (fake_avr/): the healthy fast path, the per-action timeout on a hung bus,
// and the per-tick budget that turns a slow (not dead) bus into counted
// errors instead of an overrun tick (doc §3a; code review 2026-10-11 §2.2).
#include "../src/hal/i2c.cpp"
#include "check.h"

static const uint8_t PCA = 0x40, MPU = 0x68;

static void fresh(int int_after) {
  g_twi.reset();
  g_twi.int_after = int_after;
  i2c_init();  // also refills the budget
  g_twi.twcr_reads = 0;
  g_twi.actions = g_twi.starts = g_twi.stops = g_twi.disables = 0;
}

int main() {
  uint8_t data[64];
  for (uint8_t i = 0; i < 64; i++) data[i] = (uint8_t)(i * 3);

  // Healthy bus: the PCA burst (START, address, register, 64 data bytes:
  // 67 actions, then STOP) succeeds; each action is polled
  // until TWINT (int_after reads) and only those passes are charged.
  fresh(3);
  uint16_t errs = i2c_error_count();
  CHECK(i2c_write(PCA, 0x06, data, 64) == I2C_OK);
  CHECK(g_twi.starts == 1 && g_twi.stops == 1 && g_twi.actions == 1 + 1 + 1 + 64 + 1);
  CHECK(g_twi.sent.size() == 66 && g_twi.sent[0] == (PCA << 1) && g_twi.sent[1] == 0x06 && g_twi.sent[65] == 63 * 3);
  // 67 actions x 3 reads, plus two STOP reads (TWSTO set, then clear).
  CHECKF(g_twi.twcr_reads == 67 * 3 + 2, "reads %ld", g_twi.twcr_reads);
  // Charged: 2 passes before TWINT per action, 1 for the STOP.
  CHECKF(budget == TICK_SPIN_BUDGET - 67 * 2 - 1, "budget %u", budget);
  CHECK(i2c_error_count() == errs);
  // A read: repeated START, NACK on the last byte.
  uint8_t buf[2] = {0, 0};
  CHECK(i2c_read(MPU, 0x72, buf, 2) == I2C_OK);
  CHECK(g_twi.starts == 3 && g_twi.stops == 2 && buf[0] != 0 && buf[1] != 0);

  // Hung bus: an action that never completes times out after SPIN_LIMIT
  // passes; the bus is recovered (TWI disabled and re-enabled) and the
  // error is counted.
  fresh(-1);
  errs = i2c_error_count();
  CHECK(i2c_write_byte(PCA, 0x00, 0x11) == I2C_TIMEOUT);
  CHECKF(g_twi.twcr_reads == SPIN_LIMIT, "reads %ld", g_twi.twcr_reads);
  CHECK(g_twi.actions == 1 && g_twi.disables == 1 && i2c_error_count() == errs + 1);
  CHECK(budget == TICK_SPIN_BUDGET - SPIN_LIMIT);

  // Slow bus: every action takes 2000 passes (under the per-action limit,
  // so no single action times out). Unbudgeted, the burst alone would poll
  // 67 x 2000 = 134000 passes (~75 ms). With the budget it stops after
  // TICK_SPIN_BUDGET passes, as I2C_TIMEOUT, and the bus is recovered.
  fresh(2000);
  errs = i2c_error_count();
  CHECK(i2c_write(PCA, 0x06, data, 64) == I2C_TIMEOUT);
  CHECKF(g_twi.twcr_reads <= TICK_SPIN_BUDGET + 10, "reads %ld", g_twi.twcr_reads);
  CHECK(g_twi.actions < 67 && g_twi.disables == 1 && budget == 0);
  // The rest of the tick: every transfer is refused without touching the
  // bus, and counted (so the IMU read becomes IMU_ERR, the PCA keeps its
  // pose, and i2c_err in STATS shows it).
  long reads = g_twi.twcr_reads;
  int actions = g_twi.actions;
  CHECK(i2c_read(MPU, 0x72, buf, 2) == I2C_BUDGET);
  CHECK(i2c_write(PCA, 0x06, data, 64) == I2C_BUDGET);
  CHECK(g_twi.twcr_reads == reads && g_twi.actions == actions);
  CHECK(i2c_error_count() == errs + 3);
  // The next tick refills it: a healthy bus works again.
  g_twi.int_after = 3;
  i2c_budget_reset();
  CHECK(i2c_write(PCA, 0x06, data, 64) == I2C_OK);

  // A budget spent exactly at a transfer boundary: the next one is refused
  // before START (no bus activity, no recovery).
  fresh(3);
  budget = 0;
  actions = g_twi.actions;
  CHECK(i2c_write_byte(PCA, 0x00, 0x11) == I2C_BUDGET);
  CHECK(g_twi.actions == actions && g_twi.disables == 0);

  // A slow STOP is charged to the budget too, and bounded by it.
  fresh(3);
  g_twi.sto_reads = 1000000;
  budget = 500;
  CHECK(i2c_write_byte(PCA, 0x00, 0x11) == I2C_OK);
  CHECK(budget == 0 && g_twi.twcr_reads <= 3 * 3 + 500);

  return report("i2c");
}
