// The hardware side of core/link: UART rings, PCA9685 read-back, MPU
// offsets, reboot, and the STATS / TELEMETRY values the app measures.
#include "../core/link.h"
#include "../hal/clock.h"
#include "../hal/i2c.h"
#include "../hal/mpu6050.h"
#include "../hal/pca9685.h"
#include "../hal/sys.h"
#include "../hal/uart.h"
#include "main.h"

bool link_hal_read(uint8_t* c) { return uart_read(c); }
uint8_t link_hal_tx_free(void) { return uart_tx_free(); }
uint16_t link_hal_us(void) { return (uint16_t)clock_micros(); }
void link_hal_write(const uint8_t* data, uint8_t len) { uart_write(data, len, 0); }

void link_hal_sample(LinkSample* s) {
  s->dist_mm = g_stats.dist_mm;
  s->dist_fresh = g_stats.dist_fresh;
  g_stats.dist_fresh = 0;
  uint16_t us = g_stats.tick_us_window;
  g_stats.tick_us_window = 0;
  s->tick_load = (uint8_t)(us >= 25500 ? 255 : (us + 99) / 100);
  s->overrun = g_stats.overruns != g_stats.overruns_seen;
  g_stats.overruns_seen = g_stats.overruns;
}

void link_hal_stats(kilo_stats_t* s) {
  UartStats u;
  uart_stats(&u);
  s->overruns = g_stats.overruns;
  s->max_tick_us = g_stats.max_tick_us;
  s->stack_free_min = sys_stack_free_min();
  s->rx_overflow = u.rx_overflow;
  s->i2c_err = i2c_error_count();
}

uint16_t link_hal_imu_packets(void) { return mpu_packet_count(); }

void link_hal_imu_raw(int16_t raw[6]) {
  // MotionApps 6.12 packet: accel at bytes 16..21, gyro at 22..27, big-endian.
  for (uint8_t i = 0; i < 6; i++) raw[i] = (int16_t)((g_pkt[16 + 2 * i] << 8) | g_pkt[17 + 2 * i]);
}

bool link_hal_pca_read(uint16_t counts[16]) { return pca_read_all(counts) == I2C_OK; }

void link_hal_imu_offsets(const int16_t offsets[6]) { mpu_set_offsets(offsets); }

void link_hal_reboot(void) { sys_reboot(); }
