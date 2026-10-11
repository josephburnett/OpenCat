// The body firmware's entry point: boot sequence (doc §3a "Boot sequence")
// and the polled 50 Hz loop.
//
// We define main() ourselves, so the Arduino core's main()/init() are never
// linked: no HardwareSerial, no Timer0/1/2 setup we did not ask for. The
// hal/ modules set up exactly the peripherals they use.
#include "main.h"
#include "../features.h"  // KILO_ULTRASONIC / KILO_BUZZER / KILO_IR (default on)
#include <avr/interrupt.h>
#include "../core/joints.h"
#include "../core/link.h"
#include "../core/params.h"
#include "../core/sched.h"
#include "../hal/adc.h"
#include "../hal/buzzer.h"
#include "../hal/clock.h"
#include "../hal/eeprom.h"
#include "../hal/gpio.h"
#include "../hal/i2c.h"
#include "../hal/ir.h"
#include "../hal/mpu6050.h"
#include "../hal/pca9685.h"
#include "../hal/pcint2.h"
#include "../hal/sys.h"
#include "../hal/uart.h"
#include "../hal/ultrasonic.h"

// Build ids. HELLO.fw_build is the git hash that `kilo flash` passes as
// -DKILO_BUILD=0x<8 hex digits> (git rev-parse --short=8), so the mind can
// check it flashed what it built; without it (a plain arduino-cli build) it
// is an FNV-1a hash of the compile date and time. The reset record needs a
// value that differs between any two builds (a rebuilt dirty tree has the
// same git hash), so it also mixes in the compile time.
static constexpr uint32_t fnv(const char* s, uint32_t h) {
  return *s ? fnv(s + 1, (h ^ (uint8_t)*s) * 16777619u) : h;
}
// constexpr variable: forces the hash at compile time (no string in RAM).
static constexpr uint32_t BUILD_TIME_HASH = fnv(__DATE__ " " __TIME__, 2166136261u);
#ifdef KILO_BUILD
#define FW_BUILD ((uint32_t)(KILO_BUILD))
#else
#define FW_BUILD BUILD_TIME_HASH
#endif
#define RESET_BUILD_ID (FW_BUILD ^ BUILD_TIME_HASH ^ 0x4B494C4FUL)

#define IMU_INIT_ATTEMPTS 3
#define BATT_PERIOD_TICKS BATT_SAMPLE_TICKS

Body g_body;
Config g_config;
BodyStats g_stats;
uint16_t g_counts[16];
uint8_t g_pkt[DMP_PACKET_SIZE];

static Sched sched;
static uint8_t imu_present;   // the DMP came up at boot or after a re-init
static uint8_t imu_reinit;    // runtime re-init in progress (LIMP only)
static uint32_t imu_retry_at; // after a failed attempt, wait before the next

static void run_tick(void) {
  uint32_t t0 = clock_micros();
  Body& b = g_body;
  TickIn in;
  in.imu = IMU_NONE;
  in.batt_valid = 0;
  // 1. IMU: whole packets from the FIFO, keep the newest. 2. Attitude.
  if (imu_present && !b.imu_fault && !imu_reinit) {
    in.imu = mpu_fifo_read(g_pkt);
    if (in.imu == IMU_FRESH && !attitude_from_packet(g_pkt, in.att)) in.imu = IMU_NONE;
  }
  // 3. Goals that arrived during the IMU read.
  link_poll();
  // The battery sample started on an earlier tick (non-blocking ADC).
  uint16_t raw;
  if (adc_take(&raw)) {
    in.batt_valid = 1;
    in.batt_counts = raw;
    g_stats.batt_counts = raw;
  }
  // 4-6. Reflexes, owners, crossfade, layers, limits, slew, counts.
  body_tick(b, in, g_counts);
  // 7.
  link_poll();
  // 8. One burst for all 16 channels (errors are counted by the I2C driver;
  //    the PCA keeps the previous pose, and the next tick tries again).
  pca_write_all(g_counts);
  // 9. Side sensors on their own cadences.
#if KILO_ULTRASONIC
  if (b.tick % (uint8_t)PARAM(US_PERIOD_TICKS) == 0) us_trigger(clock_micros());
#endif
  if (b.tick % BATT_PERIOD_TICKS == 0) adc_start();
#if KILO_BUZZER
  if (notes_tick(b.notes, TICK_MS)) buzzer_set(b.notes.hz);
#endif
  // 10. Telemetry, watchdog, canary, timing.
  link_tick();
  sys_wdt_kick();
  sys_reset_record.alive_tick = b.tick;
  sys_reset_record.epoch = b.epoch;  // the next boot starts past it
  uint32_t dt32 = clock_micros() - t0;
  uint16_t dt = (uint16_t)(dt32 > 0xFFFF ? 0xFFFF : dt32);
  if (dt > g_stats.max_tick_us) g_stats.max_tick_us = dt;
  if (dt > g_stats.tick_us_window) g_stats.tick_us_window = dt;
  g_stats.overruns = sched.overruns;
}

static void background(void) {
  uint32_t now = clock_micros();
  config_writer_step(g_config);
  if (g_config.saved) {
    g_config.saved = 0;
    events_push(g_body.ev, EV_EEPROM_SAVED, 0, g_body.tick, g_body.epoch);
  }
#if KILO_ULTRASONIC
  uint16_t mm;
  if (us_poll(now, &mm)) {
    g_stats.dist_mm = mm;
    g_stats.dist_fresh = 1;
  }
#endif
#if KILO_IR
  uint16_t key;
  if (ir_poll(&key)) events_push(g_body.ev, EV_IR_KEY, (int16_t)key, g_body.tick, g_body.epoch);
#endif
  // IMU re-init only while limp: the upload takes seconds of I2C, and
  // nothing may depend on attitude meanwhile (doc §3a, §3g).
  if (g_body.imu_fault && body_mode(g_body) == 1) {
    if (!imu_reinit) {
      if ((int32_t)(now - imu_retry_at) < 0) return;  // back off: a missing IMU
      mpu_init_begin(g_config.img.imu_offset);        // must not flood the bus
      imu_reinit = 1;
    }
    MpuInit r = mpu_init_step(now);
    if (r != MPU_BUSY) {
      imu_reinit = 0;
      if (r == MPU_DONE) {
        imu_present = 1;
        body_imu_recovered(g_body);
      } else {
        imu_retry_at = now + 5000000UL;  // try again in 5 s
      }
    }
  } else {
    imu_reinit = 0;  // left LIMP: abandon (restarts from scratch next time)
  }
}

int main(void) {
  // 1. Reset record (before anything can overwrite it) and the UART.
  g_stats.build = FW_BUILD;
  g_stats.reset_cause = reset_classify(sys_reset_record, RESET_BUILD_ID, g_stats.last_alive_tick);
  clock_init();
  uart_init();
  led_init();
#if KILO_BUZZER
  buzzer_init();
#endif
  adc_init();
#if KILO_ULTRASONIC
  us_init();
#endif
#if KILO_IR
  ir_init();
#endif
  pcint2_init();
  sei();
  // Config first: it is only EEPROM reads, and the warm-boot pose needs the
  // calibration to turn counts back into angles.
  config_boot(g_config);
  params_load(g_config.img);
  // 2. I2C and the PCA9685: warm (keep the pose it is driving) or cold.
  i2c_init();
  int16_t warm[DRIVEN_COUNT];
  bool is_warm = pca_probe_warm(g_counts);
  if (is_warm) {
    for (uint8_t k = 0; k < DRIVEN_COUNT; k++) {
      uint8_t j = joint_id(k);
      warm[k] = joint_angle_from_counts(g_config.img.cal, j, g_counts[joint_channel(j)]);
    }
  } else {
    pca_init_cold();
  }
  // 3. DMP bring-up, before the hardware watchdog (it blocks ~300 ms).
  led_on();
  imu_present = mpu_init_blocking(g_config.img.imu_offset, IMU_INIT_ATTEMPTS);
  led_off();
  body_init(g_body, &g_config, is_warm ? warm : 0, imu_present, sys_reset_record.epoch);
  // 4. HELLO and EVENT BOOT, then the watchdog.
  uint16_t caps = (KILO_BUZZER ? KILO_HELLO_CAPS_BUZZER : 0) | (imu_present ? KILO_HELLO_CAPS_DMP : 0) |
                  (KILO_ULTRASONIC ? KILO_HELLO_CAPS_ULTRASONIC : 0) | (KILO_IR ? KILO_HELLO_CAPS_IR : 0);
  link_init(FW_BUILD, caps);
  link_boot(g_stats.reset_cause, g_stats.last_alive_tick);
  events_push(g_body.ev, EV_BOOT, g_stats.reset_cause, 0, g_body.epoch);
  if (g_config.state == CFG_MIGRATED) events_push(g_body.ev, EV_CONFIG_MIGRATED, 0, 0, g_body.epoch);
  if (g_config.state == CFG_DEFAULTED) events_push(g_body.ev, EV_CONFIG_DEFAULTED, g_config.reason, 0, g_body.epoch);
  if (!imu_present) events_push(g_body.ev, EV_IMU_FAULT, 0, 0, g_body.epoch);
  sys_wdt_arm();
  // 5. The loop: poll the link always, tick on the 20 ms grid, otherwise
  //    do background work. Nothing here blocks.
  sched_start(sched, clock_micros());
  for (;;) {
    link_poll();
    if (sched_due(sched, clock_micros()))
      run_tick();
    else
      background();
  }
}
