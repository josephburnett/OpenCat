// mpu6050: DMP bring-up and FIFO reads for the MPU6050 at I2C 0x68.
//
// Bring-up reproduces legacy MPU6050::dmpInitialize() (MotionApps V6.12)
// register for register, then the offsets and DMP enable of legacy
// imuSetup(). It is a state machine with one small step per call, so the
// same code serves the boot (called in a loop, before the hardware watchdog
// is armed) and the runtime re-init while LIMP (one step per background()
// pass, ~4 s in total, kicking the watchdog in between).
#ifndef KILO_HAL_MPU6050_H
#define KILO_HAL_MPU6050_H
#include <stdint.h>
#include "../core/attitude.h"

#define MPU_ADDR 0x68

enum MpuInit : uint8_t { MPU_BUSY = 0, MPU_DONE = 1, MPU_FAIL = 2 };

// Start a bring-up. offsets: accel xyz, gyro xyz (config slot), copied.
void mpu_init_begin(const int16_t offsets[6]);
// One step. now_us: clock_micros(), for the two 100 ms reset waits.
MpuInit mpu_init_step(uint32_t now_us);
// Write the six offset registers now (IMU_OFFSETS_SET). I2C error or 0.
uint8_t mpu_set_offsets(const int16_t offsets[6]);
// Boot convenience: up to `attempts` full bring-ups, blocking (~300 ms each).
bool mpu_init_blocking(const int16_t offsets[6], uint8_t attempts);

// The tick's FIFO read (doc §3a step 1, Appendix B2): read FIFO_COUNT,
// then floor(count/28) whole packets, keeping the last in pkt. Never waits
// for a packet. Resets the FIFO if count > 200, or if count % 28 != 0 three
// ticks in a row (a packet boundary was lost). Returns core/body.h ImuStatus:
// 0 none, 1 fresh packet in pkt, 2 I2C error.
uint8_t mpu_fifo_read(uint8_t pkt[DMP_PACKET_SIZE]);

// Packets read since boot (STATS dmp_rate_hz is its rate) and FIFO resets.
uint16_t mpu_packet_count(void);
uint16_t mpu_fifo_resets(void);

#endif
