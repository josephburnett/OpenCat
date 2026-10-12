#include "mpu6050.h"
#include <avr/pgmspace.h>
#include "clock.h"
#include "dmp_image.h"
#include "i2c.h"

// Registers (MPU-6000/6050 register map; names as in the legacy MPU6050.h).
#define XA_OFFS_H 0x06
#define XG_OFFS_USRH 0x13
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define FIFO_EN 0x23
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define BANK_SEL 0x6D
#define MEM_START_ADDR 0x6E
#define MEM_R_W 0x6F
#define PRGM_START_H 0x70
#define FIFO_COUNTH 0x72
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75
#define USER_CTRL_FIFO_RESET 0x04

#define CHUNK 16
#define N_CHUNKS ((DMP_IMAGE_SIZE + CHUNK - 1) / CHUNK)  // 192
#define FIFO_MAX 200

// The bring-up as a script of (register, value) writes in flash, plus a few
// pseudo-registers (0xF0+, above every real register) for the steps that are
// not plain writes. One entry runs per mpu_init_step() call (the upload one
// chunk per call), so the runtime re-init never holds the loop for long.
//
// Read-modify-writes of the legacy code are written as the constants they
// produce: every one of them runs right after a device reset or right after
// we wrote the register ourselves, so the value read is known.
#define OP_PROBE 0xF0    // WHO_AM_I must say MPU6050
#define OP_WAIT 0xF1     // wait 100 ms
#define OP_UPLOAD 0xF2   // the 3062 B image, 16 B per step, verified
#define OP_OFFSETS 0xF3  // the six stored offsets
#define OP_END 0xFF

static const uint8_t SCRIPT[] PROGMEM = {
    OP_PROBE,     0,
    PWR_MGMT_1,   0x80,  // device reset (register map p.41) ...
    OP_WAIT,      0,     // ... with a 100 ms wait
    USER_CTRL,    0x07,  // full signal path reset (FIFO, I2C master, sensors) ...
    OP_WAIT,      0,     // ... and another 100 ms
    PWR_MGMT_1,   0x01,  // clock: PLL with X gyro reference
    INT_ENABLE,   0x00,  // no interrupts
    FIFO_EN,      0x00,  // the DMP fills the FIFO, not the raw sensors
    ACCEL_CONFIG, 0x00,  // +-2 g
    INT_PIN_CFG,  0x80,  // INT pin active low
    PWR_MGMT_1,   0x01,
    SMPLRT_DIV,   0x04,  // 1 kHz / (1 + 4) = 200 Hz sample rate
    CONFIG,       0x01,  // DLPF 188 Hz
    OP_UPLOAD,    0,
    PRGM_START_H, 0x04,  // DMP program start address 0x0400
    PRGM_START_H + 1, 0x00,
    GYRO_CONFIG,  0x18,  // +-2000 deg/s
    USER_CTRL,    0xC0,  // DMP_EN | FIFO_EN (legacy's comment says "reset FIFO"; it is not)
    INT_ENABLE,   0x02,  // RAW_DMP_INT_EN
    USER_CTRL,    0xC4,  // FIFO reset "just for kicks" (the bit self-clears)
    USER_CTRL,    0x40,  // DMP off: legacy dmpInitialize() ends with it disabled,
    OP_OFFSETS,   0,     // imuSetup() then writes the stored offsets
    USER_CTRL,    0xC0,  // and only then enables the DMP
    OP_END,       0,
};

static uint8_t pc;     // script position (entry index x 2)
static uint8_t chunk;  // upload progress
static uint32_t t_wait;
static uint8_t waiting;
static int16_t offs[6];
static uint8_t misalign_run;
static uint16_t packets, resets;

static uint8_t write16(uint8_t reg, int16_t v) {
  // Offset registers are big-endian (legacy writeWord).
  uint8_t b[2] = {(uint8_t)((uint16_t)v >> 8), (uint8_t)v};
  return i2c_write(MPU_ADDR, reg, b, 2);
}

static uint8_t point(uint8_t bank, uint8_t addr) {
  return i2c_write_byte(MPU_ADDR, BANK_SEL, bank) | i2c_write_byte(MPU_ADDR, MEM_START_ADDR, addr);
}

// Upload and verify one 16 B chunk of the image. 3062 B span banks 0..11;
// 256 is a multiple of 16, so a chunk never crosses a bank boundary (legacy
// writeMemoryBlock() handles that case generally).
static bool upload_chunk(uint8_t n) {
  uint16_t off = (uint16_t)n * CHUNK;
  uint8_t len = (uint8_t)(DMP_IMAGE_SIZE - off < CHUNK ? DMP_IMAGE_SIZE - off : CHUNK);
  uint8_t bank = (uint8_t)(off >> 8), addr = (uint8_t)off;
  uint8_t back[CHUNK];
  if (point(bank, addr) || i2c_write_P(MPU_ADDR, MEM_R_W, dmp_image + off, len)) return false;
  // Verify: point at the same place again and read it back.
  if (point(bank, addr) || i2c_read(MPU_ADDR, MEM_R_W, back, len)) return false;
  for (uint8_t i = 0; i < len; i++)
    if (back[i] != pgm_read_byte(dmp_image + off + i)) return false;
  return true;
}

uint8_t mpu_set_offsets(const int16_t offsets[6]) {
  // MPU6050 layout (legacy setAccelOffset/setGyroOffset with an ID below
  // 0x38): accel at XA_OFFS_H + 2 axis, gyro at XG_OFFS_USRH + 2 axis.
  uint8_t e = 0;
  for (uint8_t i = 0; i < 3; i++) {
    e |= write16((uint8_t)(XA_OFFS_H + 2 * i), offsets[i]);
    e |= write16((uint8_t)(XG_OFFS_USRH + 2 * i), offsets[3 + i]);
  }
  return e;
}

void mpu_init_begin(const int16_t offsets[6]) {
  for (uint8_t i = 0; i < 6; i++) offs[i] = offsets[i];
  pc = 0;
  chunk = 0;
  waiting = 0;
}

MpuInit mpu_init_step(uint32_t now) {
  i2c_budget_reset();  // one step is a few short transfers, outside the tick
  uint8_t reg = pgm_read_byte(&SCRIPT[pc]);
  uint8_t val = pgm_read_byte(&SCRIPT[pc + 1]);
  uint8_t e = 0;
  switch (reg) {
    case OP_END:
      misalign_run = 0;
      return MPU_DONE;
    case OP_PROBE:
      e = i2c_read(MPU_ADDR, WHO_AM_I, &val, 1);
      val = (uint8_t)((val >> 1) & 0x3F);  // legacy getDeviceID(): bits 6..1
      if (val != 0x34 && val != 0x0C) e = 1;
      break;
    case OP_WAIT:
      if (!waiting) {
        waiting = 1;
        t_wait = now;
      }
      if (now - t_wait < 100000UL) return MPU_BUSY;
      waiting = 0;
      break;
    case OP_UPLOAD:
      if (!upload_chunk(chunk)) return MPU_FAIL;
      if (++chunk < N_CHUNKS) return MPU_BUSY;
      break;
    case OP_OFFSETS:
      e = mpu_set_offsets(offs);
      break;
    default:
      e = i2c_write_byte(MPU_ADDR, reg, val);
  }
  if (e) return MPU_FAIL;
  pc = (uint8_t)(pc + 2);
  return MPU_BUSY;
}

bool mpu_init_blocking(const int16_t offsets[6], uint8_t attempts) {
  while (attempts--) {
    mpu_init_begin(offsets);
    MpuInit r;
    do {
      r = mpu_init_step(clock_micros());
    } while (r == MPU_BUSY);
    if (r == MPU_DONE) return true;
  }
  return false;
}

static void fifo_reset(void) {
  // USER_CTRL is 0xC0 while running (DMP and FIFO on): set FIFO_RESET too.
  i2c_write_byte(MPU_ADDR, USER_CTRL, 0xC0 | USER_CTRL_FIFO_RESET);
  misalign_run = 0;
  resets++;
}

uint8_t mpu_fifo_read(uint8_t pkt[DMP_PACKET_SIZE]) {
  uint8_t c[2];
  if (i2c_read(MPU_ADDR, FIFO_COUNTH, c, 2)) return 2;
  uint16_t count = (uint16_t)((c[0] << 8) | c[1]);
  if (count > FIFO_MAX) {
    // Too far behind (or overflowed, which corrupts packet alignment):
    // reading it all would cost more than waiting 10 ms for a fresh one.
    fifo_reset();
    return 0;
  }
  // A count that is not a whole number of packets is normal for a moment
  // (the DMP is mid-write); three ticks in a row means we lost alignment.
  if (count % DMP_PACKET_SIZE) {
    if (++misalign_run >= 3) {
      fifo_reset();
      return 0;
    }
  } else {
    misalign_run = 0;
  }
  uint8_t n = (uint8_t)(count / DMP_PACKET_SIZE);
  if (!n) return 0;
  // Drain whole packets; the last one read is the newest and stays in pkt.
  while (n--) {
    if (i2c_read(MPU_ADDR, FIFO_R_W, pkt, DMP_PACKET_SIZE)) return 2;
    packets++;
  }
  return 1;
}

uint16_t mpu_packet_count(void) { return packets; }
uint16_t mpu_fifo_resets(void) { return resets; }
