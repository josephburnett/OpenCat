// hal/mpu6050.cpp through the real hal/i2c.cpp against a register-level
// MPU6050 model: the DMP bring-up script (register order, the 100 ms reset
// waits, the 3062 B image uploaded and verified chunk by chunk, the stored
// offsets, DMP enabled last), its failure paths and retry, and the tick's
// FIFO read (whole packets only, the newest kept, never waiting, the reset
// on a backlog over 200 B or three misaligned counts in a row, I2C errors).
#include <cstdio>
#include <vector>
#include "../check.h"
#include "fake_avr.h"
#include "fake_i2c_devices.h"
#include "../../src/hal/i2c.cpp"
#include "../../src/hal/mpu6050.cpp"

static uint32_t g_now;
// hal/clock.h for mpu_init_blocking(): 1 ms passes per call.
uint32_t clock_micros(void) {
  g_now += 1000;
  return g_now;
}

static MpuModel mpu;
static const int16_t OFFS[6] = {-1234, 567, 1688, 220, -76, -85};

static void reset_all() {
  fake_avr_reset();
  err = 0;
  err_count = 0;
  mpu.reset();
  mpu.corrupt_at = -1;
  mpu.corrupt_once = false;
  mpu.now = &g_now;
  g_now = 0;
  g_twi.attach(MPU_ADDR, &mpu);
  i2c_init();
  packets = resets = 0;
  misalign_run = 0;
}

static void packet(uint8_t tag, uint8_t* p) {
  for (uint8_t i = 0; i < DMP_PACKET_SIZE; i++) p[i] = (uint8_t)(tag + i);
}

// Run a bring-up step by step, 1 ms apart. Returns the result; *steps the count.
static MpuInit run_init(int* steps, long* max_actions) {
  mpu_init_begin(OFFS);
  MpuInit r = MPU_BUSY;
  *steps = 0;
  *max_actions = 0;
  while (r == MPU_BUSY && *steps < 10000) {
    long a = g_twi.actions;
    r = mpu_init_step(g_now);
    if (g_twi.actions - a > *max_actions) *max_actions = g_twi.actions - a;
    g_now += 1000;
    (*steps)++;
  }
  return r;
}

int main() {
  // ---- a full bring-up ----
  {
    reset_all();
    int steps;
    long max_actions;
    CHECK(run_init(&steps, &max_actions) == MPU_DONE);
    // The register writes, in order (DMP memory and its pointers excluded).
    const uint8_t expect[][2] = {
        {0x6B, 0x80}, {0x6A, 0x07}, {0x6B, 0x01}, {0x38, 0x00}, {0x23, 0x00}, {0x1C, 0x00},
        {0x37, 0x80}, {0x6B, 0x01}, {0x19, 0x04}, {0x1A, 0x01}, {0x70, 0x04}, {0x71, 0x00},
        {0x1B, 0x18}, {0x6A, 0xC0}, {0x38, 0x02}, {0x6A, 0xC4}, {0x6A, 0x40},
        // offsets: accel X/Y/Z, gyro X/Y/Z, big-endian, interleaved as written
        {0x06, 0xFB}, {0x07, 0x2E}, {0x13, 0x00}, {0x14, 0xDC}, {0x08, 0x02}, {0x09, 0x37},
        {0x15, 0xFF}, {0x16, 0xB4}, {0x0A, 0x06}, {0x0B, 0x98}, {0x17, 0xFF}, {0x18, 0xAB},
        {0x6A, 0xC0}};
    size_t n = sizeof expect / sizeof expect[0];
    bool same = mpu.writes.size() == n;
    for (size_t i = 0; same && i < n; i++)
      same = mpu.writes[i].reg == expect[i][0] && mpu.writes[i].val == expect[i][1];
    CHECKF(same, "%zu register writes", mpu.writes.size());
    // 100 ms after the device reset, and after the signal path reset.
    if (same) {
      CHECK(mpu.writes[1].t - mpu.writes[0].t >= 100000);
      CHECK(mpu.writes[2].t - mpu.writes[1].t >= 100000);
      CHECK(mpu.writes[2].t - mpu.writes[1].t < 110000);  // and not much more
    }
    // The image arrived intact, and the DMP ends up running.
    CHECK(std::memcmp(mpu.mem, dmp_image, DMP_IMAGE_SIZE) == 0);
    CHECK(mpu.regs[0x6A] == 0xC0 && mpu.regs[0x38] == 0x02 && mpu.device_resets == 1);
    CHECK(mpu.regs[0x06] == 0xFB && mpu.regs[0x07] == 0x2E);  // -1234 = 0xFB2E
    // One small step per call: the biggest is an upload chunk (point, write
    // 16 B, point, read 16 B back).
    CHECK(max_actions <= 60);
    CHECK(i2c_error_count() == 0);
    std::printf("mpu: bring-up = %d steps (%d upload chunks), at most %ld TWI actions per step\n", steps,
                (int)N_CHUNKS, max_actions);
    // The waits do not block: a step during a wait returns BUSY at once.
    reset_all();
    mpu_init_begin(OFFS);
    CHECK(mpu_init_step(0) == MPU_BUSY);  // probe
    CHECK(mpu_init_step(0) == MPU_BUSY);  // reset
    long a = g_twi.actions;
    CHECK(mpu_init_step(5000) == MPU_BUSY);  // the wait starts at its first step
    for (uint32_t t = 5000; t < 105000; t += 1000) CHECK(mpu_init_step(t) == MPU_BUSY);
    CHECK(g_twi.actions == a && mpu.writes.size() == 1);  // no I2C while waiting
    CHECK(mpu_init_step(105000) == MPU_BUSY && mpu.writes.size() == 1);  // wait over
    CHECK(mpu_init_step(105000) == MPU_BUSY && mpu.writes.size() == 2);  // the next write follows
  }

  // ---- the probe: both accepted WHO_AM_I values, and a wrong one ----
  {
    reset_all();
    mpu.regs[0x75] = 0x18;  // bits 6..1 = 0x0C
    mpu_init_begin(OFFS);
    CHECK(mpu_init_step(0) == MPU_BUSY);
    reset_all();
    mpu.regs[0x75] = 0x70;  // MPU9250-style 0x71>>1: not accepted
    mpu_init_begin(OFFS);
    CHECK(mpu_init_step(0) == MPU_FAIL);
    CHECK(mpu.writes.empty());  // nothing written to an unknown chip
  }

  // ---- failures: no device, a bad upload; blocking retries ----
  {
    reset_all();
    g_twi.attach(MPU_ADDR, nullptr);
    mpu_init_begin(OFFS);
    CHECK(mpu_init_step(0) == MPU_FAIL);
    CHECK(!mpu_init_blocking(OFFS, 3));
    CHECK(g_twi.bytes('A').size() == 4);  // the first probe and 3 more, one SLA each
    // A byte that reads back wrong fails the bring-up at that chunk.
    reset_all();
    mpu.corrupt_at = 1000;
    int steps;
    long ma;
    CHECK(run_init(&steps, &ma) == MPU_FAIL);
    CHECK(mpu.regs[0x6A] != 0xC0);  // DMP never enabled
    // mpu_init_blocking retries: a one-off bad read-back costs one attempt.
    reset_all();
    mpu.corrupt_at = 3000;
    mpu.corrupt_once = true;
    CHECK(mpu_init_blocking(OFFS, 3));
    CHECK(mpu.regs[0x6A] == 0xC0 && mpu.device_resets == 2);
  }

  // ---- mpu_set_offsets ----
  {
    reset_all();
    const int16_t o[6] = {-1, 32767, -32768, 0, 1, -2};
    CHECK(mpu_set_offsets(o) == 0);
    CHECK(mpu.regs[0x06] == 0xFF && mpu.regs[0x07] == 0xFF);
    CHECK(mpu.regs[0x08] == 0x7F && mpu.regs[0x09] == 0xFF);
    CHECK(mpu.regs[0x0A] == 0x80 && mpu.regs[0x0B] == 0x00);
    CHECK(mpu.regs[0x13] == 0 && mpu.regs[0x14] == 0 && mpu.regs[0x16] == 1 && mpu.regs[0x17] == 0xFF &&
          mpu.regs[0x18] == 0xFE);
    g_twi.attach(MPU_ADDR, nullptr);
    CHECK(mpu_set_offsets(o) != 0);
  }

  // ---- the FIFO read ----
  {
    reset_all();
    uint8_t pkt[DMP_PACKET_SIZE], p[DMP_PACKET_SIZE];
    // Empty: no packet, one count read, no FIFO read.
    long t0 = mpu.transactions;
    CHECK(mpu_fifo_read(pkt) == 0);
    CHECK(mpu.transactions - t0 == 2);  // SLA+W (pointer) and SLA+R (2 count bytes)
    // One packet.
    packet(10, p);
    mpu.push_packet(p, DMP_PACKET_SIZE);
    CHECK(mpu_fifo_read(pkt) == 1 && std::memcmp(pkt, p, DMP_PACKET_SIZE) == 0);
    CHECK(mpu_packet_count() == 1 && mpu.fifo.empty());
    // Three: all drained, the newest kept.
    for (uint8_t k = 0; k < 3; k++) {
      packet((uint8_t)(50 + k), p);
      mpu.push_packet(p, DMP_PACKET_SIZE);
    }
    CHECK(mpu_fifo_read(pkt) == 1 && pkt[0] == 52 && pkt[27] == 52 + 27);
    CHECK(mpu_packet_count() == 4 && mpu.fifo.empty());
    // 200 B (7 packets + 4): read 7, leave the partial one; no reset.
    for (uint8_t k = 0; k < 7; k++) {
      packet((uint8_t)(100 + k), p);
      mpu.push_packet(p, DMP_PACKET_SIZE);
    }
    mpu.push_packet(p, 4);
    CHECK(mpu_fifo_read(pkt) == 1 && pkt[0] == 106);
    CHECK(mpu.fifo.size() == 4 && mpu_fifo_resets() == 0 && mpu_packet_count() == 11);
    mpu.fifo.clear();
    // 201 B: too far behind (or overflowed): reset, no packet.
    for (int i = 0; i < 201; i++) mpu.fifo.push_back(0);
    CHECK(mpu_fifo_read(pkt) == 0);
    CHECK(mpu_fifo_resets() == 1 && mpu.fifo_resets == 1 && mpu.fifo.empty());
    CHECK(!mpu.writes.empty() && mpu.writes.back().reg == 0x6A && mpu.writes.back().val == 0xC4);  // DMP and FIFO stay on
    CHECK(mpu_packet_count() == 11);
    // A full hardware FIFO (1024 B) is the same case.
    for (int i = 0; i < 1024; i++) mpu.fifo.push_back(0);
    CHECK(mpu_fifo_read(pkt) == 0 && mpu_fifo_resets() == 2);
  }

  // ---- packet alignment: three misaligned counts in a row reset ----
  {
    reset_all();
    uint8_t pkt[DMP_PACKET_SIZE], p[DMP_PACKET_SIZE];
    packet(1, p);
    mpu.push_packet(p, DMP_PACKET_SIZE);
    mpu.push_packet(p, 2);  // a boundary was lost: 2 stray bytes
    CHECK(mpu_fifo_read(pkt) == 1);  // 30: the whole packet is read (run 1)
    CHECK(mpu_fifo_read(pkt) == 0);  // 2 (run 2)
    CHECK(mpu_fifo_resets() == 0);
    CHECK(mpu_fifo_read(pkt) == 0);  // 2 (run 3): reset
    CHECK(mpu_fifo_resets() == 1 && mpu.fifo.empty());
    // A DMP mid-write is normal for a tick or two: an aligned count in
    // between restarts the run.
    reset_all();
    mpu.push_packet(p, 10);
    CHECK(mpu_fifo_read(pkt) == 0);  // run 1
    mpu.fifo.clear();
    CHECK(mpu_fifo_read(pkt) == 0);  // 0 is aligned: run 0
    mpu.push_packet(p, 10);
    CHECK(mpu_fifo_read(pkt) == 0);  // run 1
    CHECK(mpu_fifo_read(pkt) == 0);  // run 2
    CHECK(mpu_fifo_resets() == 0);
    mpu.push_packet(p, 18);  // completes the packet: 28
    CHECK(mpu_fifo_read(pkt) == 1 && mpu_fifo_resets() == 0);
    // The bring-up's end clears a run left over from before.
    reset_all();
    mpu.push_packet(p, 5);
    mpu_fifo_read(pkt);
    mpu_fifo_read(pkt);
    int steps;
    long ma;
    CHECK(run_init(&steps, &ma) == MPU_DONE);
    mpu.push_packet(p, 5);
    CHECK(mpu_fifo_read(pkt) == 0 && mpu_fifo_resets() == 0);  // run 1, not 3
  }

  // ---- I2C errors: status 2, nothing counted ----
  {
    reset_all();
    uint8_t pkt[DMP_PACKET_SIZE], p[DMP_PACKET_SIZE];
    g_twi.attach(MPU_ADDR, nullptr);
    CHECK(mpu_fifo_read(pkt) == 2);
    g_twi.attach(MPU_ADDR, &mpu);
    packet(7, p);
    mpu.push_packet(p, DMP_PACKET_SIZE);
    mpu.push_packet(p, DMP_PACKET_SIZE);
    // The count read is actions 0..6; the first packet read starts at 7.
    g_twi.arb_lost_at = g_twi.actions + 7 + 10;
    CHECK(mpu_fifo_read(pkt) == 2);
    CHECK(mpu_packet_count() == 0);
  }

  return report("hal_mpu6050");
}
