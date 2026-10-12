// Register-level models of the NyBoard's two I2C chips, as FakeTwi slaves:
// enough of the MPU6050 (register file, DMP memory banks, FIFO) and the
// PCA9685 (register file, auto-increment, ALL_LED, sleep-only prescaler,
// outputs latched at STOP) for hal/mpu6050.cpp and hal/pca9685.cpp to run
// against them through the real hal/i2c.cpp.
#ifndef KILO_TEST_FAKE_I2C_DEVICES_H
#define KILO_TEST_FAKE_I2C_DEVICES_H
#include <cstring>
#include <deque>
#include <vector>
#include "fake_avr.h"

struct RegWrite {
  uint8_t reg, val;
  uint32_t t;  // the model's clock (*now) at the write
};

// Common part: the first byte after SLA+W is the register pointer, then
// data bytes go to on_write(); reads come from on_read().
struct RegDevice : TwiDevice {
  uint8_t ptr = 0;
  bool expect_ptr = false;
  long transactions = 0;  // SLA+W or SLA+R
  void start(bool read) override {
    transactions++;
    expect_ptr = !read;
  }
  bool write(uint8_t b) override {
    if (expect_ptr) {
      ptr = b;
      expect_ptr = false;
    } else {
      on_write(ptr, b);
      ptr = next(ptr);
    }
    return true;
  }
  long reads = 0;        // data bytes read by the master
  bool last_ack = false; // whether the master ACKed the latest one (wanted more)
  uint8_t read(bool ack) override {
    reads++;
    last_ack = ack;
    uint8_t v = on_read(ptr);
    ptr = next(ptr);
    return v;
  }
  virtual void on_write(uint8_t reg, uint8_t v) = 0;
  virtual uint8_t on_read(uint8_t reg) = 0;
  virtual uint8_t next(uint8_t reg) { return (uint8_t)(reg + 1); }
};

struct MpuModel : RegDevice {
  enum { BANK_SEL = 0x6D, MEM_START = 0x6E, MEM_R_W = 0x6F, FIFO_COUNTH = 0x72, FIFO_R_W = 0x74,
         USER_CTRL = 0x6A, PWR_MGMT_1 = 0x6B, WHO_AM_I = 0x75 };
  uint8_t regs[128];
  uint8_t mem[12 * 256];
  uint8_t bank = 0, mem_addr = 0;
  std::deque<uint8_t> fifo;
  long fifo_resets = 0, device_resets = 0;
  int corrupt_at = -1;  // DMP memory offset that reads back wrong (a bad upload)
  bool corrupt_once = false;  // ... only the first time it is read
  const uint32_t* now = nullptr;
  std::vector<RegWrite> writes;  // every register write except DMP memory and its pointers
  MpuModel() { reset(); }
  void reset() {
    std::memset(regs, 0, sizeof regs);
    std::memset(mem, 0, sizeof mem);
    regs[WHO_AM_I] = 0x68;
    regs[PWR_MGMT_1] = 0x40;
    fifo.clear();
    writes.clear();
    fifo_resets = device_resets = 0;
    transactions = 0;
  }
  void push_packet(const uint8_t* p, uint8_t n) { fifo.insert(fifo.end(), p, p + n); }
  uint8_t next(uint8_t reg) override {
    return reg == FIFO_R_W || reg == MEM_R_W ? reg : (uint8_t)(reg + 1);
  }
  void on_write(uint8_t reg, uint8_t v) override {
    uint16_t m = (uint16_t)(bank * 256 + mem_addr);
    switch (reg) {
      case BANK_SEL: bank = v; return;
      case MEM_START: mem_addr = v; return;
      case MEM_R_W:
        if (m < sizeof mem) mem[m] = v;
        mem_addr++;
        return;
      default: break;
    }
    writes.push_back({reg, v, now ? *now : 0});
    if (reg == USER_CTRL && (v & 0x04)) {
      fifo.clear();
      fifo_resets++;
      v = (uint8_t)(v & ~0x04);  // self-clearing
    }
    if (reg == PWR_MGMT_1 && (v & 0x80)) {
      device_resets++;
      v = 0x40;  // back to sleep, reset bit self-clears
    }
    if (reg < 128) regs[reg] = v;
  }
  uint8_t on_read(uint8_t reg) override {
    uint16_t m = (uint16_t)(bank * 256 + mem_addr);
    switch (reg) {
      case MEM_R_W: {
        uint8_t v = m < sizeof mem ? mem[m] : 0;
        if ((int)m == corrupt_at) {
          v ^= 0x01;
          if (corrupt_once) corrupt_at = -1;
        }
        mem_addr++;
        return v;
      }
      case FIFO_COUNTH: return (uint8_t)(fifo.size() >> 8);
      case FIFO_COUNTH + 1: return (uint8_t)fifo.size();
      case FIFO_R_W: {
        if (fifo.empty()) return 0;
        uint8_t v = fifo.front();
        fifo.pop_front();
        return v;
      }
      default: return reg < 128 ? regs[reg] : 0;
    }
  }
};

struct PcaModel : RegDevice {
  enum { MODE1 = 0x00, MODE2 = 0x01, LED0 = 0x06, ALL_LED = 0xFA, PRESCALE = 0xFE };
  uint8_t regs[256];
  uint16_t out_off[16];      // the OFF count (incl. bit 12 full-off) the outputs drive
  long latches = 0;          // output updates (one per STOP that followed writes)
  long prescale_ignored = 0; // PRESCALE written while awake (the chip ignores it)
  bool dirty = false;
  std::vector<RegWrite> writes;
  PcaModel() { power_on(); }
  void power_on() {
    std::memset(regs, 0, sizeof regs);
    regs[MODE1] = 0x11;  // SLEEP | ALLCALL
    regs[MODE2] = 0x04;
    regs[PRESCALE] = 0x1E;
    for (int c = 0; c < 16; c++) regs[LED0 + 4 * c + 3] = 0x10;  // full off
    latch();
    latches = 0;
    writes.clear();
    transactions = 0;
  }
  // A chip that has been driving `counts` (what a warm boot finds).
  void driving(const uint16_t counts[16], uint8_t prescale = 24) {
    regs[MODE1] = 0x20;  // awake, auto-increment
    regs[PRESCALE] = prescale;
    for (int c = 0; c < 16; c++) {
      regs[LED0 + 4 * c] = 0;
      regs[LED0 + 4 * c + 1] = 0;
      regs[LED0 + 4 * c + 2] = (uint8_t)counts[c];
      regs[LED0 + 4 * c + 3] = (uint8_t)(counts[c] >> 8);
    }
    latch();
  }
  void latch() {
    for (int c = 0; c < 16; c++)
      out_off[c] = (uint16_t)(regs[LED0 + 4 * c + 2] | ((regs[LED0 + 4 * c + 3] & 0x1F) << 8));
    latches++;
  }
  uint8_t next(uint8_t reg) override { return (regs[MODE1] & 0x20) ? (uint8_t)(reg + 1) : reg; }
  void on_write(uint8_t reg, uint8_t v) override {
    writes.push_back({reg, v, 0});
    dirty = true;
    if (reg == PRESCALE && !(regs[MODE1] & 0x10)) {
      prescale_ignored++;
      return;
    }
    if (reg >= ALL_LED && reg <= ALL_LED + 3) {
      for (int c = 0; c < 16; c++) regs[LED0 + 4 * c + (reg - ALL_LED)] = v;
      return;
    }
    if (reg == MODE1) v = (uint8_t)(v & ~0x80);  // RESTART: writing 1 clears it
    regs[reg] = v;
  }
  uint8_t on_read(uint8_t reg) override { return reg >= ALL_LED && reg <= ALL_LED + 3 ? 0 : regs[reg]; }
  void stop() override {
    if (dirty) latch();  // MODE2.OCH = 0: outputs change on STOP
    dirty = false;
  }
};

#endif
