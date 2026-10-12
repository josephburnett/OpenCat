// hal/i2c.cpp on the fake TWI bus: the transaction state machine (START,
// SLA, data, repeated START, ACK/NACK on reads, STOP), the error codes, the
// short-circuit after the first error, the per-action spin bound, the bus
// recovery (SCL pulses until a stuck slave lets go, never driving a line
// high) and the error counter. Also measures the polling bounds that
// docs/code-review-2026-10-11.md §2.2 calls weak: a dead bus costs one
// bounded wait per transfer, a slow (not dead) bus is bounded only per
// action, so per transfer it scales with the byte count.
#include <cstdio>
#include <vector>
#include "../check.h"
#include "fake_avr.h"
#include "../../src/hal/i2c.cpp"

// A device that ACKs writes, records them, and answers reads from a script.
struct Dev : TwiDevice {
  std::vector<uint8_t> written;
  std::vector<bool> read_acks;
  std::vector<uint8_t> replies;
  size_t ri = 0;
  int nack_at = -1;  // refuse the data byte with this index
  int starts = 0, stops = 0;
  void start(bool) override { starts++; }
  bool write(uint8_t b) override {
    int i = (int)written.size();
    written.push_back(b);
    return i != nack_at;
  }
  uint8_t read(bool ack) override {
    read_acks.push_back(ack);
    return ri < replies.size() ? replies[ri++] : 0xFF;
  }
  void stop() override { stops++; }
};

// SCL pulses seen by the recovery, and a slave holding SDA low until
// `release_after` pulses (-1 = forever, 0 = not stuck).
static int scl_pulses, release_after, driven_high;
static bool scl_was_low;
static void line_watch(uint32_t) {
  if (DDRC & PORTC & (_BV(4) | _BV(5))) driven_high++;  // open drain: never drive a 1
  bool scl_low = DDRC & _BV(5);
  if (scl_low && !scl_was_low) {
    scl_pulses++;
    if (release_after >= 0 && scl_pulses >= release_after) PINC = (uint8_t)(PINC | _BV(4));
  }
  scl_was_low = scl_low;
}
static void stuck_sda(int after) {
  release_after = after;
  PINC = (uint8_t)(PINC & ~_BV(4));
}

static Dev dev;
static void reset_all() {
  fake_avr_reset();
  err = 0;
  err_count = 0;
  dev = Dev();
  g_twi.attach(0x40, &dev);
  scl_pulses = 0;
  release_after = 0;
  driven_high = 0;
  scl_was_low = false;
  fake_delay_hook = line_watch;
}

int main() {
  // ---- init: lines released with pull-ups, 400 kHz, TWI on, a STOP ----
  reset_all();
  i2c_init();
  CHECK((DDRC & 0x30) == 0 && (PORTC & 0x30) == 0x30);
  CHECK(TWBR == 12 && TWSR == 0 && g_twi.reg == _BV(TWEN));
  CHECK(scl_pulses == 0);  // SDA was high: no clocking needed
  CHECK(driven_high == 0);

  // ---- a register write ----
  {
    reset_all();
    i2c_init();
    uint8_t d[3] = {1, 2, 3};
    CHECK(i2c_write(0x40, 0x06, d, 3) == I2C_OK);
    std::vector<uint8_t> w = {0x06, 1, 2, 3};
    CHECK(dev.written == w);
    CHECK(g_twi.bytes('A') == std::vector<uint8_t>({0x80}));
    CHECK(g_twi.count('S') == 1 && g_twi.count('R') == 0 && g_twi.count('P') == 1);
    CHECK(dev.starts == 1 && dev.stops == 1);
    CHECK(g_twi.actions == 6);  // START, SLA, 4 bytes
    CHECK(i2c_error_count() == 0);
    // flash variant and one-byte variant
    static const uint8_t P[2] PROGMEM = {0xAB, 0xCD};
    dev.written.clear();
    CHECK(i2c_write_P(0x40, 0x10, P, 2) == I2C_OK);
    CHECK(dev.written == std::vector<uint8_t>({0x10, 0xAB, 0xCD}));
    dev.written.clear();
    CHECK(i2c_write_byte(0x40, 0xFE, 24) == I2C_OK);
    CHECK(dev.written == std::vector<uint8_t>({0xFE, 24}));
  }

  // ---- a register read: repeated START, ACK all but the last byte ----
  {
    reset_all();
    i2c_init();
    dev.replies = {0x11, 0x22, 0x33};
    uint8_t buf[3] = {};
    CHECK(i2c_read(0x40, 0x72, buf, 3) == I2C_OK);
    CHECK(buf[0] == 0x11 && buf[1] == 0x22 && buf[2] == 0x33);
    CHECK(dev.written == std::vector<uint8_t>({0x72}));
    CHECK(g_twi.bytes('A') == std::vector<uint8_t>({0x80, 0x81}));
    CHECK(g_twi.count('S') == 1 && g_twi.count('R') == 1 && g_twi.count('P') == 1);
    CHECK(dev.read_acks == std::vector<bool>({true, true, false}));
    // one byte: NACKed at once
    dev.read_acks.clear();
    dev.replies = {0x68};
    dev.ri = 0;
    CHECK(i2c_read(0x40, 0x75, buf, 1) == I2C_OK && buf[0] == 0x68);
    CHECK(dev.read_acks == std::vector<bool>({false}));
  }

  // ---- nobody at the address: NACK_ADDR, STOP (no recovery), short-circuit ----
  {
    reset_all();
    i2c_init();
    long dis = g_twi.disables;
    uint8_t d[64] = {};
    CHECK(i2c_write(0x41, 0x00, d, 64) == I2C_NACK_ADDR);
    CHECK(g_twi.actions == 2);  // START and SLA only: the 65 bytes were skipped
    CHECK(g_twi.count('P') == 1 && g_twi.disables == dis);
    CHECK(i2c_error_count() == 1);
    uint8_t b[2];
    CHECK(i2c_read(0x41, 0x00, b, 2) == I2C_NACK_ADDR);
    CHECK(g_twi.actions == 4 && i2c_error_count() == 2);
    // The next transfer starts clean.
    CHECK(i2c_write(0x40, 0x00, d, 1) == I2C_OK && i2c_error_count() == 2);
  }

  // ---- the slave refuses a data byte: NACK_DATA, the rest skipped ----
  {
    reset_all();
    i2c_init();
    dev.nack_at = 2;  // reg, byte 0 accepted; byte 1 refused
    uint8_t d[10] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
    CHECK(i2c_write(0x40, 0x06, d, 10) == I2C_NACK_DATA);
    CHECK(dev.written.size() == 3);
    CHECK(g_twi.count('P') == 1 && i2c_error_count() == 1);
  }

  // ---- a hung action: timeout after exactly SPIN_LIMIT polls, recovery ----
  {
    reset_all();
    i2c_init();
    g_twi.hang_at = 3;  // START, SLA, reg ok; the first data byte never completes
    uint64_t polls0 = g_twi.polls;
    long dis = g_twi.disables;
    uint64_t d0 = fake_delay_total_us;
    uint8_t d[64] = {};
    CHECK(i2c_write(0x40, 0x06, d, 64) == I2C_TIMEOUT);
    CHECK(g_twi.actions == 4);  // nothing after the hung byte
    // 3 actions answered on their first poll, the hung one polled SPIN_LIMIT times.
    CHECK(g_twi.polls - polls0 == 3 + SPIN_LIMIT);
    CHECK(g_twi.max_polls_action == SPIN_LIMIT);
    CHECK(SPIN_LIMIT == 3000);  // the documented bound (about 1 ms at 16 MHz)
    // Recovery, not a STOP through the TWI unit: TWCR off, lines by hand, TWI back on.
    CHECK(g_twi.count('P') == 0 && g_twi.disables == dis + 1 && g_twi.reg == _BV(TWEN) && TWBR == 12);
    CHECK(scl_pulses == 0);  // SDA was not held
    CHECK(i2c_error_count() == 1);
    std::printf("i2c: one hung action = %u TWCR polls; dead-bus transfer: %llu polls + %llu us of recovery delays\n",
                (unsigned)SPIN_LIMIT, (unsigned long long)(g_twi.polls - polls0),
                (unsigned long long)(fake_delay_total_us - d0));
    CHECK(fake_delay_total_us - d0 == 15);  // 5 us settle + the STOP's 2 x 5 us
  }

  // ---- a dead bus with SDA held: 9 pulses, then BUS_STUCK ----
  {
    reset_all();
    i2c_init();
    g_twi.hang_all = true;
    stuck_sda(-1);
    uint64_t d0 = fake_delay_total_us;
    uint8_t b[28];
    CHECK(i2c_read(0x40, 0x74, b, 28) == I2C_BUS_STUCK);
    CHECK(g_twi.actions == 1);  // the START hung; nothing else was tried
    CHECK(scl_pulses == 9);
    CHECK(driven_high == 0);
    CHECK(i2c_error_count() == 1);
    // Worst-case recovery delay: 5 + 9 x (5 + 5) + 5 + 5 us.
    CHECK(fake_delay_total_us - d0 == 5 + 9 * 10 + 10);
    // A slave that lets go after 3 clocks: recovered, reported as the timeout.
    reset_all();
    i2c_init();
    g_twi.hang_at = 1;
    stuck_sda(3);
    CHECK(i2c_write_byte(0x40, 0x00, 0) == I2C_TIMEOUT);
    CHECK(scl_pulses == 3 && (PINC & _BV(4)));
    // And init itself frees a bus held from before the reset.
    reset_all();
    stuck_sda(5);
    i2c_init();
    CHECK(scl_pulses == 5 && driven_high == 0);
  }

  // ---- arbitration lost (or noise) at each stage: ARB_LOST, recovered ----
  {
    for (long at = 0; at < 4; at++) {
      reset_all();
      i2c_init();
      g_twi.arb_lost_at = at;  // START, SLA, reg, data
      long dis = g_twi.disables;
      uint8_t d[4] = {};
      uint8_t e = i2c_write(0x40, 0x06, d, 4);
      CHECKF(e == I2C_ARB_LOST, "arb lost at action %ld: err %u", at, e);
      CHECK(g_twi.actions == at + 1);
      CHECK(g_twi.disables > dis);  // recovery ran
    }
    // While reading, any unexpected status is treated as arbitration lost.
    reset_all();
    i2c_init();
    g_twi.arb_lost_at = 6;  // START SLA reg RSTART SLA, byte 0 ok, byte 1 lost
    uint8_t b[4];
    CHECK(i2c_read(0x40, 0x00, b, 4) == I2C_ARB_LOST);
    CHECK(g_twi.actions == 7);
  }

  // ---- the streaming API: every call after an error is a no-op ----
  {
    reset_all();
    i2c_init();
    i2c_begin_write(0x41, 0x00);  // NACK at SLA
    long a = g_twi.actions;
    uint64_t p = g_twi.polls;
    for (int i = 0; i < 64; i++) i2c_put(0x55);
    CHECK(i2c_get(true) == 0);
    CHECK(g_twi.actions == a && g_twi.polls == p);  // not a single register access
    CHECK(i2c_end() == I2C_NACK_ADDR);
    // begin_write clears the previous transfer's error
    i2c_begin_write(0x40, 0x00);
    i2c_put(1);
    CHECK(i2c_end() == I2C_OK);
  }

  // ---- the error counter saturates ----
  {
    reset_all();
    i2c_init();
    err_count = 0xFFFE;
    CHECK(i2c_write_byte(0x41, 0, 0) == I2C_NACK_ADDR && i2c_error_count() == 0xFFFF);
    CHECK(i2c_write_byte(0x41, 0, 0) == I2C_NACK_ADDR && i2c_error_count() == 0xFFFF);
  }

  // ---- STOP that never completes: bounded, and (finding) not an error ----
  {
    reset_all();
    i2c_init();
    g_twi.stop_hangs = true;
    uint64_t sp = g_twi.stop_polls;
    CHECK(i2c_write_byte(0x40, 0, 0) == I2C_OK);  // reported as success
    CHECK(g_twi.stop_polls - sp == SPIN_LIMIT);    // after SPIN_LIMIT polls
    CHECK(i2c_error_count() == 0);                 // and not counted
  }

  // ---- a slow (not dead) bus: every action just inside the limit ----
  {
    reset_all();
    i2c_init();
    g_twi.slow_polls = SPIN_LIMIT - 1;  // TWINT on the last allowed poll
    uint64_t p0 = g_twi.polls;
    uint8_t d[64] = {};
    // The PCA9685 burst of every tick: START, SLA, register, 64 bytes.
    CHECK(i2c_write(0x40, 0x06, d, 64) == I2C_OK);
    uint64_t burst = g_twi.polls - p0;
    CHECK(burst == 67ull * SPIN_LIMIT + 2);  // + the STOP's 2 polls
    CHECK(i2c_error_count() == 0);
    // One more poll per action and the first action times out instead.
    reset_all();
    i2c_init();
    g_twi.slow_polls = SPIN_LIMIT;
    CHECK(i2c_write(0x40, 0x06, d, 64) == I2C_TIMEOUT);
    std::printf("i2c: slow bus, no error: 64-byte PCA burst = %llu TWCR polls (67 actions x %u)\n",
                (unsigned long long)burst, (unsigned)SPIN_LIMIT);
  }

  return report("hal_i2c");
}
