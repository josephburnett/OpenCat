// A register-level fake of the ATmega328P TWI unit and port C, for host
// tests that compile hal/i2c.cpp unchanged (test_i2c.cpp includes it). The
// fake <avr/io.h>, <util/twi.h>, <util/delay.h> and <avr/pgmspace.h> in this
// directory map the registers onto it. Header-only: one test uses it.
//
// Model: writing TWCR with TWINT set starts an action (START if TWSTA, STOP
// if TWSTO, otherwise sending TWDR or receiving a byte). TWINT reads back
// set after `int_after` reads of TWCR (-1 = never: a hung bus); a STOP's
// TWSTO reads set for `sto_reads` reads. TW_STATUS is what a healthy slave
// that ACKs everything produces.
#ifndef KILO_TEST_FAKE_TWI_H
#define KILO_TEST_FAKE_TWI_H
#include <cstdint>
#include <vector>

struct FakeTwi {
  int int_after = 2;    // TWCR reads until TWINT for each action (-1: never)
  int sto_reads = 1;    // TWCR reads with TWSTO still set after a STOP
  long twcr_reads = 0;  // every read of TWCR: the polling passes
  int actions = 0, starts = 0, stops = 0, disables = 0;
  std::vector<uint8_t> sent;  // bytes sent (addresses and data)
  // Registers and internal state.
  uint8_t twdr = 0, twsr = 0, twbr = 0, portc = 0, ddrc = 0, pinc = 0x30;  // SDA, SCL high
  int left = 0, sto_left = 0;
  uint8_t twcr = 0, status = 0xF8;
  bool started = false, addr_next = false;

  void reset() { *this = FakeTwi(); }

  void write_twcr(uint8_t v) {
    twcr = (uint8_t)(v & ~0x80u);  // writing TWINT 1 clears it
    if (!(v & 0x04u)) {            // TWEN off: the pins are back to the port
      disables++;
      started = false;
      return;
    }
    if (!(v & 0x80u)) return;
    actions++;
    left = int_after;
    if (v & 0x20u) {  // TWSTA
      starts++;
      status = started ? 0x10 : 0x08;
      started = addr_next = true;
    } else if (v & 0x10u) {  // TWSTO: no TWINT, TWSTO clears itself
      stops++;
      started = false;
      sto_left = sto_reads;
      left = -1;
    } else if (addr_next) {
      sent.push_back(twdr);
      addr_next = false;
      status = (twdr & 1) ? 0x40 : 0x18;
    } else if (status == 0x40 || status == 0x50) {  // receiving
      twdr = (uint8_t)actions;
      status = (v & 0x40u) ? 0x50 : 0x58;  // TWEA: ACK, else NACK
    } else {
      sent.push_back(twdr);
      status = 0x28;
    }
  }

  uint8_t read_twcr() {
    twcr_reads++;
    if (left > 0 && --left == 0) twcr |= 0x80u;
    uint8_t r = twcr;
    if (sto_left > 0) {
      r |= 0x10u;
      sto_left--;
    } else {
      twcr &= (uint8_t)~0x10u;
      r &= (uint8_t)~0x10u;
    }
    return r;
  }
};
inline FakeTwi g_twi;

struct TwcrReg {
  TwcrReg& operator=(uint8_t v) {
    g_twi.write_twcr(v);
    return *this;
  }
  operator uint8_t() const { return g_twi.read_twcr(); }
};
inline TwcrReg g_twcr_reg;

#endif
