#include "ir_nec.h"

enum : uint8_t { PH_NONE = 0, PH_LEAD = 1, PH_BITS = 2 };

void ir_nec_init(IrNec& s) {
  s.phase = PH_NONE;
  s.have_cmd = 0;
  s.key = IR_NEC_NONE;
}

// Code size matters here (it runs in the PCINT2 ISR on the AVR): past the
// timeout check the interval fits 16 bits, and the repeat window is kept in
// 256 us units (t >> 8 is a byte move).
#define T256(t) ((uint16_t)((t) >> 8))

// AVR: keep it out of line and unspecialised (noinline, noclone; the build
// uses LTO). Through the reference the fields are reached with 2-byte
// displacement loads; inlined at the fixed address of the one IrNec they
// become 4-byte lds/sts, about 60 B more.
#ifdef __AVR__
#define IR_NEC_OUT_OF_LINE __attribute__((noinline, noclone))
#else
#define IR_NEC_OUT_OF_LINE
#endif

IR_NEC_OUT_OF_LINE bool ir_nec_edge(IrNec& s, uint32_t t) {
  uint32_t dt32 = t - s.t_last;
  s.t_last = t;
  uint8_t ph = s.phase;
  if (ph == PH_NONE || dt32 > IR_NEC_TIMEOUT_US) {
    // A gap longer than the repeat window ends any held key (this also keeps
    // the 16-bit t_valid comparison below from ever wrapping).
    if (ph == PH_NONE || dt32 > IR_NEC_REPEAT_US) s.have_cmd = 0;
    s.phase = PH_LEAD;
    return false;
  }
  uint16_t dt = (uint16_t)dt32;
  if (ph == PH_LEAD) {
    if (dt >= 12000 && dt <= 15000) {
      s.bits = 0;
      s.phase = PH_BITS;
    } else if (dt >= 10000 && dt <= 12500 && s.have_cmd &&
               (uint16_t)(T256(t) - s.t_valid) <= (uint16_t)(IR_NEC_REPEAT_US >> 8)) {
      s.t_valid = T256(t);
      s.key = s.cmd | IR_NEC_REPEAT;
      return true;
    }
    return false;  // otherwise this edge is the new reference
  }
  uint32_t v = s.bits << 1;
  if (dt >= 1800 && dt <= 2800)
    v |= 1;
  else if (dt < 800 || dt > 1500) {
    s.phase = PH_LEAD;
    return false;
  }
  s.bits = v;
  if (++ph < PH_BITS + 32) {
    s.phase = ph;
    return false;
  }
  s.phase = PH_LEAD;
  uint8_t addr = (uint8_t)(v >> 24), naddr = (uint8_t)(v >> 16);
  uint8_t cmd = (uint8_t)(v >> 8), ncmd = (uint8_t)v;
  if ((uint8_t)(addr ^ naddr) != 0xFF || (uint8_t)(cmd ^ ncmd) != 0xFF) {
    s.have_cmd = 0;
    return false;
  }
  s.cmd = cmd;
  s.have_cmd = 1;
  s.t_valid = T256(t);
  s.key = cmd;
  return true;
}
