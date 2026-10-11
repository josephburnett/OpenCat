// ir_nec: a receive-only NEC infrared decoder (the Petoi remote), pure logic.
//
// The IR receiver's output idles high and goes low during a mark, so every
// NEC symbol starts with a falling edge and the decoder only needs the times
// of the falling edges (hal/ir.cpp stamps them in the pin-change ISR and
// calls ir_nec_edge there; doc §3a). Intervals between falling edges:
//
//   leader  9 + 4.5   = 13.5  ms   window 12..15    ms
//   bit 0   0.56+0.56 =  1.125 ms  window  0.8..1.5 ms
//   bit 1   0.56+1.69 =  2.25  ms  window  1.8..2.8 ms
//   repeat  9 + 2.25  = 11.25 ms   window 10..12.5  ms
//
// 32 bit intervals after the leader complete a frame (the last bit ends at
// the stop mark's falling edge). Bits are collected MSB first, as the legacy
// IRremote library did, so the legacy code 0x00FFA25D is address 0x00, ~0xFF,
// command 0xA2, ~0x5D. A frame counts only if both bytes match their
// inverses. A repeat frame (key held) yields the last command | 0x100, but
// only if a valid frame or repeat ended within IR_NEC_REPEAT_US; a lone
// repeat is ignored. Any out-of-range interval, or a gap over
// IR_NEC_TIMEOUT_US, drops the partial frame and makes the edge the new
// reference (it may be the start of a leader).
#ifndef KILO_CORE_IR_NEC_H
#define KILO_CORE_IR_NEC_H
#include <stdint.h>

#define IR_NEC_TIMEOUT_US 20000UL
#define IR_NEC_REPEAT_US 150000UL
#define IR_NEC_REPEAT 0x100
#define IR_NEC_NONE 0xFFFF  // IrNec.key when nothing is pending

struct IrNec {
  uint32_t t_last;   // the previous falling edge
  uint32_t bits;     // data bits so far, MSB first
  uint16_t t_valid;  // end of the last valid frame/repeat, in 256 us units
  uint8_t phase;     // 0 no edge yet, 1 waiting for a leader, 2+n: n bits in
  uint8_t cmd;       // the last valid command (for repeats)
  uint8_t have_cmd;  // cmd is valid and t_valid means something
  uint16_t key;      // the decoded key (command | IR_NEC_REPEAT), or IR_NEC_NONE
};

void ir_nec_init(IrNec& s);
// One falling edge at t_us (wrapping microseconds). True when it completed
// a frame or a valid repeat; the result is then in s.key.
bool ir_nec_edge(IrNec& s, uint32_t t_us);

#endif
