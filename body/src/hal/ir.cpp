#include "../features.h"
#if KILO_IR
#include "ir.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../core/ir_nec.h"

static IrNec nec;  // written by the PCINT2 ISR; key read with interrupts off

void ir_init(void) {
  ir_nec_init(nec);
  // D4 is an input after reset (DDRD4 = 0, no pull-up: the receiver drives
  // the line, and the legacy firmware used plain INPUT too). Enable PCINT20;
  // hal/pcint2.cpp enables the group. Boot code, before sei().
  PCMSK2 |= _BV(PCINT20);
}

void ir_edge(uint32_t t_us) { ir_nec_edge(nec, t_us); }

bool ir_poll(uint16_t* key) {
  uint8_t sreg = SREG;
  cli();
  uint16_t k = nec.key;
  nec.key = IR_NEC_NONE;
  SREG = sreg;
  if (k == IR_NEC_NONE) return false;
  *key = k;
  return true;
}
#endif
