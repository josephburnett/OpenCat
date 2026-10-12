// The PCINT2 path on fake registers: hal/pcint2.cpp's shared ISR (PIND edge
// detection), hal/ir.cpp's NEC capture over core/ir_nec, hal/ultrasonic.cpp's
// echo timing and hal/clock.cpp's micros (the ISR's time stamps). Each edge
// is a PIND change at a set Timer0 time followed by the interrupt, as on the
// board: the IR receiver's falling AND rising edges both interrupt (only the
// falling ones are NEC symbols), and so do the ultrasonic echo's.
#include <vector>
#include "../check.h"
#include "fake_avr.h"
#include "../../src/hal/clock.cpp"
#include "../../src/hal/pcint2.cpp"
#include "../../src/hal/ir.cpp"
#include "../../src/hal/ultrasonic.cpp"
#include "../../src/core/ir_nec.cpp"

// The Petoi remote's 21 command bytes (mind/kilo/ir.py KEYS, src/infrared.h).
static const uint8_t KEYS[21] = {0xA2, 0x62, 0xE2, 0x22, 0x02, 0xC2, 0xE0, 0xA8, 0x90, 0x68, 0x98,
                                 0xB0, 0x30, 0x18, 0x7A, 0x10, 0x38, 0x5A, 0x42, 0x4A, 0x52};

// Timer0 state that makes clock_micros() return t (a multiple of 4 us).
static void set_time(uint32_t t) {
  uint32_t q = t / 4;
  ovf = q >> 8;
  TCNT0 = (uint8_t)q;
  TIFR0 = 0;
}

// D4 (IR) and D6 (ultrasonic) levels change at time t: one interrupt.
static void pin(uint8_t bit, bool high, uint32_t t) {
  set_time(t);
  PIND = (uint8_t)(high ? PIND | _BV(bit) : PIND & ~_BV(bit));
  isr_PCINT2();
}

// The receiver output of one NEC frame (address byte a, command c, and
// their inverses unless overridden) starting at t: a 9 ms low leader, 4.5 ms
// high, 32 bits of 560 us low + 560/1690 us high, a 560 us stop mark.
static uint32_t ir_frame(uint32_t t, uint32_t v) {
  pin(4, false, t);
  pin(4, true, t + 9000);
  t += 13500;
  for (int8_t i = 31; i >= -1; i--) {
    pin(4, false, t);
    pin(4, true, t + 560);
    if (i >= 0) t += (v >> i) & 1 ? 2250 : 1125;
  }
  return t + 560;
}
static uint32_t nec_code(uint8_t addr, uint8_t cmd) {
  return ((uint32_t)addr << 24) | ((uint32_t)(uint8_t)~addr << 16) | ((uint32_t)cmd << 8) | (uint8_t)~cmd;
}
// A repeat frame: 9 ms low, 2.25 ms high, the stop mark.
static void ir_repeat(uint32_t t) {
  pin(4, false, t);
  pin(4, true, t + 9000);
  pin(4, false, t + 11250);
  pin(4, true, t + 11810);
}

static std::vector<uint16_t> keys() {
  std::vector<uint16_t> out;
  uint16_t k;
  while (ir_poll(&k)) out.push_back(k);
  return out;
}

static void reset_all() {
  fake_avr_reset();
  ovf = 0;
  prev_pind = 0xFF;
  clock_init();
  us_init();
  ir_init();
  pcint2_init();
}

int main() {
  // ---- init: the masks and Timer0 ----
  reset_all();
  CHECK(PCICR == _BV(PCIE2));
  CHECK(PCMSK2 == _BV(PCINT20));  // IR on, ultrasonic off until a ping
  CHECK(TCCR0A == 0 && TCCR0B == (_BV(CS01) | _BV(CS00)) && TIMSK0 == _BV(TOIE0));
  PCMSK2 = 0x81;
  ir_init();
  CHECK(PCMSK2 == (0x81 | _BV(PCINT20)));  // ir_init only sets its own bit

  // ---- clock_micros: 4 us per count, the overflow ISR, a pending overflow ----
  reset_all();
  CHECK(clock_micros() == 0);
  TCNT0 = 10;
  CHECK(clock_micros() == 40);
  isr_TIMER0_OVF();
  isr_TIMER0_OVF();
  CHECK(clock_micros() == (2u * 256 + 10) * 4);
  TCNT0 = 3;  // wrapped with the overflow interrupt still pending
  TIFR0 = _BV(TOV0);
  CHECK(clock_micros() == (3u * 256 + 3) * 4);
  TCNT0 = 254;
  CHECK(clock_micros() == (3u * 256 + 254) * 4);  // still counted at 254
  TCNT0 = 255;  // read just before the wrap; the flag is from just after: not counted
  CHECK(clock_micros() == (2u * 256 + 255) * 4);
  CHECK(SREG == _BV(SREG_I));
  cli();
  clock_micros();
  CHECK(SREG == 0);  // safe in an ISR: leaves interrupts off
  sei();
  set_time(0xFFFFFFFCu);  // wraps at 2^32 us
  CHECK(clock_micros() == 0xFFFFFFFCu);

  // ---- every key of the remote, through the real ISR ----
  {
    reset_all();
    uint32_t t = 1000000;
    bool all = true;
    for (uint8_t k : KEYS) {
      ir_frame(t, nec_code(0x00, k));
      std::vector<uint16_t> got = keys();
      if (got.size() != 1 || got[0] != k) all = false;
      t += 300000;
    }
    CHECK(all);
    CHECK(SREG == _BV(SREG_I));  // ir_poll restores SREG
  }

  // ---- held key: repeat frames every 108 ms yield cmd | 0x100 ----
  {
    reset_all();
    uint32_t t0 = 2000000;
    ir_frame(t0, nec_code(0x00, 0x18));
    CHECK(keys() == std::vector<uint16_t>({0x18}));
    ir_repeat(t0 + 108000);
    CHECK(keys() == std::vector<uint16_t>({0x118}));
    ir_repeat(t0 + 216000);
    ir_repeat(t0 + 324000);
    CHECK(keys() == std::vector<uint16_t>({0x118}));  // one pending key, the newest
    // A repeat long after the last frame is ignored (a lone repeat).
    ir_repeat(t0 + 1000000);
    CHECK(keys().empty());
    // A lone repeat at power-up too.
    reset_all();
    ir_repeat(500000);
    CHECK(keys().empty());
  }

  // ---- noise ----
  {
    reset_all();
    // Glitches before the leader do not stop the frame from decoding.
    uint32_t t = 3000000;
    for (int i = 0; i < 5; i++) {
      pin(4, false, t);
      pin(4, true, t + 40);
      t += 300;
    }
    ir_frame(t + 2000, nec_code(0x00, 0xA2));
    CHECK(keys() == std::vector<uint16_t>({0xA2}));
    // A glitch inside a frame drops it: no key, and no wrong key.
    reset_all();
    t = 4000000;
    pin(4, false, t);
    pin(4, true, t + 9000);
    t += 13500;
    for (int i = 0; i < 16; i++) {
      pin(4, false, t);
      pin(4, true, t + 560);
      t += 1125;
    }
    pin(4, false, t - 400);  // a glitch 400 us into a bit
    pin(4, true, t - 380);
    for (int i = 0; i < 17; i++) {
      pin(4, false, t);
      pin(4, true, t + 560);
      t += 1125;
    }
    CHECK(keys().empty());
    // The decoder recovers for the next frame.
    ir_frame(t + 50000, nec_code(0x00, 0x62));
    CHECK(keys() == std::vector<uint16_t>({0x62}));
    // A frame cut short (the remote moved away) yields nothing.
    reset_all();
    t = 5000000;
    pin(4, false, t);
    pin(4, true, t + 9000);
    t += 13500;
    for (int i = 0; i < 20; i++) {
      pin(4, false, t);
      pin(4, true, t + 560);
      t += 2250;
    }
    CHECK(keys().empty());
    ir_frame(t + 100000, nec_code(0x00, 0xE2));
    CHECK(keys() == std::vector<uint16_t>({0xE2}));
  }

  // ---- bad address / command checks ----
  {
    reset_all();
    uint32_t t = 6000000;
    ir_frame(t, 0x10FFA25DUL);  // address byte not the inverse of its complement
    CHECK(keys().empty());
    ir_frame(t + 300000, 0x00FFA25CUL);  // command inverse wrong
    CHECK(keys().empty());
    // and a repeat after a rejected frame is not a held key either
    ir_repeat(t + 408000);
    CHECK(keys().empty());
    // Another NEC remote with a consistent address is decoded (the mind
    // shows it as "? (0x45)"; the body does not filter addresses).
    ir_frame(t + 900000, nec_code(0x04, 0x45));
    CHECK(keys() == std::vector<uint16_t>({0x45}));
  }

  // ---- only falling edges of D4 count; other PORTD pins do not ----
  {
    reset_all();
    uint32_t t = 7000000;
    // A full frame where every D4 edge comes with a D3/D5/D7 toggle on its
    // own interrupt (the LED, buzzer and eyes share PORTD; inputs or not, a
    // PIND change on another bit must not be read as an IR edge).
    pin(4, false, t);
    pin(5, false, t + 100);
    pin(5, true, t + 200);
    pin(4, true, t + 9000);
    t += 13500;
    uint32_t v = nec_code(0x00, 0x90);
    for (int8_t i = 31; i >= -1; i--) {
      pin(4, false, t);
      pin(7, (i & 1) != 0, t + 100);
      pin(4, true, t + 560);
      if (i >= 0) t += (v >> i) & 1 ? 2250 : 1125;
    }
    CHECK(keys() == std::vector<uint16_t>({0x90}));
  }

  // ---- ultrasonic: trigger pulse, echo timing, timeout ----
  {
    reset_all();
    uint16_t mm = 1;
    CHECK(!us_busy() && !us_poll(0, &mm));
    PIND = (uint8_t)(PIND & ~_BV(6));  // the module holds the line low
    uint64_t d0 = fake_delay_total_us;
    us_trigger(10000000);
    CHECK(fake_delay_total_us - d0 == 22);  // 2 us low + 20 us pulse
    CHECK(!(DDRD & _BV(6)) && !(PORTD & _BV(6)));  // input, no pull-up
    CHECK(PCMSK2 == (_BV(PCINT20) | _BV(PCINT22)));
    CHECK(us_busy());
    CHECK(!us_poll(10000100, &mm));
    // Echo: high at +500 us for 5831 us (1000 mm at 343 m/s, there and back).
    pin(6, true, 10000500);
    CHECK(us_busy());
    pin(6, false, 10000500 + 5832);
    CHECK(!us_busy());
    CHECK(PCMSK2 == _BV(PCINT20));  // the ISR switched D6 off, D4 kept
    CHECK(us_poll(10020000, &mm) && mm == 5832UL * 1715 / 10000);
    CHECK(mm == 1000);
    CHECK(!us_poll(10020000, &mm));  // once per measurement
    // No echo: false until 25 ms after the trigger, then 0 mm once.
    us_trigger(11000000);
    CHECK(!us_poll(11000000 + US_TIMEOUT_US - 4, &mm));
    CHECK(us_poll(11000000 + US_TIMEOUT_US, &mm) && mm == 0);
    CHECK(PCMSK2 == _BV(PCINT20) && !us_busy());
    // The timeout compares across the 2^32 us wrap.
    us_trigger(0xFFFFF000u);
    CHECK(!us_poll(0x00000100u, &mm));
    CHECK(us_poll((uint32_t)(0xFFFFF000u + US_TIMEOUT_US), &mm) && mm == 0);
  }

  // ---- an IR frame during an echo: both are decoded ----
  {
    reset_all();
    PIND = (uint8_t)(PIND & ~_BV(6));
    uint32_t t = 20000000;
    us_trigger(t);
    // IR edges arrive while waiting for the rise and during the echo; the
    // ultrasonic sees D6 low/high on those interrupts and must not act on
    // them before its own edges.
    pin(4, false, t + 100);  // IR leader starts: D6 still low, waiting for rise
    CHECK(us_busy());
    pin(6, true, t + 400);  // echo rises
    pin(4, true, t + 9100);  // IR leader high: D6 high, waiting for fall: ignored
    CHECK(us_busy());
    uint32_t ti = t + 100 + 13500;
    uint32_t v = nec_code(0x00, 0x30);
    uint32_t t_fall = 0;
    for (int8_t i = 31; i >= -1; i--) {
      pin(4, false, ti);
      pin(4, true, ti + 560);
      if (!t_fall && ti > t + 20000) {  // the echo falls mid-frame (~3.4 m)
        t_fall = (ti + 700) & ~3u;
        pin(6, false, t_fall);
      }
      if (i >= 0) ti += (v >> i) & 1 ? 2250 : 1125;
    }
    uint16_t mm = 0;
    CHECK(t_fall != 0);
    CHECK(us_poll(ti, &mm) && mm == (t_fall - (t + 400)) * 1715 / 10000);
    CHECK(keys() == std::vector<uint16_t>({0x30}));
  }

  return report("hal_pcint");
}
