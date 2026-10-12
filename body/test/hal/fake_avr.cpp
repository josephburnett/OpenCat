// The fake ATmega328P of fake_avr.h: register storage, the TWI bus model,
// the EEPROM write model, delays and the watchdog.
#include "fake_avr.h"
#include <cstring>

volatile uint8_t SREG;
volatile uint16_t UBRR0;
volatile uint8_t UCSR0A, UCSR0B, UCSR0C, UDR0;
volatile uint8_t TCCR0A, TCCR0B, TIMSK0, TCNT0, TIFR0;
volatile uint8_t TCCR2A, TCCR2B, TIMSK2, TCNT2, OCR2A;
volatile uint8_t PORTC, DDRC, PINC, PORTD, DDRD, PIND;
volatile uint8_t PCICR, PCMSK2;
volatile uint8_t ADMUX, ADCSRA;
volatile uint16_t ADC;
volatile uint8_t MCUSR;
volatile uint8_t TWSR, TWBR, TWDR;
volatile uint8_t EEARH, EEARL, EEDR;
uint8_t fake_eeprom[1024];
FakeTwi g_twi;
FakeTwcr TWCR;
FakeEecr EECR;
uint64_t fake_delay_total_us;
void (*fake_delay_hook)(uint32_t us);
long fake_wdt_resets;
int fake_wdt_timeout;
bool fake_wdt_throw;

// The free-RAM region between _end and __stack (see fake_avr.h).
#define FAKE_STR2(x) #x
#define FAKE_STR(x) FAKE_STR2(x)
asm(".pushsection .bss\n"
    ".balign 16\n"
    ".globl fake_ram_end_\n"
    "fake_ram_end_:\n"
    ".zero " FAKE_STR(FAKE_RAM_FREE) "-1\n"
    ".globl fake_ram_stack_\n"
    "fake_ram_stack_:\n"
    ".zero 16\n"
    ".popsection\n");

// ---- TWI ----

static const uint64_t NEVER = ~(uint64_t)0;

void FakeTwi::reset() {
  for (auto& d : dev) d = nullptr;
  log.clear();
  hang_at = -1;
  hang_all = false;
  slow_polls = 0;
  arb_lost_at = -1;
  stop_hangs = false;
  actions = 0;
  polls = polls_action = max_polls_action = stop_polls = 0;
  stops = disables = 0;
  reg = 0;
  state = 0;
  cur = nullptr;
  pending = false;
  pending_left = 0;
  pending_status = 0;
  stop_left = 0;
}

void FakeTwi::write_twcr(uint8_t v) {
  if (!(v & _BV(TWEN))) {  // TWI off: the pins are plain GPIO again
    reg = v;
    state = 0;
    cur = nullptr;
    pending = false;
    stop_left = 0;
    disables++;
    return;
  }
  if (!(v & _BV(TWINT))) {  // no action: writing 0 to TWINT leaves it as it is
    reg = (uint8_t)((reg & _BV(TWINT)) | (v & ~_BV(TWINT)));
    return;
  }
  reg = (uint8_t)(v & ~_BV(TWINT));  // writing 1 clears TWINT and starts the action
  if (v & _BV(TWSTO)) {
    if (cur) cur->stop();
    log.push_back({'P', 0});
    stops++;
    cur = nullptr;
    state = 0;
    pending = false;
    stop_left = stop_hangs ? 0xFFFFFFFFu : 2;  // TWSTO clears itself a few cycles later
    return;
  }
  long idx = actions++;
  polls_action = 0;
  uint8_t st;
  if (v & _BV(TWSTA)) {
    st = state == 0 ? 0x08 : 0x10;
    log.push_back({state == 0 ? 'S' : 'R', st});
    state = 1;
    cur = nullptr;
  } else if (state == 1) {
    uint8_t sla = TWDR;
    bool rd = sla & 1;
    log.push_back({'A', sla});
    cur = dev[sla >> 1];
    if (cur) {
      cur->start(rd);
      state = rd ? 3 : 2;
      st = rd ? 0x40 : 0x18;
    } else {
      state = 4;
      st = rd ? 0x48 : 0x20;
    }
  } else if (state == 2) {
    uint8_t b = TWDR;
    log.push_back({'W', b});
    st = cur->write(b) ? 0x28 : 0x30;
  } else if (state == 3) {
    bool ack = v & _BV(TWEA);
    uint8_t b = cur->read(ack);
    TWDR = b;
    log.push_back({'r', b});
    st = ack ? 0x50 : 0x58;
  } else {
    st = 0x30;  // data after an unanswered SLA: nobody acknowledges
  }
  if (idx == arb_lost_at) {
    st = 0x38;
    state = 0;
    cur = nullptr;
  }
  pending = true;
  pending_status = st;
  pending_left = (hang_all || idx == hang_at) ? NEVER : slow_polls;
}

uint8_t FakeTwi::read_twcr() {
  polls++;
  polls_action++;
  if (polls_action > max_polls_action) max_polls_action = polls_action;
  if (pending) {
    if (pending_left == 0) {
      pending = false;
      reg = (uint8_t)(reg | _BV(TWINT));
      TWSR = pending_status;
    } else if (pending_left != NEVER) {
      pending_left--;
    }
  }
  if (stop_left) {
    stop_polls++;
    if (stop_left != 0xFFFFFFFFu && --stop_left == 0) reg = (uint8_t)(reg & ~_BV(TWSTO));
  }
  return reg;
}

std::vector<uint8_t> FakeTwi::bytes(char kind) const {
  std::vector<uint8_t> out;
  for (const TwiEvent& e : log)
    if (e.kind == kind) out.push_back(e.v);
  return out;
}

long FakeTwi::count(char kind) const {
  long n = 0;
  for (const TwiEvent& e : log) n += e.kind == kind;
  return n;
}

// ---- EEPROM ----

void FakeEecr::reset() {
  reg = 0;
  busy_left = 0;
  busy_reads = 3;
  commits = 0;
  bad_sequence = 0;
  log.clear();
}

void FakeEecr::operator=(uint8_t v) {
  bool irq_off = !(SREG & _BV(SREG_I));
  bool prev_mpe = !log.empty() && (log.back().value & _BV(EEMPE)) && !(log.back().sreg & _BV(SREG_I));
  log.push_back({v, SREG});
  if ((v & _BV(EEPE)) && !(reg & _BV(EEPE))) {
    // EEPE starts a write only if EEMPE was set by the write just before
    // (within 4 cycles, so with interrupts off throughout).
    if ((reg & _BV(EEMPE)) && prev_mpe && irq_off) {
      uint16_t a = (uint16_t)(((EEARH << 8) | EEARL) & 0x3FF);
      fake_eeprom[a] = EEDR;
      commits++;
      busy_left = busy_reads;
      reg = _BV(EEPE);  // EEMPE has cleared itself by now
      return;
    }
    bad_sequence++;
    reg = (uint8_t)(v & ~_BV(EEPE));
    return;
  }
  reg = (uint8_t)((v & ~_BV(EEPE)) | (reg & _BV(EEPE)));  // EEPE cannot be cleared by software
}

FakeEecr::operator uint8_t() {
  if (reg & _BV(EEPE)) {
    if (busy_left)
      busy_left--;
    else
      reg = (uint8_t)(reg & ~_BV(EEPE));
  }
  return reg;
}

uint8_t eeprom_read_byte(const uint8_t* addr) {
  while (EECR & _BV(EEPE)) {
  }
  return fake_eeprom[(uintptr_t)addr & 0x3FF];
}

// ---- delays and the watchdog ----

void fake_delay_us(uint32_t us) {
  fake_delay_total_us += us;
  if (fake_delay_hook) fake_delay_hook(us);
}

void wdt_enable(int timeout) {
  fake_wdt_timeout = timeout;
  if (fake_wdt_throw) throw FakeWdtFired{timeout};
}
void wdt_disable(void) { fake_wdt_timeout = -1; }
void wdt_reset(void) { fake_wdt_resets++; }

void fake_avr_reset(void) {
  SREG = _BV(SREG_I);
  UBRR0 = 0;
  UCSR0A = UCSR0B = UCSR0C = UDR0 = 0;
  TCCR0A = TCCR0B = TIMSK0 = TCNT0 = TIFR0 = 0;
  TCCR2A = TCCR2B = TIMSK2 = TCNT2 = OCR2A = 0;
  PORTC = DDRC = PORTD = DDRD = 0;
  PINC = _BV(4) | _BV(5);  // SDA, SCL pulled up
  PIND = 0xFF;
  PCICR = PCMSK2 = 0;
  ADMUX = ADCSRA = 0;
  ADC = 0;
  MCUSR = 0;
  TWSR = TWBR = TWDR = 0;
  EEARH = EEARL = EEDR = 0;
  std::memset(fake_eeprom, 0xFF, sizeof fake_eeprom);
  g_twi.reset();
  EECR.reset();
  fake_delay_total_us = 0;
  fake_delay_hook = nullptr;
  fake_wdt_resets = 0;
  fake_wdt_timeout = -1;
  fake_wdt_throw = false;
}
