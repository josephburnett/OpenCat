// Fake ATmega328P registers for the HAL host tests (body/test/hal).
//
// The real hal/*.cpp files compile unchanged on the host against this: the
// shim headers avr/io.h, avr/interrupt.h, avr/pgmspace.h, avr/eeprom.h,
// avr/wdt.h, util/delay.h and util/twi.h next to it (on the include path via
// -I body/test/hal) all land here. Most registers are plain volatile bytes the
// test sets and inspects. Three have behaviour, because the code under test
// polls them for a hardware response:
//   TWCR  drives FakeTwi, a TWI bus with pluggable slave devices and fault
//         injection (hangs, slow bus, arbitration loss, stuck STOP);
//   EECR  models the EEPROM write sequence (EEMPE then EEPE, interrupts off)
//         and the busy time of a byte write into fake_eeprom[];
//   _delay_us() calls a hook (the I2C recovery test clocks a stuck slave
//         with it) and sums the delay.
// ISR(v) defines a plain C function, so a test "raises an interrupt" by
// calling it (e.g. isr_USART_RX()). cli()/sei() clear/set bit 7 of SREG, so a
// test can check that code ran with interrupts off and restored SREG.
// Bit numbers and status codes are the ATmega328P datasheet's / avr-libc's.
#ifndef KILO_TEST_FAKE_AVR_H
#define KILO_TEST_FAKE_AVR_H
#include <stdint.h>
#include <vector>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define _BV(b) (1 << (b))

// ---- status register and interrupts ----
extern volatile uint8_t SREG;
#define SREG_I 7
#define cli() (SREG = (uint8_t)(SREG & ~_BV(SREG_I)))
#define sei() (SREG = (uint8_t)(SREG | _BV(SREG_I)))
#define ISR(vec)             \
  extern "C" void vec(void); \
  extern "C" void vec(void)
#define USART_RX_vect isr_USART_RX
#define USART_UDRE_vect isr_USART_UDRE
#define TIMER0_OVF_vect isr_TIMER0_OVF
#define TIMER2_COMPA_vect isr_TIMER2_COMPA
#define PCINT2_vect isr_PCINT2

// ---- USART0 ----
extern volatile uint16_t UBRR0;
extern volatile uint8_t UCSR0A, UCSR0B, UCSR0C, UDR0;
#define U2X0 1
#define DOR0 3
#define FE0 4
#define UDRE0 5
#define RXC0 7
#define TXEN0 3
#define RXEN0 4
#define UDRIE0 5
#define RXCIE0 7
#define UCSZ00 1
#define UCSZ01 2

// ---- Timer0, Timer2 ----
extern volatile uint8_t TCCR0A, TCCR0B, TIMSK0, TCNT0, TIFR0;
#define CS00 0
#define CS01 1
#define TOIE0 0
#define TOV0 0
extern volatile uint8_t TCCR2A, TCCR2B, TIMSK2, TCNT2, OCR2A;
#define WGM21 1
#define OCIE2A 1

// ---- ports and pin change ----
extern volatile uint8_t PORTC, DDRC, PINC, PORTD, DDRD, PIND;
extern volatile uint8_t PCICR, PCMSK2;
#define PCIE2 2
#define PCINT20 4
#define PCINT22 6

// ---- ADC ----
extern volatile uint8_t ADMUX, ADCSRA;
extern volatile uint16_t ADC;
#define ADPS0 0
#define ADPS1 1
#define ADPS2 2
#define ADSC 6
#define ADEN 7
#define REFS0 6

// ---- reset / watchdog ----
extern volatile uint8_t MCUSR;

// ---- TWI ----
extern volatile uint8_t TWSR, TWBR, TWDR;
#define TWIE 0
#define TWEN 2
#define TWWC 3
#define TWSTO 4
#define TWSTA 5
#define TWEA 6
#define TWINT 7

// A TWI slave. start(read): addressed with SLA+R or SLA+W. write(): a data
// byte from the master; return ACK. read(ack): the next byte for the master
// (ack = the master will ACK it, i.e. wants more). stop(): STOP on the bus.
struct TwiDevice {
  virtual ~TwiDevice() {}
  virtual void start(bool read) { (void)read; }
  virtual bool write(uint8_t b) {
    (void)b;
    return true;
  }
  virtual uint8_t read(bool ack) {
    (void)ack;
    return 0xFF;
  }
  virtual void stop() {}
};

struct TwiEvent {
  char kind;  // 'S' START, 'R' repeated START, 'A' SLA byte, 'W' data out, 'r' data in, 'P' STOP
  uint8_t v;  // the byte (A, W, r) or the status code (S, R)
};

struct FakeTwi {
  TwiDevice* dev[128];
  std::vector<TwiEvent> log;
  // Fault injection.
  long hang_at;         // this action (0-based count of TWINT actions) never completes; -1 none
  bool hang_all;        // every action hangs (a dead bus)
  uint32_t slow_polls;  // an action completes on the (slow_polls + 1)th TWCR read after it
  long arb_lost_at;     // this action reports arbitration lost; -1 none
  bool stop_hangs;      // TWSTO never clears
  // Measurements.
  long actions;               // TWINT actions started (START, SLA, data)
  uint64_t polls;             // TWCR reads since reset
  uint64_t polls_action;      // TWCR reads since the last action was started
  uint64_t max_polls_action;  // the most TWCR reads any one action saw
  uint64_t stop_polls;        // TWCR reads while a STOP was pending
  long stops, disables;       // STOPs sent; TWCR writes without TWEN (recovery)
  // The sim's own state.
  uint8_t reg;
  int state;  // 0 idle, 1 START sent, 2 master transmit, 3 master receive, 4 SLA NACKed
  TwiDevice* cur;
  bool pending;
  uint64_t pending_left;
  uint8_t pending_status;
  uint32_t stop_left;

  void reset();
  void attach(uint8_t addr, TwiDevice* d) { dev[addr & 0x7F] = d; }
  void write_twcr(uint8_t v);
  uint8_t read_twcr();
  // The bytes of the log of one kind, in order.
  std::vector<uint8_t> bytes(char kind) const;
  long count(char kind) const;
};
extern FakeTwi g_twi;

struct FakeTwcr {
  void operator=(uint8_t v) { g_twi.write_twcr(v); }
  operator uint8_t() { return g_twi.read_twcr(); }
};
extern FakeTwcr TWCR;

// ---- EEPROM ----
#define EERE 0
#define EEPE 1
#define EEMPE 2
extern volatile uint8_t EEARH, EEARL, EEDR;
extern uint8_t fake_eeprom[1024];

struct EecrWrite {
  uint8_t value;
  uint8_t sreg;  // SREG at the time of the write (bit 7 = interrupts on)
};
struct FakeEecr {
  uint8_t reg;
  uint32_t busy_left;   // EECR reads until EEPE clears
  uint32_t busy_reads;  // how long one byte write stays busy, in EECR reads
  long commits;         // byte writes performed
  long bad_sequence;    // EEPE set without EEMPE just before, or with interrupts on
  std::vector<EecrWrite> log;
  void reset();
  void operator=(uint8_t v);
  void operator|=(uint8_t v) { *this = (uint8_t)(reg | v); }  // sbi: no read side effect
  operator uint8_t();
};
extern FakeEecr EECR;

// ---- delays and the watchdog ----
extern uint64_t fake_delay_total_us;
extern void (*fake_delay_hook)(uint32_t us);
void fake_delay_us(uint32_t us);

extern long fake_wdt_resets;
extern int fake_wdt_timeout;  // last wdt_enable() argument, -1 = disabled
extern bool fake_wdt_throw;   // wdt_enable() throws FakeWdtFired (escapes sys_reboot's loop)
struct FakeWdtFired {
  int timeout;
};

// ---- RAM layout for the stack paint (hal/sys.cpp) ----
// On the AVR, _end (end of the globals) and __stack (RAMEND) are linker
// symbols. Here they are two labels around a FAKE_RAM_FREE-byte region that
// fake_avr.cpp defines in assembly (one object, so walking from one to the
// other stays in memory the test owns). The macros turn sys.cpp's
// `extern uint8_t _end;` into a declaration of a zero-length array at the
// label; &_end is the first free byte and &__stack the last.
#define FAKE_RAM_FREE 531
#define _end fake_ram_end_[0]
#define __stack fake_ram_stack_[0]

// Every fake register and model back to its power-on state, except that the
// SREG I bit is set (as it is inside the running firmware), PINC reads SDA
// and SCL high (pull-ups) and PIND idles high. No TWI devices attached.
void fake_avr_reset(void);

#endif
