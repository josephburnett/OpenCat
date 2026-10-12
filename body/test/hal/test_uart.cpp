// hal/uart.cpp on fake registers: the ring arithmetic (96 B TX, 192 B RX,
// neither a power of two, so the wrap is explicit), all-or-nothing writes and
// the drop hook, RX overflow and hardware-error counting, and the ISR
// contract: the UDRE interrupt is enabled whenever bytes are queued, drains
// them in order one per interrupt, and switches itself off when empty.
#include <cstdlib>
#include <deque>
#include <vector>
#include "../check.h"
#include "fake_avr.h"
#include "../../src/hal/uart.cpp"

static bool udrie() { return UCSR0B & _BV(UDRIE0); }

// One UDRE interrupt; returns the byte it sent, or -1 if it sent nothing.
static int16_t udre() {
  UDR0 = 0;
  uint8_t before = UCSR0B;
  isr_USART_UDRE();
  if (!(UCSR0B & _BV(UDRIE0)) && (before & _BV(UDRIE0))) return -1;
  return UDR0;
}

static void rx_byte(uint8_t c, uint8_t status = 0) {
  UCSR0A = status;
  UDR0 = c;
  isr_USART_RX();
}

static int drops, drop_prio, drop_len;
static void hook(uint8_t prio, uint8_t len) {
  drops++;
  drop_prio = prio;
  drop_len = len;
}

static void reset_all() {
  fake_avr_reset();
  rx_head = rx_tail = tx_head = tx_tail = 0;
  rx_overflow = rx_hw_err = 0;
  drop_hook = nullptr;
  drops = 0;
  UCSR0A = _BV(U2X0);  // optiboot leaves U2X set
  uart_init();
}

int main() {
  // ---- init: 250000 baud at 16 MHz, U2X cleared, 8N1, RX interrupt only ----
  reset_all();
  CHECK(UBRR0 == 3 && UCSR0A == 0);
  CHECK(UCSR0C == (_BV(UCSZ01) | _BV(UCSZ00)));
  CHECK(UCSR0B == (_BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0)));
  CHECK(uart_tx_free() == UART_TX_SIZE - 1);  // one slot always open: 95 B usable

  // ---- a write queues, enables UDRE, restores SREG; the ISR drains in order ----
  {
    reset_all();
    uint8_t msg[10];
    for (uint8_t i = 0; i < 10; i++) msg[i] = (uint8_t)(0xA0 + i);
    CHECK(uart_write(msg, 10, 3));
    CHECK(uart_tx_free() == 85);
    CHECK(udrie());
    CHECK(SREG == _BV(SREG_I));  // interrupts back on as they were
    cli();
    CHECK(uart_write(msg, 1, 3));
    CHECK(SREG == 0);  // and left off if they were off
    sei();
    for (uint8_t i = 0; i < 10; i++) CHECK(udre() == 0xA0 + i);
    CHECK(udre() == 0xA0);
    CHECK(uart_tx_free() == 95 && udrie());
    CHECK(udre() == -1);  // empty: the ISR switches itself off and sends nothing
    CHECK(!udrie() && uart_tx_free() == 95);
    // An empty ring at power-up: the ISR only switches itself off.
    reset_all();
    UCSR0B |= _BV(UDRIE0);
    CHECK(udre() == -1 && !udrie());
  }

  // ---- full ring and all-or-nothing refusal ----
  {
    reset_all();
    uart_set_drop_hook(hook);
    uint8_t big[100] = {};
    CHECK(uart_write(big, 95, 1));
    CHECK(uart_tx_free() == 0);
    CHECK(!uart_write(big, 1, 7));
    CHECK(drops == 1 && drop_prio == 7 && drop_len == 1);
    CHECK(uart_tx_free() == 0);
    for (int i = 0; i < 5; i++) udre();
    CHECK(uart_tx_free() == 5);
    CHECK(!uart_write(big, 6, 2));  // 6 > 5 free: nothing is queued
    CHECK(drops == 2 && drop_prio == 2 && drop_len == 6 && uart_tx_free() == 5);
    CHECK(uart_write(big, 5, 2) && uart_tx_free() == 0 && drops == 2);
    CHECK(!uart_write(big, 96, 0));  // longer than the ring itself
    // No hook: a refusal is just false.
    uart_set_drop_hook(nullptr);
    CHECK(!uart_write(big, 1, 0));
    CHECK(uart_write(big, 0, 0));  // zero bytes always fit
  }

  // ---- wrap-around: random writes and drains, checked against a model ----
  {
    reset_all();
    std::srand(12345);
    std::deque<uint8_t> model;
    std::vector<uint8_t> sent, out;
    uint8_t next = 0;
    bool invariant = true, order = true, free_ok = true;
    for (int step = 0; step < 20000; step++) {
      if (std::rand() % 2) {
        uint8_t len = (uint8_t)(std::rand() % 40);
        uint8_t buf[40];
        for (uint8_t i = 0; i < len; i++) buf[i] = next++;
        bool fits = len <= 95 - model.size();
        bool ok = uart_write(buf, len, 0);
        if (ok != fits) order = false;
        if (ok)
          for (uint8_t i = 0; i < len; i++) model.push_back(buf[i]);
        else
          next = (uint8_t)(next - len);
      } else {
        int n = std::rand() % 30;
        for (int i = 0; i < n && udrie(); i++) {
          int16_t c = udre();
          if (c < 0) break;
          if (model.empty() || c != model.front()) order = false;
          if (!model.empty()) model.pop_front();
        }
      }
      if (uart_tx_free() != 95 - model.size()) free_ok = false;
      // The ISR contract: queued bytes always have the interrupt armed.
      if (!model.empty() && !udrie()) invariant = false;
    }
    CHECK(order);
    CHECK(free_ok);
    CHECK(invariant);
  }

  // ---- RX: in order, overflow counted and dropped, hardware errors ----
  {
    reset_all();
    uint8_t c;
    CHECK(!uart_read(&c));
    for (int i = 0; i < 191; i++) rx_byte((uint8_t)i);  // 191 = capacity
    UartStats st;
    uart_stats(&st);
    CHECK(st.rx_overflow == 0 && st.rx_hw_err == 0);
    rx_byte(0xEE);
    rx_byte(0xEF);
    uart_stats(&st);
    CHECK(st.rx_overflow == 2);
    CHECK(SREG == _BV(SREG_I));  // uart_stats restores SREG
    bool ok = true;
    for (int i = 0; i < 191; i++) ok = ok && uart_read(&c) && c == (uint8_t)i;
    CHECK(ok);
    CHECK(!uart_read(&c));  // the overflowed bytes are gone, not late
    // Framing error and data overrun are counted; the byte is still stored
    // (the frame CRC decides).
    rx_byte(0x11, _BV(FE0));
    rx_byte(0x22, _BV(DOR0));
    rx_byte(0x33, _BV(FE0) | _BV(DOR0));
    rx_byte(0x44, _BV(RXC0) | _BV(UDRE0));  // other status bits are not errors
    uart_stats(&st);
    CHECK(st.rx_hw_err == 3 && st.rx_overflow == 2);
    CHECK(uart_read(&c) && c == 0x11);
    CHECK(uart_read(&c) && c == 0x22);
    CHECK(uart_read(&c) && c == 0x33);
    CHECK(uart_read(&c) && c == 0x44);
    // An error on a byte that also overflows counts both.
    for (int i = 0; i < 191; i++) rx_byte(0);
    rx_byte(0, _BV(FE0));
    uart_stats(&st);
    CHECK(st.rx_hw_err == 4 && st.rx_overflow == 3);
  }

  // ---- RX wrap-around against a model ----
  {
    reset_all();
    std::srand(777);
    std::deque<uint8_t> model;
    uint8_t next = 0;
    uint16_t lost = 0;
    bool ok = true;
    for (int step = 0; step < 20000; step++) {
      int n = std::rand() % 80;
      if (std::rand() % 2) {
        for (int i = 0; i < n; i++) {
          if (model.size() < 191)
            model.push_back(next);
          else
            lost++;
          rx_byte(next++);
        }
      } else {
        for (int i = 0; i < n; i++) {
          uint8_t c;
          bool got = uart_read(&c);
          if (got != !model.empty()) ok = false;
          if (!got) break;
          if (c != model.front()) ok = false;
          model.pop_front();
        }
      }
    }
    UartStats st;
    uart_stats(&st);
    CHECK(ok);
    CHECK(st.rx_overflow == lost && lost > 0);
  }

  return report("hal_uart");
}
