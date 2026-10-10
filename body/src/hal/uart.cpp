#include "uart.h"
#include <avr/interrupt.h>
#include <avr/io.h>

// Rings: the ISR writes rx_head and reads tx_tail; the main loop writes
// rx_tail and tx_head. Each index has exactly one writer and is one byte,
// so it is read atomically and no locking is needed for the data path.
static volatile uint8_t rx_buf[UART_RX_SIZE];
static volatile uint8_t rx_head, rx_tail;
static volatile uint8_t tx_buf[UART_TX_SIZE];
static volatile uint8_t tx_head, tx_tail;
static volatile uint16_t rx_overflow, rx_hw_err;
static uart_drop_hook drop_hook;

static inline uint8_t rx_next(uint8_t i) { return (uint8_t)(i + 1 == UART_RX_SIZE ? 0 : i + 1); }
static inline uint8_t tx_next(uint8_t i) { return (uint8_t)(i + 1 == UART_TX_SIZE ? 0 : i + 1); }

void uart_init(void) {
  UBRR0 = 3;
  UCSR0A = 0;  // optiboot leaves U2X0 set (it runs 115200 that way): clear it
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);  // 8N1
  UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
}

ISR(USART_RX_vect) {
  uint8_t st = UCSR0A;  // read status before data: reading UDR0 clears it
  uint8_t c = UDR0;
  if (st & (_BV(FE0) | _BV(DOR0))) rx_hw_err++;
  uint8_t n = rx_next(rx_head);
  if (n == rx_tail) {
    rx_overflow++;  // ring full: drop; the frame's CRC will catch it
    return;
  }
  rx_buf[rx_head] = c;
  rx_head = n;
}

ISR(USART_UDRE_vect) {
  if (tx_head == tx_tail) {
    UCSR0B &= (uint8_t)~_BV(UDRIE0);  // nothing left: stop asking
    return;
  }
  UDR0 = tx_buf[tx_tail];
  tx_tail = tx_next(tx_tail);
}

bool uart_read(uint8_t* c) {
  uint8_t t = rx_tail;
  if (t == rx_head) return false;
  *c = rx_buf[t];
  rx_tail = rx_next(t);
  return true;
}

uint8_t uart_tx_free(void) {
  int16_t used = (int16_t)tx_head - (int16_t)tx_tail;
  if (used < 0) used += UART_TX_SIZE;
  return (uint8_t)(UART_TX_SIZE - 1 - used);
}

bool uart_write(const uint8_t* data, uint8_t len, uint8_t prio) {
  if (len > uart_tx_free()) {
    if (drop_hook) drop_hook(prio, len);
    return false;
  }
  uint8_t h = tx_head;
  for (uint8_t i = 0; i < len; i++) {
    tx_buf[h] = data[i];
    h = tx_next(h);
  }
  tx_head = h;
  // UCSR0B is also written by the UDRE ISR: read-modify-write it with
  // interrupts off.
  uint8_t sreg = SREG;
  cli();
  UCSR0B |= _BV(UDRIE0);
  SREG = sreg;
  return true;
}

void uart_set_drop_hook(uart_drop_hook h) { drop_hook = h; }

void uart_stats(UartStats* s) {
  uint8_t sreg = SREG;
  cli();
  s->rx_overflow = rx_overflow;
  s->rx_hw_err = rx_hw_err;
  SREG = sreg;
}
