// uart: USART0 at 250000 baud 8N1 with interrupt-driven rings (doc §3a, §4).
//
// Why not HardwareSerial: its 64 B RX buffer is smaller than one 66 B frame,
// and it costs 157 B of RAM for features we do not use. Here the RX ring is
// 192 B (7.7 ms of input at full rate: the slack the main loop has between
// link_poll() calls) and the TX ring 96 B (one tick's worth of telemetry).
//
// 250000 baud is exact on a 16 MHz AVR: UBRR = 16e6 / (16 * 250000) - 1 = 3
// with U2X off. (U2X with UBRR = 7 is also exact; normal speed samples each
// bit 16 times instead of 8, which tolerates more noise.)
#ifndef KILO_HAL_UART_H
#define KILO_HAL_UART_H
#include <stdint.h>

#define UART_RX_SIZE 192
#define UART_TX_SIZE 96

void uart_init(void);

// Next received byte; false if the ring is empty.
bool uart_read(uint8_t* c);

// Free space in the TX ring.
uint8_t uart_tx_free(void);

// Queue `len` bytes, all or nothing. On refusal the drop hook (if set) is
// called with `prio` so the link layer can count the drop and raise a
// rate-limited TX_DROPPED event. Priority policy (doc §4): the link layer
// queues ACK/EVENT first, then TELEMETRY, JOINTS, IMU_RAW, so whatever is
// refused is always the least important frame of the tick.
bool uart_write(const uint8_t* data, uint8_t len, uint8_t prio);
typedef void (*uart_drop_hook)(uint8_t prio, uint8_t len);
void uart_set_drop_hook(uart_drop_hook h);

struct UartStats {
  uint16_t rx_overflow;  // bytes lost because the RX ring was full
  uint16_t rx_hw_err;    // framing errors and hardware overruns
};
void uart_stats(UartStats* s);

#endif
