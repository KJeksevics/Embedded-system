#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- Configuration ------------------------------------------------------- */
// Which hardware UART to use on ATmega162:
//   0 => USART0 (PD1 TXD0, PD0 RXD0)
//   1 => USART1 (PB3 TXD1, PB2 RXD1)
#ifndef UART_NUM
#define UART_NUM 1
#endif

#ifndef F_CPU
#define F_CPU 4915200UL // lab crystal 4.9152 MHz
#endif

/* ---- Public API ---------------------------------------------------------- */

// Initialize UARTx for 8N1 at 'baud' (e.g. 9600, 115200)
void uart_init(uint32_t baud);

// Non-blocking: enqueue one byte for transmit.
// Returns false if TX buffer is full (caller can retry later).
bool uart_putc(uint8_t c);

// Convenience: enqueue a buffer/string; returns bytes enqueued.
size_t uart_write(const void *data, size_t len);
size_t uart_write_str(const char *s);

// RX side: number of bytes waiting (0..RX_BUF_SIZE).
uint8_t uart_available(void);

// Non-blocking get: returns next byte (0..255) or -1 if none.
int16_t uart_getc(void);

// Optional notification when a new byte arrives (called from ISR context).
typedef void (*uart_rx_cb_t)(uint8_t byte);
void uart_set_rx_callback(uart_rx_cb_t cb);

// True when TX buffer empty AND data register empty.
bool uart_tx_idle(void);
