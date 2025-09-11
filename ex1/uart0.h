#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 4915200UL // lab crystal (4.9152 MHz)
#endif

// ---- Public API -------------------------------------------------------------

// Initialize USART0 for 8N1 at 'baud' (e.g., 9600, 115200)
void uart0_init(uint32_t baud);

// Non-blocking: enqueue one byte to transmit.
// Returns false if the TX buffer is full (caller can retry later).
bool uart0_putc(uint8_t c);

// Convenience: enqueue a buffer/string; returns number of bytes enqueued.
// (Stops early if TX buffer fills.)
size_t uart0_write(const void *data, size_t len);
size_t uart0_write_str(const char *s);

// RX side: bytes waiting?  (0..RX_BUF_SIZE)
uint8_t uart0_available(void);

// Non-blocking get: returns next byte (0..255) or -1 if none.
int16_t uart0_getc(void);

// Optional notification when a new byte arrives (called from ISR context).
typedef void (*uart0_rx_cb_t)(uint8_t byte);
void uart0_set_rx_callback(uart0_rx_cb_t cb);

// Check if TX buffer is empty (i.e., idle). Handy for flushing on shutdown.
bool uart0_tx_idle(void);
