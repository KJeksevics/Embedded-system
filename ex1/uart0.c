#include "uart0.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// ---- ATmega162 / dual USART compatibility shims ----------------------------
// (On some AVRs these are named without the trailing 0; map if needed.)
#ifndef UDR0
#define UDR0 UDR
#endif
#ifndef UBRR0H
#define UBRR0H UBRRH
#define UBRR0L UBRRL
#endif
#ifndef UCSR0A
#define UCSR0A UCSRA
#define UCSR0B UCSRB
#define UCSR0C UCSRC
#endif
#ifndef RXEN0
#define RXEN0 RXEN
#define TXEN0 TXEN
#define RXCIE0 RXCIE
#define UDRIE0 UDRIE
#define UCSZ00 UCSZ0
#define UCSZ01 UCSZ1
#endif
#ifndef UDRE0
#define UDRE0 UDRE
#define RXC0 RXC
#endif

// ---- Small ring buffers (power-of-two sizes for cheap wrap) ----------------
#define TX_BUF_SIZE 64u
#define RX_BUF_SIZE 64u
#define TX_MASK (TX_BUF_SIZE - 1)
#define RX_MASK (RX_BUF_SIZE - 1)

static volatile uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint8_t tx_head, tx_tail;
static volatile uint8_t rx_head, rx_tail;

static volatile uart0_rx_cb_t rx_cb = 0;

// ---- Driver ----------------------------------------------------------------
void uart0_init(uint32_t baud)
{
    // Baud: UBRR = F_CPU/(16*baud) - 1  (U2X0=0 for standard async)
    uint16_t ubrr = (uint16_t)((F_CPU / (16UL * baud)) - 1UL);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // Set baud
        UBRR0H = (uint8_t)(ubrr >> 8);
        UBRR0L = (uint8_t)(ubrr & 0xFF);

        // 8 data, no parity, 1 stop: UCSZ01:0 = 1,1
        UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

        // Enable RX, TX, and RX Complete interrupt. (UDRE int is enabled on demand.)
        UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

        // Reset buffers
        tx_head = tx_tail = 0;
        rx_head = rx_tail = 0;
    }
}

bool uart0_putc(uint8_t c)
{
    uint8_t h = tx_head;
    uint8_t next = (uint8_t)((h + 1) & TX_MASK);
    if (next == tx_tail)
    {
        // Buffer full -> caller can retry later
        return false;
    }

    tx_buf[h] = c;
    tx_head = next;

    // Ensure the TX empty interrupt is on to start draining the buffer
    UCSR0B |= (1 << UDRIE0);
    return true;
}

size_t uart0_write(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t n = 0;
    while (n < len)
    {
        if (!uart0_putc(p[n]))
            break;
        n++;
    }
    return n;
}

size_t uart0_write_str(const char *s)
{
    size_t n = 0;
    while (*s)
    {
        if (!uart0_putc((uint8_t)*s))
            break;
        s++;
        n++;
    }
    return n;
}

uint8_t uart0_available(void)
{
    return (uint8_t)((RX_BUF_SIZE + rx_head - rx_tail) & RX_MASK);
}

int16_t uart0_getc(void)
{
    if (rx_head == rx_tail)
        return -1;
    uint8_t c = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1) & RX_MASK);
    return (int16_t)c;
}

void uart0_set_rx_callback(uart0_rx_cb_t cb)
{
    rx_cb = cb;
}

bool uart0_tx_idle(void)
{
    // Idle when buffer empty AND data register empty
    return (tx_head == tx_tail) && (UCSR0A & (1 << UDRE0));
}

// ---- Interrupts ------------------------------------------------------------

// RX complete: grab the byte into RX ring and optionally notify
#if defined(USART0_RX_vect)
ISR(USART0_RX_vect)
#elif defined(USART_RX_vect)
ISR(USART_RX_vect)
#elif defined(USART0_RXC_vect)
ISR(USART0_RXC_vect)
#else
#error "No USART0 RX ISR vector name known for this MCU."
#endif
{
    uint8_t c = UDR0;
    uint8_t h = rx_head;
    uint8_t next = (uint8_t)((h + 1) & RX_MASK);

    if (next != rx_tail)
    { // drop if full
        rx_buf[h] = c;
        rx_head = next;
    }
    if (rx_cb)
    {
        rx_cb(c); // keep it short; called in ISR context
    }
}

// Data register empty: push next TX byte; if none, disable interrupt
#if defined(USART0_UDRE_vect)
ISR(USART0_UDRE_vect)
#elif defined(USART_UDRE_vect)
ISR(USART_UDRE_vect)
#else
#error "No USART0 UDRE ISR vector name known for this MCU."
#endif
{
    if (tx_head == tx_tail)
    {
        // Nothing to send -> stop UDRE interrupts until we enqueue again
        UCSR0B &= ~(1 << UDRIE0);
        return;
    }
    UDR0 = tx_buf[tx_tail];
    tx_tail = (uint8_t)((tx_tail + 1) & TX_MASK);
}
