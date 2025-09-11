#include "uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/* ===== Register mapping (USART0 vs USART1) =============================== */
#if (UART_NUM == 1)
/* ---- USART1 on PB3/PB2 ---- */
#define UCSRnA UCSR1A
#define UCSRnB UCSR1B
#define UCSRnC UCSR1C
#define UBRRnH UBRR1H
#define UBRRnL UBRR1L
#define UDRn UDR1
#define RXENn RXEN1
#define TXENn TXEN1
#define RXCIEn RXCIE1
#define UDRIEn UDRIE1
#define UDREn UDRE1
#define UCSZn0 UCSZ10
#define UCSZn1 UCSZ11
#define TX_PIN_DDR DDRB
#define TX_PIN_BIT PB3 /* TXD1 */
#define RX_PIN_DDR DDRB
#define RX_PIN_BIT PB2 /* RXD1 */
#define ISR_RX USART1_RX_vect
#define ISR_UDRE USART1_UDRE_vect
#elif (UART_NUM == 0)
/* ---- USART0 on PD1/PD0 ---- */
#define UCSRnA UCSR0A
#define UCSRnB UCSR0B
#define UCSRnC UCSR0C
#define UBRRnH UBRR0H
#define UBRRnL UBRR0L
#define UDRn UDR0
#define RXENn RXEN0
#define TXENn TXEN0
#define RXCIEn RXCIE0
#define UDRIEn UDRIE0
#define UDREn UDRE0
#define UCSZn0 UCSZ00
#define UCSZn1 UCSZ01
#define TX_PIN_DDR DDRD
#define TX_PIN_BIT PD1 /* TXD0 */
#define RX_PIN_DDR DDRD
#define RX_PIN_BIT PD0 /* RXD0 */
#define ISR_RX USART0_RX_vect
#define ISR_UDRE USART0_UDRE_vect
#else
#error "UART_NUM must be 0 or 1"
#endif

/* ===== Small ring buffers (power-of-two sizes) =========================== */
#define TX_BUF_SIZE 64u
#define RX_BUF_SIZE 64u
#define TX_MASK (TX_BUF_SIZE - 1)
#define RX_MASK (RX_BUF_SIZE - 1)

static volatile uint8_t tx_buf[TX_BUF_SIZE], rx_buf[RX_BUF_SIZE];
static volatile uint8_t tx_head, tx_tail, rx_head, rx_tail;
static volatile uart_rx_cb_t rx_cb = 0;

/* ===== Driver ============================================================== */
void uart_init(uint32_t baud)
{
    // UBRR = F_CPU/(16*baud) - 1 (U2X off; fine for 4915200↔9600 etc.)
    uint16_t const ubrr = (uint16_t)((F_CPU / (16UL * baud)) - 1UL);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        UBRRnH = (uint8_t)(ubrr >> 8);
        UBRRnL = (uint8_t)(ubrr & 0xFF);

        // 8 data, no parity, 1 stop
        UCSRnC = (1 << UCSZn1) | (1 << UCSZn0);

        // Enable RX, TX, and RX Complete interrupt. (UDRE enabled on demand)
        UCSRnB = (1 << RXENn) | (1 << TXENn) | (1 << RXCIEn);

        // Make TX pin output, RX pin input (not strictly required)
        TX_PIN_DDR |= (1 << TX_PIN_BIT);
        RX_PIN_DDR &= ~(1 << RX_PIN_BIT);

        tx_head = tx_tail = 0;
        rx_head = rx_tail = 0;
    }
}

bool uart_putc(uint8_t c)
{
    uint8_t h = tx_head;
    uint8_t next = (uint8_t)((h + 1) & TX_MASK);
    if (next == tx_tail)
        return false; // TX buffer full
    tx_buf[h] = c;
    tx_head = next;
    UCSRnB |= (1 << UDRIEn); // kick TX via UDRE interrupt
    return true;
}

size_t uart_write(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t n = 0;
    while (n < len)
    {
        if (!uart_putc(p[n]))
            break;
        n++;
    }
    return n;
}

size_t uart_write_str(const char *s)
{
    size_t n = 0;
    while (*s)
    {
        if (!uart_putc((uint8_t)*s))
            break;
        s++;
        n++;
    }
    return n;
}

uint8_t uart_available(void)
{
    return (uint8_t)((RX_BUF_SIZE + rx_head - rx_tail) & RX_MASK);
}

int16_t uart_getc(void)
{
    if (rx_head == rx_tail)
        return -1;
    uint8_t c = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1) & RX_MASK);
    return (int16_t)c;
}

void uart_set_rx_callback(uart_rx_cb_t cb)
{
    rx_cb = cb;
}

bool uart_tx_idle(void)
{
    return (tx_head == tx_tail) && (UCSRnA & (1 << UDREn));
}

/* ===== Interrupts ========================================================== */

// RX complete: store byte and optionally notify
ISR(ISR_RX)
{
    uint8_t c = UDRn;
    uint8_t h = rx_head;
    uint8_t next = (uint8_t)((h + 1) & RX_MASK);

    if (next != rx_tail)
    { // drop if full
        rx_buf[h] = c;
        rx_head = next;
    }
    if (rx_cb)
        rx_cb(c); // keep ISR callbacks short!
}

// Data Register Empty: push next TX byte; if none, disable interrupt
ISR(ISR_UDRE)
{
    if (tx_head == tx_tail)
    {
        UCSRnB &= ~(1 << UDRIEn);
        return;
    }
    UDRn = tx_buf[tx_tail];
    tx_tail = (uint8_t)((tx_tail + 1) & TX_MASK);
}
