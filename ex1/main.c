#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart0.h"

static void on_rx(uint8_t b)
{
    // Echo received bytes (non-blocking)
    while (!uart0_putc(b))
    { /* optional: backoff or drop */
    }
}

int main(void)
{
    uart0_init(9600); // 4.9152 MHz → UBRR=31 (exact 9600)
    uart0_set_rx_callback(on_rx);
    sei(); // enable global interrupts

    uart0_write_str("UART0 ready\r\n");

    for (;;)
    {
        // Or poll instead of callbacks:
        // int16_t ch = uart0_getc();
        // if (ch >= 0) uart0_putc((uint8_t)ch);
    }
}
