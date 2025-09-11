// ATmega162 UART self-test (works with the generic uart.h driver)
// Clock: 4.9152 MHz  (F_CPU must match your fuses/crystal)
// UART:  set UART_NUM=1 for PB3/PB2 (USART1), or UART_NUM=0 for PD1/PD0 (USART0)

#define F_CPU 4915200UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include "uart.h" // <-- from the driver I sent (uart_init/putc/getc/write_str)

static void uart_write_u32(uint32_t v)
{
    char buf[10];
    uint8_t i = 0;
    do
    {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    } while (v && i < sizeof(buf));
    while (i--)
        while (!uart_putc((uint8_t)buf[i]))
        {
        }
}

int main(void)
{
    // Optional LED heartbeat on PB0
    DDRB |= (1 << PB0);

    uart_init(9600); // exact at 4.9152 MHz (U2X off)
    sei();

    uart_write_str("\r\nUART self-test (9600 8N1)\r\n");
    uart_write_str("Type and press <Enter>. Backspace works. "
                   "I also print a tick once per second.\r\n\r\n");

    char line[64];
    uint8_t idx = 0;
    uint32_t ticks = 0;
    uint16_t ms = 0;

    for (;;)
    {
        // ---- Periodic TX: "tick N" every ~1000 ms ----
        if (ms >= 1000)
        {
            uart_write_str("tick ");
            uart_write_u32(++ticks);
            uart_write_str("\r\n");
            PORTB ^= (1 << PB0); // heartbeat
            ms = 0;
        }

        // ---- RX: drain all received bytes, echo, build a line ----
        int16_t ch;
        while ((ch = uart_getc()) >= 0)
        {
            uint8_t c = (uint8_t)ch;

            // handle Backspace / DEL
            if (c == '\b' || c == 127)
            {
                if (idx > 0)
                {
                    idx--;
                    // erase one char on terminal
                    uart_write_str("\b \b");
                }
                continue;
            }

            // Enter (CR or LF): finish the line
            if (c == '\r' || c == '\n')
            {
                uart_write_str("\r\nYou typed: ");
                uart_write(line, idx);
                uart_write_str("\r\n\r\n");
                idx = 0;
                continue;
            }

            // Normal character: append if room and echo
            if (idx < sizeof(line) - 1)
            {
                line[idx++] = (char)c;
                while (!uart_putc(c))
                {
                } // immediate echo
            }
            else
            {
                // overflow: reset and warn
                uart_write_str("\r\n(line too long)\r\n");
                idx = 0;
            }
        }

        _delay_ms(10);
        ms += 10;
    }
}
