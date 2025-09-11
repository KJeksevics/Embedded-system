#define F_CPU 4915200UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "uart0.h"

// printf -> UART
static int uart0_putchar_stdio(char c, FILE *stream)
{
    (void)stream;
    if (c == '\n')
    {
        while (!uart0_putc('\r'))
        {
        }
    } // CRLF for terminals
    while (!uart0_putc((uint8_t)c))
    {
    } // wait until queued
    return 0;
}

// scanf/getchar <- UART
static int uart0_getchar_stdio(FILE *stream)
{
    (void)stream;
    int16_t ch;
    while ((ch = uart0_getc()) < 0)
    {
    } // wait for a byte
    return ch;
}

int main(void)
{
    uart0_init(9600); // exact at 4.9152 MHz
    sei();

    // Attach stdio using fdevopen (simplest)
    FILE *uart = fdevopen(uart0_putchar_stdio, uart0_getchar_stdio);
    (void)uart; // we assign to stdin/stdout below if you like

    // Either use the returned FILE*, or set global streams:
    stdout = stdin = uart; // (optional) stderr = uart;

    puts("\r\nprintf/scanf ready @9600.\r\nType a line and press <Enter>.\r\n");

    char line[64];
    for (;;)
    {
        int c = getchar(); // blocks for one char
        putchar(c);        // echo immediately
        if (c == '\r' || c == '\n')
        {
            printf("\r\n"); // newline handling
        }
    }
}
