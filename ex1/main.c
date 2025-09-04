// main.c — ATmega162 square wave on PB0
#define F_CPU 4915200UL // Lab crystal: 4.9152 MHz
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    // Set PB0 as output
    DDRB |= (1 << PB0);

    // 10 Hz square wave: 50 ms high, 50 ms low
    while (1)
    {
        PORTB = (1 << PB0); // Toggle PB0 by writing to PIN register
        _delay_ms(50);
    }
}
