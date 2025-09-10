#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    DDRB |= (1 << PB0); // PB0 output
    while (1)
    {
        PORTB |= (1 << PB0);  // toggle
        _delay_ms(250);       // 2 Hz (250 ms high + 250 ms low)
        PORTB &= ~(1 << PB0); // toggle
        _delay_ms(250);       // 2 Hz (250 ms high + 250 ms low)
    }
}
