#include <avr/io.h>
#include <util/delay.h>
#include "pins.h"

void ultrasonic_trigger(uint8_t trig_pin) {
    PORTD &= ~(1 << trig_pin);
    _delay_us(2);
    PORTD |= (1 << trig_pin);
    _delay_us(10);
    PORTD &= ~(1 << trig_pin);
}

uint16_t ultrasonic_read(uint8_t trig_pin, uint8_t echo_pin) {
    ultrasonic_trigger(trig_pin);

    // Wait for echo to go high
    while (!(PIND & (1 << echo_pin)));

    // Start timer
    TCNT1 = 0;
    TCCR1B = (1 << CS11); // Prescaler 8

    // Wait for echo to go low
    while (PIND & (1 << echo_pin));

    // Stop timer
    TCCR1B = 0;
    uint16_t time = TCNT1;
    
    // Convert to distance (cm)
    return time / 58;
}
