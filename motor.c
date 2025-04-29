#include <avr/io.h>
#include "pins.h"
#include "motor.h"

void motor_init() {
    DDRB |= (1 << MOTOR_RIGHT_IN1) | (1 << MOTOR_RIGHT_IN2) |
            (1 << MOTOR_LEFT_IN3) | (1 << MOTOR_LEFT_IN4) |
            (1 << MOTOR_RIGHT_EN);
    DDRD |= (1 << MOTOR_LEFT_EN); // PWM on OC0A

    // Timer0 setup for Left Motor (OC0A - PD6)
    TCCR0A |= (1 << WGM01) | (1 << WGM00);  // Fast PWM
    TCCR0A |= (1 << COM0A1);               // Non-inverting mode
    TCCR0B |= (1 << CS01);                 // Prescaler 8

    // Timer2 setup for Right Motor (OC2A - PB3)
    TCCR2A |= (1 << WGM21) | (1 << WGM20);  // Fast PWM
    TCCR2A |= (1 << COM2A1);               // Non-inverting mode
    TCCR2B |= (1 << CS21);                 // Prescaler 8
}

void motor_set_left(uint8_t speed, uint8_t dir) {
    if (dir == 0) {
        PORTB &= ~(1 << MOTOR_LEFT_IN3);
        PORTB |= (1 << MOTOR_LEFT_IN4);
    } else {
        PORTB |= (1 << MOTOR_LEFT_IN3);
        PORTB &= ~(1 << MOTOR_LEFT_IN4);
    }
    OCR0A = speed;
}

void motor_set_right(uint8_t speed, uint8_t dir) {
    if (dir == 0) {
        PORTB &= ~(1 << MOTOR_RIGHT_IN1);
        PORTB |= (1 << MOTOR_RIGHT_IN2);
    } else {
        PORTB |= (1 << MOTOR_RIGHT_IN1);
        PORTB &= ~(1 << MOTOR_RIGHT_IN2);
    }
    OCR2A = speed;
}

void motor_move(uint8_t direction, uint8_t speed) {
    switch (direction) {
        case DIR_FORWARD:
            motor_set_left(speed, 1);
            motor_set_right(speed, 1);
            break;

        case DIR_BACKWARD:
            motor_set_left(speed, 0);
            motor_set_right(speed, 0);
            break;

        case DIR_LEFT:
            motor_set_left(speed, 0);
            motor_set_right(speed, 1);
            break;

        case DIR_RIGHT:
            motor_set_left(speed, 1);
            motor_set_right(speed, 0);
            break;

        case DIR_STOP:
        default:
            motor_set_left(0, 1);
            motor_set_right(0, 1);
            break;
    }
}
