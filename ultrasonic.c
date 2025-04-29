#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* ------------------ Bit Manipulation Macros ------------------ */
#define SET_BIT(REG_NAME, BIT_NUMBER)     REG_NAME |= (1 << BIT_NUMBER)
#define CLEAR_BIT(REG_NAME, BIT_NUMBER)   REG_NAME &= (~(1 << BIT_NUMBER))
#define TOGGLE_BIT(REG_NAME, BIT_NUMBER)  REG_NAME ^= (1 << BIT_NUMBER)
#define READ_BIT(REG_NAME, BIT_NUM)       ((REG_NAME >> BIT_NUM) & 1)
#define CLEAR_PORT(PORT_NAME)             PORT_NAME &= 0x00
#define SET_PORT(PORT_NAME)               PORT_NAME |= 0xFF
#define READ_PORT(PORT_NAME)              (PORT_NAME & 0xFF)

/* ------------------ Pin Definitions ------------------ */
// Motor A (Right)
#define IN1_PIN  PD2
#define IN2_PIN  PD3
#define ENA_PIN  PD6  // OC0A

// Motor B (Left)
#define IN3_PIN  PD4
#define IN4_PIN  PD5
#define ENB_PIN  PD7  // OC0B

// Ultrasonic
#define TRIG_PIN PB0
#define ECHO_PIN PB1

/* ------------------ UART Functions ------------------ */
void uart_init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)(ubrr);
    SET_BIT(UCSR0B, RXEN0);
    SET_BIT(UCSR0B, TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print(const char* str) {
    while (*str) uart_transmit(*str++);
}

void uart_print_num(uint16_t num) {
    char buffer[10];
    itoa(num, buffer, 10);
    uart_print(buffer);
}

/* ------------------ Motor Control ------------------ */
void motor_init() {
    DDRD |= (1 << IN1_PIN) | (1 << IN2_PIN) | (1 << IN3_PIN) | (1 << IN4_PIN) | (1 << ENA_PIN) | (1 << ENB_PIN);

    // Timer0 PWM
    TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1); // non-inverting
    TCCR0B |= (1 << CS01); // Prescaler 8
}

void motor_set_left(uint8_t speed, uint8_t forward) {
    if (forward) {
        SET_BIT(PORTD, IN3_PIN);
        CLEAR_BIT(PORTD, IN4_PIN);
    } else {
        CLEAR_BIT(PORTD, IN3_PIN);
        SET_BIT(PORTD, IN4_PIN);
    }
    OCR0B = speed;
}

void motor_set_right(uint8_t speed, uint8_t forward) {
    if (forward) {
        SET_BIT(PORTD, IN1_PIN);
        CLEAR_BIT(PORTD, IN2_PIN);
    } else {
        CLEAR_BIT(PORTD, IN1_PIN);
        SET_BIT(PORTD, IN2_PIN);
    }
    OCR0A = speed;
}

void motor_move(uint8_t dir, uint8_t speed) {
    switch (dir) {
        case 0: // Forward
            uart_print("FORWARD\r\n");
            motor_set_left(speed, 1);
            motor_set_right(speed, 1);
            break;
        case 1: // Backward
            uart_print("BACKWARD\r\n");
            motor_set_left(speed, 0);
            motor_set_right(speed, 0);
            break;
        case 2: // Left
            uart_print("LEFT\r\n");
            motor_set_left(speed, 0);
            motor_set_right(speed, 1);
            break;
        case 3: // Right
            uart_print("RIGHT\r\n");
            motor_set_left(speed, 1);
            motor_set_right(speed, 0);
            break;
        default: // Stop
            uart_print("STOP\r\n");
            motor_set_left(0, 1);
            motor_set_right(0, 1);
            break;
    }
}

/* ------------------ Ultrasonic ------------------ */
void ultrasonic_trigger(uint8_t trig_pin) {
    CLEAR_BIT(PORTB, trig_pin);
    _delay_us(2);
    SET_BIT(PORTB, trig_pin);
    _delay_us(10);
    CLEAR_BIT(PORTB, trig_pin);
}

uint16_t ultrasonic_read(uint8_t trig_pin, uint8_t echo_pin) {
    ultrasonic_trigger(trig_pin);

    while (!READ_BIT(PINB, echo_pin));
    TCNT1 = 0;
    TCCR1B = (1 << CS11);
    while (READ_BIT(PINB, echo_pin));
    TCCR1B = 0;
    return TCNT1 / 58;
}

/* ------------------ Main ------------------ */
int main(void) {
    uart_init(103); // 9600 baud
    motor_init();

    // Set trig as output, echo as input
    SET_BIT(DDRB, TRIG_PIN);
    CLEAR_BIT(DDRB, ECHO_PIN);

    while (1) {
        uint16_t dist = ultrasonic_read(TRIG_PIN, ECHO_PIN);
        uart_print("Distance: ");
        uart_print_num(dist);
        uart_print(" cm\r\n");

        if (dist < 15) {
            motor_move(4, 0); // Stop
        } else {
            motor_move(0, 150); // Forward
        }

        _delay_ms(300);
    }
}
