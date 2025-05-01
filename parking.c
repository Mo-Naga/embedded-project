#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* ------------------ Macros ------------------ */
#define SET_BIT(REG, BIT) (REG |= (1 << BIT))
#define CLEAR_BIT(REG, BIT) (REG &= ~(1 << BIT))
#define READ_BIT(REG, BIT) ((REG >> BIT) & 1)

/* ------------------ Pins ------------------ */
#define IN1_PIN PD2
#define IN2_PIN PD3
#define ENA_PIN PD6
#define IN3_PIN PD4
#define IN4_PIN PD5
#define ENB_PIN PD7

#define FRONT_TRIG_PIN PB1
#define FRONT_ECHO_PIN PB0 // polling

#define RIGHT_TRIG_PIN PB3
#define RIGHT_ECHO_PIN PD3 // INT1

#define LEFT_TRIG_PIN  PB5
#define LEFT_ECHO_PIN  PD2 // INT0

/* ------------------ States ------------------ */
typedef enum {
    STATE_FORWARD,
    STATE_DETECT_GAP,
    STATE_PARKING,
    STATE_STOP,
    STATE_PARKED
} State;

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
volatile uint16_t right_distance = 0;
volatile uint16_t left_distance = 0;
volatile uint8_t edge_right = 0;
volatile uint8_t edge_left = 0;
void ultrasonic_trigger(uint8_t trig_pin) {
    CLEAR_BIT(PORTB, trig_pin);
    _delay_us(2);
    SET_BIT(PORTB, trig_pin);
    _delay_us(10);
    CLEAR_BIT(PORTB, trig_pin);
}

uint16_t ultrasonic_read_polling(uint8_t trig_pin, uint8_t echo_pin) {
    ultrasonic_trigger(trig_pin);
    while (!READ_BIT(PINB, echo_pin));
    TCNT1 = 0;
    TCCR1B = (1 << CS11);
    while (READ_BIT(PINB, echo_pin));
    TCCR1B = 0;
    return TCNT1 / 58;
}

void ultrasonic_init_interrupts() {
    EICRA |= (1 << ISC00) | (1 << ISC10); // any change on INT0, INT1
    EIMSK |= (1 << INT0) | (1 << INT1);   // enable INT0, INT1

    TCCR1A = 0; // normal mode
    sei();      // global interrupt enable
}

void ultrasonic_measure_right() {
    edge_right = 0;
    ultrasonic_trigger(RIGHT_TRIG_PIN);
    while (edge_right < 2);
}

void ultrasonic_measure_left() {
    edge_left = 0;
    ultrasonic_trigger(LEFT_TRIG_PIN);
    while (edge_left < 2);
}

ISR(INT1_vect) {
    static uint16_t timer;
    if (edge_right == 0) {
        TCNT1 = 0;
        TCCR1B |= (1 << CS11);
        edge_right = 1;
    } else {
        TCCR1B = 0;
        timer = TCNT1;
        right_distance = timer / 58;
        edge_right = 2;
    }
}

ISR(INT0_vect) {
    static uint16_t timer;
    if (edge_left == 0) {
        TCNT1 = 0;
        TCCR1B |= (1 << CS11);
        edge_left = 1;
    } else {
        TCCR1B = 0;
        timer = TCNT1;
        left_distance = timer / 58;
        edge_left = 2;
    }
}

/* ------------------ Main ------------------ */
int main(void) {
    uart_init(103);
    motor_init();
    ultrasonic_init_interrupts();
    DDRB |= (1 << FRONT_TRIG_PIN) | (1 << RIGHT_TRIG_PIN) | (1 << LEFT_TRIG_PIN);
    DDRB &= ~((1 << FRONT_ECHO_PIN) | (1 << RIGHT_ECHO_PIN) | (1 << LEFT_ECHO_PIN));

    State state = STATE_FORWARD;
    uint16_t front, right, left;

    while (1) {
        front = ultrasonic_read_polling(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
        ultrasonic_measure_right();
        right = right_distance;
        ultrasonic_measure_left();
        left  = left_distance;

        uart_print("F: "); uart_print_num(front);
        uart_print(" R: "); uart_print_num(right);
        uart_print(" L: "); uart_print_num(left);
        uart_print("\r\n");

        switch (state) {
            case STATE_FORWARD:
                if (front < 5) {
                    state = STATE_STOP;
                }
                else if (right > 30 || left > 30) {
                    state = STATE_DETECT_GAP;
                }
                else {
                    motor_move(0, 150);
                }
                break;

            case STATE_DETECT_GAP:

                motor_move(0, 100);
                _delay_ms(500);
                motor_move(4, 0);
                _delay_ms(200);
                
                ultrasonic_measure_right();
                right = right_distance;
                ultrasonic_measure_left();
                left  = left_distance;

                if (right > 30 || left > 30) {
                    state = STATE_PARKING;
                } else {
                    state = STATE_FORWARD;
                }
                break;

            case STATE_PARKING:
                if (right > 30) {
                    motor_move(3, 150);
                } else if (left > 30) {
                    motor_move(2, 150);
                }
                if (front < 5 || right < 5 || left < 5) {
                    state = STATE_STOP;
                }
                if (front > 10 && right < 10 && left < 10) {
                    state = STATE_PARKED;
                }
                break;

            case STATE_STOP:
                motor_move(4, 0);
                if (front > 10) {
                    state = STATE_FORWARD;
                }
                break;

            case STATE_PARKED:
                motor_move(4, 0);
                break;
        }

        _delay_ms(200);
    }
}
