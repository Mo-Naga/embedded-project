
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
#define IN1_PORT PORTD
#define IN1_DDR  DDRD
#define IN1_PIN  PD2

#define IN2_PORT PORTD
#define IN2_DDR  DDRD
#define IN2_PIN  PD3

#define ENA_DDR  DDRD
#define ENA_PIN  PD6  // OC0A

// Motor B (Left)
#define IN3_PORT PORTD
#define IN3_DDR  DDRD
#define IN3_PIN  PD4

#define IN4_PORT PORTD
#define IN4_DDR  DDRD
#define IN4_PIN  PD7

#define ENB_DDR  DDRD
#define ENB_PIN  PD5  // OC0B

// Ultrasonic
#define BACK_TRIG_PIN PB1
#define BACK_ECHO_PIN PB0

#define RIGHT_TRIG_PIN PB3
#define RIGHT_ECHO_PIN PB2

#define LEFT_TRIG_PIN PB5
#define LEFT_ECHO_PIN PB4 


/* ------------------ Direction Macros ------------------ */
#define DIR_FORWARD  0
#define DIR_BACKWARD 1
#define DIR_LEFT     2
#define DIR_RIGHT    3
#define DIR_STOP     4

/* ------------------ Function Prototypes ------------------ */
void motor_init();
void motor_set_left(uint8_t speed, uint8_t dir);
void motor_set_right(uint8_t speed, uint8_t dir);
void motor_move(uint8_t direction, uint8_t speed);
void uart_init();
void uart_transmit(char data);
char uart_receive();



/* ------------------ Motor Init ------------------ */
void motor_init() {
    SET_BIT(IN1_DDR, IN1_PIN);
    SET_BIT(IN2_DDR, IN2_PIN);
    SET_BIT(IN3_DDR, IN3_PIN);
    SET_BIT(IN4_DDR, IN4_PIN);
    SET_BIT(ENA_DDR, ENA_PIN);
    SET_BIT(ENB_DDR, ENB_PIN);

    // Timer0 Fast PWM mode, non-inverting, prescaler 64
    TCCR0A |= (1 << WGM00) | (1 << WGM01);
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
    TCCR0B |= (1 << CS01) | (1 << CS00);
}

/* ------------------ Motor Control ------------------ */
void motor_set_left(uint8_t speed, uint8_t dir) {
    if (dir) {
        SET_BIT(IN3_PORT, IN3_PIN);
        CLEAR_BIT(IN4_PORT, IN4_PIN);
    } else {
        CLEAR_BIT(IN3_PORT, IN3_PIN);
        SET_BIT(IN4_PORT, IN4_PIN);
    }
    OCR0B = speed;
}

void motor_set_right(uint8_t speed, uint8_t dir) {
    if (dir) {
        SET_BIT(IN1_PORT, IN1_PIN);
        CLEAR_BIT(IN2_PORT, IN2_PIN);
    } else {
        CLEAR_BIT(IN1_PORT, IN1_PIN);
        SET_BIT(IN2_PORT, IN2_PIN);
    }
    OCR0A = speed;
}

void motor_move(uint8_t direction, uint8_t speed) {
    switch (direction) {
        case DIR_BACKWARD:
            motor_set_left(speed, 1);
            motor_set_right(speed, 1);
            break;
        case  DIR_FORWARD:
            motor_set_left(speed, 0);
            motor_set_right(speed, 0);
            break;
        case DIR_RIGHT:
            motor_set_left(speed, 0);
            motor_set_right(speed, 1);
            break;
        case DIR_LEFT:
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
    return TCNT1 / 117;
}


/* ------------------ UART Init ------------------ */
void uart_init() {
    // Baud Rate = 9600
    uint16_t ubrr = 103;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;

    UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable receiver and transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data, 1 stop bit
}
bool uart_available(void) {
    // RXC0 set when unread data in UDR0
    return (UCSR0A & (1 << RXC0)) != 0;
}
void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer is empty
    UDR0 = data;
}

char uart_receive() {
    while (!(UCSR0A & (1 << RXC0))); // Wait until data is received
    return UDR0;
}

/* ------------------ Main Program ------------------ */
// pin definitions
#define BACK_TRIG_PIN  PB1
#define BACK_ECHO_PIN  PB0

#define RIGHT_TRIG_PIN PB3
#define RIGHT_ECHO_PIN PB2

#define LEFT_TRIG_PIN  PB5
#define LEFT_ECHO_PIN  PB4

int main(void) {
    motor_init();
    uart_init();

    uint8_t speed = 100;   // default speed
    char    cmd   = 'S';   // current command state

    // init BACK sensor
    SET_BIT(DDRB, BACK_TRIG_PIN);
    CLEAR_BIT(DDRB, BACK_ECHO_PIN);
    // init RIGHT sensor
    SET_BIT(DDRB, RIGHT_TRIG_PIN);
    CLEAR_BIT(DDRB, RIGHT_ECHO_PIN);
    // init LEFT sensor
    SET_BIT(DDRB, LEFT_TRIG_PIN);
    CLEAR_BIT(DDRB, LEFT_ECHO_PIN);

    while (1) {
        // 1) non‐blocking check for a new byte
        if (uart_available()) {
            char in = uart_receive();
            // accept only our command chars
            if (in=='F'|| in=='B' || in=='L' || in=='R' ||
                in=='S' || in=='+' || in=='-' ||
                (in>='0' && in<='9'))
            {
                cmd = in;
            }
        }

        // 2) read your back sensor on every loop
        uint16_t d_back  = ultrasonic_read(BACK_TRIG_PIN, BACK_ECHO_PIN);
        // (optional: read left/right if you have them)
        uint16_t d_right = ultrasonic_read(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
        uint16_t d_left  = ultrasonic_read(LEFT_TRIG_PIN,  LEFT_ECHO_PIN);

        // 3) immediate obstacle override
        if (cmd == 'B' && d_back < 25) {
            motor_move(DIR_STOP, 0);
            continue;
        }
        if (cmd == 'L' && d_left < 25) {
            motor_move(DIR_STOP, 0);
            continue;
        }
        if (cmd == 'R' && d_right < 25) {
            motor_move(DIR_STOP, 0);
            continue;
        }

        // 4) speed adjustments (consume '+'/'-'/'0'..'9' immediately)
        switch (cmd) {
            case '+': if (speed <= 245) speed += 10; cmd = 'S'; break;
            case '-': if (speed >= 10)  speed -= 10; cmd = 'S'; break;
            case '0': speed =   0;       cmd = 'S'; break;
            case '1': speed =  25;       cmd = 'S'; break;
            case '2': speed =  50;       cmd = 'S'; break;
            case '3': speed =  75;       cmd = 'S'; break;
            case '4': speed = 100;       cmd = 'S'; break;
            case '5': speed = 125;       cmd = 'S'; break;
            case '6': speed = 150;       cmd = 'S'; break;
            case '7': speed = 175;       cmd = 'S'; break;
            case '8': speed = 200;       cmd = 'S'; break;
            case '9': speed = 225;       cmd = 'S'; break;
            case '10': speed =255;       cmd = 'S'; break;
            default: break;
        }

        // 5) movement
        switch (cmd) {
            case 'F': motor_move(DIR_FORWARD,  speed); break;
            case 'B': motor_move(DIR_BACKWARD, speed); break;
            case 'L': motor_move(DIR_LEFT,     speed); break;
            case 'R': motor_move(DIR_RIGHT,    speed); break;
            default:  motor_move(DIR_STOP,     0);     break;
        }
    }
}

