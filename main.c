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

// Motor A (Right side)
#define IN1_PORT PORTD
#define IN1_DDR  DDRD
#define IN1_PIN  PD2

#define IN2_PORT PORTD
#define IN2_DDR  DDRD
#define IN2_PIN  PD3

#define ENA_DDR   DDRD
#define ENA_PIN   PD6  // OC0A

// Motor B (Left side)
#define IN3_PORT PORTD
#define IN3_DDR  DDRD
#define IN3_PIN  PD4

#define IN4_PORT PORTD
#define IN4_DDR  DDRD
#define IN4_PIN  PD7

#define ENB_DDR   DDRD
#define ENB_PIN   PD5  // OC0B

/* ------------------ Direction Macros ------------------ */
#define DIR_FORWARD  0
#define DIR_BACKWARD 1
#define DIR_LEFT     2
#define DIR_RIGHT    3
#define DIR_STOP     4

/* ------------------ Motor Init ------------------ */
void motor_init() {
    // Set direction of control pins
    SET_BIT(IN1_DDR, IN1_PIN);
    SET_BIT(IN2_DDR, IN2_PIN);
    SET_BIT(IN3_DDR, IN3_PIN);
    SET_BIT(IN4_DDR, IN4_PIN);
    SET_BIT(ENA_DDR, ENA_PIN);
    SET_BIT(ENB_DDR, ENB_PIN);

    // Timer0 PWM setup (Fast PWM, non-inverting, prescaler 8)
    TCCR0A |= (1 << WGM00) | (1 << WGM01);        // Fast PWM mode
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);      // Non-inverting on OC0A and OC0B
    TCCR0B |= (1 << CS01);                        // Prescaler 8
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
    OCR0B = speed; // Set PWM for left motor (ENB)
}

void motor_set_right(uint8_t speed, uint8_t dir) {
    if (dir) {
        SET_BIT(IN1_PORT, IN1_PIN);
        CLEAR_BIT(IN2_PORT, IN2_PIN);
    } else {
        CLEAR_BIT(IN1_PORT, IN1_PIN);
        SET_BIT(IN2_PORT, IN2_PIN);
    }
    OCR0A = speed; // Set PWM for right motor (ENA)
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

/* ------------------ Main Program ------------------ */
int main(void) {
    motor_init();
    Serial.begin(9600);
    while (1) {

        for(int i=0;i<=15;i++){
            motor_move(DIR_FORWARD, 10*i);
            _delay_ms(100);
        }
        for(int i=15;i>=0;i--){
            motor_move(DIR_FORWARD, 10*i);
            _delay_ms(100);
        }


        for(int i=0;i<=15;i++){
            motor_move(DIR_LEFT, 10*i);
            _delay_ms(100);
        }
        for(int i=15;i>=0;i--){
            motor_move(DIR_LEFT, 10*i);
            _delay_ms(100);
        }


        for(int i=0;i<=15;i++){
            motor_move(DIR_BACKWARD, 10*i);
            _delay_ms(100);
        }
        for(int i=15;i>=0;i--){
            motor_move(DIR_BACKWARD, 10*i);
            _delay_ms(100);
        }

         for(int i=0;i<=15;i++){
            motor_move(DIR_RIGHT, 10*i);
            _delay_ms(100);
        }
        for(int i=15;i>=0;i--){
            motor_move(DIR_RIGHT, 10*i);
            _delay_ms(100);
        }
    

        Serial.println("STOP");
        motor_move(DIR_STOP, 0);
        _delay_ms(1000);
    }
}
