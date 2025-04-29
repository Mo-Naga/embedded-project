#ifndef PINS_H
#define PINS_H

// Ultrasonic Sensor Pins
#define TRIG_FRONT PD2
#define ECHO_FRONT PD3

#define TRIG_LEFT PD4
#define ECHO_LEFT PD5

#define TRIG_RIGHT PD6
#define ECHO_RIGHT PD7

// L298N Motor Driver Pins
#define MOTOR_RIGHT_IN1 PB0
#define MOTOR_RIGHT_IN2 PB1
#define MOTOR_RIGHT_EN  PB3  // OC2A

#define MOTOR_LEFT_IN3  PB4
#define MOTOR_LEFT_IN4  PB5
#define MOTOR_LEFT_EN   PD6  // OC0A

#endif
