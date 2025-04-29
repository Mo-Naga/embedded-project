#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define DIR_FORWARD  0
#define DIR_BACKWARD 1
#define DIR_LEFT     2
#define DIR_RIGHT    3
#define DIR_STOP     4

void motor_init();
void motor_set_left(uint8_t speed, uint8_t dir);
void motor_set_right(uint8_t speed, uint8_t dir);
void motor_move(uint8_t direction, uint8_t speed);

#endif
