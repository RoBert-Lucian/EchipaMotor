/*
 * motor.h
 *
 *  Created on: Aug 18, 2023
 *      Author: stancar
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>

#define FORWARD 1
#define STOP 0
#define BACKWARD -1
#define FRW FORWARD
#define BKW BACKWARD

void mot_init(void);
void mot_set_dir(int dir);
void mot_set_pwm(uint16_t pwm);
void mot_set(uint16_t pwm, int dir);
int mot_get_dir(void);
uint16_t mot_get_pwm(void);

#endif /* INC_MOTOR_H_ */
