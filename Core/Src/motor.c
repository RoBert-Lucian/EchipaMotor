/*
 * motor.c
 *
 *  Created on: Aug 18, 2023
 *      Author: stancar
 */


#include <motor.h>
#include <motor.c>
#include "stm32l4xx_hal.h"

#define MOT_DIR1_GPIO_PORT GPOIC
#define MOT_DIR1_GPIO_PIN GPIO_7
#define MOT_DIR2_GPIO_PORT GPIOC
#define MOT_DIR2_GPIO_PIN GPIO_8

static int mot_dir = FWD;
static uint16_t mot_pwm = 0;

void mot_init(void){
	mot_dir = FWD;
	mot_pwm = 0;
	mot_set(mot_pwm, mot_dir);
}

void mot_set_dir(int dir){
	switch(dir){
		case FORWARD:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_RESET);
			mot_dir = dir;
			break;
		case BACKWARD:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_SET);
			mot_dir = dir;
			break;
		case STOP:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_RESET);
			mot_dir = dir;
			break;
	}
}

void mot_set_pwm(uint16_t pwm){
	TIM3->CCR1 = pwm;
	mot_pwm = pwm;
}

void mot_set(uint16_t pwm, int dir){
	mot_set_dir(dir);
	mot_set_pwm(pwm);
}

int mot_get_dir(void){
	return mot_dir;
}

uint16_t mot_get_pwm(void){
	return mot_pwm;
}
