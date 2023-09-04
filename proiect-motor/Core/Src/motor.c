#include <math.h>
#include "motor.h"
#include "stm32f0xx_hal.h"

static int mot_dir;
static uint16_t mot_pwm;
static uint32_t mot_pos;
static uint32_t mot_last_sample_time;
static uint32_t overflowCtr = 0;
static float odometer;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
static float angularVelocity = 0;
static float angularAcceleration = 0;
#pragma GCC diagnostic pop

void mot_init(void){
	mot_dir = MOT_FWD;
	mot_pwm = 0;
	mot_pos = MOT_ENC_TIM->CNT;
	mot_last_sample_time = HAL_GetTick();
	mot_set(mot_pwm, mot_dir);
}

void mot_set_dir(int dir){
	switch(dir){
		case MOT_FORWARD:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_SET);
			mot_dir = dir;
			break;
		case MOT_BACKWARD:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_RESET);
			mot_dir = dir;
			break;
		case MOT_STOP:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_RESET);
			mot_dir = dir;
			break;
	}
}

void mot_set_pwm(uint16_t pwm){
	MOT_PWM_TIM->CCR1 = pwm;
	mot_pwm = pwm;
}

void mot_set(uint16_t pwm, int dir){
	mot_set_dir(dir);
	mot_set_pwm(pwm);
}

void mot_toggle_dir(){
	switch(mot_dir){
		case MOT_FORWARD: mot_dir = MOT_BACKWARD; break;
		case MOT_BACKWARD: mot_dir = MOT_FORWARD; break;
	}
	mot_set_dir(mot_dir);
}

uint32_t mot_get_pos(){
	return MOT_ENC_TIM->CNT;
}

int mot_get_dir(void){
	return mot_dir;
}

uint16_t mot_get_pwm(void){
	return mot_pwm;
}

float mot_get_vel(void){
	return 1;
}

float mot_get_odometer(void){
	static int32_t lastPos = 0;
	int32_t pos = mot_get_pos();
	int32_t dp = pos>lastPos ? pos - lastPos : lastPos - pos;
	odometer += ( (float)dp * 93 / 10000 ) * 0.22;
	lastPos = pos;
	return odometer;
}

void mot_enc_tim_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	overflowCtr++;
}
