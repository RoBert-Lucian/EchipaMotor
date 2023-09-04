// INCLUDES
#include <math.h>
#include "motor.h"
#include "stm32f0xx_hal.h"

// PRIVATE VARIABLES
static int dir;
static float targetVel;
static uint16_t pwm;
static uint32_t pos;
static uint32_t last_sample_time;
static uint32_t overflowCtr = 0;
static float odometer;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
static float angularVelocity = 0;
static float angularAcceleration = 0;
#pragma GCC diagnostic pop

// PRIVATE FUNCTION PROTOTYPES
static void motSetPWM(uint16_t);
static uint32_t motGetPos();
void motSetDirection(int);

// PUBLIC FUNCTION CODE
void motInit(void){
	dir = MOT_FWD;
	targetVel = 0;
	pos = MOT_ENC_TIM->CNT;
	last_sample_time = HAL_GetTick();
	motSetDirection(dir);
	motSetVelocity(targetVel);
}

float motGetOdometer(void){
	static int32_t lastPos = 0;
	int32_t pos = motGetPos();
	int32_t dp = pos>lastPos ? pos - lastPos : lastPos - pos;
	odometer += ( (float)dp * 93 / 10000 ) * 0.22;
	lastPos = pos;
	return odometer;
}

float motGetVelocity(void){
	return -1;
}

void motSetVelocity(float velocity){

}

void motEncTimPeriodElapsedCallback(TIM_HandleTypeDef *htim){
	overflowCtr++;
}

// PRIVATE FUNCTION CODE
void motSetPWM(uint16_t val){
	MOT_PWM_TIM->CCR1 = val;
	pwm = val;
}

uint32_t motGetPos(){
	return MOT_ENC_TIM->CNT;
}

void motSetDirection(int val){
	switch(val){
		case MOT_FORWARD:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_SET);
			dir = val;
			break;
		case MOT_BACKWARD:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_RESET);
			dir = val;
			break;
		case MOT_STOP:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_RESET);
			dir = val;
			break;
	}
}
