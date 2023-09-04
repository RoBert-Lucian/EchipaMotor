// INCLUDES
#include <math.h>
#include "motor.h"
#include "stm32f0xx_hal.h"

// PRIVATE VARIABLES
static int initalised = 0;
static int dir;
static float targetVel;
static uint32_t pos;
static uint32_t overflowCtr = 0;
static float odometer;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
static float angularVelocity = 0;
static float angularAcceleration = 0;
#pragma GCC diagnostic pop

// PRIVATE FUNCTION PROTOTYPES
static uint32_t motGetPos();
static void motSetDirection(int);
static void Error_Handler(void);

// PUBLIC FUNCTION CODE
void motInit(uint32_t odometer){
	dir = MOT_FORWARD;
	targetVel = 0;
	pos = MOT_ENC_TIM->CNT;
	initalised = 1;
	motSetVelocity(targetVel);
}

float motGetOdometer(void){
	if(!initalised) Error_Handler();
	static int32_t lastPos = 0;
	int32_t pos = motGetPos();
	int32_t dp = pos>lastPos ? pos - lastPos : lastPos - pos;
	odometer += ( (float)dp * 93 / 10000 ) * 0.22;
	lastPos = pos;
	return odometer;
}

float motGetVelocity(void){
	if(!initalised) Error_Handler();
	return -1;
}

void motSetVelocity(float velocity){
	if(!initalised) Error_Handler();
	if(velocity < 0) motSetDirection(MOT_BACKWARD);
	else motSetDirection(MOT_FORWARD);
//	MOT_PWM_TIM->CCR1 = val;
}

void motEncTimPeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(!initalised) return;
	overflowCtr++;
}

// PRIVATE FUNCTION CODE

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

void Error_Handler(void)
{
  __disable_irq();
  while (1);
}
