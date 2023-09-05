// INCLUDES
#include <math.h>
#include <stdio.h>
#include "motor.h"
#include "stm32f0xx_hal.h"

// PRIVATE DEFINES
#define MOT_FORWARD 1
#define MOT_STOP 0
#define MOT_BACKWARD -1

// PRIVATE VARIABLES
static int initalised = 0;
static float targetVel;
static uint32_t initialOdometerPulseValue = 0;
static uint32_t odometerOverflowCtr = 0;
static uint32_t timekeepingTimerOverflowCtr = 0;
static uint32_t timekeepingTimerT1 = 0;
static uint32_t timekeepingTimerT2 = 0;

// PRIVATE FUNCTION PROTOTYPES
static void motSetDirection(int);
static void Error_Handler(uint32_t err);

// PUBLIC FUNCTION CODE
void motInit(uint32_t initialOdometer){
	initialOdometerPulseValue = initialOdometer * 1000/*m to mm conversion*/ * MOT_ODO_PULSES_PER_ROT / MOT_WHEEL_LENGTH_MM;
	targetVel = 0;
	initalised = 1;
	motSetVelocity(targetVel);
}

float motGetOdometer(void){
	if(!initalised) Error_Handler(MOT_ERR_UNINITIALISED);
	int32_t pulseCount = (odometerOverflowCtr<<16) + MOT_ODO_TIM->CNT + initialOdometerPulseValue;
	return ((float)pulseCount) * MOT_WHEEL_LENGTH_MM / (MOT_GEAR_RATIO * MOT_ODO_PULSES_PER_ROT * 1000/*mm to m conversion*/ );
}

float motGetVelocity(void){
	if(!initalised) Error_Handler(MOT_ERR_UNINITIALISED);
	uint32_t timerTickCount = timekeepingTimerT2 - timekeepingTimerT1;

//	float vel = (48000000.0) / (timerTickCount * MOT_ODO_PULSES_PER_ROT);
	float vel = timerTickCount / 48000000.0;
	return vel;
}

void motSetVelocity(float velocity){
	if(!initalised) Error_Handler(MOT_ERR_UNINITIALISED);

	if(velocity < 0) motSetDirection(MOT_BACKWARD);
	else motSetDirection(MOT_FORWARD);

	targetVel = velocity;

	MOT_PWM_TIM->CCR1 = velocity != 0 ? 0xFFFF : 0x0000;
}

void motTimPeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(!initalised) return;
	if(htim->Instance == MOT_ODO_TIM)
		odometerOverflowCtr++;
	else if(htim->Instance == MOT_TIM_TIM)
		timekeepingTimerOverflowCtr++;
}

void motGpioExtiCallback(uint16_t GPIO_Pin){
	if(GPIO_Pin == MOT_INT_PIN){
		timekeepingTimerT1 = timekeepingTimerT2;
		timekeepingTimerT2 = (timekeepingTimerOverflowCtr << 16) + MOT_TIM_TIM->CNT;
	}
}

// PRIVATE FUNCTION CODE
void motSetDirection(int direction){
	switch(direction){
		case MOT_FORWARD:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_SET);
			break;
		case MOT_BACKWARD:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_RESET);
			break;
		case MOT_STOP:
			HAL_GPIO_WritePin(MOT_DIR1_GPIO_PORT, MOT_DIR1_GPIO_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOT_DIR2_GPIO_PORT, MOT_DIR2_GPIO_PIN, GPIO_PIN_RESET);
			break;
	}
}

void Error_Handler(uint32_t err)
{
  __disable_irq();
  while (1);
}
