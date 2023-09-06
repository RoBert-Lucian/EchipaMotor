// INCLUDES
#include <math.h>
#include <stdio.h>
#include "motor.h"
#include "pid.h"
#include "stm32f0xx_hal.h"

// PRIVATE DEFINES
#define MOT_FORWARD 1 /*formula value. do not modify*/
#define MOT_STOP 0
#define MOT_BACKWARD -1 /*formula value. do not modify*/

#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

// PRIVATE VARIABLES
static int initalised = 0;
static float targetVel;
static uint32_t initialOdometerPulseValue = 0;
static uint32_t odometerOverflowCtr = 0;
static uint32_t timekeepingTimerOverflowCtr = 0;
static uint32_t timekeepingTimerT1 = 0;
static uint32_t timekeepingTimerT2 = 0;
static PIDController pid = {PID_KP, PID_KI, PID_KD, PID_TAU, PID_LIM_MIN, PID_LIM_MAX, PID_LIM_MIN_INT, PID_LIM_MAX_INT, SAMPLE_TIME_S};

// PRIVATE FUNCTION PROTOTYPES
static float motGetDirection();
static void motSetDirection(int);
static void motSetPWM(uint32_t);
static void motSetVelocity(float);
static void Error_Handler(uint32_t err);

// PUBLIC FUNCTION CODE
void motInit(uint32_t initialOdometer){
	initialOdometerPulseValue = initialOdometer * 1000/*m to mm conversion*/ * MOT_ODO_PULSES_PER_ROT / MOT_WHEEL_LENGTH_MM;
	PIDController_Init(&pid);
	targetVel = 0;
	initalised = 1;
	motSetTargetVelocity(targetVel);
}

float motGetOdometer(void){
	if(!initalised) Error_Handler(MOT_ERR_UNINITIALISED);
	int32_t pulseCount = (odometerOverflowCtr<<16) + MOT_ODO_TIM->CNT + initialOdometerPulseValue;
	return ((float)pulseCount) * MOT_WHEEL_LENGTH_MM / (MOT_GEAR_RATIO * MOT_ODO_PULSES_PER_ROT * 1000/*mm to m conversion*/ );
}

float motGetVelocity(void){
	if(!initalised) Error_Handler(MOT_ERR_UNINITIALISED);
	uint32_t timerTickCount = timekeepingTimerT2 - timekeepingTimerT1;

	float vel = motGetDirection() * (48000000.0 * 60) / (timerTickCount * MOT_ODO_PULSES_PER_ROT);

	return vel;
}

void motSetTargetVelocity(float velocity){
	if(!initalised) Error_Handler(MOT_ERR_UNINITIALISED);

	targetVel = velocity;
}

void motTimPeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(!initalised) return;
	if(htim->Instance == MOT_ODO_TIM)
		odometerOverflowCtr++;
	else if(htim->Instance == MOT_TIM_TIM)
		timekeepingTimerOverflowCtr++;
	else if(htim->Instance == MOT_PID_TIM){
//		float measurement = motGetVelocity();
//		PIDController_Update(&pid, targetVel, measurement);
//		motSetVelocity(pid.out);
		motSetVelocity(targetVel);
	}
}

void motGpioExtiCallback(uint16_t GPIO_Pin){
	if(GPIO_Pin == MOT_INT_PIN){
		timekeepingTimerT1 = timekeepingTimerT2;
		timekeepingTimerT2 = (timekeepingTimerOverflowCtr << 16) + MOT_TIM_TIM->CNT;
	}
}

// PRIVATE FUNCTION CODE
static float motGetDirection(){
	if((MOT_ENC_TIM->CR1 >> 4) & 1) return MOT_BACKWARD;
	else return MOT_FORWARD;
}

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

void motSetVelocity(float vel){
	if(vel < 0){
		motSetDirection(MOT_BACKWARD);
		vel *= -1;
	} else motSetDirection(MOT_FORWARD);
	motSetPWM((uint32_t)vel);
}

void motSetPWM(uint32_t pwm){
	MOT_PWM_TIM->CCR1 = pwm;
}

void Error_Handler(uint32_t err)
{
  __disable_irq();
  while (1);
}
