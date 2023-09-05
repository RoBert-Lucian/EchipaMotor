#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f0xx_hal.h"
#include <stdint.h>

// HARDWARE CONFIGURATION
#define MOT_ODO_TIM TIM1
#define MOT_ENC_TIM TIM3
#define MOT_PWM_TIM TIM14
#define MOT_TIM_TIM TIM6
#define MOT_DIR1_GPIO_PORT GPIOB
#define MOT_DIR1_GPIO_PIN GPIO_PIN_6
#define MOT_DIR2_GPIO_PORT GPIOC
#define MOT_DIR2_GPIO_PIN GPIO_PIN_7
#define MOT_INT_PIN GPIO_PIN_0

// ERROR CODES
#define MOT_ERR 0x4D00
#define MOT_ERR_GENERALERR 0x4D01
#define MOT_ERR_NOTIMPLEMENTED 0x4D02
#define MOT_ERR_UNINITIALISED 0x4D03

// PHISICAL PARAMETERS
#define MOT_WHEEL_LENGTH_MM 215
#define MOT_ENC_PULSES_PER_ROT 22
#define MOT_ODO_PULSES_PER_ROT 11
#define MOT_GEAR_RATIO 4.76

// EXPORTED FUNCTION PROTOTYPES
void motInit(uint32_t initialOdometer);
float motGetOdometer(void);
float motGetVelocity(void);
void motSetVelocity(float velocity);

// EXPORTED CALLBACKS
void motTimPeriodElapsedCallback(TIM_HandleTypeDef *htim);
void motGpioExtiCallback(uint16_t GPIO_Pin);

#endif /* INC_MOTOR_H_ */
