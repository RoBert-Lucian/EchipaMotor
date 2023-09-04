#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f0xx_hal.h"
#include <stdint.h>

#define MOT_ENC_TIM TIM3
#define MOT_PWM_TIM TIM14
#define MOT_DIR1_GPIO_PORT GPIOB
#define MOT_DIR1_GPIO_PIN GPIO_PIN_6
#define MOT_DIR2_GPIO_PORT GPIOC
#define MOT_DIR2_GPIO_PIN GPIO_PIN_7

#define MOT_FORWARD 1
#define MOT_STOP 0
#define MOT_BACKWARD -1

void motInit(uint32_t odometer);
float motGetOdometer(void);
float motGetVelocity(void);
void motSetVelocity(float velocity);

void motEncTimPeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_MOTOR_H_ */
