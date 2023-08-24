/*
 * printf32.c
 *
 *  Created on: Aug 21, 2023
 *      Author: stancar
 */


#include "printf32.h"
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdint.h>

char printf_buf[PRINTF32_DATABUF_LEN];

void sendBuf(UART_HandleTypeDef* uart){
	HAL_UART_Transmit(uart, (uint8_t*)printf_buf, strlen(printf_buf), 0xFFFF);
}
