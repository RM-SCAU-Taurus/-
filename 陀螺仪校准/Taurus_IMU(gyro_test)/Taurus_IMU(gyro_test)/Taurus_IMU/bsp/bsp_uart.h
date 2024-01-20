#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "string.h"
#include "stdlib.h"
#include "usart.h"
#include "Status_Task.h"


void user_uart_init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart);

#endif

