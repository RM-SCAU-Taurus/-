#ifndef __STATUS_TASK_H
#define __STATUS_TASK_H

#include "Comm_Task.h"
#include "BMI088_Read.h"

#define Status_LED_PWM_Set(pwm)	TIM2->CCR2=(pwm)	//״̬��־LED

//extern osTimerId	status_timer_id;
extern uint8_t Data_Ready_Flag;

void Data_Not_Ready(void);
void Data_Ready(void);
void status_task(void const * argument);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif

