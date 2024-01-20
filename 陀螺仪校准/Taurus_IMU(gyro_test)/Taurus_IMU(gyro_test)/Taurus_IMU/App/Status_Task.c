/** 
  * @file     Status_Task.c
  * @version  v1.0
  * @date     2020.1.3
	*
  * @brief    ״̬���
	*
  *	@author   YY
  *
  */

#include "Status_Task.h"
#include "tim.h"
#include "BMI088_Read.h"
#include "IST8310_Read.h"
#include "IMU_AHRSupdate.h"
#include "Comm_Task.h"
uint8_t Data_Ready_Flag=0;	// 0:����δ����	 1�������Ѿ���

uint8_t led_dir=0;	//LED��������
uint16_t led_pwm=0;


/**
  * @brief status_task
  * @param     
  * @attention  
	* @note  
  */
void status_task(void)
{
	Status_LED_PWM_Set(0);
//	for(;;)
//	{
		if(led_dir==0)
		{
			led_pwm++;
			if(led_pwm==1000)
			{
				led_dir=1;
			}
		}
		else
		{
			led_pwm--;
			if(led_pwm==0)
			{
				led_dir=0;
			}
		}
		Status_LED_PWM_Set(led_pwm);	//���ݾ������Ϊ������
//		osDelay(1);
//	}
}


/**
  * @brief Data_Not_Ready
  * @param     
  * @attention  
	* @note  
  */
void Data_Not_Ready(void)
{
	Data_Ready_Flag=0;
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	Status_LED_PWM_Set(0);	//����δ����״̬�Ƴ���
}

/**
  * @brief Data_Ready
  * @param     
  * @attention  
	* @note  
  */
void Data_Ready(void)
{
	Status_LED_PWM_Set(1000);
	HAL_Delay(200);
	Data_Ready_Flag=1;
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//   if(htim==&htim1)
//   {
//	    BMI_Read();
//		// IST_Read();
//	 }     
//   else if(htim==&htim16)
//	 {
//			// IMU_AHRSupdate_task();
//	 }
//	 else if(htim==&htim17)
//	 {
//	    //can_msg_send_task();
//	 }
//}
