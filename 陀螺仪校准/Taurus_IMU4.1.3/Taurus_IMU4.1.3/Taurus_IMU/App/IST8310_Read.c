/**
	* @file IST8310_Read.c
	* @version 1.0
	* @date 2019.11.26
  *
  * @brief  IST8310���ݸ�������
  *
  *	@author YY
  *
  */

#include "IST8310_Read.h"

uint8_t magnetism_tx_data[12];	// [0,3]:X������� ��[4,7]:Y������� ��[8,11]:Z�������


/**
  * @brief IST_Read
  * @param 
  * @attention  
  * @note  
  */
void IST_Read(void)
{
	//uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		IST8310_original_data_read();		 //������ԭʼ���ݶ�ȡ
		IST8310_Filter();								 //�˲�����
//		osDelayUntil(&mode_wake_time,5); //Ƶ��200HZ
	}

}

