/** 
  * @file     IMU_AHRSupdate.c
  * @version  v1.0
  * @date     2020.1.10
	*
  * @brief    ��̬�Ǹ��º���
	*
  *	@author   YY
  *
  */

#include "IMU_AHRSupdate.h"

extern IMU_FLOAT_DATA_T imu_real_data;

uint8_t angle_tx_data[12];
uint8_t offset_time=0;

float flag_quaternion_intilized;
/**
  * @brief IMU_AHRSupdate_task
  * @param 
  * @attention  
  * @note  
  */
void IMU_AHRSupdate_task(void)
{
//	uint32_t mode_wake_time = osKernelSysTick();
//	for(;;)
//	{
	
	 
		IMU_Values_Convert(); //ԭʼ���ݻ���
		IMU_AHRS_Calcu();			//��̬�ǽ���
		
		Float2Byte(&imu_real_data.roll ,angle_tx_data,0);	
		Float2Byte(&imu_real_data.pitch,angle_tx_data,4);
		Float2Byte(&imu_real_data.yaw  ,angle_tx_data,8); //ת��Ϊ�ɴ��������
		if(Data_Ready_Flag==1)	//���ݾ����ŷ���
		{
		//	osSignalSet(CAN_MSG_SENDHandle, ANGLE_MSG_SEND);
		}
		
	//	osDelayUntil(&mode_wake_time,2);
//	}
	
}

