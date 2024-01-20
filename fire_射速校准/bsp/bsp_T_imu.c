/**
	* @file bsp_T_imu.c
	* @version 1.0
	* @date 2020.1.4
  *
  * @brief  Taurus ����������
  *
  *	@author YY
  */
#include "bsp_T_imu.h"
#include "bsp_can.h"
#include "gimbal_task.h"
#include "KalmanFilter.h"
#include "usart.h"
#include "status_task.h"

#include "us_tim.h"
#include "ubf.h"

extern ubf_t imu_yaw_ubf; /* ���ݸ��»����� */

ust_t ust_palstance; /* ��ʱ�� */
ust_t ust_angle; /* ��ʱ�� */
Taurus_imu_data_t   imu_data; /* ���������� */
float temp_speed;
void T_imu_calcu(uint32_t can_id,uint8_t * CAN_Rx_data)
{
    switch(can_id)
    {
        case TIMU_PALSTANCE_ID:	//���ٶ� �������� 3us����
        {
            ust_period_test(&ust_palstance);
            float palstance_buffer[2];
            memcpy(palstance_buffer,CAN_Rx_data,8);
            imu_data.wy = palstance_buffer[0];  //��������ˮƽ��ת180��ʱ��Ҫ��-��
            imu_data.wz = palstance_buffer[1];//��λLSB��/16.3835f/57.3f ֮���� rad/s
            break;
        }
        case TIMU_ANGLE_ID: //�Ƕ� �������� 1ms 0X002
        {
            ust_period_test(&ust_angle);
            float angle_buffer[2];
            memcpy(angle_buffer,CAN_Rx_data,8);
            imu_data.pitch = angle_buffer[0];  //��������ˮƽ��ת180��ʱ��Ҫ��-��
            imu_data.yaw   = angle_buffer[1];
            ubf_push(imu_yaw_ubf, &imu_data.wz);//δ֪bug�����ô��ȶ����ڶ�ȡ���ٶ�
            break;
        }
    }
}
