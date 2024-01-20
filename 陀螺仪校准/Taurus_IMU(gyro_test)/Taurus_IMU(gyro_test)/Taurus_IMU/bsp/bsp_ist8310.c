/**
	* @file bsp_ist8310.c
	* @version 1.0
	* @date 2019.11.29
  *
  * @brief  ������IST8310����
  *
  *	@author YY
  *
  */
	
#include "bsp_ist8310.h"

/**********************************************************************************************************
*�� �� ��: IST8310_Init
*����˵��: IST8310�Ĵ������ó�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
bool IST8310_Init(void)
{
	if(IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_WHOAMI) != 0x10)	//У������ƹ���״̬
		return false;
	else
	{
		IIC_Reg_Write(IST8310_ADDRESS,IST8310_REG_CNTRL2,0x04);	//�ر����ݾ����ж�
		HAL_Delay(5);
		IIC_Reg_Write(IST8310_ADDRESS,IST8310_REG_CNTRL1,0x01);	//���õ��β���ģʽ
		HAL_Delay(5);
	}
	return true;
}

/**********************************************************************************************************
*�� �� ��: IST8310_original_data_read
*����˵��: IST8310ԭʼ���ݸ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void IST8310_original_data_read(void)
{
	uint8_t mag_rec_buff[6];
	
	IIC_Reg_Write(IST8310_ADDRESS,IST8310_REG_CNTRL1,0x01);
	delay_us(2);
	
	mag_rec_buff[0] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HX_L);
	mag_rec_buff[1] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HX_H);
	imu_org_data.Mag.X = (uint16_t)mag_rec_buff[1]<<8 | mag_rec_buff[0];
	
	mag_rec_buff[2] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HY_L);
	mag_rec_buff[3] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HY_H);
	imu_org_data.Mag.Y = (uint16_t)mag_rec_buff[3]<<8 | mag_rec_buff[2];
	
	mag_rec_buff[4] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HZ_L);
	mag_rec_buff[5] = IIC_Reg_Read(IST8310_ADDRESS,IST8310_REG_HZ_H);
	imu_org_data.Mag.Z = (uint16_t)mag_rec_buff[5]<<8 | mag_rec_buff[4];
	
}



