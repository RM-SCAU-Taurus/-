/**
	* @file bsp_spi.c
	* @version 1.0
	* @date 2019.11.9
  *
  * @brief  SPI��д����
  *
  *	@author YY
  *
  */

#include "bsp_spi.h"


/*
 * ��������BMI088_Write_Reg
 * ����  ��SPIд��Ĵ���
 * ����  ��retyp:дGyro/Accel�ļĴ��� ��
					 reg:ָ���ļĴ�����ַ��
					 value��д���ֵ
 * ���  ����
 */ 
void BMI088_Write_Reg(reg_type_e regtyp, uint8_t regaddr,uint8_t value)
{
	regaddr &= 0x7f;	//�Ĵ�����ַ+д����
	regtyp==Gyro?GYRO_SS(0):ACCEL_SS(0);
	SPI_Read_Write_Byte((((uint16_t)regaddr)<<8) | (uint16_t)value);
	regtyp==Gyro?GYRO_SS(1):ACCEL_SS(1);
}


/*
 * ��������BMI088_Read_Reg
 * ����  ��SPI��ȡ�Ĵ���
*  ����  ��retyp:��ȡGyro/Accel�ļĴ���
					 regaddr:ָ���ļĴ�����ַ
 * ���  ��reg_val��reg�Ĵ�����ַ��Ӧ��ֵ
 */ 
uint8_t BMI088_Read_Reg(reg_type_e regtyp, uint8_t regaddr)
{
	uint8_t  reg_val;
	regaddr |= 0x80;	//�Ĵ�����ַ+������
	switch(regtyp)
	{
		case Gyro:	
		{	
			GYRO_SS(0);																												
			reg_val=SPI_Read_Write_Byte((((uint16_t)regaddr)<<8) | 0x00ff);																
			GYRO_SS(1);																													
			break;
		}
		case Accel:
		{
			ACCEL_SS(0);
			SPI_Read_Write_Byte((((uint16_t)regaddr)<<8) | 0x00ff);
			reg_val=SPI_Read_Write_Byte(0xffff)>>8;
			ACCEL_SS(1);
			break;
		}
	}
	return(reg_val);
}


/*
 * ��������SPI_Read_Write_Byte
 * ����  ����дһ���ֽ�
 * ����  ��TxData:Ҫд����ֽ�
 * ���  ����ȡ�����ֽ�
 */ 
uint16_t SPI_Read_Write_Byte(uint16_t TxData)
{		
	uint16_t Rxdata;
	HAL_SPI_TransmitReceive(&hspi3,(uint8_t*)&TxData,(uint8_t*)&Rxdata,1, 1000);       
	return Rxdata;  				    
}



