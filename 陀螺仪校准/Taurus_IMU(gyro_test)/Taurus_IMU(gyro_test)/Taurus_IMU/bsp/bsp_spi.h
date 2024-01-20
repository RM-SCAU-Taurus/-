#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "stm32f3xx_hal.h"
#include "spi.h"
#include "gpio.h"

#define  GYRO_SS(X)		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,(X==0)?GPIO_PIN_RESET:GPIO_PIN_SET) //������Ƭѡ�ź�
#define  ACCEL_SS(X)	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,(X==0)?GPIO_PIN_RESET:GPIO_PIN_SET) //���ٶȼ�Ƭѡ�ź�

typedef enum
{
	Gyro,
	Accel,
}reg_type_e;

uint16_t SPI_Read_Write_Byte(uint16_t TxData);
uint8_t  BMI088_Read_Reg(reg_type_e regtyp, uint8_t regaddr);
void BMI088_Write_Reg(reg_type_e regtyp, uint8_t reg,uint8_t value);


#endif




