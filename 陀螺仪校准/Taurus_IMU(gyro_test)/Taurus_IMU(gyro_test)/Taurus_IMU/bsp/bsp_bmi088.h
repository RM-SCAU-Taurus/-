#ifndef __BSP_BMI088_H
#define __BSP_BMI088_H

#include "stm32f3xx_hal.h" 
//#include "FreeRTOS.h"
//#include "task.h"
#include "main.h"
#include "tim.h"
//#include "cmsis_os.h"
#include "bsp_spi.h"
#include "BMI088_Read.h"

#define Heat_PWM_Set(pwm)	TIM17->CCR1=pwm	//���ȵ���pwm

typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}S_INT16_XYZ;

typedef struct
{
	float X;
	float Y;
	float Z;
	float init_z;
}S_FLOAT_XYZ;

typedef struct
{
	S_INT16_XYZ Gyro;
	S_INT16_XYZ Accel;
	S_INT16_XYZ Mag;
}IMU_INT16_DATA_T;

typedef struct
{
	S_FLOAT_XYZ Gyro;		//���ٶ�
	S_FLOAT_XYZ Accel;	//���ٶ�
	S_FLOAT_XYZ Mag;    //������
	
	float q[4]; 				//����Ԫ��
	
	float roll;
	float pitch;
	float yaw;					//ŷ����
	float init_tempture;
}IMU_FLOAT_DATA_T;

extern IMU_INT16_DATA_T imu_org_data;				//ԭʼ����
extern IMU_FLOAT_DATA_T imu_output_data;		//�������
extern IMU_FLOAT_DATA_T Bias;								//��ƫֵ
extern float Temperature;	
// ����BMI088�ڲ���ַ
/*******************************GYRO******************************/
#define GYRO_CHIP_ID 			0x00
#define RATE_X_LSB				0x02
#define RATE_X_MSB				0x03
#define	RATE_Y_LSB  			0x04
#define	RATE_Y_MSB  			0x05
#define RATE_Z_LSB				0x06
#define	RATE_Z_MSB  			0x07
#define GYRO_INT_STAT_1		0x0A
#define GYRO_RANGE				0x0F
#define	GYRO_BANDWIDTH		0x10
#define GYRO_LPM1					0x11
#define GYRO_SOFTRESET		0x14
#define GYRO_INT_CTRL			0x15
#define INT3_INT4_IO_CONF	0x16
#define INT3_INT4_IO_MAP	0x18
#define GYRO_SELF_TEST		0x3C
/*******************************Accelerometer******************************/
#define ACC_CHIP_ID				0x00
#define ACC_ERR_REG 			0x02
#define ACC_STATUS				0x03
#define	ACC_X_LSB 				0x12
#define ACC_X_MSB 				0x13
#define ACC_Y_LSB					0x14
#define ACC_Y_MSB					0x15
#define ACC_Z_LSB					0x16
#define ACC_Z_MSB  				0x17
#define SENSORTIME_0			0x18
#define SENSORTIME_1			0x19
#define SENSORTIME_2 			0x1A
#define ACC_INT_STAT_1		0x1D
#define TEMP_MSB					0x22
#define	TEMP_LSB					0x23
#define ACC_CONF					0x40
#define ACC_RANGE					0x41
#define INT1_IO_CTRL			0x53
#define INT2_IO_CTRL			0x54
#define INT_MAP_DATA			0x58
#define ACC_SELF_TEST			0x6D
#define ACC_PWR_CONF			0x7C
#define ACC_PWR_CTRL			0x7D
#define	ACC_SOFTRESET			0x7E


uint8_t BMI088_Init(void);
extern float Temperature;
void BMI088_original_data_read(void);
void BMI088_temp_data_read(void);

#endif


