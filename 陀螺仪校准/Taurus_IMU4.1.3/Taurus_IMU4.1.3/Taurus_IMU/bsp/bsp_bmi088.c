/**
	* @file bsp_bmi088.c
	* @version 1.0
	* @date 2019.11.10
  *
  * @brief  ��ȡbmi088���ݵײ㺯��
  *
  *	@author YY
  *
  */
	
#include "bsp_bmi088.h"
#include "pid.h"
IMU_INT16_DATA_T imu_org_data; //IMUԭʼ����
float 	 	 Temperature;	 //IMU�¶�
float     calibration_temperature;//У׼�¶�
/***************************************************************/
/*
 * ��������BMI088_Init
 * ����  ��BMI088��ʼ������
 * ����  ����
 * ���  ��0����ʼ��ʧ�� 1����ʼ���ɹ�
 */ 
/***************************************************************/
uint8_t BMI088_Init(void)
{
	HAL_GPIO_WritePin(GPIOA, ACCEL_SS_Pin, GPIO_PIN_SET); //�Ӽ��л���SPIͨ��
	HAL_Delay(1);
	BMI088_Write_Reg(Accel,ACC_PWR_CTRL,0x04); //�����Ӽ�
	HAL_Delay(55);
	BMI088_Write_Reg(Accel,ACC_PWR_CONF,0x00); //����Ӽƹ���ģʽ
	
	uint8_t Gyro_Init_Data[4][2] =
	{
		{GYRO_RANGE, 				0x00}, //����2000��/s
	//	{GYRO_BANDWIDTH,		0x01}, //�������ݸ���Ƶ��2000HZ���ڲ���ͨ�˲���230HZ
		{GYRO_BANDWIDTH,		0x02}, //�������ݸ���Ƶ��1000HZ���ڲ���ͨ�˲���116HZ
	//	  {GYRO_BANDWIDTH,		0x03}, //�������ݸ���Ƶ��1000HZ���ڲ���ͨ�˲���116HZ
		{GYRO_INT_CTRL,			0x00}, //���������ݸ����ж�
		//{GYRO_INT_CTRL,			0x08}, //�������ݸ����ж�
		{INT3_INT4_IO_MAP,	0x00}, //�����ж�������ӳ��
	};
	uint8_t Accel_Init_Data[2][2] =
	{
		//{ACC_CONF,					0x9c}, //���������������1600HZ
		//{ACC_CONF,					0x87}, //���������������800HZ
		{ACC_CONF,					0xA7}, //����ODR=1.6kHz����������ģʽ��������ģʽ��(norm_avg4)
		{ACC_RANGE,					0x03}, //��������Ϊ+-3g����ͨ�˲���234HZ
		 //{ACC_RANGE,					0x00}, //��������Ϊ+-3g����ͨ�˲���234HZ
		
	};
	HAL_Delay(55);
	
	if(BMI088_Read_Reg(Gyro,GYRO_CHIP_ID) != 0x0F) //У��������ͨ�ŵ�ַ
		return 0;
	else
	{
		for(uint8_t g = 0; g < 4; g++)
		{
			BMI088_Write_Reg(Gyro, Gyro_Init_Data[g][0], Gyro_Init_Data[g][1]);
			HAL_Delay(1);
		}
	}
	
	if(BMI088_Read_Reg(Accel,ACC_CHIP_ID) != 0x1E) //У����ٶȼ�ͨ�ŵ�ַ
		return 0;
	else
	{
		for(uint8_t a = 0; a < 2; a++)
		{
			BMI088_Write_Reg(Accel, Accel_Init_Data[a][0], Accel_Init_Data[a][1]);
			HAL_Delay(1);
		}
	}
	
	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);	//���ȵ���pwm��ʼ��
	PID_struct_init(&pid_temperature,POSITION_PID,1000,800,100,5,5);
	TIM17->CCR1 = pid_calc(&pid_temperature,Temperature,27);
		return 1;
}

/***************************************************************/
/*
 * ��������BMI088_original_data_read
 * ����  ����ȡBMI088ԭʼ����
 * ����  ����
 * ���  ����
 */ 
/***************************************************************/
void BMI088_original_data_read(void)
{
	uint16_t gyro_rec_buff[4];
	uint16_t accel_rec_buff[3];
	
	GYRO_SS(0);
	gyro_rec_buff[0]=SPI_Read_Write_Byte((((uint16_t)RATE_X_LSB)<<8) | 0x80ff);
	for(uint8_t g=1; g<4; g++)
	{
		gyro_rec_buff[g] = SPI_Read_Write_Byte(0xffff);
	}
	GYRO_SS(1);
	
	ACCEL_SS(0);
	SPI_Read_Write_Byte((((uint16_t)ACC_X_LSB)<<8) | 0x80ff);
	for(uint8_t a=0; a<3; a++)
	{
		accel_rec_buff[a] = SPI_Read_Write_Byte(0xffff);
	}
	ACCEL_SS(1);
	
	imu_org_data.Gyro.X = (gyro_rec_buff[1]&0xff00) | (gyro_rec_buff[0]&0x00ff);
	imu_org_data.Gyro.Y = (gyro_rec_buff[2]&0xff00) | (gyro_rec_buff[1]&0x00ff);
//	imu_org_data.Gyro.Y = (gyro_rec_buff[1]&0xff00) | (gyro_rec_buff[0]&0x00ff);
//	imu_org_data.Gyro.X = (gyro_rec_buff[2]&0xff00) | (gyro_rec_buff[1]&0x00ff);
	imu_org_data.Gyro.Z = (gyro_rec_buff[3]&0xff00) | (gyro_rec_buff[2]&0x00ff);
	imu_org_data.Accel.X= (accel_rec_buff[0]<<8) | (accel_rec_buff[0]>>8);
	imu_org_data.Accel.Y= (accel_rec_buff[1]<<8) | (accel_rec_buff[1]>>8);
//	imu_org_data.Accel.Y= (accel_rec_buff[0]<<8) | (accel_rec_buff[0]>>8);
//	imu_org_data.Accel.X= (accel_rec_buff[1]<<8) | (accel_rec_buff[1]>>8);
	imu_org_data.Accel.Z= (accel_rec_buff[2]<<8) | (accel_rec_buff[2]>>8);
}

/***************************************************************/
/*
 * ��������BMI088_temp_data_read
 * ����  ����ȡBMI088�¶�
 * ����  ����
 * ���  ����
 */ 
/***************************************************************/
void BMI088_temp_data_read(void)
{
	uint16_t temp_rec_buff, temp_uint11;
	int16_t  temp_int11;
	
	ACCEL_SS(0);
	
	SPI_Read_Write_Byte((((uint16_t)TEMP_MSB)<<8) | 0x80ff);
	temp_rec_buff = SPI_Read_Write_Byte(0xffff);
	
	ACCEL_SS(1);
	
	temp_uint11 = ((temp_rec_buff>>8)*8) + ((temp_rec_buff&0x00ff)/32);
	if(temp_uint11>1023)
		temp_int11 = temp_uint11-2048;
	else
		temp_int11 = temp_uint11;
	Temperature = (temp_int11*0.125)+23;
}



