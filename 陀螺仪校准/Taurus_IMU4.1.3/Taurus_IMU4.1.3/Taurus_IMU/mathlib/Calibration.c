/**
	* @file Calibration.c
	* @version 1.0
	* @date 2019.12.1
  *
  * @brief  ��ƫУ׼����
  *
  *	@author YY(Part of codes reference ��������)
  *
  */

#include "Calibration.h"
#include "pid.h"
#include "bsp_bmi088.h"
#define ABS(x)		((x>0)? (x): (-x))
IMU_FLOAT_DATA_T Bias;	//��ƫֵ
float AccRatioOffset;
void BMI088_Calibration()
{
	float Gyro_Bias_X,Gyro_Bias_Y,Gyro_Bias_Z,Accel_Bias_X,Accel_Bias_Y,Accel_Bias_Z,Acc_ratio;
	BMI088_temp_data_read();
	Bias.init_tempture = Temperature;
	for(uint16_t i=0;i<1000;i++)
	{
		BMI088_original_data_read();			//�õ�δ�˲���ԭʼ����
		BMI088_Filter();									//�˲�����
		Gyro_Bias_X  += imu_output_data.Gyro.X;
		Gyro_Bias_Y  += imu_output_data.Gyro.Y;
		Gyro_Bias_Z  += imu_output_data.Gyro.Z;
		Accel_Bias_Z += imu_output_data.Accel.Z;
		Acc_ratio += sqrt(imu_output_data.Accel.X/1365.0f*imu_output_data.Accel.X/1365.0f+imu_output_data.Accel.Y/1365.0f*imu_output_data.Accel.Y/1365.0f+imu_output_data.Accel.Z/1365.0f*imu_output_data.Accel.Z/1365.0f);
		HAL_Delay(1);
	}
	Bias.Accel.X = Accel_Bias_X/1000;
	Bias.Accel.Y = Accel_Bias_Y/1000;
	Bias.Accel.Z = Accel_Bias_Z/1000;
	Bias.Gyro.X = Gyro_Bias_X/1000;
	Bias.Gyro.Y = Gyro_Bias_Y/1000;
	Bias.Gyro.Z = Gyro_Bias_Z/1000;	
 Bias.Gyro.init_z = Gyro_Bias_Z/1000;		
  AccRatioOffset = Acc_ratio/1000.0f;
//	Bias.Accel.X = 17.0f;
//	Bias.Accel.Y = -13.5f;
//	Bias.Accel.Z = -20.0f;
	HAL_Delay(50);
}


