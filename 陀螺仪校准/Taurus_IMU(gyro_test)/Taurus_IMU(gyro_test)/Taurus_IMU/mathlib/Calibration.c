/**
	* @file Calibration.c
	* @version 1.0
	* @date 2019.12.1
  *
  * @brief  零偏校准函数
  *
  *	@author YY(Part of codes reference 无名创新)
  *
  */

#include "Calibration.h"
#include "pid.h"
#include "bsp_bmi088.h"
#include "bsp_imu.h"
#define ABS(x)		((x>0)? (x): (-x))
IMU_FLOAT_DATA_T Bias;	//零偏值
float AccRatioOffset;
void BMI088_Calibration()
{
	float Gyro_Bias_X,Gyro_Bias_Y,Gyro_Bias_Z,Accel_Bias_X,Accel_Bias_Y,Accel_Bias_Z,Acc_ratio;
	BMI088_temp_data_read();
	Bias.init_tempture = Temperature;
	bias_gyro_mode = Calibration_successful_mode;
	for(uint16_t i=0;i<10000;i++)
	{
		BMI088_original_data_read();			//得到未滤波的原始数据
		BMI088_Filter();									//滤波处理
		if(ABS(imu_output_data.Gyro.X)>=20||ABS(imu_output_data.Gyro.Y)>=20||ABS(imu_output_data.Gyro.Z)>=20)
		{
		   bias_gyro_mode = Calibration_error_mode;
		}
//		Gyro_Bias_X  += imu_output_data.Gyro.X;
//		Gyro_Bias_Y  += imu_output_data.Gyro.Y;
//		Gyro_Bias_Z  += imu_output_data.Gyro.Z;
		Gyro_Bias_X  += imu_org_data.Gyro.X;
		Gyro_Bias_Y  += imu_org_data.Gyro.Y;
		Gyro_Bias_Z  += imu_org_data.Gyro.Z;
	  Accel_Bias_X += imu_org_data.Accel.X;
		Accel_Bias_Y += imu_org_data.Accel.Y;
		Accel_Bias_Z += imu_org_data.Accel.Z;
		HAL_Delay(1);
	}
	Accel_Bias_X = Accel_Bias_X/10000/1365.0f;
	Accel_Bias_Y = Accel_Bias_Y/10000/1365.0f;
	Accel_Bias_Z = Accel_Bias_Z/10000/1365.0f;
	Bias.Accel.X = imu_output_data.Accel.X/1365.0f-0;
	Bias.Accel.Y = imu_org_data.Accel.Y/1365.0f-0;
	Bias.Accel.Z = imu_org_data.Accel.Z/1365.0f-1;
	Bias.Gyro.X = Gyro_Bias_X/10000;
	Bias.Gyro.Y = Gyro_Bias_Y/10000;
	Bias.Gyro.Z = Gyro_Bias_Z/10000;	
 Bias.Gyro.init_z = Gyro_Bias_Z/2000;		
  AccRatioOffset = Acc_ratio/1000.0f;
//	Bias.Accel.X = 17.0f;
//	Bias.Accel.Y = -13.5f;
//	Bias.Accel.Z = -20.0f;
	HAL_Delay(50);
}


