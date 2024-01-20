#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "main.h"
#include "BMI088_Read.h"
#include "data_processing.h"
#include "math.h"
typedef enum{
   hight_temperature,
	 normal,
	temperature_error
}imu_mode_e;
extern imu_mode_e imu_mode;
void IMU_Values_Convert(void);
void IMU_AHRS_Calcu(void) ;

#endif


