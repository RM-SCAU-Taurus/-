#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "main.h"
#include "BMI088_Read.h"
#include "data_processing.h"
#include "math.h"


void IMU_Values_Convert(void);
void IMU_AHRS_Calcu(void) ;
typedef enum{
   Calibration_error_mode,
	 Calibration_successful_mode
}bias_gyro_mode_e;
extern bias_gyro_mode_e bias_gyro_mode;
#endif


