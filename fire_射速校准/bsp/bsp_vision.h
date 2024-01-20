/**
  * @file bsp_vision.h
  * @version 1.0
  * @date Feb,23th 2021
	*
  * @brief  视觉信息解算
	*
  *	@author
  *
  */
#ifndef	__BSP_VISION_H__
#define __BSP_VISION_H__

#include "stdint.h"

typedef struct
{
    float now;
    float last;
} vision_data_pkg2_t;

typedef struct
{
    float now;
    float last;
    float kal;
} vision_data_pkg3_t;

typedef struct
{
    vision_data_pkg3_t vision_angle_error;
    vision_data_pkg3_t vision_angular_speed;
    vision_data_pkg3_t imu_angular_speed;
    vision_data_pkg3_t object_angular_speed;
    vision_data_pkg3_t object_tangential_speed;
    vision_data_pkg3_t object_radial_speed;
    vision_data_pkg3_t object_2D_speed;
    vision_data_pkg3_t object_speed_angle;
    vision_data_pkg3_t predict_distance;
    vision_data_pkg3_t opposite_line;
    vision_data_pkg3_t predict_angle;
} vision_yaw_data_t;

typedef struct
{
    vision_yaw_data_t yaw;
    vision_data_pkg3_t pit_angle_error;
    vision_data_pkg3_t distance;
    vision_data_pkg3_t tof;
    float period;
    uint8_t cnt;
    uint8_t eof;
} vision_data_t;

typedef struct
{
    float pit_angle_error;
    float pit_predict_angle;
    float yaw_angle_error;
    float yaw_predict_angle;
    
    volatile uint8_t aiming_flag;
    volatile uint8_t first_lost_flag;
    volatile uint8_t new_frame_flag;
} vision_output_t;

extern vision_data_t vd;
extern vision_output_t vision;

void vision_data_handler(uint8_t *Vision_Data);

#endif

