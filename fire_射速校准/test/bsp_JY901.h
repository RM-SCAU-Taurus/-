#ifndef __BSP_JY901_H
#define __BSP_JY901_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "usart.h"
#include "string.h"
#include "stdlib.h"

#define JY901_BUFF_LEN 33

typedef struct
{
    float Gyro_a_x;
    float Gyro_a_y;
    float Gyro_a_z; //静止状态下为 9.8m/s^2
	float roll;
	float pitch;
	float yaw;
	float Gyro_x;
	float Gyro_y;
	float Gyro_z;
    
    /* 以下供Z轴云台使用 */
    float gravity;
    uint32_t cnt;
    
    float acc_x;
    float acc_y;
    float acc_z;
    uint32_t stop_cnt;
    
    float z_acc[2];
    float z_spd[2];
    float z_pos;

}imu_typedef;

extern imu_typedef JY901_data;
extern uint8_t dma_JY901_data_buff[JY901_BUFF_LEN];
void JY901_original_data_read(uint8_t imu_buf[]);

#endif

