#ifndef __Z_GIMBAL_H__
#define __Z_GIMBAL_H__


#include "math_calcu.h"
#include "pid.h"
#include "bsp_can.h"

#define ZGIMBAL_ID 0X208

typedef struct
{
    /* pit ecd pid param */
    float ecd_ref;
    float ecd_fdb;
    float ecd_err;
    /* pit spd pid param */
    float spd_ref;
    float spd_fdb;
} zgimbal_pid_t;

typedef enum
{
    ZGIM_MODE_PROTECT   = 0,    //保护模式
    ZGIM_MODE_SPRING    = 1,    //弹簧自然减震模式
    ZGIM_MODE_ECD   = 2,        //钳位模式
    ZGIM_MODE_AUTO  = 3         //自适应模式
}zgim_mode_e;

typedef struct
{
    zgim_mode_e mode;
    uint8_t acc_flag;//初始化电机启动标志
    
    zgimbal_pid_t pid_param;
    pid_t ecd_pid;
    pid_t spd_pid;
    moto_measure_t moto;
    float theta;//转轴相对于竖直方向上的角度
    uint16_t init_cnt;
    int32_t  last_total_ecd;
    float zacc;

    int32_t ecd_max;
    int32_t ecd_min;

    int16_t current;
} zgimbal_t;

void zgimbal_init(void);

void zgimbal_task(void const *argu);

void zgimbal_send_task(void const *argu);

void zgimbal_data_handler(CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data);

extern zgimbal_t zgim;

#endif

