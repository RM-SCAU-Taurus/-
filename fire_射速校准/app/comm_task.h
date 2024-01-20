#ifndef __COMM_TASK_H__
#define __COMM_TASK_H__

#include "stm32f4xx_hal.h"

#define CHASSIS_CAN_TX_ID 	0x200
#define GIMBAL_CAN_TX_ID  	0x1ff

#define GIMBAL_MOTOR_MSG_SEND     ( 1 << 6 )
#define CHASSIS_MOTOR_MSG_SEND    ( 1 << 7 )
#define SHOOT_MOTOR_MSG_SEND      ( 1 << 8 )

/* motor current parameter structure */
typedef struct
{
    /* 4 chassis motor current */
    int16_t chassis_cur[4];
    /* yaw/pitch motor current */
    int16_t gimbal_cur[2];
    /* stir current */
    int16_t trigger_cur;

} motor_current_t;

typedef __packed struct
{
    uint8_t SOF; //帧头
    uint8_t imu_pit[4];//陀螺仪数据
    uint8_t imu_yaw[4];  
    uint8_t imu_pit_spd[4];
    uint8_t imu_yaw_spd[4];
    __packed struct
    {
        uint8_t  vacancy:1;		//空闲位
        uint8_t  camp 	: 1;	//阵营
        uint8_t  aiming_status : 3;	//瞄准模式
        uint8_t  shooter_speed : 3;	//射速
    } mode_msg;
    uint8_t  EOF1;	//帧尾1
    uint8_t  EOF2;	//帧尾2
} vision_tx_msg_t;

void can_msg_send_task(void const *argu);
void usart_msg_send_task(void const *argu);

extern motor_current_t motor_cur;
extern vision_tx_msg_t vision_tx_msg;

#endif
