#ifndef __pid_H
#define __pid_H

#ifdef  __PID_GLOBALS
#define __PID_EXT
#else
#define __PID_EXT extern
#endif

#include "stdint.h"

typedef enum
{
    POSITION_PID,
    DELTA_PID,
} PID_mode_e;

typedef struct __pid_t
{
    float p;
    float i;
    float d;

    float set[3];           //目标值,包含NOW， LAST， LLAST上上次
    float get[3];           //测量值
    float err[3];           //误差

    float d_error;
    float pout;             //p输出
    float iout;             //i输出
    float dout;             //d输出

    float pos_out;          //本次位置式输出
    float last_pos_out;     //上次输出
    float delta_u;          //本次增量值
    float delta_out;        //本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;

    float max_err;
    float deadband;             //err < deadband return
    float div;
    uint8_t pid_mode;
    float MaxOutput;         //输出限幅
    float IntegralLimit;     //积分限幅

    void (*f_param_init)(struct __pid_t *pid,  //PID参数初始化
                         uint8_t pid_mode,
                         float maxOutput,
                         float integralLimit,
                         float deadband,
                         float div,
                         float p, float i, float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid三个参数修改

} pid_t;

void PID_struct_init(
    pid_t* pid, uint8_t mode, float maxout, 
    float intergral_limit, float deadband, float div,
    float kp, float ki, float kd);
float pid_calc(pid_t* pid, float fdb, float ref);

/* -------------------------------------- Gimbal -------------------------------------- */
// PIT 轴 位置速度串级
__PID_EXT pid_t pid_pit_ecd;
__PID_EXT pid_t pid_pit_spd;

// YAW 轴 角度速度串级
__PID_EXT pid_t pid_yaw_angle;
__PID_EXT pid_t pid_yaw_spd;

// PIT 轴 视觉位置速度串级
__PID_EXT pid_t vision_pid_pit_ecd;
__PID_EXT pid_t vision_pid_pit_spd;

// YAW 轴 视觉角度速度串级
__PID_EXT pid_t vision_pid_yaw_angle;
__PID_EXT pid_t vision_pid_yaw_spd;

//测试YAW轴 位置速度串级 使用YAW电机反馈
__PID_EXT pid_t pid_yaw_mecd;
__PID_EXT pid_t pid_yaw_mspd;
/* -------------------------------------- Trigger -------------------------------------- */
__PID_EXT pid_t pid_trigger_ecd;
__PID_EXT pid_t pid_trigger_spd;

/* -------------------------------------- Chassis -------------------------------------- */
__PID_EXT pid_t pid_chassis_spd[4];
__PID_EXT pid_t pid_chassis_angle;//利用云台电机的编码器来实现底盘的跟随摇摆等控制
//__PID_EXT pid_t pid_chassis_cur[4];

#endif

