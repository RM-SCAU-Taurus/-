/**
  * @file bsp_vision.c
  * @version 1.0
  * @date Feb,23th 2021
	*
  * @brief  视觉信息解算
	*
  *	@author
  *
  */
#include "bsp_vision.h"
#include "control_def.h"
#include "remote_msg.h"
#include "bsp_can.h"
#include "string.h"
#include "math.h"
#include "remote_msg.h"
#include "bsp_T_imu.h"
#include "KalmanFilter.h"
#include "math_calcu.h"
#include "modeswitch_task.h"
#include "math.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "ubf.h"
#include "us_tim.h"

float test_visoin_delay = 6;/* 视觉延时参量 */
//float test_imu_spd_kp = 16.3835f;

vision_data_t vd;/* 视觉数据结构体 */
vision_output_t vision;/* 视觉计算结果输出结构体 */
ust_t ust_vision_nvic;/* 视觉串口中断计时器 */

extern ubf_t imu_yaw_ubf;/* 陀螺仪角速度缓存器 */
extern ubf_t vision_data_ubf;
float vision_data_array[20];
float var;

/**
 * 目前主要是使用了4个一阶卡尔曼滤波器
 * 1. 视觉回传的角度差分得到的相对角速度
 * 2. 视觉回传的距离
 * 3. 计算得的预测距离
 * 4. 解算得的预测角度
 *
 *  调试经验
 *  1. 看是否有视觉数据
 *  2. 绝对速度拟合三曲线，调延时
 *  3. 
 *
 *  遗留问题
 *  1. 通信延时精确确定
 *  2. can通信延时的问题
 */

/**
  * @brief 视觉信息解算
  * @param
  * @attention
	* @note
  */
void vision_data_handler(uint8_t *Vision_Data)
{
    /* 接收视觉信息 */
    memcpy(&vd.yaw.vision_angle_error.now, Vision_Data, 4);     /* YAW轴相对角度（°） */
    memcpy(&vd.pit_angle_error.now,(Vision_Data+4), 4);         /* PIT轴相对角度（°） */
    memcpy(&vd.distance.now, (Vision_Data+8), 4);                       /* 距离（mm） */
    memcpy(&vd.tof.now, (Vision_Data+12), 4);                    /* 子弹飞行时间（s） */
    memcpy(&vd.cnt, (Vision_Data+16), 1);                                   /* 计数位 */
    memcpy(&vd.eof, (Vision_Data+17), 1);                                     /* 帧尾 */
    
    /* 计算数据方差 */
    ubf_push(vision_data_ubf, &vd.yaw.vision_angle_error.now);
    uint32_t real_num = ubf_pop_into_array_new2old(vision_data_ubf, vision_data_array, 0, 20);
    arm_var_f32(vision_data_array, real_num, &var);

    /* 不加40ms的通信与相应延时 */
    vd.tof.now -= 0.04f;  
    Output_Limit(vd.tof.now, vd.tof.now, 0);

    /* 获取视觉运算周期 */
    vd.period = 8;/* 单位 ms */
//    vd.period = ust_period_test(&ust_vision_nvic);
    
//    static uint32_t last_time, time;
//    time = HAL_GetTick();
//    vd.period = time - last_time;
//    last_time = time;

    /* 单位统一：标准单位制 */
    vd.period *= 1.0e-3f;/* 周期（s）*/
    vd.distance.now *= 1.0e-3f;/* 距离（m） */

    /* 符号统一：俯视逆时针为负 */
    vd.yaw.vision_angle_error.now = -vd.yaw.vision_angle_error.now;
    
    if (vd.distance.now)  //前后两帧都识别到目标
    {
        /* 数据异常处理 */
//        //飞行时间
//        if (vd.tof.now >= 2.0f || vd.tof.now <= -2.0f)
//            vd.tof.now = vd.tof.last;
//        else    vd.tof.last = vd.tof.now;
//        
//        //PIT轴角度
//        if (vd.pit_angle_error.now >= 40 || vd.pit_angle_error.now <= -40)
//            vd.pit_angle_error.now = vd.pit_angle_error.last;
//        else    vd.pit_angle_error.last = vd.pit_angle_error.now;
//        
//        //YAW轴角度
//        if (vd.yaw.vision_angle_error.now >= 50 || vd.yaw.vision_angle_error.now <= -50)
//            vd.yaw.vision_angle_error.now = vd.yaw.vision_angle_error.last;

        /* 陀螺仪相对角速度 */
        vd.yaw.imu_angular_speed.now = 
            *(float*)ubf_pop(imu_yaw_ubf, (uint32_t)(vd.period*1e3f*test_visoin_delay))
            /16.3835f; /* 单位:LSB->(°/s) 陀螺仪内置滤波*/
        
        /* 视觉相对角速度 */
        vd.yaw.vision_angle_error.kal = 
            Kalman1FilterCalc(&kalman_yaw_angle_error, vd.yaw.vision_angle_error.now); /* 云台控制使用 */
        vd.yaw.vision_angular_speed.now = (vd.yaw.vision_angle_error.kal - vd.yaw.vision_angle_error.last) 
            / vd.period;  /* 单位:°/s */
        vd.yaw.vision_angular_speed.kal = Kalman1FilterCalc(&kalman_yaw_aim_speed, vd.yaw.vision_angular_speed.now);
        vd.yaw.vision_angle_error.last = vd.yaw.vision_angle_error.kal;
        
        /* 目标切向绝对角速度（°/s） */
        vd.yaw.object_angular_speed.now = vd.yaw.vision_angular_speed.kal + vd.yaw.imu_angular_speed.now;
        
        /* 目标切向绝对线速度矢量（m/s） */
        vd.distance.kal = Kalman1FilterCalc(&kalman_vision_distance, vd.distance.now);
        vd.yaw.object_tangential_speed.now = vd.yaw.object_angular_speed.now * (2*PI)/360.0f * vd.distance.kal;
        
        /* 目标径向相对速度矢量（m/s） */
        vd.yaw.object_radial_speed.now = (vd.distance.kal - vd.distance.last) / vd.period;
        vd.yaw.object_radial_speed.kal = Kalman1FilterCalc(&kalman_vision_dr, vd.yaw.object_radial_speed.now);
        vd.distance.last = vd.distance.kal;
        
        /* 目标二维速度模值 */
        vd.yaw.object_2D_speed.now = sqrtf(powf(vd.yaw.object_radial_speed.kal, 2) + 
            powf(vd.yaw.object_tangential_speed.now, 2));/* 单位:m/s */
        
        /* 目标二维速度角度 */
        vd.yaw.object_speed_angle.now = 
            atanf(vd.yaw.object_radial_speed.kal
            /ABS(vd.yaw.object_tangential_speed.now))+ PI/2;/* 只要径向速度为正就为钝角（rad） */
        
        /* 目标运动位移模值预测（m） */
        vd.tof.kal = Kalman1FilterCalc(&kalman_bullet_time, vd.tof.now);
        vd.yaw.predict_distance.now = vd.yaw.object_2D_speed.now * (vd.tof.kal + vd.period/2.0f);
        
        /* 余弦定理，几何对边（m） */
        vd.yaw.opposite_line.now = sqrtf(powf(vd.yaw.predict_distance.now, 2) + powf(vd.distance.kal, 2) 
            - 2 * vd.yaw.predict_distance.now * vd.distance.kal * cosf(vd.yaw.object_speed_angle.now));
        vd.yaw.opposite_line.kal = Kalman1FilterCalc(&kalman_predict_distance, vd.yaw.opposite_line.now);
        
        /* 正弦定理，预测角度解算（°） */
        vd.yaw.object_angular_speed.kal = 
            Kalman1FilterCalc(&kalman_object_angular_speed, vd.yaw.object_angular_speed.now);/* 相位匹配 */
        if (ABS(vd.yaw.object_angular_speed.kal) > 5.0f) /* 设置预测启动阈值 */
        {
            vd.yaw.predict_angle.now = SIGN(vd.yaw.object_angular_speed.kal)
                * asinf(vd.yaw.opposite_line.kal / sinf(vd.yaw.object_speed_angle.now) * vd.yaw.predict_distance.now)
                * 360 / (2*PI); /* 角度制 */
        }
        else
        {
            vd.yaw.predict_angle.now = 0;
        }
        /* 预测角度数据处理 */
        if (isnan(vd.yaw.predict_angle.now))
        {
            if(isnan(vd.yaw.predict_angle.last))
                vd.yaw.predict_angle.last = vd.yaw.predict_angle.now = 0;
            else
                vd.yaw.predict_angle.now = vd.yaw.predict_angle.last;
        }
        else
        {
            vd.yaw.predict_angle.last = vd.yaw.predict_angle.now;
        }
        vd.yaw.predict_angle.kal = Kalman1FilterCalc(&kalman_predict_angle, vd.yaw.predict_angle.now);
        
        /* 输出数据 */
        vision.pit_angle_error = vd.pit_angle_error.now; /* PIT角度偏差 */
        vision.yaw_angle_error = vd.yaw.vision_angle_error.kal; /* YAW轴角度偏差 */
        vision.yaw_predict_angle = vd.yaw.predict_angle.kal; /* YAW轴角度偏差预测 */
        
        vision.aiming_flag = 1; /* 标记当前识别到目标 */
        vision.first_lost_flag = 1; /* 标记当前没有丢帧 */
        vision.new_frame_flag = 1; /* 标记当前数据信息为最新 */
    }
    else
    {
        vision.aiming_flag = 0; /* 标记当前未识别到目标 */
        vision.new_frame_flag = 0; /* 标记没有解算出有效新数据 */
        vision.yaw_predict_angle = 0; /* 预测角度置零 */
        
        memset(&vd.yaw.vision_angle_error, 0, sizeof(vision_data_pkg3_t));
        memset(&vd.pit_angle_error, 0, sizeof(vision_data_pkg3_t));
        memset(&vd.distance, 0, sizeof(vision_data_pkg3_t));
        memset(&vd.tof, 0, sizeof(vision_data_pkg3_t));
        
        Kalman1FilterDeinit(&kalman_yaw_angle_error);
        Kalman1FilterDeinit(&kalman_yaw_aim_speed);
        Kalman1FilterDeinit(&kalman_vision_distance);
        Kalman1FilterDeinit(&kalman_vision_dr);
        Kalman1FilterDeinit(&kalman_bullet_time);
        Kalman1FilterDeinit(&kalman_predict_distance);
        Kalman1FilterDeinit(&kalman_object_angular_speed);
    }
}


