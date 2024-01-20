#ifndef __KALMAN_FILTERS_H__
#define __KALMAN_FILTERS_H__

#ifdef  __KALMAN_FILTERS_H_GLOBALS
#define __KALMAN_FILTERS_EXT
#else
#define __KALMAN_FILTERS_EXT extern
#endif

#include "stdint.h"
#include "arm_math.h"

#define mat         arm_matrix_instance_f32
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

//一阶卡尔曼滤波结构体定义,都是标量
typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
    float B;      //输入项参数
    float Q;      //状态方程噪声
    float R;      //观测方程噪声
    float H;
} Kalman1_param_t;

//二阶卡尔曼五条重要参数中间变量定义，用于计算，创建函数的第一个输入
typedef struct
{
    float raw_value;
    float filtered_value[2];     //卡尔曼滤波的返回值，0是位置1是速度
    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman2_filter_t;

//二阶卡尔曼数组初始化，创建函数的第二个输入
typedef struct
{
    float raw_value;
    float filtered_value[2];        //卡尔曼滤波的返回值，0是位置1是速度
    float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];
    float A_data[4];
    float H_data[4];
    float Q_data[4];
    float R_data[4];
} Kalman2_param_t;

typedef struct  //视觉目标速度测量
{
    int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
    int freq;
    int last_time;//上次受到目标角度的时间
    float last_position;//上个目标角度
    float speed;//速度
    float last_speed;//上次速度
    float processed_speed;//速度计算结果
} speed_calc_data_t;

/* 卡尔曼滤波器初始化 */
void Kalman_init(void);

/* 卡尔曼滤波器计算与创建函数 */
void Kalman1FilterCreate(Kalman1_param_t *p,float Q,float R);
void Kalman1FilterDeinit(Kalman1_param_t *p);
float Kalman1FilterCalc(Kalman1_param_t* p,float dat);

void Kalman2FilterCreate(kalman2_filter_t *F,  Kalman2_param_t *I);
float *Kalman2FilterCalc(kalman2_filter_t *F, float signal1, float signal2);

float Kalman2Filter_calc2(kalman2_filter_t *F, float signal1, float signal2);
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);

/* 视觉自瞄卡尔曼滤波器 */
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_imu_pit;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_yaw_angle_error;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_yaw_aim_speed;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_yaw_abs_speed;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_yaw_imu_speed;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_vision_distance;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_vision_dr;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_bullet_time;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_predict_angle;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_2D_spd_arc;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_predict_distance;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_object_angular_speed;//符号相位匹配

__KALMAN_FILTERS_EXT Kalman1_param_t kalman_yaw_energy;
__KALMAN_FILTERS_EXT Kalman1_param_t kalman_pit_energy;

/* 二阶卡尔曼测试 */
__KALMAN_FILTERS_EXT kalman2_filter_t kalman2_pit_filter;
__KALMAN_FILTERS_EXT kalman2_filter_t kalman2_yaw_filter;
__KALMAN_FILTERS_EXT float *pit_kf_result;
__KALMAN_FILTERS_EXT float *yaw_kf_result;

__KALMAN_FILTERS_EXT kalman2_filter_t kalman2_test1_filter;
__KALMAN_FILTERS_EXT float *test1_kf_result;
__KALMAN_FILTERS_EXT kalman2_filter_t kalman2_test2_filter;
__KALMAN_FILTERS_EXT float *test2_kf_result;

#endif

