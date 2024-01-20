#define __GIMBAL_TASK_GLOBALS
#include "gimbal_task.h"
#include "control_def.h"
#include "usart.h"

#include "math_calcu.h"
#include "pid.h"
#include "func_generator.h"
#include "KalmanFilter.h"

#include "bsp_can.h"
#include "bsp_TriggerMotor.h"
#include "remote_msg.h"
#include "bsp_T_imu.h"
#include "bsp_vision.h"

#include "chassis_task.h"
#include "status_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"

#define VISION_PREDICT_NUM 4

extern TaskHandle_t can_msg_send_task_t;

/* �������� */
static void gimbal_vision_pid_calcu(void);
static void gimbal_pid_calcu(void);

// #define __TEST_GIMBAL_pid__
#ifdef __TEST_GIMBAL_pid__
#define PERIOD 1500
/* pit��sqr���� */
FGT_sqr_t test_sqr_pit = 
{
    .Td = GIMBAL_PERIOD,
    .time = 0,
    .max = GIMBAL_PIT_MAX - 200,
    .min = GIMBAL_PIT_MIN + 200,
    .dc = 0,
    
    .Th = PERIOD/2,
    .Tl = PERIOD/2,
    .high = GIMBAL_PIT_MAX - 200,
    .low = GIMBAL_PIT_MIN + 200
};  

/* yaw��sqr���� */
FGT_sqr_t test_sqr_yaw = 
{
    .Td = GIMBAL_PERIOD,
    .time = 0,
    .max = 90 + 25,
    .min = 90 - 25,
    .dc = 0,
    
    .Th = PERIOD/2,
    .Tl = PERIOD/2,
    .high = 90 + 25,
    .low = 90 - 25
};
#endif

void gimbal_param_init(void)
{
    memset(&gimbal, 0, sizeof(gimbal_t));
    /* pit ����PID���� */
//    PID_struct_init(&pid_pit_ecd, POSITION_PID, 8000, 0, 0, 0,
//                    pid_pit_ecd_P, pid_pit_ecd_I, pid_pit_ecd_D);
//    PID_struct_init(&pid_pit_spd, POSITION_PID, 28000,20000, 0, 0,
//                    pid_pit_spd_P, pid_pit_spd_I, pid_pit_spd_D);
    /* YAW ����PID���� */
//    PID_struct_init(&pid_yaw_angle, POSITION_PID, 8000, 0, 0, 0,
//                    pid_yaw_angle_P, pid_yaw_angle_I, pid_yaw_angle_D);
//    PID_struct_init(&pid_yaw_spd, POSITION_PID, 28000, 20000, 0, 0,
//                    pid_yaw_spd_P, pid_yaw_spd_I, pid_yaw_spd_D);

    /* ������̨PID���� */
    PID_struct_init(&pid_pit_ecd, POSITION_PID, 10000, 0, 0, 0,
                    12, 0, 0);
    PID_struct_init(&pid_pit_spd, POSITION_PID, 30000,20000, 0, 0,
                    12, 0.15f, 0);
                    
    PID_struct_init(&pid_yaw_angle, POSITION_PID, 10000, 0, 0, 0,
                    170, 0, 0);
    PID_struct_init(&pid_yaw_spd, POSITION_PID, 30000, 20000, 0, 0,
                    11, 0.15f, 0);
                    
    /* �Ӿ���̨PID���� */
    PID_struct_init(&vision_pid_pit_ecd, POSITION_PID, 10000, 0, 0, 0,
                    12, 0, 0);
    PID_struct_init(&vision_pid_pit_spd, POSITION_PID, 30000,20000, 0, 0,
                    12, 0.15f, 0);
    PID_struct_init(&vision_pid_yaw_angle, POSITION_PID, 10000, 0, 0, 0,
                    170, 0, 0);
    PID_struct_init(&vision_pid_yaw_spd, POSITION_PID, 30000, 20000, 0, 0,
                    11, 0.15f, 0);

    /* ������ YAW ���� PID ���� */
    PID_struct_init(&pid_yaw_mecd, POSITION_PID, 5000, 0, 0, 0,
                    pid_yaw_mecd_P, pid_yaw_mecd_I, pid_yaw_mecd_D);
    PID_struct_init(&pid_yaw_mspd, POSITION_PID, 28000, 20000, 0, 0,
                    pid_yaw_mspd_P,pid_yaw_mspd_I, pid_yaw_mspd_D);

    /* ң�����������趨 */
    scale.ch1 = RC_CH1_SCALE;
    scale.ch2 = RC_CH2_SCALE;
}

/* ================================== TEST PARAM ================================== */
FGT_sin_t test_s =
{
    .Td = 1,
    .time = 0,
    .max = GIMBAL_YAW_CENTER_OFFSET + 800,
    .min = GIMBAL_YAW_CENTER_OFFSET - 800,
    .dc = GIMBAL_YAW_CENTER_OFFSET,
    .T = 800,
    .A = 250,
    .phi = 0,
    .out = 0
};
/* ================================== TEST PARAM ================================== */

extern vision_tx_msg_t vision_tx_msg;
float test_kp_angle_error = 2.0f;
float test_kp_predict = 1.0f;
/**
  * @brief gimbal_task
  */
void gimbal_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();

    for(;;)
    {
        taskENTER_CRITICAL();

        switch (ctrl_mode)
        {
        case PROTECT_MODE:
        {
            gimbal.pid.pit_ecd_ref   = GIMBAL_PIT_CENTER_OFFSET;
            gimbal.pid.yaw_angle_ref = imu_data.yaw;
            gimbal.pid.yaw_mecd_ref  = GIMBAL_YAW_CENTER_OFFSET;
            for( uint8_t i=0; i<2; i++ )	gimbal.current[i] = 0;
        }
        break;
        case REMOTER_MODE:
        {
            gimbal.pid.pit_ecd_ref += rc.ch2 * scale.ch2;
            if (vision.aiming_flag)
            {
                if (vision.new_frame_flag)//���յ��µ�����
                {
                    vision.new_frame_flag = 0;//����0,�ó���־���Ӿ��ж�
                    gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb + vision.yaw_angle_error + vision.yaw_predict_angle;//
                }
            }
            else if (vision.first_lost_flag)
            {
                vision.first_lost_flag = 0;
                gimbal.pid.pit_ecd_ref   = gimbal.pid.pit_ecd_fdb;
                gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
            }
            else
            {
                gimbal.pid.pit_ecd_ref   += rc.ch2 * scale.ch2;
                gimbal.pid.yaw_angle_ref += rc.ch1 * scale.ch1;
                gimbal.pid.yaw_mecd_ref  += rc.ch1 * (-0.008f);
            }
            gimbal_pid_calcu();
//            gimbal_vision_pid_calcu();
            
            
#ifdef __TEST_GIMBAL_pid__
            /* ��̨ PID ����ʱʹ�� */
            FGT_sqr_cal(&test_sqr_pit);dv
            gimbal.pid.pit_ecd_ref   = test_sqr_pit.out;
            FGT_sqr_cal(&test_sqr_yaw);
            gimbal.pid.yaw_angle_ref   = test_sqr_yaw.out;
#endif
        }
        break;
        case KEYBOARD_MODE:
        {
            if( chassis.mode == CHASSIS_MODE_KEYBOARD_SUPPLY )
            {
                gimbal.pid.pit_ecd_ref = GIMBAL_PIT_CENTER_OFFSET;
                gimbal.pid.yaw_angle_ref += rc.mouse.x *  KEYBOARD_SCALE_YAW_SUPPLY;
            }
            else
            {
                gimbal.pid.pit_ecd_ref   += rc.mouse.y *  KEYBOARD_SCALE_PIT;
                gimbal.pid.yaw_angle_ref += rc.mouse.x *  KEYBOARD_SCALE_YAW;
            }
            gimbal_pid_calcu();
        }
        break;
        case VISION_MODE:
        {
//            if( vision.distance )
//            {
//                vision.aim_flag = 1;

//                switch( vision_mode )
//                {
//                case VISION_MODE_AUTO:
//                    vision_autoaiming_calcu();
//                    break;
//                case VISION_MODE_bENERGY:
//                case VISION_MODE_sENERGY:
//                    vision_energy_calcu();
//                    break;
//                case VISION_MODE_ANTIROTATE:
//                    vision_antirotate_calu();
//                    break;
//                case VISION_MODE_SENTRY:
//                    vision_sentry_calcu();
//                    break;
//                default:
//                    break;
//                }
//            }
//            else if( vision.aim_flag == 1 )
//            {
//                gimbal.pid.pit_ecd_ref   = gimbal.pid.pit_ecd_fdb;
//                gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
//                vision.aim_flag = 0;
//            }
//            else
//            {
//                gimbal.pid.pit_ecd_ref   += rc.mouse.y * KEYBOARD_SCALE_PIT;
//                gimbal.pid.yaw_angle_ref += rc.mouse.x * KEYBOARD_SCALE_YAW;
//            }
        }
        default:
            break;
        }
        
//        memcpy(motor_cur.gimbal_cur, gimbal.current, sizeof(gimbal.current));
        osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);
        taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, GIMBAL_PERIOD);
    }
}

static void gimbal_vision_pid_calcu(void)
{
    gimbal.pid.pit_ecd_ref = data_limit(gimbal.pid.pit_ecd_ref, GIMBAL_PIT_MAX, GIMBAL_PIT_MIN);	//�?标值限�?
    gimbal.pid.pit_ecd_fdb = moto_pit.ecd;
    gimbal.pid.pit_ecd_err = circle_error(gimbal.pid.pit_ecd_ref, gimbal.pid.pit_ecd_fdb, 8191);
    pid_calc(&vision_pid_pit_ecd, gimbal.pid.pit_ecd_fdb, gimbal.pid.pit_ecd_fdb + gimbal.pid.pit_ecd_err);

    gimbal.pid.pit_spd_ref = pid_pit_ecd.pos_out;   //PID外环�?标�?
    gimbal.pid.pit_spd_fdb = imu_data.wy;			 //pit角速度反�?�传进PID结构�?
    pid_calc(&vision_pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);
    gimbal.current[1] = pid_pit_spd.pos_out;
    
    if(gimbal.pid.yaw_angle_ref<0)            gimbal.pid.yaw_angle_ref += 360;
    else if(gimbal.pid.yaw_angle_ref>360)     gimbal.pid.yaw_angle_ref -= 360;	//�?标值限�?
    gimbal.pid.yaw_angle_fdb = imu_data.yaw;  //陀螺仪角度反�??
    gimbal.pid.yaw_angle_err = circle_error(gimbal.pid.yaw_angle_ref, gimbal.pid.yaw_angle_fdb, 360);
    pid_calc(&vision_pid_yaw_angle, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_fdb + gimbal.pid.yaw_angle_err);
    gimbal.pid.yaw_spd_ref = pid_yaw_angle.pos_out;
    gimbal.pid.yaw_spd_fdb = imu_data.wz;  //陀螺仪速度反�??
    pid_calc(&vision_pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
    gimbal.current[0] = pid_yaw_spd.pos_out;
}

/* 云台串级PID控制 */
static void gimbal_pid_calcu(void)
{
    /*------------------------pit轴串�?pid计算------------------------*/
    //位置反�?�：编码器位�?
    //速度反�?�：陀螺仪速度
    gimbal.pid.pit_ecd_ref = data_limit(gimbal.pid.pit_ecd_ref, GIMBAL_PIT_MAX, GIMBAL_PIT_MIN);	//�?标值限�?
    gimbal.pid.pit_ecd_fdb = moto_pit.ecd;
    gimbal.pid.pit_ecd_err = circle_error(gimbal.pid.pit_ecd_ref, gimbal.pid.pit_ecd_fdb, 8191);
    pid_calc(&pid_pit_ecd, gimbal.pid.pit_ecd_fdb, gimbal.pid.pit_ecd_fdb + gimbal.pid.pit_ecd_err);

    gimbal.pid.pit_spd_ref = pid_pit_ecd.pos_out;   //PID外环�?标�?
    gimbal.pid.pit_spd_fdb = imu_data.wy;			 //pit角速度反�?�传进PID结构�?
    pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);

    gimbal.current[1] = pid_pit_spd.pos_out;


    /*------------------------yaw轴串�?pid计算------------------------*/
    //位置反�?�：陀螺仪角度
    //速度反�?�：陀螺仪WZ
    if(gimbal.pid.yaw_angle_ref<0)            gimbal.pid.yaw_angle_ref += 360;
    else if(gimbal.pid.yaw_angle_ref>360)     gimbal.pid.yaw_angle_ref -= 360;	//�?标值限�?
    gimbal.pid.yaw_angle_fdb = imu_data.yaw;  //陀螺仪角度反�??
    gimbal.pid.yaw_angle_err = circle_error(gimbal.pid.yaw_angle_ref, gimbal.pid.yaw_angle_fdb, 360);
    pid_calc(&pid_yaw_angle, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_fdb + gimbal.pid.yaw_angle_err);

    gimbal.pid.yaw_spd_ref = pid_yaw_angle.pos_out;
    gimbal.pid.yaw_spd_fdb = imu_data.wz;  //陀螺仪速度反�??
    pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);

    gimbal.current[0] = pid_yaw_spd.pos_out;

    //位置反�?�：编码器位�?
    //速度反�?�：陀螺仪WZ
    //注意：测试发射器时用。使用时，需要注释掉底盘，保证编码值绝对，下方电流来源切换
//    if( gimbal.pid.yaw_mecd_ref >8191 )
//        gimbal.pid.yaw_mecd_ref -= 8191;
//    else if( gimbal.pid.yaw_mecd_ref < 0 )
//        gimbal.pid.yaw_mecd_ref += 8191;
//    gimbal.pid.yaw_mecd_fdb = moto_yaw.ecd;
//    gimbal.pid.yaw_mecd_err = circle_error(gimbal.pid.yaw_mecd_ref,gimbal.pid.yaw_mecd_fdb,8191);
//    pid_calc(&pid_yaw_mecd, gimbal.pid.yaw_mecd_fdb, gimbal.pid.yaw_mecd_fdb + gimbal.pid.yaw_mecd_err);

//    gimbal.pid.yaw_mspd_ref = pid_yaw_mecd.pos_out;
//    gimbal.pid.yaw_mspd_fdb = imu_data.wz;  //陀螺仪速度反�??
//    pid_calc(&pid_yaw_mspd, gimbal.pid.yaw_mspd_fdb, gimbal.pid.yaw_mspd_ref);
//    gimbal.current[0] = pid_yaw_mspd.pos_out;
}
