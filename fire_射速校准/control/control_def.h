#ifndef _CONTROL_DEF_H_
#define _CONTROL_DEF_H_

#include "stdint.h"

/* ������� */
//С����1��   3
//С����2��   4
//С����3��   5
//���ֵ�ǹ    6
#define Robot_Number 5

/*----------------------------- the whole system ----------------------------- */
// task period
#define GIMBAL_PERIOD	  1
#define CHASSIS_PERIOD    2
#define SHOOT_PERIOD      2
#define USART_SEND_PERIOD 2
#define MODESWITCH_PERIOD 6
#define STATUS_PERIOD     500

// gimbal test pid param
#define pid_yaw_mecd_P 8.0f
#define pid_yaw_mecd_I 0.0f
#define pid_yaw_mecd_D 0.0f

#define pid_yaw_mspd_P 10.0f
#define pid_yaw_mspd_I 0.1f
#define pid_yaw_mspd_D 0.0f

/*----------------------------- manipulator preference ----------------------------- */
/* special function key and key status definition */
// chassis control key (status) define
#define KEY_CHASSIS_FIGHT       KB_G  //�����ֲ���
#define FLAG_CHASSIS_FIGHT      kb_status[KEY_CHASSIS_FIGHT]

#define KEY_CHASSIS_ROTATE      KB_F
#define FLAG_CHASSIS_ROTATE     kb_status[KEY_CHASSIS_ROTATE]

// shoot control key (status) define
#define KEY_SHOOT_FRIC          KB_V
#define FLAG_SHOOT_FRIC         kb_status[KEY_SHOOT_FRIC]

#define KEY_SHOOT_HOUSE         KB_B
#define FLAG_SHOOT_HOUSE        kb_status[KEY_SHOOT_HOUSE]

// vision control key status define
#define KEY_VISION_sENERGY      KB_Q
#define FLAG_VISION_sENERGY     kb_status[KEY_VISION_sENERGY]

#define KEY_VISION_bENERGY      KB_E
#define FLAG_VISION_bENERGY     kb_status[KEY_VISION_bENERGY]

#define KEY_VISION_ENERGY_DIR   KB_Z

#define KEY_VISION_ANTIROTATE   KB_R
#define FLAG_VISION_ANTIROTATE  kb_status[KEY_VISION_ANTIROTATE]

#define KEY_VISION_SENTRY       KB_X
#define FLAG_VISION_SENTRY      kb_status[KEY_VISION_SENTRY]

/* ========================================== Infantry s1 ========================================== */

#if ( Robot_Number == 3 )

/* ========================================== Infantry s2 ========================================== */

#elif ( Robot_Number == 4 )

/*-----------------------------shoot-----------------------------*/
#define SPEED_PWM_14        600     //14m/s����  ��������Ϊ15m/s  ����ʱ��13.8-14.6 ����ʱ��14.3-14.6
#define SPEED_PWM_17        650     //16m/s����  ��������Ϊ18m/s  ����ʱ��15.8-16.7 ����ʱ��16.9-17.2
#define SPEED_PWM_28        920     //28m/s����  ��������Ϊ30m/s  ����ʱ��25.0-28.2 ����ʱ��28.0-28.5

#define LOW_SPEED           SPEED_PWM_14   //У׼ʱ�ĳ�1100 (�ճ���ʱ������) ���Բ���ʱ���������ó�200
#define MID_SPEED	        SPEED_PWM_17	
#define HIGH_SPEED	        SPEED_PWM_28

/* ������PWM�ڼ���ӦTIM */
#define PWM_SEND_TIM12   (&htim12)
#define PWM_SEND_CH1    TIM12->CCR1 //PB14
#define PWM_SEND_CH2    TIM12->CCR2 //PB15

#define PWM_SEND_TIM3   (&htim3)
#define PWM_SEND_CH3    TIM3->CCR3  //PB0
#define PWM_SEND_CH4    TIM3->CCR4  //PB1

/* ���ո� */
#define COVER_PWM_OPEN      600
#define COVER_PWM_CLOSE     2250
#define Magazine_PWM        PWM_SEND_CH3
#define Magazine_Time_CH    PWM_SEND_TIM3, TIM_CHANNEL_3

/* ǹ��PWM IO�� */
#define FricMotor_PWM1      PWM_SEND_CH1  //��Ħ����
#define FricMotor_PWM2      PWM_SEND_CH4  //��Ħ����
#define FricMotor_Time_CH1  PWM_SEND_TIM12, TIM_CHANNEL_1
#define FricMotor_Time_CH2  PWM_SEND_TIM3 , TIM_CHANNEL_4

/* ���� PID ���� */
#define PID_TRIGGER_ECD_P 0.3f
#define PID_TRIGGER_ECD_I 0.0f
#define PID_TRIGGER_ECD_D 0.3f

#define PID_TRIGGER_SPD_P 5.0f
#define PID_TRIGGER_SPD_I 0.008f
#define PID_TRIGGER_SPD_D 0.0f

/* ����Ƶ�� */
#define TRIGGER_PERIOD    90  //������ڣ�ms��

/*-----------------------------chassis---------------------------*/
#define RC_CH4_SCALE    12     
#define RC_CH3_SCALE    12

#define SPEED_0W        1000.0f  //����ϵͳ���̹��ʵ�Ϊ������ʱ
#define SPEED_SUPPLY    600.0f
#define SPEED_45W  		3400.0f
#define SPEED_50W		4800.0f
#define SPEED_55W		4900.0f
#define SPEED_60W		5000.0f
#define SPEED_80W		6000.0f
#define SPEED_100W      6200.0f
#define SPEED_120W      7600.0f
#define SPEED_SUPERCAP  8000.0f

#define SUPERCAP_MAX_VOLAGE	23.7f		//������������ѹ
/*-----------------------------gimbal----------------------------*/
#define Reduction_ratio			    1.0f	//pit����ٱ�
#define RC_CH2_SCALE                0.003f
#define RC_CH1_SCALE                ( -0.0005f )

#define KEYBOARD_SCALE_PIT          ( -0.06f )
#define KEYBOARD_SCALE_YAW	        ( -0.001f )
#define KEYBOARD_SCALE_YAW_SUPPLY   ( -0.0006f )

#define GIMBAL_PIT_CENTER_OFFSET    2760
#define GIMBAL_PIT_MAX              3555
#define GIMBAL_PIT_MIN              2460

#define GIMBAL_YAW_CENTER_OFFSET    2700
#define GIMBAL_YAW_BETWEEN_ECD      ( 8191 / 8 )
#define FIGHT_OFFSET_ERR            ( -1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI )

/* YAW��PIDϵ�� */
#define pid_yaw_angle_P 180.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_spd_P 20.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

/* PIT��PIDϵ�� */
#define pid_pit_ecd_P 9.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 8.0f
#define pid_pit_spd_I 0.2f
#define pid_pit_spd_D 0.0f

/* ========================================== Infantry s3 ========================================== */

#elif ( Robot_Number == 5 )

/*-----------------------------shoot-----------------------------*/
#define SPEED_PWM_14        515     //14m/s����  ��������Ϊ15m/s  ����ʱ��13.8-14.6 ����ʱ��14.3-14.6
#define SPEED_PWM_17        570     //16m/s����  ��������Ϊ18m/s  ����ʱ��15.8-16.7 ����ʱ��16.9-17.2
#define SPEED_PWM_28        900     //28m/s����  ��������Ϊ30m/s  ����ʱ��25.0-28.2 ����ʱ��28.0-28.5

#define LOW_SPEED           SPEED_PWM_14   //У׼ʱ�ĳ�1100 (�ճ���ʱ������) ���Բ���ʱ���������ó�200
#define MID_SPEED	        SPEED_PWM_17	
#define HIGH_SPEED	        SPEED_PWM_28

/* ������PWM�ڼ���ӦTIM */
#define PWM_SEND_TIM12   (&htim12)
#define PWM_SEND_CH1    TIM12->CCR1 //PB14
#define PWM_SEND_CH2    TIM12->CCR2 //PB15

#define PWM_SEND_TIM3   (&htim3)
#define PWM_SEND_CH3    TIM3->CCR3  //PB0
#define PWM_SEND_CH4    TIM3->CCR4  //PB1

/* ���ո� */
#define COVER_PWM_OPEN      500
#define COVER_PWM_CLOSE     2400
#define Magazine_PWM        PWM_SEND_CH3
#define Magazine_Time_CH    PWM_SEND_TIM3, TIM_CHANNEL_3

/* ǹ��PWM IO�� */
#define FricMotor_PWM1      PWM_SEND_CH1  //��Ħ����
#define FricMotor_PWM2      PWM_SEND_CH4  //��Ħ����
#define FricMotor_Time_CH1  PWM_SEND_TIM12, TIM_CHANNEL_1
#define FricMotor_Time_CH2  PWM_SEND_TIM3 , TIM_CHANNEL_4

/* ���� PID ���� */
#define PID_TRIGGER_ECD_P 0.3f
#define PID_TRIGGER_ECD_I 0.0f
#define PID_TRIGGER_ECD_D 0.3f

#define PID_TRIGGER_SPD_P 5.0f
#define PID_TRIGGER_SPD_I 0.008f
#define PID_TRIGGER_SPD_D 0.0f

/* ����Ƶ�� */
#define TRIGGER_PERIOD    90  //������ڣ�ms��

/*-----------------------------chassis---------------------------*/
#define RC_CH4_SCALE    12     
#define RC_CH3_SCALE    12

#define SPEED_0W        1000.0f  //����ϵͳ���̹��ʵ�Ϊ������ʱ
#define SPEED_SUPPLY    2000.0f
#define SPEED_45W  		3400.0f
#define SPEED_50W		4800.0f
#define SPEED_55W		4900.0f
#define SPEED_60W		5000.0f
#define SPEED_80W		6000.0f
#define SPEED_100W      6200.0f
#define SPEED_120W      7600.0f
#define SPEED_SUPERCAP  8000.0f

#define SUPERCAP_MAX_VOLAGE	23.7f		//������������ѹ
/*-----------------------------gimbal----------------------------*/
#define Reduction_ratio			    1.0f	//pit����ٱ�
#define RC_CH2_SCALE                0.008f
#define RC_CH1_SCALE                ( -0.0005f )

#define KEYBOARD_SCALE_PIT          ( -0.06f )
#define KEYBOARD_SCALE_YAW	        ( -0.001f )
#define KEYBOARD_SCALE_YAW_SUPPLY   ( -0.0006f )

#define GIMBAL_PIT_CENTER_OFFSET    2760
#define GIMBAL_PIT_MAX              3555
#define GIMBAL_PIT_MIN              2460

#define GIMBAL_YAW_CENTER_OFFSET    6848
#define GIMBAL_YAW_BETWEEN_ECD      ( 8191 / 8 )
#define FIGHT_OFFSET_ERR            ( -1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI )

/* YAW��PIDϵ�� */
#define pid_yaw_angle_P 180.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_spd_P 20.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

/* PIT��PIDϵ�� */
#define pid_pit_ecd_P 10.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 10.0f
#define pid_pit_spd_I 0.2f
#define pid_pit_spd_D 0.0f

/* pit ����PID���� */
//    PID_struct_init(&pid_pit_ecd, POSITION_PID, 8000, 0, 0, 0,
//                    13, 0, 0);
//    PID_struct_init(&pid_pit_spd, POSITION_PID, 28000,20000, 0, 0,
//                    15, 0.1f, 0);

#elif ( Robot_Number == 6 )

/*-----------------------------shoot-----------------------------*/
#define SPEED_PWM_14        510     //14m/s����  ��������Ϊ15m/s  ����ʱ��13.8-14.6 ����ʱ��14.3-14.6
#define SPEED_PWM_17        570     //16m/s����  ��������Ϊ18m/s  ����ʱ��15.8-16.7 ����ʱ��16.9-17.2
#define SPEED_PWM_28        900     //28m/s����  ��������Ϊ30m/s  ����ʱ��25.0-28.2 ����ʱ��28.0-28.5

#define LOW_SPEED           SPEED_PWM_14   //У׼ʱ�ĳ�1100 (�ճ���ʱ������) ���Բ���ʱ���������ó�200
#define MID_SPEED	        SPEED_PWM_17	
#define HIGH_SPEED	        SPEED_PWM_28

/* ������PWM�ڼ���ӦTIM */
#define PWM_SEND_TIM12   (&htim12)
#define PWM_SEND_CH1    TIM12->CCR1 //PB14
#define PWM_SEND_CH2    TIM12->CCR2 //PB15

#define PWM_SEND_TIM3   (&htim3)
#define PWM_SEND_CH3    TIM3->CCR3  //PB0
#define PWM_SEND_CH4    TIM3->CCR4  //PB1

/* ���ո� */
#define COVER_PWM_OPEN      500
#define COVER_PWM_CLOSE     2400
#define Magazine_PWM        PWM_SEND_CH3
#define Magazine_Time_CH    PWM_SEND_TIM3, TIM_CHANNEL_3

/* ǹ��PWM IO�� */
#define FricMotor_PWM1      PWM_SEND_CH1  //��Ħ����
#define FricMotor_PWM2      PWM_SEND_CH4  //��Ħ����
#define FricMotor_Time_CH1  PWM_SEND_TIM12, TIM_CHANNEL_1
#define FricMotor_Time_CH2  PWM_SEND_TIM3 , TIM_CHANNEL_4

/* ���� PID ���� */
#define PID_TRIGGER_ECD_P 0.3f
#define PID_TRIGGER_ECD_I 0.0f
#define PID_TRIGGER_ECD_D 0.0f

#define PID_TRIGGER_SPD_P 5.0f
#define PID_TRIGGER_SPD_I 0.1f
#define PID_TRIGGER_SPD_D 0.0f

/* ����Ƶ�� */
#define TRIGGER_PERIOD    86  //������ڣ�ms��,��С����50ms

/*-----------------------------chassis---------------------------*/
#define RC_CH4_SCALE    12     
#define RC_CH3_SCALE    12

#define SPEED_0W        1000.0f  //����ϵͳ���̹��ʵ�Ϊ������ʱ
#define SPEED_SUPPLY    2000.0f
#define SPEED_45W  		3400.0f
#define SPEED_50W		4800.0f
#define SPEED_55W		4900.0f
#define SPEED_60W		5000.0f
#define SPEED_80W		6000.0f
#define SPEED_100W      6200.0f
#define SPEED_120W      7600.0f
#define SPEED_SUPERCAP  8000.0f

#define SUPERCAP_MAX_VOLAGE	23.7f		//������������ѹ
/*-----------------------------gimbal----------------------------*/
#define Reduction_ratio			    1.0f	//pit����ٱ�
#define RC_CH2_SCALE                0.008f
#define RC_CH1_SCALE                ( -0.0005f )

#define KEYBOARD_SCALE_PIT          ( -0.06f )
#define KEYBOARD_SCALE_YAW	        ( -0.001f )
#define KEYBOARD_SCALE_YAW_SUPPLY   ( -0.0006f )

#define GIMBAL_PIT_CENTER_OFFSET    2760
#define GIMBAL_PIT_MAX              3555
#define GIMBAL_PIT_MIN              2460

#define GIMBAL_YAW_CENTER_OFFSET    6848
#define GIMBAL_YAW_BETWEEN_ECD      ( 8191 / 8 )
#define FIGHT_OFFSET_ERR            ( -1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI )

/* YAW��PIDϵ�� */
#define pid_yaw_angle_P 180.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_spd_P 20.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

/* PIT��PIDϵ�� */
#define pid_pit_ecd_P 10.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 4.0f
#define pid_pit_spd_I 0.05f
#define pid_pit_spd_D 0.0f

#endif

#endif
