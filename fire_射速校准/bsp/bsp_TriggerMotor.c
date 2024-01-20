#include "bsp_TriggerMotor.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "gimbal_task.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "control_def.h"
#include "shoot_task.h"
#include "math_calcu.h"
#include "string.h"
#include "bsp_judge.h"
#include "remote_msg.h"
#include "bsp_vision.h"
#include "bsp_T_imu.h"
#include "status_task.h"

extern shoot_t shoot;
extern TaskHandle_t can_msg_send_task_t;

static void TriggerMotor_pidcal(void);

void TriggerMotor_init(void)
{
    PID_struct_init(&pid_trigger_ecd, POSITION_PID, 4320, 0, 0, 0, 
                    PID_TRIGGER_ECD_P, PID_TRIGGER_ECD_I, PID_TRIGGER_ECD_D);
    PID_struct_init(&pid_trigger_spd, POSITION_PID, 10000, 8000, 0, 0,
                    PID_TRIGGER_SPD_P, PID_TRIGGER_SPD_I, PID_TRIGGER_SPD_D);
}

static void TriggerMotor_pidcal(void)
{
    shoot.barrel.pid.trigger_ecd_fdb = motor_trigger.total_ecd;
    pid_calc(&pid_trigger_ecd, shoot.barrel.pid.trigger_ecd_fdb, shoot.barrel.pid.trigger_ecd_ref);
    shoot.barrel.pid.trigger_spd_ref = pid_trigger_ecd.pos_out;  //λ�û�

    shoot.barrel.pid.trigger_spd_fdb = motor_trigger.speed_rpm;
    pid_calc(&pid_trigger_spd, shoot.barrel.pid.trigger_spd_fdb, shoot.barrel.pid.trigger_spd_ref);
    shoot.barrel.current = pid_trigger_spd.pos_out;  //�ٶȻ�

    motor_cur.trigger_cur = shoot.barrel.current;
}

void TriggerMotor_control(void)
{
    /* �ֲ�ȫ�ֱ��� */
    static uint16_t frequency_cnt = 0;	//��Ƶ����
    static uint8_t shoot_enable = 1;    //���������ص���ʹ�ܱ�־
    static uint32_t shoot_time, shoot_last_time;//�����������

    switch( shoot.stir_mode )
    {
        case STIR_MODE_PROTECT:  //���̱���ģʽ�����ֹ��ԣ�����
        {
            frequency_cnt = 0;  //��ʱ������0�������ǰһ������ֹ
            shoot.barrel.shoot_period = 0;
            
            shoot.barrel.pid.trigger_ecd_ref = motor_trigger.total_ecd;
            pid_trigger_spd.iout  = 0;
            motor_cur.trigger_cur = 0;
        }
        break;
        case STIR_MODE_STOP:  //����ֹͣģʽ�����־�ֹ������
        {
            frequency_cnt = 0;  //��ʱ������0�������ǰһ������ֹ
            shoot.barrel.shoot_period = 0;
            TriggerMotor_pidcal();
        }
        break;
        case STIR_MODE_SINGLE:  //���̵���ģʽ��������ǹ����ֻ��Ӧһ��
        case STIR_MODE_SERIES:  //��������ģʽ��������ǹ����������Ӧ
        {
            /* ����ң�����л���������ģʽ */
            if( rc_FSM_check(RC_LEFT_LU) )  //ң�����ϵ�ǰ���󲦸�������
            {
                shoot.stir_mode = STIR_MODE_SINGLE;  //����
            }
            else if( rc_FSM_check(RC_LEFT_RU) )  //ң�����ϵ�ǰ���󲦸�������
            {
                shoot.stir_mode = STIR_MODE_SERIES;  //����
            }
            frequency_cnt++;
            shoot.barrel.pid.trigger_ecd_error = shoot.barrel.pid.trigger_ecd_ref - shoot.barrel.pid.trigger_ecd_fdb;
            if( STIR_MODE_SINGLE == shoot.stir_mode )  //���̵���ģʽ
            {
                if( ( rc.mouse.l == 0 && ctrl_mode == VISION_MODE )|| \
                    ( rc.ch5 == 0 && ctrl_mode == REMOTER_MODE ) )
                    shoot_enable = 1;
                if( shoot_enable \
                    && (rc.mouse.l || rc.ch5 == 660) \
                    && ABS(shoot.barrel.pid.trigger_ecd_error) < 1.0f * TRIGGER_MOTOR_ECD \
                     )  //��������&& shoot.barrel.heat_remain >= MIN_HEAT
                {
                    shoot_enable = 0;
                    shoot.barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
                    shoot.barrel.heat += 10;
                }
            }
            else if( STIR_MODE_SERIES == shoot.stir_mode )
            {
                if( ( rc.mouse.l || ctrl_mode == REMOTER_MODE ) \
                    && frequency_cnt * SHOOT_PERIOD >= shoot.trigger_period \
                    && ABS(shoot.barrel.pid.trigger_ecd_error) < 1.0f * TRIGGER_MOTOR_ECD \
                    )//һ�����ڴ�һ��  ��Ƶ����  && shoot.barrel.heat_remain >= MIN_HEAT
                {
                    frequency_cnt = 0;
                    /* ��һ���ӵ� */
                    shoot.barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
                    shoot.barrel.heat += 10;
                    /* ��ȡ������� */
                    shoot_time = osKernelSysTick();
                    shoot.barrel.shoot_period = shoot_time - shoot_last_time;
                    shoot_last_time = shoot_time;
                }
            }
            TriggerMotor_pidcal();
        }
        break;
        default: break;
    }
    osSignalSet(can_msg_send_task_t, SHOOT_MOTOR_MSG_SEND);
}

