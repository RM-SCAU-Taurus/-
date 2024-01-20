#define __SHOOT_TASK_GLOBALS

#include "shoot_task.h"
#include "control_def.h"
#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"
#include "bsp_CoverServo.h"
#include "bsp_FricMotor.h"
#include "bsp_TriggerMotor.h"
#include "bsp_judge.h"
#include "bsp_vision.h"
#include "remote_msg.h"
#include "comm_task.h"
#include "modeswitch_task.h"

static void ShootParam_Update(void);
static void shoot_mode_sw(void);
static void house_init(void);
static void house_control(void);

void shoot_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        shoot_mode_sw();  //���ģʽ�л�
        FricMotor_Control();	//Ħ���ֵ������
        TriggerMotor_control();	//�����������
        house_control();        //���ոǿ���

        osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
    }
}

void shoot_init(void)
{
    /* �������ײ��ʼ�� */
    FricMotor_init();   //Ħ���ֳ�ʼ��
    TriggerMotor_init();//���̳�ʼ��
    house_init();       //���ճ�ʼ��
    /* ������ģʽ��ʼ�� */
    shoot.firc_mode     = FIRC_MODE_STOP;
    shoot.stir_mode     = STIR_MODE_PROTECT;
    shoot.house_mode    = HOUSE_MODE_PROTECT;  //�ϵ籣��ģʽ�����ո�����
    /* ǹ�ܲ�����ʼ�� */
    shoot.trigger_period = TRIGGER_PERIOD;
    shoot.barrel.cooling_rate   = 10;
    shoot.barrel.heat_max       = 50;
    shoot.shoot_speed           = 15;
}

static void shoot_mode_sw(void)
{
    /* ϵͳ��ʷ״̬�� */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;

    /* ���²���ϵͳ���� */
    ShootParam_Update();

    /* �Ӿ����ٵ�λ���� */
    if( shoot.shoot_speed == 15 )			shoot.shoot_speed_vision = 1;
    else if( shoot.shoot_speed == 18 )		shoot.shoot_speed_vision = 2;
    else if( shoot.shoot_speed == 30 )		shoot.shoot_speed_vision = 3;

    /* ģʽ�л� */
    switch( ctrl_mode )
    {
    case PROTECT_MODE:
    {
        LASER_DOWN;
        shoot.firc_mode = FIRC_MODE_STOP;
        shoot.stir_mode = STIR_MODE_PROTECT;
        shoot.house_mode= HOUSE_MODE_PROTECT;
    }
    break;
    case REMOTER_MODE:
    {
        /* Ħ���ֺͲ���ģʽ�л� */
        switch( rc.sw2 )
        {
        case RC_UP:
        {
            LASER_UP;
//            LASER_DOWN;
            shoot.firc_mode = FIRC_MODE_STOP;
            shoot.stir_mode = STIR_MODE_STOP;
        }
        break;
        case RC_MI:
        {
            LASER_UP;
            shoot.firc_mode = FIRC_MODE_RUN;  //����Ħ����
            shoot.stir_mode = STIR_MODE_STOP;
        }
        break;
        case RC_DN:
        {
            LASER_UP;
            shoot.firc_mode = FIRC_MODE_RUN;  //���ֿ���Ħ����
            if( shoot.barrel.heat_max == 120 )
                shoot.stir_mode = STIR_MODE_SERIES;  //�����Զ��������������ƣ�
            else if( shoot.barrel.heat_max == 180 )
                shoot.stir_mode = STIR_MODE_SINGLE;  //�����ֶ�����
            else
                shoot.stir_mode = STIR_MODE_SERIES;  //�����Զ�����
        }
        break;
        default:
            break;
        }
        /* ���ո�ģʽ�л� */
        static uint8_t house_switch_enable = 1;
        if( last_ctrl_mode != REMOTER_MODE )
            shoot.house_mode = HOUSE_MODE_CLOSE;
        if( rc.ch5 == 0 )   house_switch_enable = 1;
        if( house_switch_enable && rc.ch5 == -660 )  //�л����տ���״̬��־λ
        {
            house_switch_enable = 0;
            shoot.house_mode = (shoot_house_mode_e) (!(uint8_t)shoot.house_mode);  //���ص��ո�
        }
    }
    break;
    case VISION_MODE:
    case KEYBOARD_MODE:
    {
        /* Ħ����ģʽ�л� */
        if( Game_Robot_Status.mains_power_shooter_output )  //��������õ�����
        {
            if( key_scan_clear(KEY_SHOOT_FRIC) )
                shoot.firc_mode = (shoot_firc_mode_e)(!(uint8_t)shoot.firc_mode);  //����Ħ����
        }
        else
        {
            shoot.firc_mode = FIRC_MODE_STOP;  //Ħ���ֶϵ磬�������������Ħ����
        }
        /* ����ģʽ�л� */
        if( shoot.firc_mode == FIRC_MODE_RUN )  //��Ħ���ֺ�
        {
            LASER_UP;  //�򿪼���
            if( vision_mode == VISION_MODE_bENERGY || vision_mode == VISION_MODE_sENERGY )  //�������أ�����ģʽ
            {
                shoot.stir_mode = STIR_MODE_SINGLE;
            }
            else  //����ģʽ�£�����ģʽ
            {
                shoot.stir_mode = STIR_MODE_SERIES;
            }
        }
        else
        {
            LASER_DOWN;
            shoot.stir_mode = STIR_MODE_STOP;  //Ħ����û������������
        }
        /* ���ո�ģʽ�л� */
        keyboard_scan(KEY_SHOOT_HOUSE);
        if( last_ctrl_mode != KEYBOARD_MODE )  //�״ν������ģʽ���رյ��ո�
        {
            shoot.house_mode = HOUSE_MODE_CLOSE;
            key_status_clear(KEY_SHOOT_HOUSE);
        }
        if( FLAG_SHOOT_HOUSE == KEY_RUN )
            shoot.house_mode = HOUSE_MODE_OPEN;  //���������θ����������򿪵��ո�
        else if( FLAG_SHOOT_HOUSE == KEY_END )
            shoot.house_mode = HOUSE_MODE_CLOSE;  //����ż���θ����������رյ��ո�
    }
    break;
    default:
        break;
    }
    /* ��ʷ״̬���� */
    last_ctrl_mode = ctrl_mode;
}

/* ����������ϵͳ���ݸ��� */
static void ShootParam_Update(void)
{
    /* ���²���ϵͳ���� */
    if( Game_Robot_Status.shooter_id1_17mm_speed_limit != 0 )
    {
        shoot.shoot_speed = Game_Robot_Status.shooter_id1_17mm_speed_limit;  //��������
        shoot.barrel.heat_max = Game_Robot_Status.shooter_id1_17mm_cooling_limit;  //ǹ����������
        shoot.barrel.cooling_rate = Game_Robot_Status.shooter_id1_17mm_cooling_rate;  //ǹ����ȴ����
    }
    /* ���� ģ�����ϵͳ ���� */
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f;  //��ǰǹ�ܣ����ۣ�����
    if( shoot.barrel.heat < 0 )	  shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat;  //��ǰǹ�ܣ����ۣ�ʣ������
}

static void house_init(void)
{
    //HAL_TIM_PWM_Start(Magazine_Time_CH);  //��ʼϵͳΪ����ģʽ���������PWM������
    Magazine_PWM = COVER_PWM_CLOSE;
}

//uint16_t test_pwm_open = 600;
static void house_control(void)
{
    switch( shoot.house_mode )
    {
    case HOUSE_MODE_OPEN:  //�򿪵��ո�
    {
        HAL_TIM_PWM_Start(Magazine_Time_CH);
        Magazine_PWM = COVER_PWM_OPEN;//COVER_PWM_OPEN
    }
    break;
    case HOUSE_MODE_CLOSE:  //�رյ��ո�
    {
        HAL_TIM_PWM_Start(Magazine_Time_CH);
        Magazine_PWM = COVER_PWM_CLOSE;
    }
    break;
    case HOUSE_MODE_PROTECT:  //���ո�����
    {
        HAL_TIM_PWM_Stop(Magazine_Time_CH);
        Magazine_PWM = COVER_PWM_CLOSE;
    }
    default:
        break;
    }
}
