#include "bsp_FricMotor.h"
#include "tim.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "control_def.h"
#include "status_task.h"

static void FricGunControl(uint16_t pwm, int16_t err_1, int16_t err_2);

Slope_Struct shoot_Fric_pwm;

void FricMotor_init(void)
{
    //900-2000
    //����ʱ�����Ŵ����
    HAL_TIM_PWM_Start(FricMotor_Time_CH1);
    HAL_TIM_PWM_Start(FricMotor_Time_CH2);

    FricMotor_PWM1 = Init_PWM;
    FricMotor_PWM2 = Init_PWM;
    /* ��ʼ��Ħ����б�º��� */
    shoot_Fric_pwm.limit_target = Init_PWM;
    shoot_Fric_pwm.real_target  = Init_PWM;
    shoot_Fric_pwm.change_scale = 0.5;

    /* ��ʼ������ */
    shoot.shoot_speed = 15;
}

/**
  * @func   		void FricGunControl(uint8_t Control)
  * @bref			Ħ������ͣ
  * @param[in]      Control��0Ϊֹͣ��1Ϊ����
  * @retval         void
  * @note			900������ת
  */
static void FricGunControl(uint16_t pwm, int16_t err_1, int16_t err_2)
{
    shoot_Fric_pwm.limit_target = Init_PWM+pwm;

    if( lock_flag )			//��������ܹ����PWM
    {
        Slope_On(&shoot_Fric_pwm); //��Ħ����б������

        FricMotor_PWM1 = shoot_Fric_pwm.real_target + err_1;
        FricMotor_PWM2 = shoot_Fric_pwm.real_target + err_2;
    }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      

/* Ħ����ת�ٵ��� */
//int16_t pwm_1 = 0, pwm_2 = 0;
//uint16_t test_pwm = 590;

/**
  * @name     FricMotor_Control
  * @brief    Ħ���ֿ���
  * @param    None
  * @retval   None
  * @note     ���۵�˫ǹ�ܣ�һ������ģʽ��Ħ���ֶ��򿪣����̻����ǹ��ģʽ��ͬ��������
  */
void FricMotor_Control(void)
{
    uint16_t speed_pwm;
    switch( shoot.firc_mode )
    {
        case FIRC_MODE_STOP:
        {
            FricGunControl(0, 0, 0);		//Ħ����ͣת
            break;
        }
        case FIRC_MODE_RUN:
        {
            speed_pwm = 1100;
            break;
        }
        default:
        {
            break;
        }
    }
    /* Ħ���ֿ��� */
    FricGunControl(speed_pwm, 0, 5);
}
