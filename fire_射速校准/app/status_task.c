#include "status_task.h"
#include "tim.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "bsp_T_imu.h"
#include "stdbool.h"
#include "remote_msg.h"
#include "modeswitch_task.h"

status_t status;
static void status_deinit(void);
static void status_restore(void);

/**
  * @brief status_task
  * @param
  * @attention
	* @note
  */
void status_task(void const *argu)
{
    for(;;)
    {
        static uint16_t cnt = 0;
        static uint8_t led_status = 0;

        taskENTER_CRITICAL();

        rc.init_status = rc_FSM(status.rc_status);  //����ң�����ĳ�ʼ״̬

        /* ң����ͨ��״̬��� */
        //ң��ͨ�����ڴ�Լ14ms��������ÿ100ms��鲢���һ���жϱ�־
        cnt++;
        if( rc_normal_flag && status.gyro_status[0] && status.gyro_status[1] )  //��ͨ������ʱ(ͨ�Ŵ����б�־��1)
        {
            status.rc_status = 1;  //ϵͳ״̬��־��1����ϵͳģʽ�л������м��
            rc_normal_flag = 0;  //���ң���������жϱ�־

            /* LED ״̬��ʾ */
            if( cnt == 1 )
            {
                /* ������״̬ LED4 */
                if( status.gyro_status[0] && status.gyro_status[1] &&\
                        (imu_data.wz * imu_data.pitch * imu_data.yaw) != 0)
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
            }
            if( cnt == 3 )
            {
                /* ����״̬ LED3 */
                if( status.chassis_status[0] && status.chassis_status[1] && \
                        status.chassis_status[2] && status.chassis_status[3] )
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
            }
            if( cnt == 5 )
            {
                /* ��̨״̬ LED2 */
                if( status.gimbal_status[0] && status.gimbal_status[1] && \
                        status.gimbal_status[2] )
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
            }
            if( cnt == 7 )
            {
                /* �Ӿ����ʿ���״̬ LED1 */
                if( status.power_control )  //status.vision_status
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);

                /* �ı�����״̬ ��ʼ��һ��ѭ������ */
                led_status = !led_status;
                cnt = 0;
            }
        }
        else  //ң������������ʧ��
        {
            status.rc_status = 0;  //ң��״̬��־��0
            if(cnt >= 2)
            {
                led_status = !led_status;
                cnt = 0;

                if( led_status )
                    status_init();  //LEDȫ��Ϩ��
                else
                    status_deinit();  //ȫ������
            }
        }
        /* ��λ */
        status_restore();
        taskEXIT_CRITICAL();
        osDelay(100);
    }
}

void status_init(void)  //���
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_SET);  //LED_D
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_SET);  //LED_C
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);  //LED_B
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);  //LED_A
}

static void status_deinit(void)  //����
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_RESET);  //LED_D
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_RESET);  //LED_C
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);  //LED_B
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);  //LED_A
}

static void status_restore(void)  //�����־λ
{
    status.power_control = 0;
    for(int i = 0; i<2; i++)
        status.gyro_status[i] = 0;
    for(int i = 0; i<4; i++)
        status.chassis_status[i] = 0;
    for(int i = 0; i<3; i++)
        status.gimbal_status[i] = 0;
}
