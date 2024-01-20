#include "comm_task.h"
#include "bsp_can.h"
#include "bsp_vision.h"
#include "cmsis_os.h"
#include "modeswitch_task.h"
#include "bsp_can.h"
#include "bsp_T_imu.h"
#include "control_def.h"

#include "bsp_usart.h"
#include "remote_msg.h"
#include "bsp_judge.h"
#include "shoot_task.h"

#include "us_tim.h"

motor_current_t motor_cur;
vision_tx_msg_t vision_tx_msg = {0};

/**
  * @brief can_msg_send_task
  * @param
  * @attention
  * @note
  */
void can_msg_send_task(void const *argu)
{
    osEvent event;
    for(;;)
    {
        event = osSignalWait(GIMBAL_MOTOR_MSG_SEND  | \
                             CHASSIS_MOTOR_MSG_SEND | \
                             SHOOT_MOTOR_MSG_SEND, osWaitForever);
        if( event.status == osEventSignal )
        {
            if( ctrl_mode==PROTECT_MODE || !lock_flag )
            {
                for(int i=0; i<4; i++)		motor_cur.chassis_cur[i]= 0;
                for(int i=0; i<2; i++)		motor_cur.gimbal_cur[i] = 0;
                motor_cur.trigger_cur = 0;
                can1_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
                can2_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
                can1_send_message(CHASSIS_CAN_TX_ID,0, 0, 0, 0);
            }
            else if( lock_flag )  //�����������ݲŸ�����
            {
                if( event.value.signals & GIMBAL_MOTOR_MSG_SEND )
                {
                    /* С���� */
                    can1_send_message(GIMBAL_CAN_TX_ID, motor_cur.gimbal_cur[0], 0, 0, 0);
                    can2_send_message(GIMBAL_CAN_TX_ID, 0, motor_cur.gimbal_cur[1], motor_cur.trigger_cur, 0);
                }
                if( event.value.signals & CHASSIS_MOTOR_MSG_SEND )
                {
                    can1_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur[0], motor_cur.chassis_cur[1], motor_cur.chassis_cur[2], motor_cur.chassis_cur[3]);
                }
                can1_send_supercap();
            }
        }
    }
}

ust_t ust_vision_send;
/**
  * @brief usart_msg_send_task
  */
#define VISION_TX_BUFF_LEN 20
void usart_msg_send_task(void const *argu)
{
    uint8_t vision_tx_buf[VISION_TX_BUFF_LEN];
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        taskENTER_CRITICAL();
//        ust_interval_test_start(&ust_vision_send);
        /* ֡ͷ */
        vision_tx_msg.SOF = 0x11;
        /* ��ֵ���������� */
        float imu_data_temp_buf;
        imu_data_temp_buf = ((moto_pit.ecd - GIMBAL_PIT_CENTER_OFFSET) / (22.752f));//��λ����
        memcpy(vision_tx_msg.imu_pit, &imu_data_temp_buf, 4);
        imu_data_temp_buf = imu_data.yaw;//��λ����
        memcpy(&vision_tx_msg.imu_yaw, &imu_data_temp_buf, 4);
        imu_data_temp_buf = -1.0f * imu_data.wy / 16.384f;//��λ����/s
        memcpy(&vision_tx_msg.imu_pit_spd, &imu_data_temp_buf, 4);
        imu_data_temp_buf = imu_data.wz / 16.384f;//��λ����/s
        memcpy(&vision_tx_msg.imu_yaw_spd, &imu_data_temp_buf, 4);

        /* ��ֵ������Ϣ */
        vision_tx_msg.mode_msg.shooter_speed = shoot.shoot_speed_vision;
        /* ��ֵ�Ӿ�ģʽ */
        if( vision_mode == VISION_MODE_sENERGY )
        {
            key_status_clear(KEY_VISION_sENERGY);  //���С�������̰���״̬
            vision_tx_msg.mode_msg.aiming_status = 1;	//������
        }
        else if( vision_mode == VISION_MODE_bENERGY )
        {
            vision_tx_msg.mode_msg.aiming_status = 2;	//С����
        }
        else if( vision_mode == VISION_MODE_ANTIROTATE )
            vision_tx_msg.mode_msg.aiming_status = 0;   //��С����
        else
            vision_tx_msg.mode_msg.aiming_status = 0;	//���飨СԤ�⣩ �� ���ڱ�ģʽ����Ԥ�⣩
        
        /* ��ֵ��Ӫ */
        if( Game_Robot_Status.robot_id > 100 )		vision_tx_msg.mode_msg.camp = 1;  //���
        else 										vision_tx_msg.mode_msg.camp = 0;  //����

        /* ֡β */
        vision_tx_msg.EOF1 = 0x22;
        vision_tx_msg.EOF2 = 0x33;

        /* ����Ӿ�������Ϣ */
        memcpy(vision_tx_buf, &vision_tx_msg, VISION_TX_BUFF_LEN);
        ust_period_test(&ust_vision_send);
        HAL_UART_Transmit_DMA(&huart6, vision_tx_buf, VISION_TX_BUFF_LEN);  // С����5���Ǵ���6�������Ǵ���4
//        ust_interval_test_end(&ust_vision_send);
        taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, 1);//USART_SEND_PERIOD
    }
}

