#define __BSP_CAN_GLOBALS

#include "bsp_can.h"
#include "pid.h"
#include "chassis_task.h"
#include "status_task.h"
#include "string.h"
#include "bsp_T_imu.h"
#include "bsp_powerlimit.h"

#include "z_gimbal.h"

CAN_TxHeaderTypeDef Tx1Message;
CAN_RxHeaderTypeDef Rx1Message;
CAN_TxHeaderTypeDef Tx2Message;
CAN_RxHeaderTypeDef Rx2Message;

uint8_t CAN1_Rx_data[8];
uint8_t CAN1_Tx_data[8];
uint8_t CAN2_Rx_data[8];
uint8_t CAN2_Tx_data[8];

/**
  * @brief     CAN接受中断回调函数
  * @param     CAN_Rx_data ：CAN节点反馈的数据帧
  * @attention
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan == &hcan1)
    {
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0, &Rx1Message, CAN1_Rx_data);
        switch (Rx1Message.StdId)
        {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
            static uint8_t i;
            i = Rx1Message.StdId - CAN_3508_M1_ID;
            encoder_data_handler(&moto_chassis[i],&hcan1,CAN1_Rx_data);
            status.chassis_status[i] = 1;
            break;
        }
        case CAN_YAW_MOTOR_ID:
        {
            encoder_data_handler(&moto_yaw, hcan,CAN1_Rx_data);
            status.gimbal_status[0] = 1;
            break;
        }
        case POWER_CONTROL_ID:
        {
            Power_data_handler(Rx1Message.StdId,CAN1_Rx_data);
            status.power_control = 1;
            break;
        }
#ifdef __TEST_ZGIMBAL__
        case ZGIMBAL_ID:
        {
            zgim.moto.msg_cnt <= 50 ? (zgim.moto.msg_cnt++, get_moto_offset(&zgim.moto, &hcan1,CAN1_Rx_data)) : zgimbal_data_handler(hcan, CAN1_Rx_data);
            break;
        }
#endif
        default:
        {
            break;
        }
        };
        __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if( hcan == &hcan2 )
    {
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0, &Rx2Message, CAN2_Rx_data);
        switch( Rx2Message.StdId )
        {
        case TIMU_PALSTANCE_ID:
        {
            T_imu_calcu(Rx2Message.StdId, CAN2_Rx_data);
            status.gyro_status[0] = 1;
        }
        case TIMU_ANGLE_ID:
        {
            T_imu_calcu(Rx2Message.StdId, CAN2_Rx_data);
            status.gyro_status[1] = 1;
            break;
        }
        case CAN_PIT_MOTOR_ID:
        {
            encoder_data_handler(&moto_pit, hcan,CAN2_Rx_data);
            status.gimbal_status[1] = 1;
            /* 当机械零点在pit轴运动范围内,拓展编码值范围 */
//            if( gimbal.pid.pit_ecd_fdb < 5000 )  //确定电机编码值在零点附近时，当正向越界，从8191到1阶跃
//            {
//                gimbal.pid.pit_ecd_fdb += 8191;
//            }
            break;
        }
        case CAN_TRIGGER_MOTOR1_ID:  //上枪管拨盘2006
        {
            motor_trigger.msg_cnt++ <= 50 ? get_moto_offset(&motor_trigger, &hcan2,CAN2_Rx_data) : encoder_data_handler(&motor_trigger,&hcan2,CAN2_Rx_data);
            status.gimbal_status[2] = 1;
            break;
        }
        }
        __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
    ptr->ecd        = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
    //转子转速
    ptr->speed_rpm     = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
    ptr->given_current = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);

    //机械角度
    ptr->last_ecd = ptr->ecd;
    ptr->ecd      = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);

    //相对开机后的角度
    if (ptr->ecd - ptr->last_ecd > 4096)
        ptr->round_cnt--;
    else if (ptr->ecd - ptr->last_ecd < -4096)
        ptr->round_cnt++;

    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
}

void can1_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    uint8_t FreeTxNum = 0;

    Tx1Message.StdId = TX_ID;
    Tx1Message.IDE 	 = CAN_ID_STD;
    Tx1Message.RTR   = CAN_RTR_DATA;
    Tx1Message.DLC   = 0x08;

    CAN1_Tx_data[0] = iq1 >> 8;
    CAN1_Tx_data[1] = iq1;
    CAN1_Tx_data[2] = iq2 >> 8 ;
    CAN1_Tx_data[3] = iq2;
    CAN1_Tx_data[4] = iq3 >> 8;
    CAN1_Tx_data[5] = iq3;
    CAN1_Tx_data[6] = iq4 >> 8;
    CAN1_Tx_data[7] = iq4;

    //查询发送邮箱是否为空
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,CAN1_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

void can2_send_message(int16_t TX_ID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    uint8_t FreeTxNum = 0;

    Tx2Message.StdId = TX_ID;
    Tx2Message.IDE 	 = CAN_ID_STD;
    Tx2Message.RTR   = CAN_RTR_DATA;
    Tx2Message.DLC   = 0x08;

    CAN2_Tx_data[0] = iq1 >> 8;
    CAN2_Tx_data[1] = iq1;
    CAN2_Tx_data[2] = iq2 >> 8 ;
    CAN2_Tx_data[3] = iq2;
    CAN2_Tx_data[4] = iq3 >> 8;
    CAN2_Tx_data[5] = iq3;
    CAN2_Tx_data[6] = iq4 >> 8;
    CAN2_Tx_data[7] = iq4;

    //查询发送邮箱是否为空
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
    }

    HAL_CAN_AddTxMessage(&hcan2, &Tx2Message,CAN2_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

/**
  * @brief  init the can transmit and receive
  * @param  None
  */
void can_device_init(void)
{
    //can1 filter config
    CAN_FilterTypeDef  can_filter;

    can_filter.FilterBank           = 0;
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = 0x0000;
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
    can_filter.SlaveStartFilterBank = 0;
    can_filter.FilterActivation     = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
    while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);

    can_filter.FilterBank           = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter);
    while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK);

    HAL_Delay(100);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}

void pit_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
    //转子转速
    ptr->speed_rpm     = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
    ptr->given_current = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);

    //机械角度
    ptr->last_ecd = ptr->ecd;
    ptr->ecd      = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);
    if(ptr->ecd<=2000)	ptr->ecd+=8191;
    //相对开机后的角度
    if (ptr->ecd - ptr->last_ecd > 4096)
        ptr->round_cnt--;
    else if (ptr->ecd - ptr->last_ecd < -4096)
        ptr->round_cnt++;

    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
}
