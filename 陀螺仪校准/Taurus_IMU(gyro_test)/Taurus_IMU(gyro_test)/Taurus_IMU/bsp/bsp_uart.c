/** 
  * @file     bsp_uart.c
  * @version  v2.0
  * @date     2019.11.18
	*
  * @brief    ��������
	*
  *	@author   YY
  *
  */
	
#include "bsp_uart.h"
#include "bsp_JY901.h"
uint8_t JY901_receive_buff[JY901_BUFLEN];
/**
* @brief ʹ�ܴ��ڿ����ж�,��������DMA����
* @param  ��
* @retval ��
*/
void user_uart_init()
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)JY901_receive_buff, JY901_BUFLEN);
}


/**
* @brief ���ڿ����ж�
* @param UART_HandleTypeDef *huart
* @retval ��
*/
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //�ж��Ƿ��ǿ����ж�
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);                     	//��������жϱ�־�������һֱ���Ͻ����жϣ�
		HAL_UART_DMAStop(huart);
		USAR_UART_IDLECallback(huart);                       	  //�����жϴ�����
	}
}


/**
* @brief ���ڿ����жϻص�����
* @param ���ھ��
* @retval ��
*/
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance== USART1 )
	{
		JY901_original_data_read(JY901_receive_buff); 			//JY901���ڶ�ȡԭʼ����
	}
	memset(JY901_receive_buff,0,sizeof(JY901_receive_buff));                    //������ջ�����
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)JY901_receive_buff, JY901_BUFLEN);  //������ʼDMA����
}


