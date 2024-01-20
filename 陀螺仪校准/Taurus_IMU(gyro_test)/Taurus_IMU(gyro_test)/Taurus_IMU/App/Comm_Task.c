/** 
  * @file     Comm_Task.c
  * @version  v1.0
  * @date     2020.1.3
	*
  * @brief    can½»»¥ÈÎÎñ
	*
  *	@author   YY
  *
  */

#include "Comm_Task.h"
#include "Status_Task.h"

/**
  * @brief can_msg_send_task
  * @param 
  * @attention  
  * @note  
  */
void can_msg_send_task(void)
{
//	osEvent event;
//	for(;;)
//	{
	//	event = osSignalWait(PALSTANCE_MSG_SEND  | \
		                     ANGLE_MSG_SEND , osWaitForever);
		
	//	if (event.status == osEventSignal)
  //  {
	//		if (event.value.signals & PALSTANCE_MSG_SEND)
  //    {
	    if(Data_Ready_Flag==1)
			{	
				send_palstance_message();
				send_angle_message();
			}
	//		}
			
//			if (event.value.signals & ANGLE_MSG_SEND)
//      {
//				send_angle_message();
//			}
			
//		}
//	}

}

