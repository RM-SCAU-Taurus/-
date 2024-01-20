/**
	* @file bsp_iic.c
	* @version 1.0
	* @date 2019.11.26
  *
  * @brief  ���IIC����
  *
  *	@author YY(Part of codes reference ALIENTEK)
  *
  */

#include "bsp_iic.h"

/**********************************************************************************************************
*�� �� ��: IIC_Soft_Init
*����˵��: ���IIC��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void IIC_Soft_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE(); 
	
	GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10;
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  
	GPIO_Initure.Pull=GPIO_PULLUP;          
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	IIC_SDA(GPIO_PIN_SET);
	IIC_SCL(GPIO_PIN_SET);  
}

/**********************************************************************************************************
*�� �� ��: IIC_Start
*����˵��: ����IIC��ʼ�ź�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     					//SDA�����
	IIC_SDA(GPIO_PIN_SET);	  	  
	IIC_SCL(GPIO_PIN_SET);
	delay_us(4);
 	IIC_SDA(GPIO_PIN_RESET);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL(GPIO_PIN_RESET);//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  

/**********************************************************************************************************
*�� �� ��: IIC_Stop
*����˵��: ����IICֹͣ�ź�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();							//SDA�����
	IIC_SCL(GPIO_PIN_RESET);
	IIC_SDA(GPIO_PIN_RESET);//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL(GPIO_PIN_SET); 
	IIC_SDA(GPIO_PIN_SET);	//����I2C���߽����ź�
	delay_us(4);							   	
}

/**********************************************************************************************************
*�� �� ��: IIC_Wait_Ack
*����˵��: �ȴ�Ӧ���źŵ���
*��    ��: ��
*�� �� ֵ: 1������Ӧ��ʧ��
					 0������Ӧ��ɹ�
**********************************************************************************************************/
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      					 //SDA����Ϊ����  
	IIC_SDA(GPIO_PIN_SET);	delay_us(1);	   
	IIC_SCL(GPIO_PIN_SET);	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(GPIO_PIN_RESET);   
	return 0;  
} 

/**********************************************************************************************************
*�� �� ��: IIC_Ack
*����˵��: ����ACKӦ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL(GPIO_PIN_RESET);
	SDA_OUT();
	IIC_SDA(GPIO_PIN_RESET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET);
}

/**********************************************************************************************************
*�� �� ��: IIC_NAck
*����˵��: ������ACKӦ��	
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL(GPIO_PIN_RESET);
	SDA_OUT();
	IIC_SDA(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET);
}		

/**********************************************************************************************************
*�� �� ��: IIC_Send_Byte
*����˵��: IIC����һ���ֽ�	
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/		  
void IIC_Send_Byte(uint8_t txd)
{                        
	uint8_t t;   
	SDA_OUT(); 	    
	IIC_SCL(GPIO_PIN_RESET);	//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
		if(txd&0x80)
			IIC_SDA(GPIO_PIN_SET);
		else
			IIC_SDA(GPIO_PIN_RESET);
		txd<<=1; 	  
		delay_us(2);  
		IIC_SCL(GPIO_PIN_SET);
		delay_us(2); 
		IIC_SCL(GPIO_PIN_RESET);	
		delay_us(2);
	}	 
} 	

/**********************************************************************************************************
*�� �� ��: IIC_Read_Byte
*����˵��: IIC��ȡһ���ֽ�	
*��    ��: ��
*�� �� ֵ: �� 
**********************************************************************************************************/	
uint8_t IIC_Read_Byte(void)
{
	uint8_t receive=0;
	SDA_IN();									 //SDA����Ϊ����
  for(uint8_t i=0;i<8;i++)
	{
		receive<<=1;
		IIC_SCL(GPIO_PIN_RESET); 
		delay_us(2);
		IIC_SCL(GPIO_PIN_SET);
		if(READ_SDA)
			receive++;  
		delay_us(1);
  }	
	return receive;
}

/**********************************************************************************************************
*�� �� ��: IIC_Reg_Write
*����˵��: �����Ĵ���д��(�ð�λ��ַ)
*��    ��: �ӻ���ַ �Ĵ�����ַ д������
*�� �� ֵ: д��״̬
**********************************************************************************************************/
bool IIC_Reg_Write(uint8_t SlaveAddress,uint8_t Reg_Address,uint8_t Reg_data)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress+0);	//�ӻ���ַ��Write
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return false;
	}
	IIC_Send_Byte(Reg_Address);
	IIC_Wait_Ack();
	IIC_Send_Byte(Reg_data);
	IIC_Wait_Ack();
	IIC_Stop();
	return true;
}

/**********************************************************************************************************
*�� �� ��: IIC_Reg_Read
*����˵��: �����Ĵ�����ȡ(�ð�λ��ַ)
*��    ��: �ӻ���ַ �Ĵ�����ַ
*�� �� ֵ: ����
**********************************************************************************************************/
uint8_t IIC_Reg_Read(uint8_t SlaveAddress,uint8_t Reg_Address)
{
	uint8_t REG_data;
	IIC_Start();
	IIC_Send_Byte(SlaveAddress+0);	//�ӻ���ַ��Write
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return false;
	}
	IIC_Send_Byte(Reg_Address);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(SlaveAddress+1);	//�ӻ���ַ��Read
	IIC_Wait_Ack();
	
	REG_data = IIC_Read_Byte();
	
	IIC_NAck();
	IIC_Stop();
	
	return REG_data;
}




