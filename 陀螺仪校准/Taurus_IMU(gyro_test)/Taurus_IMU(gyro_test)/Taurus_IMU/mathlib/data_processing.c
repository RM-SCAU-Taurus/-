/**
	* @file data_processing.c
	* @version 1.0
	* @date 2019.11.26
  *
  * @brief  һЩ���ݴ�����
  *
  *	@author YY
  *
  */
	
#include "data_processing.h"


/***************************************************************/
/*
 * ��������Float2Byte
 * ����  ���������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ
 * ����  ��target:Ŀ�굥��������
					 buf:��д������
					 beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
 * ���  ����
 */ 
/***************************************************************/
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
// Fast inverse square-root
*******************************************************************************/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


