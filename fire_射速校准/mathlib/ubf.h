/**
 * @file ubf.h
 * @brief ʵʱ���»��λ�����
 * @note ʹ�÷����ο� .c�ļ��·�
 * 
 * @author ZZJ (WX:z1429881129)
 * @version 1.0 2022-02-25 For the first time to submit
 *          1.1 2022-03-04 Add the function to clear the buffer
 *          1.2 2022-03-14 Add the function to get datas into a 1D array
 * @date 2022-02-25
 * @copyright Copyright (c) Taurus 2022
 * 
 */
#ifndef __UPDATE_BUFFER_H__
#define __UPDATE_BUFFER_H__

#include "stdint.h"

/**
 * @brief ������ָ���������
 */
typedef struct _ubf_t* ubf_t;

/**
 * @brief ����������
 * 
 * @param num ���ݵ�Ԫ����
 * @param size ���ݵ�Ԫռ�ֽ��� 
 * @return ubf_t ������ָ��
 */
ubf_t ubf_create(uint32_t num, uint32_t size);

/**
 * @brief ɾ�����������ͷ��ڴ�
 * 
 * @param ubf ��ɾ���Ļ�����ָ��
 */
void ubf_delete(ubf_t ubf);

/**
 * @brief Ͷι����
 * 
 * @param ubf ������ָ��
 * @param pdata ����ָ��
 * @return uint8_t 
 */
uint8_t ubf_push(ubf_t ubf, const void* pdata);

/**
 * @brief ��ȡ k ʱ�̵�����
 * 
 * @param ubf ������ָ��
 * @param k ʱ��������k��[0, num-1]
 * @return void* ��������ָ��
 */
void* ubf_pop(ubf_t ubf, uint32_t k);

/**
 * @brief ��ȡ�������е�ǰ��������
 * 
 * @param ubf ������ָ��
 * @return uint32_t �������е�ǰ��������
 */
uint32_t ubf_get_stock(ubf_t ubf);

/**
 * @brief ��ջ�����
 * 
 * @param ubf ������ָ��
 */
void ubf_clear(ubf_t ubf);

/**
 * @brief ȡ���������������ݣ��������ݴ��µ������г����鷵��
 * 
 * @param ubf ������ָ��
 * @param pdata ���������׵�ַ���û�������
 * @param k ʱ��������k��[0, num-1]
 * @param num �������ݸ���
 * @return uint32_t ʵ�ʷ������ݸ���
 */
uint32_t ubf_pop_into_array_new2old(ubf_t ubf, void* pdata, uint32_t k, uint32_t num);

/**
 * @brief ȡ���������������ݣ��������ݴӾɵ������г����鷵��
 * 
 * @param ubf ������ָ��
 * @param pdata ���������׵�ַ���û�������
 * @param k ʱ��������k��[0, num-1]
 * @param num �������ݸ���
 * @return uint32_t ʵ�ʷ������ݸ���
 */
uint32_t ubf_pop_into_array_old2new(ubf_t ubf, void* pdata, uint32_t k, uint32_t num);

#endif
