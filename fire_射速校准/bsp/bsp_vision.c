/**
  * @file bsp_vision.c
  * @version 1.0
  * @date Feb,23th 2021
	*
  * @brief  �Ӿ���Ϣ����
	*
  *	@author
  *
  */
#include "bsp_vision.h"
#include "control_def.h"
#include "remote_msg.h"
#include "bsp_can.h"
#include "string.h"
#include "math.h"
#include "remote_msg.h"
#include "bsp_T_imu.h"
#include "KalmanFilter.h"
#include "math_calcu.h"
#include "modeswitch_task.h"
#include "math.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "ubf.h"
#include "us_tim.h"

float test_visoin_delay = 6;/* �Ӿ���ʱ���� */
//float test_imu_spd_kp = 16.3835f;

vision_data_t vd;/* �Ӿ����ݽṹ�� */
vision_output_t vision;/* �Ӿ�����������ṹ�� */
ust_t ust_vision_nvic;/* �Ӿ������жϼ�ʱ�� */

extern ubf_t imu_yaw_ubf;/* �����ǽ��ٶȻ����� */
extern ubf_t vision_data_ubf;
float vision_data_array[20];
float var;

/**
 * Ŀǰ��Ҫ��ʹ����4��һ�׿������˲���
 * 1. �Ӿ��ش��ĽǶȲ�ֵõ�����Խ��ٶ�
 * 2. �Ӿ��ش��ľ���
 * 3. ����õ�Ԥ�����
 * 4. ����õ�Ԥ��Ƕ�
 *
 *  ���Ծ���
 *  1. ���Ƿ����Ӿ�����
 *  2. �����ٶ���������ߣ�����ʱ
 *  3. 
 *
 *  ��������
 *  1. ͨ����ʱ��ȷȷ��
 *  2. canͨ����ʱ������
 */

/**
  * @brief �Ӿ���Ϣ����
  * @param
  * @attention
	* @note
  */
void vision_data_handler(uint8_t *Vision_Data)
{
    /* �����Ӿ���Ϣ */
    memcpy(&vd.yaw.vision_angle_error.now, Vision_Data, 4);     /* YAW����ԽǶȣ��㣩 */
    memcpy(&vd.pit_angle_error.now,(Vision_Data+4), 4);         /* PIT����ԽǶȣ��㣩 */
    memcpy(&vd.distance.now, (Vision_Data+8), 4);                       /* ���루mm�� */
    memcpy(&vd.tof.now, (Vision_Data+12), 4);                    /* �ӵ�����ʱ�䣨s�� */
    memcpy(&vd.cnt, (Vision_Data+16), 1);                                   /* ����λ */
    memcpy(&vd.eof, (Vision_Data+17), 1);                                     /* ֡β */
    
    /* �������ݷ��� */
    ubf_push(vision_data_ubf, &vd.yaw.vision_angle_error.now);
    uint32_t real_num = ubf_pop_into_array_new2old(vision_data_ubf, vision_data_array, 0, 20);
    arm_var_f32(vision_data_array, real_num, &var);

    /* ����40ms��ͨ������Ӧ��ʱ */
    vd.tof.now -= 0.04f;  
    Output_Limit(vd.tof.now, vd.tof.now, 0);

    /* ��ȡ�Ӿ��������� */
    vd.period = 8;/* ��λ ms */
//    vd.period = ust_period_test(&ust_vision_nvic);
    
//    static uint32_t last_time, time;
//    time = HAL_GetTick();
//    vd.period = time - last_time;
//    last_time = time;

    /* ��λͳһ����׼��λ�� */
    vd.period *= 1.0e-3f;/* ���ڣ�s��*/
    vd.distance.now *= 1.0e-3f;/* ���루m�� */

    /* ����ͳһ��������ʱ��Ϊ�� */
    vd.yaw.vision_angle_error.now = -vd.yaw.vision_angle_error.now;
    
    if (vd.distance.now)  //ǰ����֡��ʶ��Ŀ��
    {
        /* �����쳣���� */
//        //����ʱ��
//        if (vd.tof.now >= 2.0f || vd.tof.now <= -2.0f)
//            vd.tof.now = vd.tof.last;
//        else    vd.tof.last = vd.tof.now;
//        
//        //PIT��Ƕ�
//        if (vd.pit_angle_error.now >= 40 || vd.pit_angle_error.now <= -40)
//            vd.pit_angle_error.now = vd.pit_angle_error.last;
//        else    vd.pit_angle_error.last = vd.pit_angle_error.now;
//        
//        //YAW��Ƕ�
//        if (vd.yaw.vision_angle_error.now >= 50 || vd.yaw.vision_angle_error.now <= -50)
//            vd.yaw.vision_angle_error.now = vd.yaw.vision_angle_error.last;

        /* ��������Խ��ٶ� */
        vd.yaw.imu_angular_speed.now = 
            *(float*)ubf_pop(imu_yaw_ubf, (uint32_t)(vd.period*1e3f*test_visoin_delay))
            /16.3835f; /* ��λ:LSB->(��/s) �����������˲�*/
        
        /* �Ӿ���Խ��ٶ� */
        vd.yaw.vision_angle_error.kal = 
            Kalman1FilterCalc(&kalman_yaw_angle_error, vd.yaw.vision_angle_error.now); /* ��̨����ʹ�� */
        vd.yaw.vision_angular_speed.now = (vd.yaw.vision_angle_error.kal - vd.yaw.vision_angle_error.last) 
            / vd.period;  /* ��λ:��/s */
        vd.yaw.vision_angular_speed.kal = Kalman1FilterCalc(&kalman_yaw_aim_speed, vd.yaw.vision_angular_speed.now);
        vd.yaw.vision_angle_error.last = vd.yaw.vision_angle_error.kal;
        
        /* Ŀ��������Խ��ٶȣ���/s�� */
        vd.yaw.object_angular_speed.now = vd.yaw.vision_angular_speed.kal + vd.yaw.imu_angular_speed.now;
        
        /* Ŀ������������ٶ�ʸ����m/s�� */
        vd.distance.kal = Kalman1FilterCalc(&kalman_vision_distance, vd.distance.now);
        vd.yaw.object_tangential_speed.now = vd.yaw.object_angular_speed.now * (2*PI)/360.0f * vd.distance.kal;
        
        /* Ŀ�꾶������ٶ�ʸ����m/s�� */
        vd.yaw.object_radial_speed.now = (vd.distance.kal - vd.distance.last) / vd.period;
        vd.yaw.object_radial_speed.kal = Kalman1FilterCalc(&kalman_vision_dr, vd.yaw.object_radial_speed.now);
        vd.distance.last = vd.distance.kal;
        
        /* Ŀ���ά�ٶ�ģֵ */
        vd.yaw.object_2D_speed.now = sqrtf(powf(vd.yaw.object_radial_speed.kal, 2) + 
            powf(vd.yaw.object_tangential_speed.now, 2));/* ��λ:m/s */
        
        /* Ŀ���ά�ٶȽǶ� */
        vd.yaw.object_speed_angle.now = 
            atanf(vd.yaw.object_radial_speed.kal
            /ABS(vd.yaw.object_tangential_speed.now))+ PI/2;/* ֻҪ�����ٶ�Ϊ����Ϊ�۽ǣ�rad�� */
        
        /* Ŀ���˶�λ��ģֵԤ�⣨m�� */
        vd.tof.kal = Kalman1FilterCalc(&kalman_bullet_time, vd.tof.now);
        vd.yaw.predict_distance.now = vd.yaw.object_2D_speed.now * (vd.tof.kal + vd.period/2.0f);
        
        /* ���Ҷ������ζԱߣ�m�� */
        vd.yaw.opposite_line.now = sqrtf(powf(vd.yaw.predict_distance.now, 2) + powf(vd.distance.kal, 2) 
            - 2 * vd.yaw.predict_distance.now * vd.distance.kal * cosf(vd.yaw.object_speed_angle.now));
        vd.yaw.opposite_line.kal = Kalman1FilterCalc(&kalman_predict_distance, vd.yaw.opposite_line.now);
        
        /* ���Ҷ���Ԥ��ǶȽ��㣨�㣩 */
        vd.yaw.object_angular_speed.kal = 
            Kalman1FilterCalc(&kalman_object_angular_speed, vd.yaw.object_angular_speed.now);/* ��λƥ�� */
        if (ABS(vd.yaw.object_angular_speed.kal) > 5.0f) /* ����Ԥ��������ֵ */
        {
            vd.yaw.predict_angle.now = SIGN(vd.yaw.object_angular_speed.kal)
                * asinf(vd.yaw.opposite_line.kal / sinf(vd.yaw.object_speed_angle.now) * vd.yaw.predict_distance.now)
                * 360 / (2*PI); /* �Ƕ��� */
        }
        else
        {
            vd.yaw.predict_angle.now = 0;
        }
        /* Ԥ��Ƕ����ݴ��� */
        if (isnan(vd.yaw.predict_angle.now))
        {
            if(isnan(vd.yaw.predict_angle.last))
                vd.yaw.predict_angle.last = vd.yaw.predict_angle.now = 0;
            else
                vd.yaw.predict_angle.now = vd.yaw.predict_angle.last;
        }
        else
        {
            vd.yaw.predict_angle.last = vd.yaw.predict_angle.now;
        }
        vd.yaw.predict_angle.kal = Kalman1FilterCalc(&kalman_predict_angle, vd.yaw.predict_angle.now);
        
        /* ������� */
        vision.pit_angle_error = vd.pit_angle_error.now; /* PIT�Ƕ�ƫ�� */
        vision.yaw_angle_error = vd.yaw.vision_angle_error.kal; /* YAW��Ƕ�ƫ�� */
        vision.yaw_predict_angle = vd.yaw.predict_angle.kal; /* YAW��Ƕ�ƫ��Ԥ�� */
        
        vision.aiming_flag = 1; /* ��ǵ�ǰʶ��Ŀ�� */
        vision.first_lost_flag = 1; /* ��ǵ�ǰû�ж�֡ */
        vision.new_frame_flag = 1; /* ��ǵ�ǰ������ϢΪ���� */
    }
    else
    {
        vision.aiming_flag = 0; /* ��ǵ�ǰδʶ��Ŀ�� */
        vision.new_frame_flag = 0; /* ���û�н������Ч������ */
        vision.yaw_predict_angle = 0; /* Ԥ��Ƕ����� */
        
        memset(&vd.yaw.vision_angle_error, 0, sizeof(vision_data_pkg3_t));
        memset(&vd.pit_angle_error, 0, sizeof(vision_data_pkg3_t));
        memset(&vd.distance, 0, sizeof(vision_data_pkg3_t));
        memset(&vd.tof, 0, sizeof(vision_data_pkg3_t));
        
        Kalman1FilterDeinit(&kalman_yaw_angle_error);
        Kalman1FilterDeinit(&kalman_yaw_aim_speed);
        Kalman1FilterDeinit(&kalman_vision_distance);
        Kalman1FilterDeinit(&kalman_vision_dr);
        Kalman1FilterDeinit(&kalman_bullet_time);
        Kalman1FilterDeinit(&kalman_predict_distance);
        Kalman1FilterDeinit(&kalman_object_angular_speed);
    }
}


