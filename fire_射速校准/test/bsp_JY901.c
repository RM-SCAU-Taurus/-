/** 
  * @file bsp_JY901.c
  * @version 2.0
  * @date 2020.1.2
	*
  * @brief  JY901数据读取
	*
  *	@author YY
  *
  */
  
/*
正方向：


*/

#include "bsp_JY901.h"
#include "math.h"

#ifndef ABS
    #define ABS(x)		((x>0)? (x): (-(x)))
#endif

#ifndef Output_Limit
    #define Output_Limit(output,max,min) \
            ((output)<=(max) && (output)>=(min)? output: ((output)>(max)? (output = max):(output = min)))
#endif

imu_typedef JY901_data;
uint8_t dma_JY901_data_buff[JY901_BUFF_LEN];

float gravity_buff;


struct angle_func
{
    float pit_sin;
    float pit_cos;
    float rol_sin;
    float rol_cos;
    float yaw_sin;
    float yaw_cos;
    
}test_func_value;

/**
  * @brief JY901解算
  * @param 
  * @attention  
  * @note  
  */
void JY901_original_data_read(uint8_t imu_buf[])
{
    /* IMU数据解码（右手系） */
	if (imu_buf[0] == 0x55 && imu_buf[1] == 0x51)//三轴加速度
	{
	   JY901_data.Gyro_a_x=-((short)(imu_buf[3]<<8)|imu_buf[2])/32768.0f*16.0f*9.8f;
	   JY901_data.Gyro_a_y=((short)(imu_buf[5]<<8)|imu_buf[4])/32768.0f*16.0f*9.8f;
       JY901_data.Gyro_a_z=-((short)(imu_buf[7]<<8)|imu_buf[6])/32768.0f*16.0f*9.8f;
	}
	if (imu_buf[0+11] == 0x55 && imu_buf[1+11] == 0x52)//三轴角速度
	{
	   JY901_data.Gyro_x=((short)(imu_buf[3+11]<<8)|imu_buf[2+11])/32768.0f*2000.0f;
	   JY901_data.Gyro_y=-((short)(imu_buf[5+11]<<8)|imu_buf[4+11])/32768.0f*2000.0f;
       JY901_data.Gyro_z=-((short)(imu_buf[7+11]<<8)|imu_buf[6+11])/32768.0f*2000.0f; 
	}
    if (imu_buf[0+22] == 0x55 && imu_buf[1+22] == 0x53)//欧拉角
	{
		JY901_data.roll  = ((short)(imu_buf[3+22]<<8)|imu_buf[2+22])/32768.0f*180.0f;
		JY901_data.pitch = ((short)(imu_buf[5+22]<<8)|imu_buf[4+22])/32768.0f*180.0f;
		JY901_data.yaw   = ((short)(imu_buf[7+22]<<8)|imu_buf[6+22])/32768.0f*180.0f;
	}
    test_func_value.pit_sin = sin(JY901_data.pitch);
    test_func_value.pit_cos = cos(JY901_data.pitch);
    
    test_func_value.rol_sin = sin(JY901_data.roll);
    test_func_value.rol_cos = cos(JY901_data.roll);
    
    test_func_value.yaw_sin = sin(JY901_data.yaw);
    test_func_value.yaw_cos = cos(JY901_data.yaw);
    
    /* 计算Z轴位移 */
    if (JY901_data.cnt < 100)
    {
        /* 初始化重力加速度常量 */
        JY901_data.cnt++;
        gravity_buff += sqrt(JY901_data.Gyro_a_x * JY901_data.Gyro_a_x + 
                             JY901_data.Gyro_a_y * JY901_data.Gyro_a_y + 
                             JY901_data.Gyro_a_z * JY901_data.Gyro_a_z);
        if (JY901_data.cnt >= 100)
            JY901_data.gravity = gravity_buff/100.0f;
    }
    else
    {
        /* 三轴加速度与重力加速度解耦 */
        JY901_data.acc_x = JY901_data.Gyro_a_x + 
                                JY901_data.gravity * sin(JY901_data.pitch);
        JY901_data.acc_y = JY901_data.Gyro_a_y - 
                                JY901_data.gravity * cos(JY901_data.pitch) * sin(JY901_data.roll);
        JY901_data.acc_z = JY901_data.Gyro_a_z + 
                                JY901_data.gravity * cos(JY901_data.pitch) * cos(JY901_data.roll);

//        /* 静止死区 */
//        if (JY901_data.acc_z >= -0.001f || JY901_data.acc_z <= 0.001f)
//            JY901_data.z_acc[1] = 0;
//        else
//            JY901_data.z_acc[1] = JY901_data.acc_z;
//            
//        /* 梯形积分得速度 */
//        JY901_data.z_spd[1] += 0.5f * (JY901_data.z_acc[1] + JY901_data.z_acc[0]) * 0.001f;
//        
//        /* 刹停检测 */
//        if (JY901_data.z_acc[1] == 0)
//            JY901_data.stop_cnt++;
//        else
//            JY901_data.stop_cnt = 0;
//        if (JY901_data.stop_cnt >= 25)
//        {
//            JY901_data.z_spd[1] = 0;
//            JY901_data.z_spd[0] = 0;
//        }
//        
//        /* 梯形积分得位移 */
//        JY901_data.z_pos += 0.5f * (JY901_data.z_spd[1] + JY901_data.z_spd[0]) * 0.001f;
//        
//        /* 历史数据迭代 */
//        JY901_data.z_spd[0] = JY901_data.z_spd[1];
//        JY901_data.z_acc[0] = JY901_data.z_acc[1];
    }
}

