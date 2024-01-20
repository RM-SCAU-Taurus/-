/** 
  * @file     bsp_imu.c
  * @version  v1.0
  * @date     2020.1.10
	*
  * @brief    IMU�������
	*
  *	@author   YY
  *
  */

#include "bsp_imu.h"
#include "KalmanFilter.h"
#include "Calibration.h"
//#define Kp 1.35f    // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.002f   // integral gain governs rate of convergence of gyroscope biases
extern float AccRatioOffset;
bias_gyro_mode_e bias_gyro_mode;
volatile float exInt, eyInt, ezInt;  // ������
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;
//float halfT=0.001;	 //�������ڵ�һ��
float halfT = 0.0005;
float Kp=0.1f;
//float Ki=0.002f;
float Ki=0.0f;
float ahrs_count;
float ahrs_norm;
float ez_test;
float norm_test;
//float test_vz;
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;

volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ ms

IMU_FLOAT_DATA_T imu_real_data;
float pitch_angle_test,kf2_pitch_angle;
float *kf2_pitch;


/**
  * @brief IMU_Values_Convert
  * @param 
  * @attention  
  * @note  IMU���ݵ�λ����
  */
void IMU_Values_Convert(void)
{
	imu_real_data.Gyro.X = imu_output_data.Gyro.X/16.384f/57.3f;
	imu_real_data.Gyro.Y = imu_output_data.Gyro.Y/16.384f/57.3f;
	imu_real_data.Gyro.Z = imu_output_data.Gyro.Z/16.384f/57.3f; //���ٶȵ�λLSB->rad/s
	imu_real_data.Accel.X = imu_output_data.Accel.X/1365.0f;
	imu_real_data.Accel.Y = imu_output_data.Accel.Y/1365.0f;
	imu_real_data.Accel.Z = imu_output_data.Accel.Z/1365.0f; 			//���ٶ�AD->g
	
	imu_real_data.Mag.X = -imu_output_data.Mag.X;
	imu_real_data.Mag.Y = imu_output_data.Mag.Y;
	imu_real_data.Mag.Z = imu_output_data.Mag.Z; //����������ϵ��һ
  //test_vz+=(imu_real_data.Accel.Z-AccRatioOffset)*0.001;
}


/**
  * @brief IMU_AHRS_Calcu
  * @param 
  * @attention  
  * @note  
  */
void IMU_AHRS_Calcu(void) 
{
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

	  ahrs_count++;
    gx = imu_real_data.Gyro.X;
    gy = imu_real_data.Gyro.Y;
    gz = imu_real_data.Gyro.Z;
//    ax = imu_real_data.Accel.X;
//    ay = imu_real_data.Accel.Y;
//    az = imu_real_data.Accel.Z;
    ax = imu_real_data.Accel.X;
    ay = imu_real_data.Accel.Y;
    az = imu_real_data.Accel.Z;

//    mx = imu_real_data.Mag.X;
//    my = imu_real_data.Mag.Y;
//    mz = imu_real_data.Mag.Z;		
		
    //������ƽ�����㷨
    norm = invSqrt(ax*ax + ay*ay + az*az);   
    norm_test = 1/norm;    
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //�ѼӼƵ���ά����ת�ɵ�λ������
 //   norm = invSqrt(mx*mx + my*my + mz*mz); 
   if(fabs(norm_test-10)>3)
    		// Kp =0;
	      Kp=0;
	 else 
		   // Kp=0.03;
	 Kp = 0.7;
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
    ez_test = ez;
    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
			exInt = exInt + ex * Ki * halfT;
			eyInt = eyInt + ey * Ki * halfT;	
			ezInt = ezInt + ez * Ki * halfT;
			// �ò���������PI����������ƫ
			gx = gx + Kp*ex + exInt;
			gy = gy + Kp*ey + eyInt;
			gz = gz + Kp*ez + ezInt;
    }
    // ��Ԫ��΢�ַ���
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // ��Ԫ���淶��
	if(ahrs_count>2000)
	{ norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
	}
  if(ahrs_count<2000)
		{
		   ahrs_norm =sqrtf(imu_real_data.Accel.X*imu_real_data.Accel.X+imu_real_data.Accel.Y*imu_real_data.Accel.Y+imu_real_data.Accel.Z*imu_real_data.Accel.Z);
		   if(fabs(ahrs_norm-1)<0.1f && imu_real_data.Gyro.X<0.1f && imu_real_data.Gyro.Y<0.1f && imu_real_data.Gyro.Z <0.1f  )
			 {
			    //  flag_quaternion_intilized =1;
				    if(imu_real_data.Accel.Z/ahrs_norm >=0)
						{
//						   q0 =sqrtf((1.0 + imu_real_data.Accel.Z/ahrs_norm)/2.0f);
//							 q1 = imu_real_data.Accel.Y/ahrs_norm/sqrtf(2.0 * (imu_real_data.Accel.Z/ahrs_norm + 1.0));
//						   q2 = -imu_real_data.Accel.X/ahrs_norm/sqrtf(2.0 *(imu_real_data.Accel.Z/ahrs_norm +1.0));
//							 q3 = 0;
							 q0 =sqrtf((1.0f + imu_real_data.Accel.Z/ahrs_norm)/2.0f);
							 q1 = imu_real_data.Accel.Y/ahrs_norm/sqrtf(2.0f * (imu_real_data.Accel.Z/ahrs_norm + 1.0f));
						   q2 = -imu_real_data.Accel.X/ahrs_norm/sqrtf(2.0f *(imu_real_data.Accel.Z/ahrs_norm +1.0f));
							 q3 = 0;
						}
						else
						{
						  q0 = imu_real_data.Accel.Y / ahrs_norm / sqrtf(2.0f * (1.0f - imu_real_data.Accel.Z/ahrs_norm));
							q1 =sqrtf((1.0f - imu_real_data.Accel.Z/ahrs_norm));
							q2 = 0;
							q3 = imu_real_data.Accel.X/ahrs_norm/sqrtf(2.0f *(1.0f - imu_real_data.Accel.Z/ahrs_norm));
						}
			 }
		}
    imu_real_data.q[0] = q0;
		imu_real_data.q[1] = q1;
		imu_real_data.q[2] = q2;
		imu_real_data.q[3] = q3;		
		//��Ԫ��ת��ŷ����
		imu_real_data.yaw 	= atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3+180.0f; // yaw��ǶȾ�����������������
		imu_real_data.pitch = asin(-2*q1*q3 + 2*q0*q2)* 57.3;         							  			 // pitch
    imu_real_data.roll  = atan2(2*q2*q3  + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3; 			 // roll 
		
		//   kf2_pitch = Kalman2Filter_calc(pitch_angle_test,imu);     
//		imu_real_data.roll = asin(-2*q1*q3 + 2*q0*q2)* 57.3;         							  			 // roll
//    imu_real_data.pitch  = atan2(2*q2*q3  + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3; 			 // pitch 

}


