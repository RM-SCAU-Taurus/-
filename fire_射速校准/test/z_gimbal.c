#ifdef __TEST_ZGIMBAL__

#include "z_gimbal.h"

#include "cmsis_os.h"

#include "math.h"
#include "filter.h"
#include "KalmanFilter.h"
#include "DataScope_DP.h"
#include "usart.h"

#include "bsp_JY901.h"

#define ZGIMBAL_ECD_MAX 11000
#define ZGIMBAL_ECD_MIN -8000
#define ZGIMBAL_ECD_MID 3650

#define ZGIMBAL_SEND_SIGN 0X01


osThreadId zgimbal_task_t;
osThreadId zgimbal_send_task_t;
osThreadId zgimbal_reset_task_t;
osMutexId zgimbal_mutex_t;

zgimbal_t zgim;

int32_t test_ecd_set = 0;

LPFOfilter_t LPFO_filter = 
{
    .mode = LPFO_MODE_N,
    .K = 0.2f
};

void zgimbal_data_handler(CAN_HandleTypeDef* hcan, uint8_t * CAN_Rx_data)
{
    //转子转速
    zgim.moto.speed_rpm     = (int16_t)(CAN_Rx_data[2] << 8 | CAN_Rx_data[3]);
    zgim.moto.given_current = (int16_t)(CAN_Rx_data[4] << 8 | CAN_Rx_data[5]);

    //机械角度
    zgim.moto.last_ecd = zgim.moto.ecd;
    zgim.moto.ecd      = (uint16_t)(CAN_Rx_data[0] << 8 | CAN_Rx_data[1]);

    //相对开机后的角度
    if (zgim.moto.ecd - zgim.moto.last_ecd > 4096)
        zgim.moto.round_cnt--;
    else if (zgim.moto.ecd - zgim.moto.last_ecd < -4096)
        zgim.moto.round_cnt++;

    zgim.moto.total_ecd = zgim.moto.round_cnt * 8192 + zgim.moto.ecd - zgim.moto.offset_ecd;
}

static void zgimbal_pid_cal(float set)
{
    zgim.pid_param.ecd_ref = data_limit(set, zgim.ecd_max, zgim.ecd_min);
    zgim.pid_param.ecd_fdb = zgim.moto.total_ecd;
    zgim.pid_param.ecd_err = zgim.pid_param.ecd_ref - zgim.pid_param.ecd_fdb;

    zgim.pid_param.spd_ref = pid_calc(&zgim.ecd_pid, zgim.pid_param.ecd_fdb, zgim.pid_param.ecd_fdb + zgim.pid_param.ecd_err);
    zgim.pid_param.spd_fdb = zgim.moto.speed_rpm;
    zgim.current = pid_calc(&zgim.spd_pid, zgim.pid_param.spd_fdb, zgim.pid_param.spd_ref);
}

static void zgimbal_angle_solve(void)
{
    zgim.theta = ABS(zgim.ecd_max - zgim.moto.total_ecd) * (2*pi/(8191*19));
}

float rpm_set = 2000;//初始化时电机转速
float zgim_init_pos = 0.3f;//初始位置位置 5
float zgim_acc2spd = -80.0f;
uint32_t zgim_init_time = 50;//单侧行程校准停留时间(/ms)
float zgim_stop_acc = 2;//判断无抖动的加速度界限

void zgimbal_task(void const *argu)
{
    uint32_t task_wake_time = osKernelSysTick();
    for(;;)
    {
        taskENTER_CRITICAL();
        /* z轴加速度反馈处理 */
        LPFOfilter_cal(&LPFO_filter, (JY901_data.Gyro_a_z - 9.87353909f));
        zgim.zacc = Output_Limit(LPFO_filter.now_after_data, 25, -25);
        
        zgimbal_angle_solve();
        
        if (zgim.mode == ZGIM_MODE_PROTECT)
        {
            zgim.init_cnt = 0;
            zgim.acc_flag = 0;
            zgim.current = 0;
        }
        else
        {
            /* 行程校准 */
            if (zgim.mode != ZGIM_MODE_PROTECT && zgim.moto.msg_cnt >= 51 && zgim.init_cnt < 2 * zgim_init_time)//读取完电机初始位置
            {
                if (zgim.init_cnt < zgim_init_time)//正行程校准
                {
                    zgim.spd_pid.f_pid_reset(&zgim.spd_pid, 3, 0, 0);//去除I
                    zgim.pid_param.spd_fdb = zgim.moto.speed_rpm;
                    zgim.current = pid_calc(&zgim.spd_pid, zgim.pid_param.spd_fdb, rpm_set);//给初速度
                    if (zgim.pid_param.spd_fdb > 300)
                        zgim.acc_flag = 1;
                    if (zgim.acc_flag && ABS(zgim.pid_param.spd_fdb) < 30)
                        zgim.init_cnt++;
                    if (zgim.init_cnt == zgim_init_time)
                    {
                        zgim.acc_flag = 0;
                        zgim.ecd_max = zgim.moto.total_ecd;
                    }
                }
                else if (zgim.init_cnt < 2 * zgim_init_time)//反行程校准
                {
                    zgim.spd_pid.f_pid_reset(&zgim.spd_pid, 3, 0, 0);//去除I
                    zgim.pid_param.spd_fdb = zgim.moto.speed_rpm;
                    zgim.current = pid_calc(&zgim.spd_pid, zgim.pid_param.spd_fdb, -rpm_set);//给初速度
                    if (zgim.pid_param.spd_fdb < -300)
                        zgim.acc_flag = 1;
                    if (zgim.acc_flag && ABS(zgim.pid_param.spd_fdb) < 30)
                        zgim.init_cnt++;
                    if (zgim.init_cnt == 2 * zgim_init_time)
                    {
                        zgim.spd_pid.f_pid_reset(&zgim.spd_pid, 6, 0, 0);
                        zgim.ecd_min = zgim.moto.total_ecd;
                    }
                }
                zgim.last_total_ecd = zgim.moto.total_ecd;
            }
            else/* Z轴云台正常运行 */
            {
                if (zgim.mode == ZGIM_MODE_ECD)//单位置环
                    zgimbal_pid_cal((zgim.ecd_max + zgim.ecd_min)*zgim_init_pos);
                else if (zgim.mode == ZGIM_MODE_AUTO)//单速度环 + 位置限幅
                {
//                    if (ABS(zgim.zacc) <= zgim_stop_acc)//没有检测到抖动时
//                    {
//                        zgimbal_pid_cal((zgim.ecd_max + zgim.ecd_min)*zgim_init_pos);
//                    }
//                    else if (zgim.moto.total_ecd >= zgim.ecd_max)
//                    {
//                        
//                    }
//                    else if (zgim.moto.total_ecd <= zgim.ecd_min)
//                    {
//                        
//                    }
//                    else
//                    {
                        zgim.current = pid_calc(&zgim.spd_pid, zgim.pid_param.spd_fdb, zgim.zacc*zgim_acc2spd*sin(zgim.theta));//给初速度
                        if (zgim.moto.total_ecd >= zgim.ecd_max || zgim.moto.total_ecd <= zgim.ecd_min)
                            zgim.current = pid_calc(&zgim.spd_pid, zgim.pid_param.spd_fdb, 0);//给初速度
//                    }
                }
                else//弹簧或者弹簧减震模式
                {
                    zgim.current = 0;
                }
            }
        }
        /* 波形显示 */
        static uint8_t debug_i;
        debug_i++;
        if (debug_i == 1)
        {
            DataWave(&huart3);
            debug_i=0;
        }
        taskEXIT_CRITICAL();
        
        osSignalSet(zgimbal_send_task_t, ZGIMBAL_SEND_SIGN);
        osDelayUntil(&task_wake_time, 1);
    }
}

void zgimbal_send_task(void const *argu)
{
    osEvent event;
    for(;;)
    {
        event = osSignalWait(ZGIMBAL_SEND_SIGN, osWaitForever);
        if( event.status == osEventSignal)
        {
            can1_send_message(0x1ff, 0, 0, 0, zgim.current);
        }
    }
}

void zgimbal_reset_task(void const *argu)
{
//    osStatus status;
//    uint32_t task_wake_time = osKernelSysTick();
//    static uint32_t task_wake_cnt;
//    for (;;)
//    {
//        if (task_wake_cnt >= 10)
//        {
//            
//        }
//    
//    
//        osMutexWait(zgimbal_mutex_t, osWaitForever);

//        zgimbal_pid_cal((zgim.ecd_max + zgim.ecd_min)*zgim_init_pos);//位置速度串级PID控制复位
//        task_wake_cnt++;
//        if (task_wake_cnt >= 50)//每复位一次持续ms
//        {
//            osMutexRelease(zgimbal_mutex_t);//释放互斥量
//        }
//        osDelayUntil(&task_wake_time, 1);

//    }
}


static void zgimbal_rtos_init(void)
{
    /* 创建z轴云台线程 */
    osThreadDef(zgimablTask, zgimbal_task, osPriorityHigh, 0, 512);
    zgimbal_task_t = osThreadCreate(osThread(zgimablTask), NULL);

    /* 创建z轴云台电流发送线程 */
    osThreadDef(zgimablSendTask, zgimbal_send_task, osPriorityRealtime, 0, 512);
    zgimbal_send_task_t = osThreadCreate(osThread(zgimablSendTask), NULL);
    
    /* 创建z轴云台复位线程 */
    osThreadDef(zgimablResetTask, zgimbal_reset_task, osPriorityHigh, 0, 512);
    zgimbal_reset_task_t = osThreadCreate(osThread(zgimablResetTask), NULL);
    
    /* 创建互斥量 */
    osMutexDef(zgimbalMutex);
    zgimbal_mutex_t = osMutexCreate(osMutex(zgimbalMutex));
}

void zgimbal_init(void)
{
    zgimbal_rtos_init();
    PID_struct_init(&zgim.ecd_pid, POSITION_PID, 8000, 0, 0, 0,
                    0.2, 0, 0);
    PID_struct_init(&zgim.spd_pid, POSITION_PID, 16384, 11000, 10, 1000,
                    6, 0.1f, 0);
}


#endif

