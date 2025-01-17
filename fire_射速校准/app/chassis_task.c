#include "chassis_task.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "string.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "math_calcu.h"
#include "math.h"
#include "control_def.h"
#include "bsp_powerlimit.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "bsp_vision.h"

#include "func_generator.h"
extern TaskHandle_t can_msg_send_task_t;

chassis_t chassis;

static void chassis_mode_switch(void)
{
    /* 系统历史状态机 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;

    /* 单次触发使能标志 */
    static uint8_t spin_flag = 0;

    /* 底盘状态机 */
    switch( ctrl_mode )
    {
    case PROTECT_MODE:  //能量模式和保护模式下，底盘行为相同
    {
        chassis.mode = CHASSIS_MODE_PROTECT;
    }
    break;
    case REMOTER_MODE:
    {
        if( last_ctrl_mode != REMOTER_MODE )  //切入遥控模式，初始化底盘模式
            chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
        /* 底盘小陀螺模式 */
        if( rc.ch5 == 0 )   spin_flag = 1;
        if( rc.ch5 == 660 && spin_flag )
        {
            spin_flag = 0;
            if( chassis.mode == CHASSIS_MODE_REMOTER_ROTATE )
            {
                chassis.spin_dir = -chassis.spin_dir;  //小陀螺反向
                chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
            }
            else if( chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW )
                chassis.mode = CHASSIS_MODE_REMOTER_ROTATE;
        }
    }
    break;
    case VISION_MODE:
    {
        if( vision_mode == VISION_MODE_bENERGY || vision_mode == VISION_MODE_sENERGY )
        {
            chassis.mode = CHASSIS_MODE_PROTECT;  //进入底盘保护模式（击打能量机关）
        }
        else if( key_scan_clear(KEY_CHASSIS_ROTATE) )  //进入小陀螺模式
        {
            chassis.spin_dir = -chassis.spin_dir;
            chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
        }
        else if( key_scan_clear(KEY_CHASSIS_FIGHT) )  //进入迎敌模式
        {
            chassis.fight_dir = -chassis.fight_dir;
            chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
        }
        else
        {
            chassis.mode = chassis.mode;  //保持上个状态
        }
    }
    break;
    case KEYBOARD_MODE:
    {
        //键盘模式下：(跟随，陀螺，迎敌三种模式相互切换),(跟随与补给模式相互切换)
        /* 系统状态机状态切换，子状态机初始化 */
        if( last_ctrl_mode == VISION_MODE )  //从视觉模式进入键鼠模式
            chassis.mode = chassis.mode;  //保持上一个模式
        else if( last_ctrl_mode != KEYBOARD_MODE )  //从其他模式进入键鼠模式
            chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;  //初始化底盘为跟随模式
        /* 底盘模式切换 */
        switch( chassis.mode )
        {
        case CHASSIS_MODE_KEYBOARD_FOLLOW:  //键盘跟随模式下
        {
            keyboard_scan(KEY_SHOOT_HOUSE);
            if( key_scan_clear(KEY_CHASSIS_ROTATE) )  //进入小陀螺模式
            {
                chassis.spin_dir = -chassis.spin_dir;
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            }
            else if( key_scan_clear(KEY_CHASSIS_FIGHT) )  //进入迎敌模式
            {
                chassis.fight_dir = -chassis.fight_dir;
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
            }
            else if( FLAG_SHOOT_HOUSE == KEY_RUN )  //进入补给模式
            {
                chassis.mode = CHASSIS_MODE_KEYBOARD_SUPPLY;
            }
        }
        break;
        case CHASSIS_MODE_KEYBOARD_ROTATE:  //键盘陀螺模式下
        {
            if( key_scan_clear(KEY_CHASSIS_ROTATE) )  //恢复跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            else if( key_scan_clear(KEY_CHASSIS_FIGHT) )  //进入迎敌模式
            {
                chassis.fight_dir = -chassis.fight_dir;
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
            }
        }
        break;
        case CHASSIS_MODE_KEYBOARD_FIGHT:  //键盘迎敌模式下
        {
            if( key_scan_clear(KEY_CHASSIS_FIGHT) )  //恢复跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            else if( key_scan_clear(KEY_CHASSIS_ROTATE) )  //进入小陀螺模式
            {
                chassis.spin_dir = -chassis.spin_dir;
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            }
        }
        break;
        case CHASSIS_MODE_KEYBOARD_SUPPLY:  //键盘补给模式下
        {
            keyboard_scan(KEY_SHOOT_HOUSE);
            if( FLAG_SHOOT_HOUSE == KEY_END )  //进入跟随模式
            {
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            }
        }
        break;
        default:
        {
            chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
        }
        break;
        }
    }
    break;
    default:
        break;
    }
    /* 系统历史状态更新 */
    last_ctrl_mode = ctrl_mode;
}


/**
  * @brief chassis_task
  * @param
  * @attention
	* @note
  */
void chassis_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        taskENTER_CRITICAL();
        chassis_mode_switch();
        switch( chassis.mode )
        {
        case CHASSIS_MODE_PROTECT:  //底盘保护模式
        {
            chassis.spd_input.vx = 0;
            chassis.spd_input.vy = 0;
            chassis.spd_input.vw = 0;
        }
        break;
        case CHASSIS_MODE_REMOTER_FOLLOW:  //底盘遥控跟随模式
        case CHASSIS_MODE_REMOTER_ROTATE:  //底盘遥控陀螺模式
        {
            chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
            chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
            chassis.angle_error  =  chassis.position_error * (2.0f*PI/8191.0f);
            chassis.spd_input.vx = 1.0f*(float)(rc.ch4*scale.ch4 * cos(chassis.angle_error) + (-1.0f)*rc.ch3*scale.ch3 * sin(chassis.angle_error));
            chassis.spd_input.vy = 1.0f*(float)(rc.ch4*scale.ch4 * sin(chassis.angle_error) - (-1.0f)*rc.ch3*scale.ch3 * cos(chassis.angle_error));
            if( chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW )
                chassis.spd_input.vw = -1.0f*pid_calc(&pid_chassis_angle,chassis.position_ref,chassis.position_ref + chassis.position_error);
            else if( chassis.mode == CHASSIS_MODE_REMOTER_ROTATE )
            {
                if( chassis.wheel_max <= 6000 )
                    chassis.spd_input.vw = chassis.spin_dir * chassis.wheel_max;
                else    chassis.spd_input.vw = chassis.spin_dir * 6000;
            }
        }
        break;
        case CHASSIS_MODE_KEYBOARD_FOLLOW:  //底盘键盘跟随模式
        case CHASSIS_MODE_KEYBOARD_ROTATE:  //底盘键盘陀螺模式
        case CHASSIS_MODE_KEYBOARD_FIGHT:   //底盘键盘迎敌模式
        {
            /* 键盘输入斜坡处理 */
            chassis_ramp();
            /* 底盘平移速度设定计算 */
            if( chassis.mode == CHASSIS_MODE_KEYBOARD_FIGHT )
            {
                chassis.position_ref = (GIMBAL_YAW_CENTER_OFFSET + chassis.fight_dir * GIMBAL_YAW_BETWEEN_ECD) % 8191;
                chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
                chassis.angle_error =  chassis.position_error * (2.0f*PI/8191.0f);
                chassis.spd_input.vx = 1.0f *(float)(chassis_x_ramp.out * cos(chassis.angle_error + chassis.fight_dir * FIGHT_OFFSET_ERR) + (-1.0f)*chassis_y_ramp.out * sin(chassis.angle_error + chassis.fight_dir * FIGHT_OFFSET_ERR));
                chassis.spd_input.vy = 1.0f *(float)(chassis_x_ramp.out * sin(chassis.angle_error + chassis.fight_dir * FIGHT_OFFSET_ERR) - (-1.0f)*chassis_y_ramp.out * cos(chassis.angle_error + chassis.fight_dir * FIGHT_OFFSET_ERR));
            }
            else if( chassis.mode == CHASSIS_MODE_KEYBOARD_FOLLOW || chassis.mode == CHASSIS_MODE_KEYBOARD_ROTATE )
            {
                chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
                chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
                chassis.angle_error =  chassis.position_error * (2.0f*PI/8191.0f);
                chassis.spd_input.vx = 1.0f *(float)(chassis_x_ramp.out * cos(chassis.angle_error) + (-1.0f)*chassis_y_ramp.out * sin(chassis.angle_error));
                chassis.spd_input.vy = 1.0f *(float)(chassis_x_ramp.out * sin(chassis.angle_error) - (-1.0f)*chassis_y_ramp.out * cos(chassis.angle_error));
            }
            /* 底盘自旋速度设定计算 */
            if( chassis.mode == CHASSIS_MODE_KEYBOARD_FOLLOW || chassis.mode == CHASSIS_MODE_KEYBOARD_FIGHT )
                chassis.spd_input.vw = -1.0f*pid_calc(&pid_chassis_angle,chassis.position_ref, chassis.position_ref + chassis.position_error);
            else if( chassis.mode == CHASSIS_MODE_KEYBOARD_ROTATE )
            {
                if( chassis.wheel_max <= 6000 )
                    chassis.spd_input.vw = chassis.spin_dir * chassis.wheel_max;
                else    chassis.spd_input.vw = chassis.spin_dir * 6000;
            }
        }
        break;
        case CHASSIS_MODE_KEYBOARD_SUPPLY:  //底盘补给模式
        {
            /* 键盘输入斜坡处理 */
            chassis_ramp();
            /* 角度误差解算 */
            chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
            chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
            chassis.angle_error =  chassis.position_error * (2.0f*PI/8191.0f);
            /* 麦轮底盘控制 */
            chassis.spd_input.vx = 1.0f *(float)(chassis_x_ramp.out * cos(chassis.angle_error) + (-1.0f)*chassis_y_ramp.out * sin(chassis.angle_error));
            chassis.spd_input.vy = -(1.0f *(float)(chassis_x_ramp.out * sin(chassis.angle_error) - (-1.0f)*chassis_y_ramp.out * cos(chassis.angle_error))); //左右反向
            chassis.spd_input.vw = -1.0f*pid_calc(&pid_chassis_angle,chassis.position_ref, chassis.position_ref + chassis.position_error);
        }
        break;
        default:
            break;
        }
        /* 底盘速度分配 */
        chassis_spd_distribution();
        /* 电机电流 PID 计算 */
        for (int i = 0; i < 4; i++)
        {
            chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
            chassis.current[i] = (int16_t)pid_calc(&pid_chassis_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
        }
        /* 功率及超级电容控制 */
        //PowerParam_Update();
        SuperCap_Control();
        //Power_Control(chassis.current);
        /* 波形显示 */
        static uint8_t debug_i;
        debug_i++;
        if( debug_i == 10  )  //&& ctrl_mode != PROTECT_MODE
        {
            //FGT_sin_cal(&test_sin);
            DataWave(&huart3);
            debug_i=0;
        }
        /* 发送电流，控制电机 */
//        memcpy(motor_cur.chassis_cur,chassis.current, sizeof(chassis.current));
        osSignalSet(can_msg_send_task_t, CHASSIS_MOTOR_MSG_SEND);
        taskEXIT_CRITICAL();

        osDelayUntil(&mode_wake_time, CHASSIS_PERIOD);
    }
}

void chassis_init()
{
    for(uint8_t i=0; i<4; i++)
    {
        PID_struct_init(&pid_chassis_spd[i], POSITION_PID, 10000, 5000, 0, 0,
                        7.0f, 0.0f, 0.0f);
    }
    PID_struct_init(&pid_chassis_angle, POSITION_PID, 6000, 0, 0, 0,
                    7.0f, 0.0f, 0.0f);
    scale.ch3 = RC_CH3_SCALE;
    scale.ch4 = RC_CH4_SCALE;
    chassis.mode = CHASSIS_MODE_PROTECT;
    chassis.mode = chassis.mode;
    chassis.fight_dir = 1;
    chassis.spin_dir  = 1;

    chassis.wheel_max = 6000;  /* 关掉功率控制时需要初始化 */
}


/**
	* @brief 麦轮解算函数
	* @param input : ?=+vx(mm/s)  ?=+vy(mm/s)  ccw=+vw(deg/s)
	*        output: every wheel speed(rpm)
	* @note  1=FL 2=FR 3=BL 4=BR
	*/
void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
    int16_t wheel_rpm[4];
    wheel_rpm[0] =  vx + vy - vw;
    wheel_rpm[1] = -vx + vy - vw;
    wheel_rpm[2] =  vx - vy - vw;
    wheel_rpm[3] = -vx - vy - vw;

    memcpy(speed, wheel_rpm, 4 * sizeof(int16_t));
}

/**
	* @brief          扭腰模式计算底盘角度设定值
	* @author
	* @param[in]      void
	* @retval         void
	*/
void sparate_move(void)
{
    if(ABS(chassis.position_error) <= 300 )
        chassis.position_ref = moto_yaw.ecd;
    if(chassis.position_error > 300 )
    {
        chassis.position_ref = gimbal.yaw_center_offset - 300;
        if(chassis.position_ref > 8191)
            chassis.position_ref = chassis.position_ref - 8191;
    }
    if(chassis.position_error < -300 )
    {
        chassis.position_ref = gimbal.yaw_center_offset + 300;
        if(chassis.position_ref < 0)
            chassis.position_ref = chassis.position_ref + 8191;
    }
    chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
}

/**
	* @brief          底盘速度分配函数，输入速度超过最大轮速时，将输入的速度按比例分配到三个轴向上
	* @author
	* @param[in]      void
	* @retval         void
	*/
void chassis_spd_distribution(void)
{
    float  wheel_spd_input_buf[4];
    float  wheel_spd_total = 0;  //总轮速
    float  distribution_temp = 1.0f;	//限制比例

    /* 麦轮逆运动学原设定速度解算 */
    mecanum_calc(chassis.spd_input.vx,chassis.spd_input.vy, chassis.spd_input.vw, chassis.wheel_spd_input);

    /* 计算总速度 */
    for(int i=0; i<4; i++)
    {
        wheel_spd_input_buf[i]=ABS(chassis.wheel_spd_input[i]);
        wheel_spd_total += wheel_spd_input_buf[i];
    }
    /* 计算速度减小比例系数 */
    if(wheel_spd_total > (chassis.wheel_max * 4.0f))  //判断最大速度是否超额
    {
        distribution_temp = wheel_spd_total / (chassis.wheel_max * 4.0f); //超额速度除最大速度得分配比例
    }
    /* 速度重分配 并 限幅 */
    for(uint8_t j=0; j<4; j++)
    {
        chassis.wheel_spd_ref[j] = chassis.wheel_spd_input[j] / distribution_temp ;
        chassis.wheel_spd_ref[j] = data_limit(chassis.wheel_spd_ref[j], 8500, -8500);  //电机转速最高到8900
    }
    /* 麦轮正运动学处理后设定速度解算 */
    chassis.spd_ref.vx = (+chassis.wheel_spd_ref[0] - chassis.wheel_spd_ref[1] + chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;
    chassis.spd_ref.vy = (+chassis.wheel_spd_ref[0] + chassis.wheel_spd_ref[1] - chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;
    chassis.spd_ref.vw = (-chassis.wheel_spd_ref[0] - chassis.wheel_spd_ref[1] - chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;

    chassis.spd_error = chassis.spd_ref.vx + chassis.spd_ref.vy + chassis.spd_ref.vw
                        - chassis.spd_fdb.vx - chassis.spd_fdb.vy - chassis.spd_fdb.vw;

    /* 麦轮正运动学反馈速度解算 */
    chassis.spd_fdb.vx = (+moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm + moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;
    chassis.spd_fdb.vy = (+moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;
    chassis.spd_fdb.vw = (-moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;

    //计算目标速度和反馈速度差值，大于一定值用超级电容放电

//    mecanum_calc(chassis.spd_input.vx,chassis.spd_input.vy, chassis.spd_input.vw, chassis.wheel_spd_input);
//    for(int j=0; j<4; j++)
//    {
//        chassis.wheel_spd_ref[j] = chassis.wheel_spd_input[j];
//        chassis.wheel_spd_ref[j] = data_limit(chassis.wheel_spd_ref[j],8500,-8500);  //电机转速最高到8900
//    }

}

/**
  * @brief          底盘斜坡启动函数，通过斜坡函数映射目标速度
  * @author
  * @param[in]      void
  * @retval         void
  */
void chassis_ramp(void)
{
    if(rc.kb.bit.W)
    {
        ramp_calc(&chassis_x_ramp,1.0f,chassis.keyboard_input, chassis.wheel_max, 0.0f);
    }
    else if(rc.kb.bit.S)
    {
        ramp_calc(&chassis_x_ramp,1.0f,-chassis.keyboard_input, 0.0f, -chassis.wheel_max);
    }
    else
    {
        if(chassis_x_ramp.out > 0)
        {
            ramp_calc(&chassis_x_ramp,1.0f,-chassis.keyboard_input,chassis.wheel_max, 0.0f);
        }
        else if(chassis_x_ramp.out < 0)
        {
            ramp_calc(&chassis_x_ramp,1.0f,chassis.keyboard_input, 0.0f, -chassis.wheel_max);
        }
    }
    if(rc.kb.bit.D)
    {
        ramp_calc(&chassis_y_ramp,1.0f,chassis.keyboard_input, chassis.wheel_max, 0.0f);
    }
    else if(rc.kb.bit.A)
    {
        ramp_calc(&chassis_y_ramp,1.0f,-chassis.keyboard_input, 0.0f, -chassis.wheel_max);
    }
    else
    {
        if(chassis_y_ramp.out > 0)
        {
            ramp_calc(&chassis_y_ramp,1.0f,-chassis.keyboard_input, chassis.wheel_max, 0.0f);
        }
        else if(chassis_y_ramp.out < 0)
        {
            ramp_calc(&chassis_y_ramp,1.0f,chassis.keyboard_input, 0.0f, -chassis.wheel_max);
        }
    }
}

/**
  * @brief          底盘S型启动函数，通过S型函数映射目标速度
  * @author
  * @param[in]      void
  * @retval         void
  */
void chassis_sigmoid(void)
{
    static float x_speed_sigmoid,y_speed_sigmoid;
    static float x_input,y_input;
    static float x_sigmoid,y_sigmoid;
    if(rc.kb.bit.W) 					x_input += SIGMOID_PERIOD;
    else if(rc.kb.bit.S)			x_input -= SIGMOID_PERIOD;
    else
    {
        if(x_input > 0)					x_input -= SIGMOID_PERIOD;
        else if(x_input < 0)		x_input += SIGMOID_PERIOD;
    }
    if(rc.kb.bit.D) 					y_input += SIGMOID_PERIOD;
    else if(rc.kb.bit.A)			y_input -= SIGMOID_PERIOD;
    else
    {
        if(y_input > 0)					y_input -= SIGMOID_PERIOD;
        else if(y_input < 0)		y_input += SIGMOID_PERIOD;
    }

    if(x_input >= (2*SIGMOID_MAX))					x_input=  (2*SIGMOID_MAX);
    else if(x_input <= -(2*SIGMOID_MAX))		x_input= -(2*SIGMOID_MAX);

    if(y_input >= (2*SIGMOID_MAX))					y_input=  (2*SIGMOID_MAX);
    else if(y_input <= -(2*SIGMOID_MAX))		y_input= -(2*SIGMOID_MAX);

    if(x_input <= ABS(SIGMOID_PERIOD) && x_input >= -ABS(SIGMOID_PERIOD))
        x_sigmoid = 0;
    else if(x_input >= ABS(SIGMOID_PERIOD))
        x_sigmoid = Sigmoid_function(x_input);
    else if(x_input <= -ABS(SIGMOID_PERIOD))
        x_sigmoid = -Sigmoid_function(x_input);

    if(y_input <= ABS(SIGMOID_PERIOD) && y_input >= -ABS(SIGMOID_PERIOD))
        y_sigmoid = 0;
    else if(y_input >= ABS(SIGMOID_PERIOD))
        y_sigmoid = Sigmoid_function(y_input);
    else if(y_input <= -ABS(SIGMOID_PERIOD))
        y_sigmoid = -Sigmoid_function(y_input);

    x_speed_sigmoid = x_sigmoid * chassis.wheel_max;
    y_speed_sigmoid = y_sigmoid * chassis.wheel_max;

    chassis.spd_input.vx = (float)(x_speed_sigmoid * cos(chassis.angle_error) + (-1.0f)*y_speed_sigmoid * sin(chassis.angle_error));
    chassis.spd_input.vy = (float)(x_speed_sigmoid * sin(chassis.angle_error) - (-1.0f)*y_speed_sigmoid * cos(chassis.angle_error));
}

