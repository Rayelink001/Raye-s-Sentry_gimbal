#include "gimbal_task.h"

#include "cmsis_os.h"       //OS API

#include "can_receive.h"    //电机
#include "DT16_control.h"   //遥控器
#include "vision_task.h"    //视觉
#include "pid.h"            //PID

#include "wt16c.h"          //外置陀螺仪

#include "monitor_task.h"   //监控
#include "shoot_task.h"

#include "cover.h"  //弹仓盖
#include "led_control_task.h"   //指示灯


//云台机构 初始化
void gimbal_init(void);

//云台机构 控制量设置            
void gimbal_control_set(void);
//云台机构 手控角度控制
void gimbal_manual_angle_limit(fp32 yaw_add, fp32 pitch_add);
//IMU反馈控制量设置 
void gimbal_vision_angle_limit(void);
//云台机构 数据反馈                    
void gimbal_data_feedback(void);
//云台闭环控制      
void gimbal_control_loop(void); 
//编码器反馈闭环控制
void gimbal_motor_imu_loop(gimbal_motor_t *motor, pid_data_t *pid_speed, pid_data_t *pid_pos);
//陀螺仪反馈闭环控制
void gimbal_motor_encode_loop(gimbal_motor_t *motor, pid_data_t *pid_speed, pid_data_t *pid_pos);
//云台控制电流发送控制
void gimbal_control_current_sent(void);
//获取相对机械角度
fp32 motor_ecd_to_degree(int16_t ecd, int16_t offset_ecd);
//弹仓控制
void cover_control(void);

//云台机构控制全局数据
gimbal_control_data_t gimbal_control_data =
{
    .control_mode = GIMBAL_CONTROL_MODE_FREE,
    .last_control_mode = GIMBAL_CONTROL_MODE_FREE,

    .yaw_motor = 
    {
        .encode_speed_pid = 
        {
            .Kp = 0.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .max_out = 25000.0f,
            .max_iout = 5000.0f
        },
        .encode_pos_pid =
        {
            .Kp = 10.0f,
            .Ki = 0.0f,
            .Kd = 4.0f,
            .max_out = 20.0f,
            .max_iout = 0.0f
        },
        .vision_encode_pos_pid =
        {
            .Kp = 0.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .max_out = 0.0f,
            .max_iout = 0.0f
        },
        .imu_speed_pid =
        {
            .Kp = 45.0f,    //30
            .Ki = 0.1f,
            .Kd = 0.0f,
            .max_out = 25000.0f,
            .max_iout = 10000.0f
        },
        .imu_pos_pid =
        {
            .Kp = 15.0f,
            .Ki = 1.0f,
            .Kd = 0.0f,
            .max_out = 1000.0f,
            .max_iout = 0.0
        },
        .vision_imu_pos_pid =
        {
            .Kp = 1.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .max_out = 50.0f,
            .max_iout = 0.0f
        },
        .offset_ecd = 2728,
    },

    .pitch_motor =
    {
        .encode_speed_pid = 
        {
            .Kp = 80.0f,
            .Ki = 10.0f,
            .Kd = 0.0f,
            .max_out = 25000.0f,
            .max_iout = 7000.0f
        },
        .encode_pos_pid =
        {
            .Kp = 10.0f,
            .Ki = 0.0f,
            .Kd = 4.0f,
            .max_out = 20.0f,
            .max_iout = 0.0f
        },
        .vision_encode_pos_pid =
        {
            .Kp = 0.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .max_out = 0.0f,
            .max_iout = 0.0f
        },
        .imu_speed_pid =
        {
            .Kp = 118.0f,
            .Ki = 0.15f,
            .Kd = 0.1f,
            .max_out = 25000.0f,
            .max_iout = 12000.0f
        },
        .imu_pos_pid =
        {
            .Kp = 18.0f,
            .Ki = 0.0f,
            .Kd = 2.0f,
            .max_out = 1000.0f,
            .max_iout = 0.0f
        },
        .vision_imu_pos_pid =
        {
            .Kp = 18.0f,
            .Ki = 0.0f,
            .Kd = 2.0f,
            .max_out = 50.0f,
            .max_iout = 0.0f
        },
        .offset_ecd = 50,
        .offset_angle = 0.0f,
        .max_ecd_angle = 520,
        .min_ecd_angle = -360
    }
}; 

//使用Ozone调试速度环的时候需要一个速度的调试全局变量
#if yaw_pid_speed_debug_mode || pitch_pid_speed_debug_mode
    fp32 debug_set_speed = DEBUG_SET_SPEED;  
#endif

//云台主任务
void gimbal_task(void const *pvParameters)  
{
    vTaskDelay(10);                 //等待一下系统加载
    gimbal_init();

    cover_init();   //弹仓盖初始化

    while(1)
    {
        gimbal_control_set();
        cover_control();    //弹仓控制
        gimbal_data_feedback();               
        gimbal_control_loop();          
        gimbal_control_current_sent();

        
        vTaskDelay(1);          //任务频率控制
    }
}

void gimbal_init(void) //云台初始化  
{ 
    //遥控器数据指针绑定
    gimbal_control_data.gimbal_RC = get_rc_ctrl_point();
    //电机数据指针绑定
    gimbal_control_data.yaw_motor.motor_data_ref = get_yaw_motor_measure_point();
	gimbal_control_data.pitch_motor.motor_data_ref = get_pitch_motor_measure_point();
    //陀螺仪数据指针绑定
	gimbal_control_data.ins_data = get_wt61c_data_point();
    gimbal_control_data.euler_angle = get_euler_angle_real_point();
    //视觉数据指针绑定
    gimbal_control_data.vision_data= get_vision_data_point();

    /* PID初始化 */
    pid_init_gimbal(&gimbal_control_data.yaw_motor.encode_speed_pid);
    pid_init_gimbal(&gimbal_control_data.pitch_motor.encode_speed_pid);
    pid_init_gimbal(&gimbal_control_data.yaw_motor.encode_pos_pid);
    pid_init_gimbal(&gimbal_control_data.pitch_motor.encode_pos_pid);
    pid_init_gimbal(&gimbal_control_data.yaw_motor.vision_encode_pos_pid);
    pid_init_gimbal(&gimbal_control_data.pitch_motor.vision_encode_pos_pid);
    pid_init_gimbal(&gimbal_control_data.yaw_motor.imu_speed_pid);
    pid_init_gimbal(&gimbal_control_data.pitch_motor.imu_speed_pid);
    pid_init_gimbal(&gimbal_control_data.yaw_motor.imu_pos_pid);
    pid_init_gimbal(&gimbal_control_data.pitch_motor.imu_pos_pid);
    pid_init_gimbal(&gimbal_control_data.yaw_motor.vision_imu_pos_pid);
    pid_init_gimbal(&gimbal_control_data.pitch_motor.vision_imu_pos_pid);

    //初始化 最大、最小机械角度
    gimbal_control_data.pitch_motor.max_ecd_to_degree_angle = motor_ecd_to_degree(gimbal_control_data.pitch_motor.max_ecd_angle, gimbal_control_data.pitch_motor.offset_ecd);
    gimbal_control_data.pitch_motor.min_ecd_to_degree_angle = motor_ecd_to_degree(gimbal_control_data.pitch_motor.min_ecd_angle, gimbal_control_data.pitch_motor.offset_ecd);
    
    gimbal_control_data.last_key = gimbal_control_data.gimbal_RC->key.v;
    //云台数据更新
    gimbal_data_feedback();
}

//云台数据反馈
void gimbal_data_feedback(void) 
{
    //电机角度 编码器反馈
    gimbal_control_data.yaw_motor.encode_pos_ref = gimbal_control_data.yaw_motor.motor_data_ref->ecd;
    gimbal_control_data.pitch_motor.encode_pos_ref = gimbal_control_data.pitch_motor.motor_data_ref->ecd;

    //YAW过零 编码器
    if((gimbal_control_data.yaw_motor.encode_pos_set - gimbal_control_data.yaw_motor.encode_pos_ref) > HALF_ECD_RANGE)
    {
        gimbal_control_data.yaw_motor.encode_pos_ref += ECD_RANGE;
    }
    else if((gimbal_control_data.yaw_motor.encode_pos_set - gimbal_control_data.yaw_motor.encode_pos_ref) < -HALF_ECD_RANGE)
    {    
        gimbal_control_data.yaw_motor.encode_pos_ref -= ECD_RANGE;
    }
    
    //pitch过零 编码器
    if((gimbal_control_data.pitch_motor.encode_pos_set  -  gimbal_control_data.pitch_motor.encode_pos_ref) > HALF_ECD_RANGE)
    {
        gimbal_control_data.pitch_motor.encode_pos_ref += ECD_RANGE;
    }
    else if((gimbal_control_data.pitch_motor.encode_pos_set - gimbal_control_data.pitch_motor.encode_pos_ref) < -HALF_ECD_RANGE)
    {    
        gimbal_control_data.pitch_motor.encode_pos_ref -= ECD_RANGE;
    }

    //电机角速度 编码器反馈
    gimbal_control_data.yaw_motor.encode_speed_ref = gimbal_control_data.yaw_motor.motor_data_ref->speed_rpm;
    gimbal_control_data.pitch_motor.encode_speed_ref =  gimbal_control_data.pitch_motor.motor_data_ref->speed_rpm;
    //更新陀螺仪角速度数据
    gimbal_control_data.yaw_motor.imu_speed_ref = gimbal_control_data.ins_data->gyro[2].f;
    gimbal_control_data.pitch_motor.imu_speed_ref = gimbal_control_data.ins_data->gyro[0].f;
    //更新陀螺仪角度数据
    gimbal_control_data.yaw_motor.imu_pos_ref = gimbal_control_data.euler_angle->real_yaw;
    gimbal_control_data.pitch_motor.imu_pos_ref = gimbal_control_data.euler_angle->real_pitch;

    gimbal_control_data.last_key = gimbal_control_data.gimbal_RC->key.v;
}

//云台机构 状态量设置
void gimbal_control_set(void)
{
    //模式更新
    gimbal_control_data.last_control_mode =  gimbal_control_data.control_mode;   
    //遥控器 右拨杆 控制模式选择
    switch (gimbal_control_data.gimbal_RC->rc.s[RC_SWITCH_RIGHT])
    {
        case RC_SW_UP://  右上档：视觉控制模式 以IMU为反馈的PID闭环控制模式
            gimbal_control_data.control_mode = GIMBAL_CONTROL_MODE_VISION_IMU;
            break;

        case RC_SW_MID://  右中档：以IMU为反馈的PID闭环控制模式 
            gimbal_control_data.control_mode = GIMBAL_CONTROL_MODE_IMU;
            break;

        case RC_SW_DOWN:// 右下档：云台无力模式
            gimbal_control_data.control_mode = GIMBAL_CONTROL_MODE_FREE;
            break;
        
        default: 
            break;
    }

    //如果是手动控制则获取欧拉角增量，若是视觉控制则直接获得欧拉角量
    switch (gimbal_control_data.control_mode)
    {
        case GIMBAL_CONTROL_MODE_IMU:
        case GIMBAL_CONTROL_MODE_ENCODE:
        {
            //杆量转为电机角度增量
            fp32 add_yaw_angle = 0.0f, add_pitch_angle = 0.0f;
            int16_t yaw_channel = 0, pitch_channel = 0;

            //摇杆死区判断
            rc_deadband_limit(gimbal_control_data.gimbal_RC->rc.ch[RIGHT_CHANNAL_X],yaw_channel,GIMBAL_RC_DEADLINE)
            rc_deadband_limit(gimbal_control_data.gimbal_RC->rc.ch[RIGHT_CHANNAL_Y],pitch_channel,GIMBAL_RC_DEADLINE)
            //遥控量映射转换 + 键鼠控制
            add_yaw_angle = yaw_channel * YAW_RC_CEN + gimbal_control_data.gimbal_RC->mouse.x * YAW_MOUSE_RC_CEN;
            add_pitch_angle = pitch_channel * PIT_RC_CEN  + gimbal_control_data.gimbal_RC->mouse.y * PIT_MOUSE_RC_CEN;
            //控制角度限制
            gimbal_manual_angle_limit(add_yaw_angle, add_pitch_angle);
            break;
        }
            
        case GIMBAL_CONTROL_MODE_VISION_IMU:
        case GIMBAL_CONTROL_MODE_VISION_ENCODE:
        {
            //获取视觉增量
            // fp32 add_yaw_angle = 0.0f, add_pitch_angle = 0.0f;

            // add_yaw_angle = gimbal_control_data.vision_data->yaw_angle.f;
            // add_pitch_angle =  gimbal_control_data.vision_data->pitch_angle.f;
            gimbal_control_data.yaw_motor.imu_pos_set = gimbal_control_data.yaw_motor.imu_pos_ref - gimbal_control_data.vision_data->yaw_angle.f;
            gimbal_control_data.pitch_motor.imu_pos_set =  gimbal_control_data.vision_data->pitch_angle.f;
            gimbal_vision_angle_limit();//对控制量进行限制;
            break;
        }

        default:
            break;
    }
    
    //若发生模式切换 进行复位
    if(gimbal_control_data.control_mode != gimbal_control_data.last_control_mode)
    {
        gimbal_control_data.yaw_motor.imu_pos_set =  gimbal_control_data.yaw_motor.imu_pos_ref;
        gimbal_control_data.pitch_motor.imu_pos_set = 0.0f;
    }

}

//云台机构 手动控制 角度限位
void gimbal_manual_angle_limit(fp32 yaw_add, fp32 pitch_add)
{
    switch (gimbal_control_data.control_mode)
    {
        case GIMBAL_CONTROL_MODE_IMU:
        {
            //线性无限增减
            gimbal_control_data.yaw_motor.imu_pos_set += yaw_add;
            
            //限位控制
            fp32 bias_angle;          //pitch误差角度
            fp32 pitch_encode_angle;  //当前pitch编码器角度
            bias_angle = gimbal_control_data.pitch_motor.imu_pos_set - gimbal_control_data.pitch_motor.imu_pos_ref;                                 //误差角度 = imu目标角度 - imu反馈角度
            pitch_encode_angle = motor_ecd_to_degree(gimbal_control_data.pitch_motor.encode_pos_ref, gimbal_control_data.pitch_motor.offset_ecd);   //获取电机当前相对机械角度

            //若 当前相对机械角度 + 误差角度 + 增量角度 > 最大相对机械角度
            if(pitch_encode_angle + bias_angle + pitch_add > gimbal_control_data.pitch_motor.max_ecd_to_degree_angle)
            {
                //且增量角度 > 0 即还在增加, 则对增量进行调整
                if(pitch_add > 0.0f)
                {
                    pitch_add = gimbal_control_data.pitch_motor.max_ecd_to_degree_angle - pitch_encode_angle - bias_angle;
                }

            }
            //若 当前相对机械角度 + 误差角度 + 增量角度 < 最小相对机械角度
            else if(pitch_encode_angle + bias_angle + pitch_add < gimbal_control_data.pitch_motor.min_ecd_to_degree_angle)
            {
                //且增量角度 < 0 即还在减小, 则对增量进行调整
                if(pitch_add < 0.0f)
                {
                    pitch_add = gimbal_control_data.pitch_motor.min_ecd_to_degree_angle -  pitch_encode_angle - bias_angle;
                }

            }

            gimbal_control_data.pitch_motor.imu_pos_set += pitch_add;
            
            break;
        }
        case GIMBAL_CONTROL_MODE_ENCODE:
        {
            gimbal_control_data.yaw_motor.encode_pos_set += yaw_add;
            gimbal_control_data.pitch_motor.encode_pos_set += pitch_add;

            //直接进行相对角度限制
            if(gimbal_control_data.pitch_motor.encode_pos_set > gimbal_control_data.pitch_motor.max_ecd_angle)
            {
                gimbal_control_data.pitch_motor.encode_pos_set = gimbal_control_data.pitch_motor.max_ecd_angle;
            }
            else if(gimbal_control_data.pitch_motor.encode_pos_set < gimbal_control_data.pitch_motor.min_ecd_angle)
            {
                gimbal_control_data.pitch_motor.encode_pos_set = gimbal_control_data.pitch_motor.min_ecd_angle;
            }

            break;
        }    
        default:
            break;
    }
}

//云台机构 视觉控制 角度限位
void gimbal_vision_angle_limit(void)
{
    switch (gimbal_control_data.control_mode)
    {
        case GIMBAL_CONTROL_MODE_VISION_IMU:
            
            if(gimbal_control_data.pitch_motor.encode_pos_ref >= gimbal_control_data.pitch_motor.max_ecd_angle)
            {
                gimbal_control_data.pitch_motor.imu_pos_set = gimbal_control_data.pitch_motor.imu_pos_ref;
            }
            else if(gimbal_control_data.pitch_motor.encode_pos_ref <= gimbal_control_data.pitch_motor.min_ecd_angle)
            {
                gimbal_control_data.pitch_motor.imu_pos_set = gimbal_control_data.pitch_motor.imu_pos_ref;
            }

            break;
        case GIMBAL_CONTROL_MODE_VISION_ENCODE:
            /* code */
            break;
        default:
            break;
    }
}

/* 云台闭环控制 */
void gimbal_control_loop(void) 
{

    //除无力开环模式外，进行闭环控制
    if(gimbal_control_data.control_mode == GIMBAL_CONTROL_MODE_FREE)
    {
        gimbal_control_data.yaw_motor.given_current = 0;
        gimbal_control_data.pitch_motor.given_current = 0;
        return;
    }

    //设置电机闭环模式 如有需要在此处进行电流反转;
    switch (gimbal_control_data.control_mode)
    {
        case GIMBAL_CONTROL_MODE_IMU:
        {
            gimbal_motor_imu_loop(&gimbal_control_data.yaw_motor, &gimbal_control_data.yaw_motor.imu_speed_pid, &gimbal_control_data.yaw_motor.imu_pos_pid);
            gimbal_motor_imu_loop(&gimbal_control_data.pitch_motor, &gimbal_control_data.pitch_motor.imu_speed_pid, &gimbal_control_data.pitch_motor.imu_pos_pid); 
            break;
        }

        case GIMBAL_CONTROL_MODE_ENCODE:
        {
            gimbal_motor_encode_loop(&gimbal_control_data.yaw_motor, &gimbal_control_data.yaw_motor.encode_speed_pid, &gimbal_control_data.yaw_motor.encode_pos_pid);
            gimbal_motor_encode_loop(&gimbal_control_data.pitch_motor, &gimbal_control_data.pitch_motor.encode_speed_pid, &gimbal_control_data.pitch_motor.encode_pos_pid); 
            break;
        }

        case GIMBAL_CONTROL_MODE_VISION_IMU:
        {
            gimbal_motor_imu_loop(&gimbal_control_data.yaw_motor, &gimbal_control_data.yaw_motor.imu_speed_pid, &gimbal_control_data.yaw_motor.vision_imu_pos_pid);
            gimbal_motor_imu_loop(&gimbal_control_data.pitch_motor, &gimbal_control_data.pitch_motor.imu_speed_pid, &gimbal_control_data.pitch_motor.vision_imu_pos_pid);
            break;
        }

        case GIMBAL_CONTROL_MODE_VISION_ENCODE:
        {
            gimbal_motor_encode_loop(&gimbal_control_data.yaw_motor, &gimbal_control_data.yaw_motor.encode_speed_pid, &gimbal_control_data.yaw_motor.encode_pos_pid);
            gimbal_motor_encode_loop(&gimbal_control_data.pitch_motor, &gimbal_control_data.pitch_motor.encode_speed_pid, &gimbal_control_data.pitch_motor.encode_pos_pid);
            break;
        }
        
        default:
            break;
    }
}

//云台机构 以IMU为反馈的闭环控制方式
void gimbal_motor_imu_loop(gimbal_motor_t *motor, pid_data_t *pid_speed, pid_data_t *pid_pos)
{
     #if yaw_pid_speed_debug_mode || pitch_pid_speed_debug_mode //速度环debug   
        motor->current_set = pid_calc(pid_speed, motor->imu_speed_ref, debug_set_speed);    //角速度转电压
    #else //串级角度环
        motor->imu_speed_set = pid_calc(pid_pos, motor->imu_pos_ref, motor->imu_pos_set);   //位置转角速度
        motor->current_set = pid_calc(pid_speed, motor->imu_speed_ref, motor->imu_speed_set);  //角速度转电压
    #endif

   //赋值给定电压
    motor->given_current = (int16_t)(motor->current_set);
}

//云台机构 以编码器为反馈的闭环控制方式
void gimbal_motor_encode_loop(gimbal_motor_t *motor, pid_data_t *pid_speed, pid_data_t *pid_pos)
{
    #if yaw_pid_speed_debug_mode || pitch_pid_speed_debug_mode //速度环debug
        motor->current_set = pid_calc(pid_speed, motor->encode_pos_ref, debug_set_speed);    //角速度转电压
    #else //串级角度环
        motor->encode_speed_set = pid_calc(pid_pos, motor->encode_pos_ref, motor->encode_pos_set);    //位置转角速度
        motor->current_set = pid_calc(pid_speed, motor->encode_speed_ref, motor->encode_speed_set);  //角速度转电压
    #endif

   //赋值给定电压
    motor->given_current = (int16_t)(motor->current_set);
}

//云台控制电流发送
void gimbal_control_current_sent(void)
{
    if(monitor_device_is_error(RC_BEAT)) //若遥控器失联 或电机失联
    {         
        CAN_cmd_gimbal(0, 0);
    }
    else
    {
        
        //CAN_cmd_gimbal(0,0);                                                          //发送控制电流
        //CAN_cmd_gimbal(-gimbal_control_data.yaw_motor.given_current, 0);              //调试yaw
        //CAN_cmd_gimbal(0, gimbal_control_data.pitch_motor.given_current);             //调试pitch
        CAN_cmd_gimbal(gimbal_control_data.yaw_motor.given_current, gimbal_control_data.pitch_motor.given_current);
                    
    }
}

fp32 motor_ecd_to_degree(int16_t ecd, int16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }
    
    return relative_ecd * MOTOR_ECD_TO_ANGLE;
}

int cover_flag = 0;

void cover_control(void)
{
    if(get_cover_state_point() == (cover_state_e)COVER_ON &&  (gimbal_control_data.gimbal_RC->key.v & KEY_PRESSED_OFFSET_R) && ((gimbal_control_data.last_key & KEY_PRESSED_OFFSET_R) == 0) && switch_is_mid(gimbal_control_data.gimbal_RC->rc.s[RC_SWITCH_LEFT]))
    {
        cover_off();
        get_led_state()->cover_state.state = LED_STATE_GREEN;
    }
	else if (get_cover_state_point() == (cover_state_e)COVER_OFF && (gimbal_control_data.gimbal_RC->key.v & KEY_PRESSED_OFFSET_R) && ((gimbal_control_data.last_key & KEY_PRESSED_OFFSET_R) == 0) && switch_is_mid(gimbal_control_data.gimbal_RC->rc.s[RC_SWITCH_LEFT]))
    {
        cover_on();
        get_led_state()->cover_state.state = LED_STATE_OFF;
    }

    if(get_cover_state_point() == (cover_state_e)COVER_ON && switch_is_mid(gimbal_control_data.gimbal_RC->rc.s[RC_SWITCH_LEFT]) && get_shoot_control_mode() == SHOOT_LOCK && cover_flag == 0)
    {
        cover_off();
        get_led_state()->cover_state.state = LED_STATE_GREEN;
        cover_flag = 1;
    }
    else if(get_cover_state_point() == (cover_state_e)COVER_OFF && switch_is_down(gimbal_control_data.gimbal_RC->rc.s[RC_SWITCH_LEFT]) && get_shoot_control_mode() == SHOOT_LOCK && cover_flag == 1)
    {
        cover_on();
        get_led_state()->cover_state.state = LED_STATE_OFF;
        cover_flag = 0;
    }
}
