#include "shoot_task.h"
#include <stdlib.h>
#include "cmsis_os.h"
#include "gimbal_task.h"
#include <math.h>
#include "monitor_task.h"
#include "can_receive.h"
void shoot_init(void);                                                      //发射机构初始化
void shoot_mode_set(shoot_control_data_t *shoot_mode_set);                  //发射机构状态机设置
void shoot_feedback_update(shoot_control_data_t *shoot_feedback_update);    //发射数据反馈更新
void shoot_fric_stop(void);                                                 //摩擦轮关闭函数
void shoot_trigger_motor_turn_back(void);                                   //堵弹反转处理函数
void shoot_single_control(void);                                            //单发控制
gimbal_control_data_t gim_data;
shoot_control_data_t shoot_control_data;    //发射机构全局数据

void shoot_task(void const *pvParameters)
{  
    shoot_init();//发射机构初始化

    while (1)
    {
        if(gim_data.GAME_STATE == 1 )
        {gim_data.sentry_state = 1;}
        else if (gim_data.GAME_STATE == 0)
        {gim_data.sentry_state = 0;}

        //若摩擦轮-拨弹盘电机下线-不执行任何发射相关的内容-防止电机出现23赛季英雄电调报错的问题
        //while()   //不执行任何命令内容死循环

        shoot_mode_set(&shoot_control_data);        //发射机构状态机设置
        shoot_feedback_update(&shoot_control_data); //发射机构数据反馈更新

             if(shoot_control_data.shoot_mode == SHOOT_READY_TO_SHOOT || shoot_control_data.shoot_mode == SHOOT_READY_TO_SINGLE_SHOOT &&(shoot_control_data.gim_da->vision_data->is_middle == 0))
        {

            //退出发射准备状态的时间
            shoot_control_data.out_to_ready_shoot_time = xTaskGetTickCount(); 
            
            //判断时间阈值大小
            if(shoot_control_data.out_to_ready_shoot_time - shoot_control_data.in_to_ready_shoot_time <= TIME_OF_CHANGE_SHOOT_MODE)
            {
                shoot_control_data.got = 1;
                shoot_control_data.shoot_mode = SHOOT_READY_TO_SINGLE_SHOOT;
            }
            else if(shoot_control_data.out_to_ready_shoot_time - shoot_control_data.in_to_ready_shoot_time > TIME_OF_CHANGE_SHOOT_MODE)
            {
                 shoot_control_data.got = 2;
                shoot_control_data.shoot_mode = SHOOT_READY_TO_CONTINUE_SHOOT;
            }
        }

        //如果枪口过热，则需要等待CD冷却

        //模式下的基本参数设置
        switch (shoot_control_data.shoot_mode)
        {
            case SHOOT_UNLOCK:
            case SHOOT_READY:
                shoot_control_data.got = 3;
                shoot_control_data.tri_set_speed = TRI_STOP_SPEED;
                break;
            case SHOOT_SINGLE_SHOOT:
                //转动一定角度判定 或 摩擦轮掉速判定 和卡弹反拨处理
                shoot_control_data.got = 4 ;
                shoot_single_control();
                shoot_trigger_motor_turn_back();
                break;
            case SHOOT_CONTINUE_SHOOT:
                //持续转动 与 卡弹反拨处理
                shoot_control_data.got = 5 ; 
                shoot_control_data.tri_set_speed = TRI_TARGGET_SPEED;
                shoot_trigger_motor_turn_back();
                break;
            default:
                break;
        }
        
        //摩擦轮开启或关闭
        if(shoot_control_data.shoot_mode == SHOOT_REARY_TO_LOCK || shoot_control_data.shoot_mode == SHOOT_LOCK) 
        {
            shoot_control_data.got =6;
            shoot_control_data.tri_set_speed = TRI_STOP_SPEED;
            shoot_control_data.trigger_given_current = 0;
            shoot_fric_stop();  //摩擦轮关闭函数
        }
        else
        {
            //操作手指示灯提示
            //摩擦轮控制
            shoot_control_data.got = 7;
            shoot_control_data.fric_set_speed = FRIC_TARGGET_SPEED;
            shoot_control_data.fric_left_given_current = (int16_t) pid_calc(&shoot_control_data.fric_left_pid, shoot_control_data.fric_left_ref_speed, shoot_control_data.fric_set_speed);
            shoot_control_data.fric_right_given_current = (int16_t) pid_calc(&shoot_control_data.fric_right_pid, shoot_control_data.fric_right_ref_speed, shoot_control_data.fric_set_speed);
            shoot_control_data.trigger_given_current = (int16_t) pid_calc(&shoot_control_data.trigger_pid, shoot_control_data.tri_ref_speed, shoot_control_data.tri_set_speed);
        }
        //发送电机数据电流数据 - 遥控器离线或电机离线都关闭电机
        if(monitor_device_is_error(RC_BEAT))
        {
            shoot_control_data.shoot_mode = SHOOT_REARY_TO_LOCK;
            shoot_control_data.got = 8;
        }
        
        //通过时间阈值 去设定 单发还是多发模式。如果小于某一时间阈值，归中拨杆，则开启单发。若大于这一阈值，则为连发。
        if(shoot_control_data.gim_da->vision_data->is_find_target == 1 ) 
            {
                //开启摩擦轮
                shoot_control_data.got = 9;
                shoot_control_data.fric_set_speed = FRIC_TARGGET_SPEED;
                shoot_control_data.fric_left_given_current = (int16_t) pid_calc(&shoot_control_data.fric_left_pid, shoot_control_data.fric_left_ref_speed, shoot_control_data.fric_set_speed);
                shoot_control_data.fric_right_given_current = (int16_t) pid_calc(&shoot_control_data.fric_right_pid, shoot_control_data.fric_right_ref_speed, shoot_control_data.fric_set_speed);
                shoot_control_data.trigger_given_current = (int16_t) pid_calc(&shoot_control_data.trigger_pid, shoot_control_data.tri_ref_speed, shoot_control_data.tri_set_speed);
                if(fabs(shoot_control_data.gim_da->vision_data->yaw_angle.f)<= 4.8f && shoot_control_data.gim_da->vision_data->is_middle == 1 && shoot_control_data.gim_da->vision_data->dis.f > 0.8f &&  shoot_control_data.gim_da->vision_data->dis.f < 10.2f && shoot_control_data.gim_da->referfroma->shoot_heat <= 400)
                {
                    shoot_control_data.got = 10;
                    shoot_control_data.tri_set_speed = TRI_TARGGET_SPEED;
                    shoot_trigger_motor_turn_back();
                }
            }
            else if( shoot_control_data.gim_da->vision_data->is_find_target == 0 &&( shoot_control_data.gim_da->gimbal_RC->rc.s[0] ==1|| shoot_control_data.gim_da->gimbal_RC->rc.s[0] == 2) )
            {
                shoot_control_data.got =11;
                 shoot_control_data.tri_set_speed = TRI_STOP_SPEED;
                 shoot_control_data.trigger_given_current = 0;
                 shoot_fric_stop();//摩擦轮关闭函数
            }

        CAN_cmd_shoot(shoot_control_data.fric_left_given_current,-shoot_control_data.fric_right_given_current, shoot_control_data.trigger_given_current);
        vTaskDelay(SHOOT_TASK_CONTROL_TIME_MS);	//任务频率控制
	}
}

void shoot_init(void)//发射机构初始化
{
    
    const fp32 trigger_speed_pid[3] = {TRI_SPEED_KP,TRI_SPEED_KI,TRI_SPEED_KD};                         //拨弹盘速度环PID参数
    const fp32 trigger_pos_pid[3] = {TPI_POS_KP,TPI_POS_KI,TPI_POS_KD};                                 //拨弹盘速度环PID参数
    const fp32 fric_left_speed_pid[3] = {FRIC_LEFT_SPEED_KP,FRIC_LEFT_SPEED_KI,FRIC_LEFT_SPEED_KD};     //左摩擦轮速度环PID参数
    const fp32 fric_right_speed_pid[3] = {FRIC_RIGHT_SPEED_KP,FRIC_RIGHT_SPEED_KI,FRIC_RIGHT_SPEED_KD}; //右摩擦轮速度环PID参数

    pid_init(&shoot_control_data.trigger_pid,trigger_speed_pid,TRIGGER_SPEED_PID_MAX_OUT,TRIGGER_SPEED_PID_MAX_IOUT);              //拨弹盘速度环PID初始化
    pid_init(&shoot_control_data.trigger_pos_pid,trigger_pos_pid,TRIGGER_POS_PID_MAX_OUT,TRIGGER_POS_PID_MAX_IOUT);               //拨弹盘位置环PID初始化
    pid_init(&shoot_control_data.fric_left_pid,fric_left_speed_pid,FRIC_LEFT_SPEED_PID_MAX_OUT,FRIC_LEFT_SPEED_PID_MAX_IOUT);     //左摩擦轮速度环PID初始化
    pid_init(&shoot_control_data.fric_right_pid,fric_right_speed_pid,FRIC_RIGHT_SPEED_PID_MAX_OUT,FRIC_RIGHT_SPEED_PID_MAX_IOUT); //右摩擦轮速度环PID初始化

    shoot_control_data.shoot_rc = get_rc_ctrl_point();  //遥控数据指针绑定
    shoot_control_data.gim_da = get_gim_point();  
    shoot_control_data.shoot_tri_motor = get_trigger_motor_measure_point();//拨弹盘电机数据指针绑定
    shoot_control_data.shoot_fric_left_motor = get_left_fric_motor_measure_point();//左摩擦轮电机数据指针绑定
    shoot_control_data.shoot_fric_right_motor = get_right_fric_motor_measure_point();//右摩擦轮电机数据指针绑定
    gim_data.gimbal_RC = get_rc_ctrl_point();
    shoot_control_data.shoot_mode = SHOOT_LOCK; //初始化发射机构状态为SHOOT_LOCK
    shoot_control_data.last_switch = RC_SW_MID; //初始化拨杆上一时刻的状态为RC_SW_MID

    shoot_control_data.shoot_cover_state = SHOOT_COVER_OFF; //初始化舱盖为关闭状态

    shoot_control_data.wait_stop_time = 0;          //初始化停止等待时间
    shoot_control_data.single_shoot_move_flag = 0;  //初始化单次发射移动标志位
    shoot_control_data.bloack_flag = 0;             //初始化堵转标志位

    //发射机构数据更新
}

void shoot_mode_set(shoot_control_data_t *shoot_mode_set)   //发射机构状态机设置
{
    //上拨 解上锁操作 - 开启关闭摩擦轮
    if(shoot_mode_set->shoot_mode == SHOOT_LOCK && switch_is_mid(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && switch_is_up(shoot_mode_set->last_switch))
    {
        shoot_mode_set->shoot_mode = SHOOT_UNLOCK;
    }
    else if(shoot_mode_set->shoot_mode != SHOOT_LOCK && switch_is_up(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]))
    { 
        shoot_mode_set->shoot_mode = SHOOT_REARY_TO_LOCK;   //除LOCK模式下的所有模式，当拨杆推到最上档时都进行预上锁状态
    }
    else if(shoot_mode_set->shoot_mode == SHOOT_REARY_TO_LOCK && switch_is_mid(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && switch_is_up(shoot_mode_set->last_switch))
    {
        shoot_mode_set->shoot_mode = SHOOT_LOCK;            //在预上锁状态下，归中拨杆，进行上锁
    }

    //上锁状态下 开关弹仓盖
    if(shoot_mode_set->shoot_mode == SHOOT_LOCK && switch_is_down(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && switch_is_mid(shoot_mode_set->last_switch))
    {
        if(shoot_mode_set->shoot_cover_state == SHOOT_COVER_OFF)
        {
            shoot_mode_set->shoot_cover_state = SHOOT_COVER_ON;
        }
        else if(shoot_mode_set->shoot_cover_state == SHOOT_COVER_ON)
        {
            shoot_mode_set->shoot_cover_state = SHOOT_COVER_OFF;
        }
    }
    
    //等待摩擦轮速度到达预定速度57
    if(shoot_mode_set->shoot_mode == SHOOT_UNLOCK && switch_is_mid(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT])/* && fric_left_speed == MAX_speed && fric_right_speed == MAX_speed*/)
    {

        shoot_mode_set->shoot_mode = SHOOT_READY;

    }
    
    //发射模式选择
    if((shoot_mode_set->shoot_mode == SHOOT_READY || shoot_mode_set->shoot_mode == SHOOT_SINGLE_SHOOT) && switch_is_down(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && switch_is_mid(shoot_mode_set->last_switch))    //发射状态判定
    {
        shoot_mode_set->shoot_mode = SHOOT_READY_TO_SHOOT;
        shoot_control_data.single_shoot_move_flag = 0;
        shoot_control_data.in_to_ready_shoot_time = xTaskGetTickCount();    //记录进入时间
    }
    else if(shoot_mode_set->shoot_mode == SHOOT_CONTINUE_SHOOT && switch_is_mid(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && switch_is_down(shoot_mode_set->last_switch)) //连发模式归中关闭
    {
        shoot_mode_set->shoot_mode = SHOOT_READY;
    }
    else if(shoot_mode_set->shoot_mode == SHOOT_READY_TO_SINGLE_SHOOT && switch_is_mid(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && switch_is_down(shoot_mode_set->last_switch))   //单发模式归中触发
    {
        shoot_mode_set->shoot_mode = SHOOT_SINGLE_SHOOT;
    }
    else if(shoot_mode_set->shoot_mode == SHOOT_READY_TO_CONTINUE_SHOOT && switch_is_down(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]))   //连发模式保持触发
    {
        shoot_mode_set->shoot_mode = SHOOT_CONTINUE_SHOOT;
    }



    //键盘鼠标控制 - 需要维持中档 才能进行键盘鼠标操作。
    
    //中档可以使用键盘 按下F键开启摩擦轮
    if (shoot_mode_set->shoot_mode == SHOOT_LOCK && switch_is_mid(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && (shoot_mode_set->shoot_rc->key.v & KEY_PRESSED_OFFSET_F) && ((shoot_mode_set->last_key & KEY_PRESSED_OFFSET_F) == 0))
    {
        shoot_mode_set->shoot_mode = SHOOT_UNLOCK;
    }
    else if (shoot_mode_set->shoot_mode != SHOOT_LOCK && switch_is_mid(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && (shoot_mode_set->shoot_rc->key.v & KEY_PRESSED_OFFSET_F) && ((shoot_mode_set->last_key & KEY_PRESSED_OFFSET_F) == 0))
    {
        shoot_mode_set->shoot_mode = SHOOT_REARY_TO_LOCK;
    }

    //鼠标左键单击 单发
    if(shoot_mode_set->shoot_mode == SHOOT_READY && switch_is_mid(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && (shoot_mode_set->shoot_rc->mouse.press_l && shoot_mode_set->last_press_l == 0))
    {
        shoot_mode_set->shoot_mode = SHOOT_SINGLE_SHOOT;
    }

    //鼠标右键长按 连发 松手关闭
    if((shoot_mode_set->shoot_mode == SHOOT_READY || shoot_mode_set->shoot_mode == SHOOT_CONTINUE_SHOOT) && switch_is_mid(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && shoot_mode_set->shoot_rc->mouse.press_r)
    {
        shoot_mode_set->shoot_mode = SHOOT_CONTINUE_SHOOT;
    }
    else if(shoot_mode_set->shoot_mode == SHOOT_CONTINUE_SHOOT && switch_is_mid(shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT]) && (shoot_mode_set->shoot_rc->mouse.press_r == 0 && shoot_mode_set->last_press_r))
    {
        shoot_mode_set->shoot_mode = SHOOT_READY;
    }

    shoot_mode_set->last_switch = shoot_mode_set->shoot_rc->rc.s[RC_SWITCH_LEFT];   //上一挡位数据更新
    shoot_mode_set->last_key = shoot_mode_set->shoot_rc->key.v;                     //上一次按键
    shoot_mode_set->last_press_l = shoot_mode_set->shoot_rc->mouse.press_l;         //鼠标左键上一状态更新
    shoot_mode_set->last_press_r = shoot_mode_set->shoot_rc->mouse.press_r;         //鼠标右键上一状态更新
}

void shoot_feedback_update(shoot_control_data_t *shoot_feedback_update) //发射数据反馈更新
{
    //摩擦轮角速度 反馈
    shoot_feedback_update->fric_left_ref_speed = shoot_feedback_update->shoot_fric_left_motor->speed_rpm;
    shoot_feedback_update->fric_right_ref_speed = -shoot_feedback_update->shoot_fric_right_motor->speed_rpm;

    //拨弹盘角速度 反馈
    shoot_feedback_update->tri_ref_speed = shoot_feedback_update->shoot_tri_motor->speed_rpm;

    //拨弹盘角度位置 反馈
    shoot_feedback_update->tri_angle_ecd_ref = shoot_feedback_update->shoot_tri_motor->ecd;

    if(fabs((shoot_feedback_update->shoot_tri_motor->ecd - shoot_feedback_update->shoot_tri_motor->last_ecd)) > 4095)
    {
        if(shoot_feedback_update->shoot_tri_motor->ecd  < shoot_feedback_update->shoot_tri_motor->last_ecd)
            shoot_feedback_update->tri_output_angle_ecd_ref += 8192 - shoot_feedback_update->shoot_tri_motor->last_ecd + shoot_feedback_update->shoot_tri_motor->ecd;
        else
            shoot_feedback_update->tri_output_angle_ecd_ref -= 8192 - shoot_feedback_update->shoot_tri_motor->last_ecd + shoot_feedback_update->shoot_tri_motor->ecd;
    }
    else
    {
        shoot_feedback_update->tri_output_angle_ecd_ref += shoot_feedback_update->shoot_tri_motor->ecd - shoot_feedback_update->shoot_tri_motor->last_ecd;
    }

    if(fabs(shoot_feedback_update->tri_output_angle_ecd_ref) >= FULL_ECD)
        shoot_feedback_update->tri_output_angle_ecd_ref = 0;

}

void shoot_fric_stop(void) //摩擦轮关闭函数
{
    //闭环控制停止摩擦轮
    if(shoot_control_data.shoot_mode == SHOOT_REARY_TO_LOCK)
    {
        //设定摩擦轮目标转速值为0
        shoot_control_data.fric_set_speed = FRIC_STOP_SPEED;
        //闭环控制
        shoot_control_data.fric_left_given_current = (int16_t) pid_calc(&shoot_control_data.fric_left_pid, shoot_control_data.fric_left_ref_speed, shoot_control_data.fric_set_speed);
        shoot_control_data.fric_right_given_current = (int16_t) pid_calc(&shoot_control_data.fric_right_pid, shoot_control_data.fric_right_ref_speed, shoot_control_data.fric_set_speed);

        //当摩擦轮反馈转速为0时且等待一段时间后，切换为开环控制
        if(shoot_control_data.fric_left_ref_speed == 0 && shoot_control_data.fric_right_ref_speed == 0)
        {
            shoot_control_data.wait_stop_time++;
            if(shoot_control_data.wait_stop_time > SHOOT_WAIT_STOP_TIME)
            {
                shoot_control_data.shoot_mode = SHOOT_LOCK;
                shoot_control_data.wait_stop_time = 0;
            }
        }
    }
    //当摩擦轮速度为0时-开环控制-允许人为手动拨动摩擦轮
    else 
    {
        shoot_control_data.fric_left_given_current = 0;
        shoot_control_data.fric_right_given_current = 0;
    }
}

void shoot_trigger_motor_turn_back(void)    //速度环的堵弹反转处理函数
{
    if( shoot_control_data.block_time < BLOCK_TIME)  //堵转时间
    {
        shoot_control_data.tri_set_speed = shoot_control_data.tri_set_speed;
    }
    else
    {
        shoot_control_data.tri_set_speed = -shoot_control_data.tri_set_speed;
        shoot_control_data.bloack_flag = 1; //堵转标志位
    }

    if(fabs(shoot_control_data.tri_ref_speed)< BLOCK_TRIGGER_SPEED && shoot_control_data.block_time < BLOCK_TIME)    //若小于一定速度一段时间 则判定为堵转
    {
        shoot_control_data.block_time++;
        shoot_control_data.reverse_time = 0;
    }
    else if (shoot_control_data.block_time == BLOCK_TIME && shoot_control_data.reverse_time < REVERSE_TIME)   //堵转反转时间
    {
        shoot_control_data.reverse_time++;
    }
    else
    {
        shoot_control_data.block_time = 0;
        shoot_control_data.bloack_flag = 0; //堵转标志位
    }
}

void shoot_trigger_motor_turn_back_single(void)    //单发位置环的堵弹反转处理函数
{
    if( shoot_control_data.block_time < BLOCK_TIME)  //堵转时间
    {
        shoot_control_data.tri_set_speed = shoot_control_data.tri_set_speed;
    }
    else
    {
        shoot_control_data.tri_set_speed = -shoot_control_data.tri_set_speed;
        shoot_control_data.bloack_flag = 1; //堵转标志位
    }

    if(fabs(shoot_control_data.tri_ref_speed)< BLOCK_TRIGGER_SPEED && shoot_control_data.block_time < BLOCK_TIME)    //若小于一定速度一段时间 则判定为堵转
    {
        shoot_control_data.block_time++;
        shoot_control_data.reverse_time = 0;
    }
    else if (shoot_control_data.block_time == BLOCK_TIME && shoot_control_data.reverse_time < REVERSE_TIME)   //堵转反转时间
    {
        shoot_control_data.reverse_time++;
    }
    else
    {
        shoot_control_data.block_time = 0;
        shoot_control_data.bloack_flag = 0; //堵转标志位
    }
}

void shoot_single_control(void) //单发控制
{
    if (shoot_control_data.single_shoot_move_flag == 0)//每次拨动一定的角度
    {
        shoot_control_data.tri_output_angle_ecd_set = shoot_control_data.tri_output_angle_ecd_ref + SINGLE_ADD_ANGLE_ECD; //设定目标角度

        if(shoot_control_data.tri_output_angle_ecd_set > FULL_ECD)
        {
            shoot_control_data.tri_output_angle_ecd_set -= FULL_ECD;
        }
        shoot_control_data.single_shoot_move_flag = 1;
    }
    else if(shoot_control_data.single_shoot_move_flag == 1 && shoot_control_data.bloack_flag == 0)
    {
        //若一段时间内没有转动到规定角度，则对位置进行反转。

        shoot_control_data.tri_set_speed = pid_calc(&shoot_control_data.trigger_pos_pid, shoot_control_data.tri_output_angle_ecd_ref, shoot_control_data.tri_output_angle_ecd_set);
    }

    if(fabs(shoot_control_data.tri_set_speed) < 500 && fabs(shoot_control_data.tri_output_angle_ecd_ref - shoot_control_data.tri_output_angle_ecd_set) < 100)    //PID输出速度为0 且反馈位置与设定位置相同时，则说明达到位置。
    {
        shoot_control_data.single_shoot_move_flag = 0;
        shoot_control_data.shoot_mode = SHOOT_UNLOCK;
    }
}

