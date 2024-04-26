#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#include "struct_typedef.h"
#include "gimbal_task.h"
#include "CAN_receive.h"
#include "DT16_control.h"
#include "pid.h"

#define SHOOT_TASK_CONTROL_TIME_MS 1

#define FRIC_STOP_SPEED 0.0f
#define FRIC_TARGGET_SPEED 6300.0f //设定摩擦轮的角速度

#define TRI_STOP_SPEED 0.0f
#define TRI_TARGGET_SPEED 2500.0f //设定拨弹盘角速度

//#define FRIC_M3508_RATE_OF_ANGULAR_VELOCITY_TO_LINEAR_VELOCITY 0.0005f      //摩擦轮角速度到线速度的转换率 = 摩擦轮半径(m) / 60;
//#define TRI_M2006_RATE_OF_ANGULAR_VELOCITY_TO_LINEAR_VELOCITY  0.000138f   //拨弹盘角速度到线速度的转换率 = 拨弹盘半径(m) / (36(减速比) * 60)

#define TRIGGER_MOTOR 2006 //这里输入拨弹盘的电机类型 2006 或3508 需要注意的是这里都是官方标件的电机 1：36 与 1：19

#if TRIGGER_MOTOR == 3508
#define SINGLE_ADD_ANGLE_ECD 2800//2730
#define SINGLE_SHOOT_SPEED 5000
#define FULL_ECD 155629
#elif TRIGGER_MOTOR == 2006
#define SINGLE_ADD_ANGLE_ECD 18950   //18432//2500//3277
#define SINGLE_SHOOT_SPEED 2000
#define FULL_ECD 294912
#endif

#define TIME_OF_CHANGE_SHOOT_MODE 1000 //单连发模式切换阈值
#define SHOOT_WAIT_STOP_TIME 25       //等待时间阈值

#define BLOCK_TRIGGER_SPEED 2000.0f    //堵转速度判定
#define BLOCK_TIME  800             //堵转时间判定
#define REVERSE_TIME 800            //堵转修复时间

/* 拨弹盘电机速度PID */
#define TRI_SPEED_KP 10.0f
#define TRI_SPEED_KI 1.0f
#define TRI_SPEED_KD 0.0f
#define TRIGGER_SPEED_PID_MAX_OUT 10000.0f
#define TRIGGER_SPEED_PID_MAX_IOUT 8000.0f

/* 拨弹盘电机位置PID */
#define TPI_POS_KP 3.0f
#define TPI_POS_KI 0.0f
#define TPI_POS_KD 0.0f
#define TRIGGER_POS_PID_MAX_OUT 3000.0f
#define TRIGGER_POS_PID_MAX_IOUT 0.0f

/* 左摩擦轮电机速度PID */
#define FRIC_LEFT_SPEED_KP 13.0f
#define FRIC_LEFT_SPEED_KI 0.0f
#define FRIC_LEFT_SPEED_KD 0.0f
#define FRIC_LEFT_SPEED_PID_MAX_OUT 16000.0f
#define FRIC_LEFT_SPEED_PID_MAX_IOUT 5000.0f

/* 右边摩擦轮电机速度PID */
#define FRIC_RIGHT_SPEED_KP 13.0f
#define FRIC_RIGHT_SPEED_KI 0.0f
#define FRIC_RIGHT_SPEED_KD 0.0f
#define FRIC_RIGHT_SPEED_PID_MAX_OUT 16000.0f
#define FRIC_RIGHT_SPEED_PID_MAX_IOUT 5000.0f

#define SHOOT_COVER_OFF 0
#define SHOOT_COVER_ON  1

//状态模式列表
typedef enum
{
    SHOOT_LOCK = 0,                 //发射机构上锁状态 - 摩擦轮关闭
    SHOOT_UNLOCK,                   //发射机构解锁状态 - 摩擦轮开启
    SHOOT_REARY_TO_LOCK,            //发射机构预上锁状态 - 摩擦轮关闭
    SHOOT_READY,                    //发射预备状态
    SHOOT_READY_TO_SHOOT,           //单连发判定状态
    SHOOT_READY_TO_SINGLE_SHOOT,    //判定为单发模式状态
    SHOOT_READY_TO_CONTINUE_SHOOT,  //判定为连发模式状态
    SHOOT_SINGLE_SHOOT,             //单发状态
    SHOOT_CONTINUE_SHOOT,           //连发状态
    
    SHOOT_FRIC_REARY_TO_STOP    //摩擦轮关闭状态

} shoot_control_mode_e;

//发射机构数据结构体
typedef struct
{
    shoot_control_mode_e shoot_mode;//发射机构状态机
    
    const RC_ctrl_t *shoot_rc;//遥控器数据

    const motor_measure_t *shoot_tri_motor;       //拨单盘电机数据
    const motor_measure_t *shoot_fric_left_motor; //左摩擦轮电机数据
    const motor_measure_t *shoot_fric_right_motor;//右摩擦轮电机数据
    const  gimbal_control_data_t *gim_da;
    pid_data_t trigger_pid;      //拨弹盘速度环PID
    pid_data_t trigger_pos_pid;  //拨弹盘位置环PID
    pid_data_t fric_left_pid;    //左摩擦轮速度环PID
    pid_data_t fric_right_pid;   //右摩擦轮速度环PID

    fp32 fric_set_speed;        //摩擦轮 目标角速度 RPM
    fp32 fric_left_ref_speed;   //左摩擦轮 反馈角速度 RPM
    fp32 fric_right_ref_speed;  //右摩擦轮 反馈角速度 RPM

    fp32 tri_set_speed; //拨弹盘 目标角速度 RPM
    fp32 tri_ref_speed; //拨弹盘 反馈角速度 RPM

    int32_t tri_angle_ecd_set;  //拨弹盘 目标角度 ECD
    int32_t tri_angle_ecd_ref;  //拨弹盘 反馈角度 ECD

    int32_t tri_output_angle_ecd_set;   //拨弹盘 目标输出角度 ECD
    int32_t tri_output_angle_ecd_ref;   //拨弹盘 反馈输出角度 ECD

    int16_t tri_output_count;          //拨弹盘 圈速记录

    int16_t trigger_given_current;  //给定拨弹盘电机的电流值
    int16_t fric_left_given_current;  //给定左摩擦轮电机的电流值
    int16_t fric_right_given_current;  //给定右摩擦轮电机的电流值

    uint32_t in_to_ready_shoot_time; //进入预备发射的时间
    uint32_t out_to_ready_shoot_time; //推出预备发射的时间

    uint16_t block_time;        //堵转时间
    uint16_t reverse_time;    //堵转修复时间
    bool_t bloack_flag;       //堵转标志位 0 无堵 1 堵了

    uint16_t wait_stop_time;    //等待停止时间

    char last_switch; //上一次的挡位
    uint16_t last_key;      //上一次按键状态
    uint8_t last_press_l;   //上一次鼠标左键状态
    uint8_t last_press_r;   //上一次鼠标右键状态

    bool_t single_shoot_move_flag; //单发拨弹盘运动标识符 0不运动  1运动

    int16_t shoot_cover_state; //弹仓盖当前状态
    int8_t got;

} shoot_control_data_t;

#endif
