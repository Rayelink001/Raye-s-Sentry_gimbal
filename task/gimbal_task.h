#ifndef __GIMBAL_TASK__
#define __GIMBAL_TASK__

#include "DT16_control.h"
#include "can_receive.h"
#include "pid.h"

#include "wt16c.h"
#include "vision_task.h"

#define yaw_pid_speed_debug_mode 0         //yaw轴电机速度环调试模式 0关 1开 - 正常使用时应保持关闭
#define pitch_pid_speed_debug_mode 0       //pitch轴电机速度环调试模式 0关 1开 - 正常使用时应保持关闭
#define DEBUG_SET_SPEED 1                  //yaw轴速度环调试设定速度

#define GIMBAL_RC_DEADLINE 10 //遥控死区
#define YAW_RC_CEN -0.00002f//0.05f//-0.00005f//YAW-RC通道转换率
#define YAW_MOUSE_RC_CEN -0.001f
#define PIT_RC_CEN 0.0002///-0.004f//-0.000009f//PITCH-RC通道转换率
#define PIT_MOUSE_RC_CEN -0.0006f

#define YAW_OFFSET_ECD  2034            //yaw回正ECD值
#define YAW_OFFSET_ANGLE 57.7386475f    //pitch回正 角度值

#define PITCH_OFFSET_ECD 737           //pitch回正 ECD值
#define PITCH_OFFSET_ANGLE 0.0f         //pitch回正 角度值
#define PITCH_MAX_RELATIVE_ANGLE 1150	 //pitch最大机械角度
#define PITCH_MIN_RELATIVE_ANGLE 465	//pitch最小机械角度

#define ECD_RANGE 8191  //编码值范围
#define HALF_ECD_RANGE 4096 //编码值范围的一半
#define ANGLE_RANGE 360 //角度范围
#define HALF_ANGLE_RANGE 180   //角度范围的一半

#define MOTOR_ECD_TO_RAD   0.000766990394f    //编码值转换弧度值 2PI/8192
#define MOTOR_RAD_TO_ECD   1303.797316049191f //弧度值转化为编码值 8192/2PI
#define MOTOR_ECD_TO_ANGLE 0.04907793633369f  //编码值转换为角度值 360/8192
#define MOTOR_ANGLE_TO_ECD 22.755555555555f   //角度值转化为编码值 8192/360

typedef enum
{
    GIMBAL_CONTROL_MODE_FREE = 0,           //云台无力模式
    GIMBAL_CONTROL_MODE_ENCODE,             //编码器反馈的控制闭环模式
    GIMBAL_CONTROL_MODE_IMU,                //陀螺仪反馈的控制闭环模式
    GIMBAL_CONTROL_MODE_VISION_IMU,         //视觉控制下的IMU闭环控制模式
    GIMBAL_CONTROL_MODE_VISION_ENCODE       //视觉控制下的编码器闭环控制模式
} gimbal_control_mode_e;

typedef struct
{
    const motor_measure_t *motor_data_ref;  //电机数据反馈
    
    pid_data_t encode_speed_pid;        //encode - 速度环PID参数
    pid_data_t encode_pos_pid;          //encode - 位置环PID参数
    pid_data_t vision_encode_pos_pid;   //视觉 - encode - 位置环PID参数

    pid_data_t imu_speed_pid;           //imu - 速度环PID参数
    pid_data_t imu_pos_pid;             //imu - 位置环PID参数
    pid_data_t vision_imu_pos_pid;      //视觉 - imuencode - 位置环PID参数
    pid_data_t vision_imu_speed_pid;
    uint16_t offset_ecd;                //归中初始值 电机编码值
    fp32 offset_angle;                  //归中初始值 IMU角度值 
    int16_t max_ecd_angle;              //编码器 最大机械角度 ecd值
    int16_t min_ecd_angle;              //编码器 最小机械角度 ecd值
    fp32    max_ecd_to_degree_angle;    //最大机械角度 °
    fp32    min_ecd_to_degree_angle;    //最小机械角度 °

    int16_t encode_speed_set;           //编码器 - 电机角速度目标值 rpm
    int16_t encode_speed_ref;           //编码器 - 电机角速度反馈值 rpm
    int16_t encode_pos_set;             //编码器 电机角度反馈值
    int16_t encode_pos_ref;             //编码器 电机角度反馈值
    
    fp32 imu_pos_set;                   //IMU - 电机目标角度值 °
    fp32 imu_pos_ref;                   //IMU - 电机反馈角度值 °
    fp32 imu_speed_set;                 //IMU - 电机角速度目标值 °/s
    fp32 imu_speed_ref;                 //IMU - 电机角速度反馈值 °/s

    fp32 current_set;                   //电流设定值
    int16_t given_current;              //给定电流值
    fp32 forward_current_set;            
    fp32 fk;
    fp32 max_forwar;
    fp32 relative_angle_set;
} gimbal_motor_t;

typedef struct
{
    gimbal_control_mode_e control_mode;         //当前云台控制模式
    gimbal_control_mode_e last_control_mode;    //上一云台控制模式
    
    const RC_ctrl_t *gimbal_RC;         //遥控器数据指针
    const vision_data_t *vision_data;   //视觉数据指针
    const wt61c_data_t *ins_data;       //外置陀螺仪数据
    const euler_angle_t *euler_angle;   //世界坐标系下的角度值
    const receive_broad_a_t *referfroma;
    gimbal_motor_t yaw_motor;    //yaw轴数据
    gimbal_motor_t pitch_motor;  //pitch轴数据
    int8_t sentry_state;
    int8_t vision_connect;
    int8_t GAME_STATE;
} gimbal_control_data_t; //云台数据

extern const gimbal_control_data_t *get_gim_point(void) ;
#endif
