#ifndef __PID_H__
#define __PID_H__

#include "main.h"
#include "struct_typedef.h"
#include "user_lib.h"

typedef struct
{
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_data_t;

typedef struct  //不同于速度环PID 云台的角度PID有另外的写法
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

extern void pid_init(pid_data_t *pid, const float PID[3], float max_out, float max_iout);
extern void pid_init_gimbal(pid_data_t *pid);
extern void pid_init_chassis(pid_data_t *pid, const float PID[5]);
extern float pid_calc(pid_data_t *pid, float ref, float set);

extern void gimbal_pid_init(gimbal_PID_t *pid, const fp32 PID[3], fp32 maxout, fp32 max_iout);
extern fp32 gimbal_pid_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

#endif
