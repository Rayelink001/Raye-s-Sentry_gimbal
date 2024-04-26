#ifndef __MONITOR_TASK__
#define __MONITOR_TASK__

#include "struct_typedef.h"

#define MONITOR_TASK_INIT_TIME 57   //监控任务空闲等待时间

/* 设备心跳通信列表 这里需要额外注意一一对应关系 以免设备码与实际设备不符的问题发生*/
enum errorlist
{
    RC_BEAT = 0,//RC心跳通信
    GIMBAL_YAW_BEAT,//云台YAW电机心跳通信
    GIMBAL_PITCH_BEAT,//云台PITCH电机心跳通信
    ERROR_LIST_LENGHT,//错误设备数
};

/* 错误信息 */
typedef __packed struct
{
    uint32_t current_time; //最近一次刷新hook时间
    uint32_t last_time; //上一次刷新hook的时间
    
    uint32_t lost_time; //离线时间
    uint32_t work_time; //上线时间

    uint16_t set_offline_time; //设置的离线判断时间
    uint16_t set_online_time;   //设置的上线判读时间

    uint8_t enable; //使能开关 0为关闭 1为开启
    uint8_t priority;   //优先级

    uint8_t error_exist;  //错误存在标志位 0为非 1为是
    uint8_t is_lost;    //离线丢失标志位 0为非 1为是
    uint8_t data_is_error;  //数据错误标志位 0为非 1为是

    fp32 frequency; //监控轮的频率

    void (*solve_lost_fun)(void);   //离线异常处理函数
    bool_t (*data_is_error_fun)(void);  //数据错误判断函数
    void (*solve_data_error_fun)(void); //数据错误处理函数
    
}error_t;


extern void monitor_beat_hook(uint8_t device_num); //心跳通信hook
extern bool_t monitor_device_is_error(uint8_t device_num); //获取对应设备监控状态 返回值为0是 则说明状态离线
extern bool_t monitor_device_is_lost(uint8_t device_num);

#endif
