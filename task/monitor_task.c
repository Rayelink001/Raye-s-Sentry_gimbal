/* 监控任务：用于反疯车 和 自检 */
#include "monitor_task.h"

#include "cmsis_os.h"

void monitor_init(uint32_t time);   //监控任务初始化

error_t error_list[ERROR_LIST_LENGHT + 1];

void monitor_task(void const *pvParameters)//监控任务
{
    uint32_t system_current_time;//系统当前时间
    system_current_time = xTaskGetTickCount();//获取系统当前时间

    //监控任务初始化
    monitor_init(system_current_time);
    //空闲等待初始化时间
    vTaskDelay(MONITOR_TASK_INIT_TIME);

    while (1)
    {
        //获取当前检测轮的时间
        system_current_time = xTaskGetTickCount();
    
        //错误标志位置0
        error_list[ERROR_LIST_LENGHT].is_lost = 0;  
        error_list[ERROR_LIST_LENGHT].error_exist = 0;

        for ( int i = 0; i < ERROR_LIST_LENGHT; i++)
        {
            //判断设备是否使能 若无使能：直接略过该设备的自检
            if (error_list[i].enable == 0)
            {
                continue;   //这里用于直接跳出这一轮的for   
            }

            //离线判断
            if (system_current_time - error_list[i].current_time > error_list[i].set_offline_time)
            {
				
                //记录错误以及离线时间
                if(error_list[i].error_exist == 0)
                {
                    error_list[i].is_lost = 1;
                    error_list[i].error_exist = 1;
                    error_list[i].lost_time = system_current_time;
                }

                //判断错误优先级，保存等级最高的错误码，用于依次提示警报
                if(error_list[i].priority > error_list[ERROR_LIST_LENGHT].priority)
                {
                    error_list[ERROR_LIST_LENGHT].priority = i;
                }

                //错误标志位置1                
                error_list[ERROR_LIST_LENGHT].is_lost = 1;
                error_list[ERROR_LIST_LENGHT].error_exist = 1;

                //若有提供错误处理函数，则运行错误处理函数    
                if(error_list[i].solve_lost_fun != NULL)
                {
                    error_list[i].solve_lost_fun();
                }
            }

            //上线稳定工作判断
            else if(system_current_time - error_list[i].current_time < error_list[i].set_online_time)
            {
                //刚刚上线，可能存在数据不稳定，只记录不丢失
                 error_list[i].is_lost = 0;
                 error_list[i].error_exist = 1; 
            }

            //上线判断
            else
            {
                error_list[i].is_lost = 0;  

                //判断数据是否存在错误
                if(error_list[i].data_is_error != NULL)
                {
                    error_list[i].error_exist = 1;
                }
                else
                {
                    error_list[i].error_exist = 0;
                }

                //计算频率
                if(error_list[i].current_time > error_list[i].last_time)
                {
                    error_list[i].frequency = configTICK_RATE_HZ / (fp32)(error_list[i].current_time - error_list[i].last_time);
                }
            }
        }
    }
}

void monitor_init(uint32_t time)//监控任务初始化
{
    //设置 设备监控的 离线时间阈值 上线稳定时间阈值 优先级
    //这个表需要与monitor_task.h 中的 errorlist 对应
    uint16_t set_monitor_state[ERROR_LIST_LENGHT][3] =
    {
        {20, 0, 15},//RC_BEAT
        {40, 0, 11},//CHASSIS_MOTOR1_BEAT
        {40, 0, 10},//CHASSIS_MOTOR2_BEAT
    };

    //初始化操作
    for(uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_monitor_state[i][0];
        error_list[i].set_online_time = set_monitor_state[i][1];
        error_list[i].priority =  set_monitor_state[i][2];

        error_list[i].solve_lost_fun = NULL;

        error_list[i].enable = 1;
        error_list[i].error_exist = 0;
        error_list[i].is_lost = 1;
        error_list[i].data_is_error = 1;
        error_list[i].frequency = 0.0f;
        error_list[i].current_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }
}

void monitor_beat_hook(uint8_t device_num)//心跳通信hook
{
    error_list[device_num].last_time = error_list[device_num].current_time; //记录上一次hook时间
    error_list[device_num].current_time = xTaskGetTickCount();  //记录本次hook时间
    
    //若设备丢失
    if (error_list[device_num].is_lost) 
    {
        error_list[device_num].is_lost = 0; 
        error_list[device_num].work_time = error_list[device_num].current_time; 
    }
    
    //若提供数据错误判断函数时，进行判断,否则不进行判断
    if (error_list[device_num].data_is_error_fun != NULL) 
    {
        //若数据错误发生
        if (error_list[device_num].data_is_error_fun())
        {
            error_list[device_num].error_exist = 1;
            error_list[device_num].data_is_error = 1;

            //当有提供数据错误处理函数时,尝试进行处理
            if (error_list[device_num].solve_data_error_fun != NULL)
            {
                error_list[device_num].solve_data_error_fun();
            }
            
        }
        //若数据不发生错误
        else
        {
            error_list[device_num].data_is_error = 0;
        }
    }

    //无数据错误判断函数
    else
    {
        error_list[device_num].data_is_error = 0;
    }
}

//获取对应设备监控状态 返回值为0是 则说明状态离线
bool_t monitor_device_is_error(uint8_t device_num)
{
    return (error_list[device_num].error_exist == 1);
}

//获取对应设备监控状态 返回值为0是 则说明状态离线
bool_t monitor_device_is_lost(uint8_t device_num)
{
    return (error_list[device_num].is_lost == 1);
}
