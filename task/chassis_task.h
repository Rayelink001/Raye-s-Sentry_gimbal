#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "DT16_control.h"

#define SHIFT_FAST 1            //是否启用shift加速
#define SHIFT_FAST_RATIO 1.4    //加速率 建议大于1
#define COVER_UNCOLSE 1         //是否启用仓盖未关闭减速提示
#define COVER_UNCOLSE_RATIO 0.5 //减速率 建议介于0-1之间

#define VX_MAX_SPEED 660//660        //键盘正常运动最大速度 - 请符合操作手中不断调整设置
#define VY_MAX_SPEED 500//660
#define WZ_MAX_SPEED 1000//660
#define WHEN_IT_MOVE_WZ_MAX_SPEED 700//660

#define CHASSIS_RC_DEADLINE 5   //摇杆死区

typedef struct
{
    const RC_ctrl_t *chassis_RC; //遥控器数据指针

    int16_t vx_speed;
    int16_t vy_speed; 
    int16_t wz_speed;
    int8_t  chassis_mode;

    bool_t cover_flag;  //弹仓关闭标志位 0是关 1是开

} chassis_control_t; //底盘数据

#endif
