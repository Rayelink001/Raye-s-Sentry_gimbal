#include "chassis_task.h"

#include "main.h"
#include "cmsis_os.h"       //OS API

#include "DT16_control.h"
#include "can_receive.h"
#include "monitor_task.h"

#include "led_control_task.h"

//负责将摇杆数据发送给底盘
void chassis_init(void);
void chassis_get_speed(void);

chassis_control_t chssis_control_data;

void chassis_task(void const *pvParameters)  
{
    vTaskDelay(10); //等待一下系统加载
		
    chassis_init();  //底盘初始化

    while(1)
    {
        chassis_get_speed();//获取遥控数据并进行处理

        //若遥控器，则发送停止数据
        if(monitor_device_is_error(RC_BEAT))
        {
            CAN_cmd_chassis(0, 0, 0, RC_SW_DOWN);
        }
        else//否则发送底盘遥控数据
        {
						//CAN_cmd_chassis(chssis_control_data.vx_speed, chssis_control_data.vy_speed, chssis_control_data.wz_speed, RC_SW_DOWN);//chssis_control_data.chassis_mode//);
						CAN_cmd_chassis(chssis_control_data.vx_speed, chssis_control_data.vy_speed, chssis_control_data.wz_speed, chssis_control_data.chassis_mode);
        }
   
        vTaskDelay(10);
    }
}

//底盘任务初始化
void chassis_init(void)
{
    chssis_control_data.chassis_RC = get_rc_ctrl_point();
}

//从遥控中得到底盘控制量
void chassis_get_speed(void)
{
    chssis_control_data.chassis_mode = chssis_control_data.chassis_RC->rc.s[RC_SWITCH_RIGHT];
    //chssis_control_data.chassis_mode =RC_SW_DOWN;	//gimbal debug

    //rc摇杆量 - 设置死区
    rc_deadband_limit(chssis_control_data.chassis_RC->rc.ch[LEFT_CHANNAL_X], chssis_control_data.vx_speed, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chssis_control_data.chassis_RC->rc.ch[LEFT_CHANNAL_Y], chssis_control_data.vy_speed, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chssis_control_data.chassis_RC->rc.ch[WHEEL_CHANNAL], chssis_control_data.wz_speed, CHASSIS_RC_DEADLINE);

    //键盘操纵量 映射为int16_t类型
    if (chssis_control_data.chassis_RC->key.v & KEY_PRESSED_OFFSET_W)   //前进或后退
    {
        chssis_control_data.vx_speed = VX_MAX_SPEED;
    }
    else if (chssis_control_data.chassis_RC->key.v & KEY_PRESSED_OFFSET_S)
    {
        chssis_control_data.vx_speed = -VX_MAX_SPEED;
    }

    if (chssis_control_data.chassis_RC->key.v & KEY_PRESSED_OFFSET_A)    //左平移或右平移
    {
        chssis_control_data.vy_speed = -VY_MAX_SPEED;
    }
    else if (chssis_control_data.chassis_RC->key.v & KEY_PRESSED_OFFSET_D)
    {
        chssis_control_data.vy_speed = VY_MAX_SPEED;
    }

    if(chssis_control_data.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL) //旋转 ctrl
    {
			if(chssis_control_data.vx_speed != 0 && chssis_control_data.vy_speed != 0)
			{
        chssis_control_data.wz_speed = WHEN_IT_MOVE_WZ_MAX_SPEED;
			}
			else
			{
				chssis_control_data.wz_speed = WZ_MAX_SPEED;
			}
    }
		
		if(chssis_control_data.wz_speed != 0)
		{
			get_led_state()->spin_state.state = LED_STATE_YELLOW; 
		}
		else
		{
			get_led_state()->spin_state.state = LED_STATE_OFF; 
		}
		

    //按下shift加速：在最大移动速度上再快个50% - 自定调整
    #if SHIFT_FAST
    if(chssis_control_data.chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
    {
        chssis_control_data.vx_speed *= SHIFT_FAST_RATIO;
        chssis_control_data.vy_speed *= SHIFT_FAST_RATIO;
        chssis_control_data.wz_speed *= SHIFT_FAST_RATIO;
    }
    #endif

    //弹仓盖关闭提示：如果弹仓盖未关闭则速度最大移动速度降低至原来的50%以提示操作手 - 自定调整
    #if COVER_UNCOLSE
    if(chssis_control_data.cover_flag)
    {
        chssis_control_data.vx_speed *= COVER_UNCOLSE_RATIO;
        chssis_control_data.vy_speed *= COVER_UNCOLSE_RATIO;
        chssis_control_data.wz_speed *= COVER_UNCOLSE_RATIO;
    }
    #endif

}
