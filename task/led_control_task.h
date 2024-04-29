#ifndef _LED_CONTROL_TASK_
#define _LED_CONTROL_TASK_

#include "struct_typedef.h"

typedef enum
{
	LED_STATE_OFF = 0,
	LED_STATE_GREEN,
	LED_STATE_YELLOW,
	LED_STATE_RED
}led_state_e;

typedef struct
{
	uint8_t id;		//LED ID
	led_state_e state;	//颜色
}led_state_t;

typedef struct
{
	led_state_t all_robo_state;		//全车机构状态
	led_state_t supercap_state;		//超电机构状态
	led_state_t shoot_state;		//发射机构状态
	led_state_t cover_state;		//弹仓盖状态
	led_state_t auto_shoot_state;	//自瞄状态
	led_state_t spin_state;			//小陀螺状态
}robo_led_state_t;

extern robo_led_state_t *get_led_state(void);

#endif
