#include "led_control_task.h"

#include "main.h"
#include "cmsis_os.h"

#include "RGB.h"

robo_led_state_t robo_led_state = 
{
	.all_robo_state =
	{
		.id = 0,
		.state = LED_STATE_GREEN
	},
	
	.supercap_state = 
	{
		.id = 1,
		.state = LED_STATE_OFF
	},
	
	.shoot_state =
	{
		.id = 2,
		.state = LED_STATE_GREEN
	},
	
	.cover_state =
	{
		.id = 3,
		.state = LED_STATE_GREEN
	},
	
	.auto_shoot_state =
	{
		.id = 4,
		.state = LED_STATE_OFF
	},
	
	.spin_state =
	{
		.id = 5,
		.state = LED_STATE_OFF
	}
};

void led_control(led_state_t led);

void led_control_task(void const *pvParameters)  
{
    vTaskDelay(10);	//等待一下系统加载
		
		RGB_BLACK(6);	//初始化
		HAL_Delay(5);
	
    while(1)
    {

      	led_control(robo_led_state.all_robo_state);
		led_control(robo_led_state.supercap_state);
		led_control(robo_led_state.shoot_state);
		led_control(robo_led_state.cover_state);
		led_control(robo_led_state.auto_shoot_state);
		led_control(robo_led_state.spin_state);
			
      vTaskDelay(10);
    }
}


//状态控制集合函数
void led_control(led_state_t led)
{
	RGB_BLACK(6);

	switch(led.state)
	{
		case LED_STATE_OFF: 
			RGB_ON_BLACK(led.id);
			break;
		
		case LED_STATE_GREEN: 
			RGB_ON_GREEN(led.id);	
			break;

		case LED_STATE_YELLOW: 
			RGB_ON_OEANGE(led.id);		
			break;

		case LED_STATE_RED: 
			RGB_ON_RED(led.id);
			break;

		default:
			break;
	}
	
	HAL_Delay(1);
}

//获取led状态指针
robo_led_state_t *get_led_state(void)
{
	return &robo_led_state;
} 

