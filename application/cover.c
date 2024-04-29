#include "cover.h"

#include "main.h"

extern TIM_HandleTypeDef htim8;

cover_state_e cover_state;

//弹仓盖 开
void cover_on(void)
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1000);
    cover_state = COVER_ON;
}

//弹仓盖 关
void cover_off(void)
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 2020);
    cover_state = COVER_OFF;
}

//弹仓盖初始化
void cover_init(void)
{
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

    cover_off();
}

cover_state_e get_cover_state_point(void)
{
    return cover_state;
}



