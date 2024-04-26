#ifndef DT16_CONTROL_H
#define DT16_CONTROL_H

#include "bsp_DT16.h"

#define SBUS_MONITOR_SWITCH 1 //遥控器监控开关 0关 1开 当没有使用遥控器的时候需要进行关闭，否则云台可能无法使用

/* 遥控器数据长度设置 */
#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

/* 遥控死区限制 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/* 遥杆拨杆通道值宏定义 */
#define RIGHT_CHANNAL_X 0      //右摇杆X通道
#define RIGHT_CHANNAL_Y 1       //右摇杆Y通道
#define LEFT_CHANNAL_X 3        //左摇杆X通道
#define LEFT_CHANNAL_Y 2        //左摇杆Y通道
#define WHEEL_CHANNAL 4         //拨杆通达

/*遥控器物理中心偏移补偿值*/
#define RC_CH0_VALUE_OFFSET      ((uint16_t)1024) //右x摇杆
#define RC_CH1_VALUE_OFFSET      ((uint16_t)1024) //右y摇杆
#define RC_CH2_VALUE_OFFSET      ((uint16_t)1024) //左x摇杆
#define RC_CH3_VALUE_OFFSET      ((uint16_t)1024) //左u摇杆
#define RC_CH4_VALUE_OFFSET      ((uint16_t)1024) //拨轮

/* ----------------------- 遥控器开关 定义----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

#define RC_SWITCH_LEFT 1       //左拨码开关
#define RC_SWITCH_RIGHT 0      //右拨码开关
/* ----------------------- 键盘按键 定义-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- 遥控数据结构体 ----------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;


extern void DT16_control_init(void); ///DT16遥控初始化
extern void DT16_usart3_to_rc_data(void); //USART3数据处理函数 放入USART3_IRQHandler中使用
extern const RC_ctrl_t *get_rc_ctrl_point(void); //获取遥控器数据指针

#endif
