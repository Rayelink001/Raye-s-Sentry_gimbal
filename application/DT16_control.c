#include "DT16_control.h"
#include "main.h"

#include "monitor_task.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

RC_ctrl_t rc_ctrl;  //遥控器控制数据

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM]; //原始数据缓冲区

void DT16_control_init(void) //DT16遥控初始化
{
    DT16_init(sbus_rx_buf[0],sbus_rx_buf[1],SBUS_RX_BUF_NUM);
}

const RC_ctrl_t *get_rc_ctrl_point(void) //获取遥控器数据指针
{
    return &rc_ctrl;
}

void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) //DT16遥控协议数据解析
{
    if(sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    //解析遥控器数据
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;                             //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;                      //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) &0x07ff; //!< Channel 2
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;                      //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                                            //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                                         //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                                         //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                                       //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                                       //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                                       //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                                         //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                                      //!< Channel 4

    //遥控物理中心数值偏移补偿修正
    rc_ctrl->rc.ch[0] -= RC_CH0_VALUE_OFFSET;   //右x摇杆
    rc_ctrl->rc.ch[1] -= RC_CH1_VALUE_OFFSET;   //右y摇杆
    rc_ctrl->rc.ch[2] -= RC_CH2_VALUE_OFFSET;   //左x摇杆 
    rc_ctrl->rc.ch[3] -= RC_CH3_VALUE_OFFSET;   //右y摇杆
    rc_ctrl->rc.ch[4] -= RC_CH4_VALUE_OFFSET;   //拨轮

}

void DT16_usart3_to_rc_data(void) //usart3串口中断
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR * UART_FLAG_IDLE)
    {
        //__HAL_UART_CLEAR_PEFLAG(&huart3);  //DJI的原版操作 但奇偶校验错误有那么频繁吗？
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);  //这里按照逻辑 改为清除IDLE标志位

        static uint16_t this_time_rx_len = 0;

        //这里是用第二种方法手动切换缓冲区 未来可以尝试下第一种方法的自动切换
        if((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)  
        {
            
            __HAL_DMA_DISABLE(&hdma_usart3_rx); //失效DMA

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR; //获取接收数据长度,接收数据长度 = 设定长度 - 剩余长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;    //重置剩余长度为设定长度
            
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;     //切缓冲区2

            __HAL_DMA_ENABLE(&hdma_usart3_rx); //使能DMA

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                //监控防疯
                #if SBUS_MONITOR_SWITCH
                    monitor_beat_hook(RC_BEAT);
                #endif
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart3_rx); //失效DMA

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR; //获取接收数据长度,接收数据长度 = 设定长度 - 剩余长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;    //重置剩余长度为设定长度
            
            hdma_usart3_rx.Instance->CR &= ~(DMA_SxCR_CT);  //切缓冲区1

            __HAL_DMA_ENABLE(&hdma_usart3_rx); //使能DMA

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                //监控防疯
                #if SBUS_MONITOR_SWITCH
                    monitor_beat_hook(RC_BEAT);
                #endif
            }
        }
    }
}
