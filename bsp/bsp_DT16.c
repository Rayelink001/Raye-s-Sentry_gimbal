#include "bsp_DT16.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* DT16遥控接收器初始化函数 */
void DT16_init(uint8_t *rx1_buf,uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    SET_BIT(huart3.Instance->CR3,USART_CR3_DMAR);   //使能DMA接收器

    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);    //使能串口空闲中断
    
    __HAL_DMA_DISABLE(&hdma_usart3_rx);   //失效DMA数据流 以配置DMA寄存器
    while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)//检查是否完成失效DMA
    {
		  __HAL_DMA_DISABLE(&hdma_usart3_rx);//未完成则继续失效DMA 直至完成为止（必须关掉, 因为不能再DMA数据流开启时，配置DMA寄存器）
    }

    //DMA寄存器配置
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);//配置外设地址
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);//配置 内存缓冲区1 地址
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);//配置 内存缓冲区2 地址
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;//配置DMA传数据流长度
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);   //开启双缓冲区模式
   
    __HAL_DMA_ENABLE(&hdma_usart3_rx);  //使能DMA数据流
    
}

/* 
    https://blog.csdn.net/qq_53671582/article/details/123547534
    这篇文章详细说明了以上的所有操作，请为所有电控组软件方向成员将这篇文章通读。
*/
