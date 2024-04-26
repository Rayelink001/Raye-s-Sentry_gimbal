#include "bsp_usart.h"

#include "wt16c.h"
#include "usart.h"
#include "cmsis_os.h"
#include "queue.h"

extern osMessageQId VisionQueueHandle;
uint8_t wt61c_rx_buff;

uint32_t zhongduanhz;

uint8_t usart6_rx_buff[USART6_RX_BUFF_SIZE];
uint8_t usart6_rx_buf_pos = 0;	//USART6 本次回调接收的数据在缓冲区的起点
uint8_t usart6_rx_length = 0;	//USART6 本次回调接收数据的长度

void usart_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, usart6_rx_buff, sizeof(usart6_rx_buff));	//USART6 裁判系统串口 DMA空闲中断接收
	//HAL_UART_Receive_DMA(&huart6, &usart6_rx_buff, sizeof(usart6_rx_buff));//DMA传输
	
	HAL_UART_Receive_DMA(&huart1, &wt61c_rx_buff, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
			wt16c_data_process(wt61c_rx_buff);
			HAL_UART_Receive_DMA(&huart1, &wt61c_rx_buff, 1);//DMA传输
    }
	// else if(huart->Instance == USART6)
	// {
	// 	xQueueSendFromISR(VisionQueueHandle, &usart6_rx_buff, 0);
	// 	HAL_UART_Receive_DMA(&huart6, &usart6_rx_buff, sizeof(usart6_rx_buff));//DMA传输
	// }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART6)
	{	
		usart6_rx_length = Size - usart6_rx_buf_pos;
		
		for(int i = usart6_rx_buf_pos; i < usart6_rx_length; i++)
		{	
			xQueueSendFromISR(VisionQueueHandle, &usart6_rx_buff[i], 0);
		}
		
		usart6_rx_buf_pos += usart6_rx_length;
    
		if (usart6_rx_buf_pos >= (uint16_t)USART6_RX_BUFF_SIZE)
		{
			usart6_rx_buf_pos = 0;	//缓冲区用完后，返回 0 处重新开始
		}

		//HAL_UARTEx_ReceiveToIdle_DMA(&huart6, usart6_rx_buff, sizeof(usart6_rx_buff));
		zhongduanhz++;
	}
}
