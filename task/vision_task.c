#include "vision_task.h"    

#include "cmsis_os.h"
#include "queue.h"    //队列支持
#include "crc.h"      //CRC校验用
#include "wt16c.h"

#include "usart.h"

#include <string.h>

extern osMessageQId VisionQueueHandle;

vision_data_t vision_data; //视觉全局数据

void vision_data_parsed(uint8_t data[VISION_RX_DATA_LENGTH]); //视觉数据解包函数
const vision_data_t *get_vision_data_point(void); //获得视觉数据指针
void vision_data_transmit(const wt61c_data_t *data);

uint8_t tx_buff[50];

//数据缓冲区
uint8_t referee_received_data_buff[VISION_RX_DATA_LENGTH] = {0};

void vision_task(void const *pvParameters)
{
    /* 视觉解析状态机 */
    vision_parsed_state_e vision_parsed_state_flag = VISION_PARSED_STATE_SOF;
    /* 解析出来的指针ID */
    uint16_t vision_parsed_cmd_id = 0;
    /* 已接收的数据长度 */    
    uint16_t vision_parsed_received_length = 0;

    while (1)
    {
        uint8_t vision_rx_data = 0;
				
				//if(0)
        if(xQueueReceive(VisionQueueHandle, &vision_rx_data, portMAX_DELAY) == pdTRUE)
        {
            referee_received_data_buff[vision_parsed_received_length++] = vision_rx_data;

            switch (vision_parsed_state_flag)
            {
                case VISION_PARSED_STATE_SOF: //寻找SOF
                {
                    /*初始化 状态机变量*/     
                    vision_parsed_cmd_id = 0;         //清零命令码

                    if(vision_rx_data == VISION_SOF)  //寻找到帧头
                    {
                        vision_parsed_state_flag = VISION_PARSED_STATE_CMDID;   
                    }
                    else
                    {
                        vision_parsed_received_length = 0; //清零已获数据长度
                    }

                    break;
                }

                case VISION_PARSED_STATE_CMDID: //获取命令码
                {
                    vision_parsed_state_flag = VISION_PARSED_STATE_CRC8;
                    break;
                }

                case VISION_PARSED_STATE_CRC8: //CRC8校验
                {
                    if(verify_CRC8_check_sum(referee_received_data_buff,3))
                    {
                        vision_parsed_state_flag = VISION_PARSED_STATE_DATA;
                    }
                    else
                    {
                        vision_parsed_state_flag = VISION_PARSED_STATE_SOF;
                    }
                    break;
                }

                case VISION_PARSED_STATE_DATA: //获取数据 帧头+数据长度 固定
                {
                    if(vision_parsed_received_length == 20)
                    {
                        vision_parsed_state_flag = VISION_PARSED_STATE_CRC16_HIGH;
                    }
                    break;
                }

                case VISION_PARSED_STATE_CRC16_HIGH: //获取CRC16校验码 高八位
                {
                    vision_parsed_state_flag = VISION_PARSED_STATE_CRC16_LOW;
                    break;
                }

                case VISION_PARSED_STATE_CRC16_LOW: //获取CRC16校验码 低八位
                {
                    if(verify_CRC16_check_sum(referee_received_data_buff,22)) //若通过CRC16校验 进行数据解包
                    {
                        vision_data_parsed(referee_received_data_buff);
                    }
										
                    vision_parsed_state_flag = VISION_PARSED_STATE_SOF; //无论是否通过校验 都清零重新查找帧头SOF
                    vision_parsed_received_length = 0; //清零已获数据长度

                    break;
                }
                 
                default:
                    break;
            }
        }
       
	    vTaskDelay(1);
    }    
}

//视觉发送任务
void vision_send_task(void const *pvParameters)
{
	vTaskDelay(10);
	vision_data.wt61c_data = get_wt61c_data_point();    //绑定陀螺仪数据
	while(1)
	{
		vision_data_transmit(vision_data.wt61c_data);   //发送陀螺仪数据
		vTaskDelay(10);
	}
}

void vision_data_parsed(uint8_t data[VISION_RX_DATA_LENGTH])   //视觉数据解包
{
    memmove(&vision_data.pitch_angle,&data[3],4);   //获取pitch角度
    memmove(&vision_data.yaw_angle,&data[7],4);     //获取yaw角度
    memmove(&vision_data.dis,&data[11],4);          //获取目标距离  
    //对角度数据进行处理
    vision_data.is_switched = data[15];     //目标是否切换
    vision_data.is_find_target = data[16];  //是否扫描到目标
    vision_data.is_spinningl = data[17];    //目标是否小陀螺
    vision_data.is_middle =  data[18];      //目标是否在打击范围内
}

void vision_data_transmit(const wt61c_data_t *data) //视觉数据发送 - 这里主要给 设置的视觉模式 四元数 陀螺仪数据 弹丸初速度
{
	
    tx_buff[0] = VISION_SOF;
    tx_buff[1] = 0x01;
    append_CRC8_check_sum(tx_buff,3);
	
	//外置陀螺仪 wt61c 四元数 角速度 加速度
        
    tx_buff[3] = data->quat[0].c[0];
    tx_buff[4] = data->quat[0].c[1];
    tx_buff[5] = data->quat[0].c[2];
    tx_buff[6] = data->quat[0].c[3];

    tx_buff[7] = data->quat[1].c[0];
    tx_buff[8] = data->quat[1].c[1];
    tx_buff[9] = data->quat[1].c[2];
    tx_buff[10] = data->quat[1].c[3];
    
    tx_buff[11] = data->quat[2].c[0];
    tx_buff[12] = data->quat[2].c[1];
    tx_buff[13] = data->quat[2].c[2];
    tx_buff[14] = data->quat[2].c[3];
        
    tx_buff[15] = data->quat[3].c[0];
    tx_buff[16] = data->quat[3].c[1];
    tx_buff[17] = data->quat[3].c[2];
    tx_buff[18] = data->quat[3].c[3];
    
    tx_buff[19] = data->gyro[0].c[0];
    tx_buff[20] = data->gyro[0].c[1];
    tx_buff[21] = data->gyro[0].c[2];
    tx_buff[22] = data->gyro[0].c[3];
    
    tx_buff[23] = data->gyro[1].c[0];
    tx_buff[24] = data->gyro[1].c[1];
    tx_buff[25] = data->gyro[1].c[2];
    tx_buff[26] = data->gyro[1].c[3];
    
    tx_buff[27] = data->gyro[2].c[0];
    tx_buff[28] = data->gyro[2].c[1];
    tx_buff[29] = data->gyro[2].c[2];
    tx_buff[30] = data->gyro[2].c[3];
    
    tx_buff[31] = data->accel[0].c[0];
    tx_buff[32] = data->accel[0].c[1];
    tx_buff[33] = data->accel[0].c[2];
    tx_buff[34] = data->accel[0].c[3];
    
    tx_buff[35] = data->accel[1].c[0];
    tx_buff[36] = data->accel[1].c[1];
    tx_buff[37] = data->accel[1].c[2];
    tx_buff[38] = data->accel[1].c[3];
    
    tx_buff[39] = data->accel[2].c[0];
    tx_buff[40] = data->accel[2].c[1];
    tx_buff[41] = data->accel[2].c[2];
    tx_buff[42] = data->accel[2].c[3];
    
	//弹速
    tx_buff[43] = (uint8_t)((int32_t)vision_data.bullet_speed >> 24);     //tx_buff[43] - tx_buff[46] fp32 = 4X8
    tx_buff[44] = (uint8_t)((int32_t)vision_data.bullet_speed >> 16);
    tx_buff[45] = (uint8_t)((int32_t)vision_data.bullet_speed >> 8);
    tx_buff[46] = (uint8_t)((int32_t)vision_data.bullet_speed);

    tx_buff[47] = 0x00;

    append_CRC16_check_sum(tx_buff,50);         //tx_buff[48] - tx_buff[49] CRC16校验
    
		HAL_UART_Transmit_DMA(&huart6,tx_buff,sizeof(tx_buff));
}

const vision_data_t *get_vision_data_point(void) //获取视觉数据指针
{
    return &vision_data;
}
