#include "bsp_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;    //定义 CAN接收过滤器配置定义结构体变量

    can_filter_st.FilterActivation = ENABLE;    //使能CAN接收过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;   //启用掩码模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;  //设置过滤器位宽32bit
    can_filter_st.FilterIdHigh = 0x0000;    //设置过滤器验证码的高16位
    can_filter_st.FilterIdLow = 0x0000; //设置过滤器验证码的低16位
    can_filter_st.FilterMaskIdHigh = 0x0000; //设置过滤器掩码的高16位
    can_filter_st.FilterMaskIdLow = 0x0000; //设置过滤器掩码的低16位
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//设置经过滤后的数据存储到CAN_RX_FIFO0当中
    can_filter_st.FilterBank = 0;   //设置过滤器编号
    
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st); //对CAN1过滤器进行配置
    HAL_CAN_Start(&hcan1);  //使能CAN1
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //激活CAN1接收  
    
    can_filter_st.SlaveStartFilterBank = 14; //根据说明启用多个CAN总线时需要额外进行配置
    can_filter_st.FilterBank = 14;   //设置过滤器编号
    
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);   //对CAN2过滤器进行配置
    HAL_CAN_Start(&hcan2);  //使能CAN2
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);  //激活CAN2接收  
    
    //通过CAN过滤器的接收报文 将通过HAL_CAN_RxFifo0MsgPendingCallback函数处理
}

//查阅互联网资料这里会有个“想使用CAN2就得配置CAN1”的神奇操作在这里，暂时按下不做处理
//如果“不配置CAN1 CAN2就用不了”的情况出现了，请及时修改这里的初始化
