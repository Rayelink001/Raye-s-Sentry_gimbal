#include "can_receive.h"

#include "main.h"
#include "struct_typedef.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
receive_broad_a_t broad_type_a;
motor_measure_t motor_gimbal_data[2]; //云台二电机 YAW PIT
motor_measure_t motor_shoot_data[3];	//发射三电机 left right trigger

CAN_TxHeaderTypeDef can1_tx_message;
CAN_TxHeaderTypeDef can2_tx_message;

uint8_t can1_send_data[8];
uint8_t can2_send_data[8];

void get_cmd_form_broad_type_a(uint8_t rx_data[8]);

//CAN接收回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    static uint8_t i = 0;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
            case CAN_YAW_MOTOR_ID:
            case CAN_PITCH_MOTOR_ID:
            {
                i = rx_header.StdId - CAN_YAW_MOTOR_ID; //获取对应i值
                get_motor_measure(&motor_gimbal_data[i], rx_data);
                break;
            }
            case 0x100:
            {
                get_cmd_form_broad_type_a( rx_data);
                break;
            }
            default:
            break;
        }
    }
    else if(hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
            case CAN_FRIC_LEFT_ID:	    //左摩擦轮
            {
                get_motor_measure(&motor_shoot_data[0], rx_data);
                break;
            }
            case CAN_FRIC_RIGHT_ID:	    //右摩擦轮
            {
                get_motor_measure(&motor_shoot_data[1], rx_data);
                break;
            }
            case CAN_TRIGGER_MOTOR_ID:  //拨弹电机
            {
                get_motor_measure(&motor_shoot_data[2], rx_data);
                break;
            }
            
            default:
            break;
        }
    }
}
void get_cmd_form_broad_type_a(uint8_t rx_data[8])
{
    broad_type_a.shoot_heat = (int16_t)(rx_data[0] << 8 | rx_data[1]);
    broad_type_a.game_progress =(int16_t) rx_data[2]; //case 0x30 delay5秒
    broad_type_a.gametime = (int16_t)(rx_data[3] << 8 | rx_data[4]);
}

//发送给底盘的控制信号CAN2(0x301) chassis_vx前后方向速度 前为正，chassis_vy左右方向速度 左为正，chassis_wz 旋转方向速度，顺时针为正
void CAN_cmd_chassis(int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_wz, uint8_t rc_switch)
{
    uint32_t send_mail_box;
	can2_tx_message.IDE = CAN_ID_STD;
    can2_tx_message.RTR = CAN_RTR_DATA;
    can2_tx_message.DLC = 0x08;
	
    can2_tx_message.StdId = CAN_CMD_CHASSIS_ID;
	
    can2_send_data[0] = (chassis_vx >> 8);
    can2_send_data[1] = chassis_vx;
    can2_send_data[2] = (chassis_vy >> 8);
    can2_send_data[3] = chassis_vy;
    can2_send_data[4] = (chassis_wz >> 8);
    can2_send_data[5] = chassis_wz;
    can2_send_data[6] = rc_switch;
    can2_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &can2_tx_message, can2_send_data, &send_mail_box);
}

//发送云台电机控制电流CAN2(0x205,0x206)
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch) 
{
    uint32_t send_mail_box;
		
	can2_tx_message.IDE = CAN_ID_STD;
    can2_tx_message.RTR = CAN_RTR_DATA;
    can2_tx_message.DLC = 0x08;
	
    can2_tx_message.StdId = CAN_GIMBAL_ALL_ID;

    can2_send_data[0] = (yaw >> 8);
    can2_send_data[1] = yaw;
    can2_send_data[2] = (pitch >> 8);
    can2_send_data[3] = pitch;
    can2_send_data[4] = 0;
    can2_send_data[5] = 0;
    can2_send_data[6] = 0;
    can2_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &can2_tx_message, can2_send_data, &send_mail_box);
}

//发送发射机构电机控制电流CAN1(0x201,0x202，0x03)
void CAN_cmd_shoot(int16_t fric_left, int16_t fric_right, int16_t trigger) 
{
    uint32_t send_mail_box;
    can1_tx_message.StdId = CAN_FRIC_ALL_ID;
    can1_tx_message.IDE = CAN_ID_STD;
    can1_tx_message.RTR = CAN_RTR_DATA;
    can1_tx_message.DLC = 0x08;
    can1_send_data[0] = (fric_left >> 8);
    can1_send_data[1] = fric_left;
    can1_send_data[2] = (fric_right >> 8);
    can1_send_data[3] = fric_right;
    can1_send_data[4] = (trigger >> 8);
    can1_send_data[5] = trigger;
    can1_send_data[6] = 0;
    can1_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &can1_tx_message, can1_send_data, &send_mail_box);
}

//返回摩擦轮左电机 3508电机数据指针
const motor_measure_t *get_left_fric_motor_measure_point(void)  
{
    return &motor_shoot_data[0];
}

//返回摩擦轮右电机 3508电机数据指针
const motor_measure_t *get_right_fric_motor_measure_point(void)  
{
    return &motor_shoot_data[1];
}

//返回拨弹电机 2006电机数据指针 注意2006使用C610电调 因此没有温度值
const motor_measure_t *get_trigger_motor_measure_point(void)  
{
    return &motor_shoot_data[2];
}


//返回yaw 6020电机数据指针
const motor_measure_t *get_yaw_motor_measure_point(void) 
{
    return &motor_gimbal_data[0];
}

//返回pitch 6020电机数据指针
const motor_measure_t *get_pitch_motor_measure_point(void) 
{
    return &motor_gimbal_data[1];
}
const receive_broad_a_t *get_U_measure_point(void) 
{
    return &broad_type_a;
}

