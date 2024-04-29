#ifndef __VISION_TASK__
#define __VISION_TASK__

#include "struct_typedef.h"
#include "wt16c.h"

#define VISION_RX_DATA_LENGTH 22//视觉最大数据长度
#define VISION_DATA_LENGTH 17//数据段长度大小

#define VISION_SOF 0xA5//视觉数据帧头SOF

/* 视觉数据识别码ID　*/
#define CMDID0 0x00 //关闭视觉
#define CMDID1 0x01 //自瞄
#define CMDID2 0x02	//前哨
#define CMDID3 0x03 //小符
#define CMDID4 0x04 //大符

//视觉通信解包状态机
typedef enum
{
  VISION_PARSED_STATE_SOF = 0,        //寻找帧起始
  VISION_PARSED_STATE_CMDID,          //获取模式命令码
  VISION_PARSED_STATE_CRC8,           //CRC8帧头校验
  VISION_PARSED_STATE_DATA,           //获取视觉数据
  VISION_PARSED_STATE_CRC16_HIGH,     //获取CRC16校验码 高八位
  VISION_PARSED_STATE_CRC16_LOW       //获取CRC16校验码 低八位

} vision_parsed_state_e;

//视觉数据结构体
typedef struct
{
    uint8_t vision_mode;              //当前视觉模式 
    fp32 quat[4];                     //四元数 uint8 
    fp32 gyro[3];                     //陀螺仪 - 角速度
    fp32 acc[3];                      //加速度计 -加速度
	
		const wt61c_data_t *wt61c_data;					//外置陀螺仪数据指针
    fp32 bullet_speed;                //弹丸初速度
    
    fp32uint8 yaw_angle;         //yaw轴角度
    fp32uint8 pitch_angle;       //pitch角度
    fp32uint8 dis;               //目标距离
    uint8_t is_switched;        //是否切换目标
    uint8_t is_find_target;     //是否扫到目标
    uint8_t is_spinningl;       //目标是否处于小陀螺状态
    uint8_t is_middle;          //目标是否达到开火范围
} vision_data_t;

extern const vision_data_t *get_vision_data_point(void);	//获取视觉数据指针
extern void vision_data_transmit(const wt61c_data_t *data); //转发视觉数据 - 陀螺仪数据和视觉数据一起转发
#endif
