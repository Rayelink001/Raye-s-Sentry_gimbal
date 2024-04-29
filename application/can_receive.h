#ifndef __CAN_RECEIVE_H__
#define __CAN_RECEIVE_H__

#include "struct_typedef.h"

/* 与底盘A板的通讯信息 */
#define CAN_CMD_CHASSIS_ID 0x101    //发送底盘控制命令

/* 云台电机 ID标识 */
#define CAN_GIMBAL_ALL_ID    0x1FF
#define CAN_YAW_MOTOR_ID     0x205
#define CAN_PITCH_MOTOR_ID   0x206

/* 机构电机 ID标识 */
#define CAN_FRIC_ALL_ID    0x200
#define CAN_FRIC_LEFT_ID   0x201
#define CAN_FRIC_RIGHT_ID  0x202
#define CAN_TRIGGER_MOTOR_ID 0x203

//RoboMaster电机数据结构
typedef struct
{
    uint16_t ecd;         //霍尔编码值机械角度
    int16_t speed_rpm;    //转速
    int16_t given_current;//电流  
    uint8_t temperate;  //温度
    int16_t last_ecd;   //上一次机械角度
} motor_measure_t;

extern void CAN_cmd_chassis(int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_wz, uint8_t rc_switch);

extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch);

extern void CAN_cmd_shoot(int16_t fric_left, int16_t fric_right, int16_t trigger);

extern const motor_measure_t *get_right_fric_motor_measure_point(void);

extern const motor_measure_t *get_left_fric_motor_measure_point(void);

extern const motor_measure_t *get_trigger_motor_measure_point(void);

extern const motor_measure_t *get_yaw_motor_measure_point(void);

extern const motor_measure_t *get_pitch_motor_measure_point(void);

#endif
