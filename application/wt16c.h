#ifndef __WT16C_H__
#define __WT16C_H__

#include "struct_typedef.h"

#define WT61C_DATA_READY 0
#define WT61C_FINED_SOF 1

#define WT61C_DATA_SOF 0x55 
#define WT61C_ACCEL_ID 0x51
#define WT61C_GYRO_ID 0x52
#define WT61C_ANGLE_ID 0x53

//fp32转uint8联合体
typedef union
{
	fp32 f;
	uint8_t c[4];
	
} fp32uint8;

typedef struct
{
    fp32uint8 gyro[3];	//角速度
    fp32uint8 accel[3];	//加速度
    fp32uint8 angle[3];	//角度
		fp32uint8 quat[4];	//四元数

}wt61c_data_t;

typedef struct
{
  fp32 real_pitch;
  fp32 real_roll;
  fp32 real_yaw;
}euler_angle_t;

extern void wt16c_data_process(uint8_t rx_buff);
extern const wt61c_data_t *get_wt61c_data_point(void);
extern const euler_angle_t *get_euler_angle_real_point(void);

#endif
