#include "wt16c.h"
#include "vision_task.h"
#include "arm_math.h"

#include "main.h"

#include <math.h>
euler_angle_t euler_angle;
wt61c_data_t wt61c_data;    //wt61c数据

uint8_t wt61c_data_buff[11];
uint8_t wt61c_process_flag = WT61C_DATA_READY;

float roll_round, pitch_round, yaw_round;

static int8_t wt61c_data_length = 0;
static int16_t wt61c_buff[3];
unsigned char crc_sum_check(const unsigned char *buf, int len);	//校验和
void wt61c_get_quaternions(void);//计算四元数

void wt16c_data_process(uint8_t rx_buff)    //wt61c数据处理与归档
{
    float last_pitch_angle = wt61c_data.angle[0].f;
    float last_roll_angle = wt61c_data.angle[1].f;
    float last_yaw_angle = wt61c_data.angle[2].f;

    wt61c_data_buff[wt61c_data_length] = rx_buff;
		wt61c_data_length++;
	
    switch (wt61c_process_flag)
    {
        case WT61C_DATA_READY:
            if(rx_buff == WT61C_DATA_SOF)
            {
	            wt61c_process_flag = WT61C_FINED_SOF;
            }
            else
            {
                wt61c_data_length = 0;
            }
        break;

        case WT61C_FINED_SOF:
            if(wt61c_data_length >= 11)
            {
                if(crc_sum_check(wt61c_data_buff,10) == wt61c_data_buff[10]) //校验
                {
                    switch (wt61c_data_buff[1]) //根据帧头数据
                    {
                        case WT61C_ACCEL_ID:
													for(int8_t i = 0; i < 3; i++)
													{
															wt61c_buff[i] = (wt61c_data_buff[3+2*i] << 8 | wt61c_data_buff[2+2*i]);
															wt61c_data.accel[i].f = (fp32)wt61c_buff[i] / 32768.0f * 156.8f;
													}                
                           break;
                        case WT61C_GYRO_ID:
													for(int8_t i = 0; i < 3; i++)
													{
															wt61c_buff[i] = (wt61c_data_buff[3+2*i] << 8 | wt61c_data_buff[2+2*i]);
															wt61c_data.gyro[i].f = (fp32)wt61c_buff[i] / 32768.0f * 2000.0f;
													}     
                            break;
                        case WT61C_ANGLE_ID:
													for(int8_t i = 0; i < 3; i++)
													{
															wt61c_buff[i] = (wt61c_data_buff[3+2*i] << 8 | wt61c_data_buff[2+2*i]);
															wt61c_data.angle[i].f = (fp32)wt61c_buff[i] / 32768.0f * 180.0f;
													} 

												//过零处理
												
												// if(wt61c_data.angle[0].f - last_pitch_angle > 300.0f)
												// 		roll_round --;
												// else if(wt61c_data.angle[0].f - last_pitch_angle < -300.0f)
												// 		roll_round ++;
												// euler_angle.real_pitch = wt61c_data.angle[0].f + roll_round * 360;
												
												// if(wt61c_data.angle[1].f - last_roll_angle > 300.0f)
												// 		roll_round --;
												// else if(wt61c_data.angle[1].f - last_roll_angle < -300.0f)
												// 		roll_round ++;
												// euler_angle.real_roll = wt61c_data.angle[1].f + roll_round * 360;
												

                            break;
                        default:
                            break;
                        }

                                                                            euler_angle.real_pitch = wt61c_data.angle[0].f;

												if(wt61c_data.angle[2].f - last_yaw_angle > 300.0f)
														yaw_round --;
												else if(wt61c_data.angle[2].f - last_yaw_angle < -300.0f)
														yaw_round ++;
												euler_angle.real_yaw = wt61c_data.angle[2].f + yaw_round * 360;

												wt61c_get_quaternions();//计算四元数
                                               


										//vision_data_transmit(wt61c_data);//发送给视觉 - 会导致陀螺仪解算延迟巨大
										wt61c_data_length = 0;
                    wt61c_process_flag = WT61C_DATA_READY;
									}
									else
									{
                    wt61c_data_length = 0;
                    wt61c_process_flag = WT61C_DATA_READY;
									}
            }
        break;

        default:
        break;
    }
}

unsigned char crc_sum_check(const unsigned char *buf, int len)  
{  
    int iSum = 0; 
    for (int i = 0;i < len;i++)  
    {  
        iSum += buf[i];  
    }  
    iSum %= 0x100;   //也可以&0xff
    return (unsigned char)iSum;  
}

const wt61c_data_t *get_wt61c_data_point(void) 
{
    return &wt61c_data;
}

const euler_angle_t *get_euler_angle_real_point(void) 
{
    return &euler_angle;
}

//通过既得角度转化为出四元数
void wt61c_get_quaternions(void)
{

    fp32 cy = arm_cos_f32(euler_angle.real_yaw * 0.5f / 57.92f);
    fp32 sy = arm_sin_f32(euler_angle.real_yaw * 0.5f / 57.92f);
    fp32 cp = arm_cos_f32(euler_angle.real_pitch * 0.5f / 57.92f);
    fp32 sp = arm_sin_f32(euler_angle.real_pitch * 0.5f / 57.92f);
    fp32 cr = arm_cos_f32(euler_angle.real_roll * 0.5f / 57.92f);
    fp32 sr = arm_sin_f32(euler_angle.real_roll * 0.5f / 57.92f);
 
    wt61c_data.quat[0].f = cy * cp * cr + sy * sp * sr;	//w
    wt61c_data.quat[1].f = cy * cp * sr - sy * sp * cr;	//x
    wt61c_data.quat[2].f = sy * cp * sr + cy * sp * cr;	//y
    wt61c_data.quat[3].f = sy * cp * cr - cy * sp * sr;	//z
}

