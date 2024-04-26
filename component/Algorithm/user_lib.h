#ifndef __USER_LIB__
#define __USER_LIB__

#include "struct_typedef.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

//绝对限制
extern void abs_limit(fp32 *num, fp32 Limit);

//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

// 三维向量归一化
extern float *Norm3d(float *v);

// 计算模长
extern float NormOf3d(float *v);

// 三维向量叉乘v1 x v2
extern void Cross3d(float *v1, float *v2, float *res);

// 三维向量点乘
extern float Dot3d(float *v1, float *v2);

#endif
