#ifndef _COVER_H_
#define _COVER_H_

#include "struct_typedef.h"


typedef enum 
{
    COVER_OFF = 0,
    COVER_ON
}cover_state_e;

extern void cover_init(void);   //弹仓盖初始化
extern void cover_off(void);    //弹仓盖 关
extern void cover_on(void);     //弹仓盖 开
extern cover_state_e get_cover_state_point(void);

#endif
