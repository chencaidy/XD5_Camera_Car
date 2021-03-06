#include "common.h"

void Motor_Config(void);			//RS540电机初始化
void SD5_Config(void);				//S-D5舵机初始化
void Encoder_Config(void);		//编码器初始化


/*********************************************************** 
函数名称：Motor_ChangeDuty
函数功能：改变电机占空比
入口参数：val - 占空比(-10000 ~ 10000，负值为反转) 
出口参数：无 
***********************************************************/
void Motor_ChangeDuty(s16 val);

/*********************************************************** 
函数名称：SD5_ChangeDuty
函数功能：改变舵机占空比
入口参数：val - 占空比(0-10000, 中值5000, 已校准) 
出口参数：无 
***********************************************************/
void SD5_ChangeDuty(u16 val);

/*********************************************************** 
函数名称：Encoder_GetCounter
函数功能：获取编码器脉冲数
入口参数：无
出口参数：一段时间内脉冲数 
***********************************************************/
s16 Encoder_GetCounter(void);

/*********************************************************** 
函数名称：Encoder_GetSpeed
函数功能：将一段时间内的脉冲数转化为速度
入口参数：cnt - 一段时间的脉冲数
					ti  - 采样间隔(us)
出口参数：小车当前速度(毫米/秒)
***********************************************************/
s32 Encoder_GetSpeed(s32 cnt, u16 ti);
