#include "common.h"
#include "Comm_Slave.h"
#include "FTM_Drv.h"
#include "PIT_Drv.h"
#include "PID.h"

extern u8 PIT_Flag;

s16 SpeedGoal=10000;		//当前值可以关闭 PID，见 while(1)中代码

void main()
{
	Comm_Config();
	Motor_Config();
	SD5_Config();
	Encoder_Config();
	PIT_Config(10);
	
	while(1)
	{
		if(PIT_Flag==1)
		{
			if(SpeedGoal<=5000)
			{
				SpeedPID(SpeedGoal);		//开启PID控制器
			}
			else
			{
				Motor_ChangeDuty(0);		//关闭PID控制器
				GetSpeed();							//保持速度的获取
			}		
			
			PIT_Flag = 0;	
		}
	}
}
