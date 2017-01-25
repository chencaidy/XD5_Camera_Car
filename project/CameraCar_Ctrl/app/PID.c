#include "common.h"
#include "PID.h"
#include "FTM_Drv.h"

double SpeedKP=5, SpeedKI=0.5, SpeedKD=0.6;		//*速度P、I、D
double SpeedPWM=0;
s32 err=0, err_last=0, err_lastbefore=0;

s32 Count, Speed, PWM;

void GetSpeed()
{
	Count = Encoder_GetCounter();
	Speed = Encoder_GetSpeed(Count, 10000);
}

void SpeedPID(s32 GoalSpeed)	//速度PID调节,增量式
{
	s32 ActualSpeed;
	double IncrementSpeed;

	GetSpeed();
	ActualSpeed = Speed;
	
	err = GoalSpeed-ActualSpeed;
	IncrementSpeed = SpeedKP*(err-err_last)+SpeedKI*err+SpeedKD*(err-2*err_last+err_lastbefore);
	SpeedPWM += IncrementSpeed;
	err_lastbefore = err_last;
	err_last = err;
	
	if(SpeedPWM>10000)
	{SpeedPWM = 10000;}
	else if(SpeedPWM<-10000)
	{SpeedPWM = -10000;}
	
	PWM = (int)SpeedPWM;
	
	Motor_ChangeDuty(PWM);
}
