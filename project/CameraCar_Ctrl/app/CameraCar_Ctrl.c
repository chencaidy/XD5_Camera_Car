#include "common.h"
#include "Comm_Slave.h"
#include "FTM_Drv.h"
#include "PIT_Drv.h"
#include "PID.h"

extern u8 PIT_Flag;

s16 SpeedGoal=10000;		//��ǰֵ���Թر� PID���� while(1)�д���

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
				SpeedPID(SpeedGoal);		//����PID������
			}
			else
			{
				Motor_ChangeDuty(0);		//�ر�PID������
				GetSpeed();							//�����ٶȵĻ�ȡ
			}		
			
			PIT_Flag = 0;	
		}
	}
}
