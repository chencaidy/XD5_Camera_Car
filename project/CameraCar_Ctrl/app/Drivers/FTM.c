#include "common.h"

#define SD5_FTM						FTM2			//���FTM��
#define SD5_Ch 						FTM_Ch0		//������ͨ����
#define SD5_Pin 					PTB18			//���PWM�������

#define Motor_FTM					FTM0			//PWM���FTM��
#define Motor_PWM1_Ch 		FTM_Ch2		//���PWM1ͨ����
#define Motor_PWM1_Pin 		PTA5			//���PWM1�������
#define Motor_PWM2_Ch 		FTM_Ch1		//���PWM2ͨ����
#define Motor_PWM2_Pin 		PTA4			//���PWM2�������

#define Encoder_FTM				FTM1			//��������FTM��
#define Encoder_A 				PTA12			//��������Aͨ��
#define Encoder_B 				PTA13			//��������Bͨ��

u16 SD5_Conf[3] = {210, 3910, 6090}; 		//{�����ֵƫ��, ��������, �Ҵ������}//240

/********************************************************/
/*                ������ �����غ��� ������                  */
/********************************************************/
FTM_InitTypeDef ftm_init_structure;
void Motor_Config()
{
	ftm_init_structure.FTM_Ftmx = Motor_FTM;
	ftm_init_structure.FTM_Mode = FTM_MODE_PWM;
	ftm_init_structure.FTM_PwmFreq = 10000;
	ftm_init_structure.FTM_PwmDeadtimeCfg = DEADTIME_CH01;
	ftm_init_structure.FTM_PwmDeadtimeDiv = DEADTIME_DIV1;
	ftm_init_structure.FTM_PwmDeadtimeVal = 0;
	LPLD_FTM_Init(ftm_init_structure);
	LPLD_FTM_PWM_Enable(Motor_FTM, Motor_PWM1_Ch, 0, Motor_PWM1_Pin, ALIGN_RIGHT);
	LPLD_FTM_PWM_Enable(Motor_FTM, Motor_PWM2_Ch, 0, Motor_PWM2_Pin, ALIGN_RIGHT);
}

void Motor_ChangeDuty(s16 val)
{
	if(val>=0)
	{
		LPLD_FTM_PWM_ChangeDuty(Motor_FTM, Motor_PWM1_Ch, 0);
		LPLD_FTM_PWM_ChangeDuty(Motor_FTM, Motor_PWM2_Ch, val);
	}
	else
	{
		LPLD_FTM_PWM_ChangeDuty(Motor_FTM, Motor_PWM2_Ch, 0);
		LPLD_FTM_PWM_ChangeDuty(Motor_FTM, Motor_PWM1_Ch, -val);
	}
}

/********************************************************/
/*                ������ �����غ��� ������                  */
/********************************************************/
void SD5_Config()
{
	ftm_init_structure.FTM_Ftmx = SD5_FTM;
	ftm_init_structure.FTM_Mode = FTM_MODE_PWM;
	ftm_init_structure.FTM_PwmFreq = 300;
	ftm_init_structure.FTM_PwmDeadtimeCfg = DEADTIME_CH01;
	ftm_init_structure.FTM_PwmDeadtimeDiv = DEADTIME_DIV1;
	ftm_init_structure.FTM_PwmDeadtimeVal = 0;
	LPLD_FTM_Init(ftm_init_structure);
	LPLD_FTM_PWM_Enable(SD5_FTM, SD5_Ch, 5000-SD5_Conf[0], SD5_Pin, ALIGN_RIGHT);
}

void SD5_ChangeDuty(u16 val)
{
	if(val<SD5_Conf[1])
	{val = SD5_Conf[1];}
	
	if(val>SD5_Conf[2])
	{val = SD5_Conf[2];}
	
	LPLD_FTM_PWM_ChangeDuty(SD5_FTM, SD5_Ch, val-SD5_Conf[0]);
}

/********************************************************/
/*               ������ ��������غ��� ������                 */
/********************************************************/
void Encoder_Config()
{
	ftm_init_structure.FTM_Ftmx = Encoder_FTM;
	ftm_init_structure.FTM_Mode = FTM_MODE_QD;
	ftm_init_structure.FTM_QdMode = QD_MODE_CNTDIR;
	LPLD_FTM_Init(ftm_init_structure);
	LPLD_FTM_QD_Enable(Encoder_FTM, Encoder_A, Encoder_B);
}

s16 Encoder_GetCounter()
{
	s16 Count;
	
	Count = -LPLD_FTM_GetCounter(Encoder_FTM);
	LPLD_FTM_ClearCounter(Encoder_FTM);
	
	return Count;
}

s32 Encoder_GetSpeed(s32 cnt, u16 ti)
{
	s32 Count;
	double Speed;
	
	Count = cnt;
	Speed = Count*144586/ti;
	//��Counter/512*0.419*6.283*0.02812*1000=(����/��λ����ʱ��)��
	//����������һȦ�����������ֱȡ�2�С���̥�뾶(m)��1000���Ŵ�
	
	return (int)Speed;
}
/********************************************************/
