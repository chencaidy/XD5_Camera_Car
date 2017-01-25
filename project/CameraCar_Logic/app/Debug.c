#include "Debug.h"
#include "HMI.h"
#include "OV7725.h"
#include "OLED12864.h"
#include "Comm_Master.h"

/**  ���� value  **************************************************************/
PIT_InitTypeDef pit_init_structure;
char str[32];		//��ʱ������
u8 PIT_Flag=0;		//PIT�жϱ�־
u8 MPU6050_Cnt=0;			//MPU6050���ݽ��ռ���
u16 LoopTime=0;		//Whileѭ������

/**  ���� extern  *************************************************************/
extern s16 Accel[3], Gyro[3], Angle[3];
extern u8  Ctrl_Msg, Img_Show, MPU6050_Msg;
extern s16 Ctrl_Val, GoalSpeed;
/******************************************************************************/

void PIT_Isr()
{PIT_Flag = 1;}

void PIT_Config()
{
	pit_init_structure.PIT_Pitx = PIT0;
	pit_init_structure.PIT_PeriodMs = 1000;
	pit_init_structure.PIT_Isr = PIT_Isr;
	
	LPLD_PIT_Init(pit_init_structure);
	LPLD_PIT_EnableIrq(pit_init_structure);
}

void Main_Test()
{
	//ѭ���ƴ�
	LoopTime++;
	
	//K60���Ƶ�Ԫͨ��
	if(Ctrl_Msg!=0)
	{
		switch(Ctrl_Msg)
		{
			case SPEED_MSG:
				GoalSpeed = Ctrl_Val;
				break;
				
			case DUOJI_MSG:
				SD5_ChangeDuty(Ctrl_Val);
				break;
		}
		Ctrl_Msg = 0;
	}
	
	//OLED��ʾ����
	if(Img_Show==1)
	{OLED_DrawImage();}
	
	//��̬�������
	if(MPU6050_Msg!=0)
	{				
		MPU6050_Cnt++;
		if(MPU6050_Cnt==50)
		{
			MPU6050_Cnt = 0;
			switch(MPU6050_Msg)
			{
				case 1:
					sprintf((int8*)str, "add 3,0,%d", Angle[0]/100+90);
					HMI_SendString(str);
					sprintf((int8*)str, "add 3,1,%d", Angle[1]/100+90);
					HMI_SendString(str);
					sprintf((int8*)str, "add 3,2,%d", Angle[2]/100+90);
					HMI_SendString(str);
					break;
				case 2:
					sprintf((int8*)str, "add 3,0,%d", Gyro[0]/100+90);
					HMI_SendString(str);
					sprintf((int8*)str, "add 3,1,%d", Gyro[1]/100+90);
					HMI_SendString(str);
					sprintf((int8*)str, "add 3,2,%d", Gyro[2]/100+90);
					HMI_SendString(str);
					break;
				case 3:
					sprintf((int8*)str, "add 3,0,%d", Accel[0]/2+90);
					HMI_SendString(str);
					sprintf((int8*)str, "add 3,1,%d", Accel[1]/2+90);
					HMI_SendString(str);
					sprintf((int8*)str, "add 3,2,%d", Accel[2]/2+90);
					HMI_SendString(str);
					break;
			}
		}
	}
	
	//PIT��ʱ��
	if(PIT_Flag==1)
	{
		//1s�� while���д���
    sprintf((int8*)str, "%5d", LoopTime);		
		OLED_WriteStr(11, 0, str, 0);
		LoopTime = 0;
    
    //����ͷ�ɼ�FPS
    //sprintf((int8*)str, "%5d", OV7725_GetFPS());		
		//OLED_WriteStr(11, 1, str, 0);
		
		//����
    //s16 spd;
    //GetSpeed(&spd);   
		//sprintf((int8*)str, "%5d", spd);		
		//OLED_WriteStr(11, 2, str, 0);
		
		PIT_Flag = 0;
	}
}