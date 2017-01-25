#include "Menu.h"
#include "HMI.h"
#include "BlackBox.h"
#include "OLED12864.h"
#include "Comm_Master.h"
#include "MPU6050_Soft.h"

/**  各种 Flag  ***************************************************************/
u8  Ctrl_Msg=0, MPU6050_Msg=0, Img_Show=1;
/**  各种 Valve  **************************************************************/
s16 Ctrl_Val=0;
/**  各种 extern  *************************************************************/
extern u8 Image_Convent[CAMERA_H][CAMERA_W];
extern u8 Image_Line[10];
extern u8 Image_Pval[10];
extern u8 Brake_Cnt;
extern char RxBuffer[RxBufferSize];

void HMI_MsgReceived()
{
	switch(RxBuffer[0])
	{
		case PAGE_RTN:
      if(RxBuffer[1]==4)
			{PrintPID();}
      if(RxBuffer[1]==7)
			{PrintBrake();}
			if(RxBuffer[1]!=5)
			{MPU6050_Msg = 0;}
			break;
			
		case BUTTON_RTN:
      //控制面板页面按钮
      if(RxBuffer[1]==2 && RxBuffer[2]==12)
			{Img_Show = 0; OLED_Clear(0);}
			//摄像头页面按钮
			if(RxBuffer[1]==3 && RxBuffer[2]==3)
			{OV7725_RefreshRate(4);}
			if(RxBuffer[1]==3 && RxBuffer[2]==4)
			{OV7725_RefreshRate(3);}
			if(RxBuffer[1]==3 && RxBuffer[2]==5)
			{OV7725_RefreshRate(2);}
			if(RxBuffer[1]==3 && RxBuffer[2]==6)
			{OV7725_RefreshRate(1);}
			if(RxBuffer[1]==3 && RxBuffer[2]==12)
			{Img_Show = 1;}
			if(RxBuffer[1]==3 && RxBuffer[2]==13)
			{Img_Show = 0; OLED_Clear(0);}
      //舵机页面按钮
      if(RxBuffer[1]==4 && RxBuffer[2]==39)
			{PrintPID();}
			//姿态页面按钮
			if(RxBuffer[1]==5 && RxBuffer[2]==9)
			{MPU6050_Msg = 1;}
			if(RxBuffer[1]==5 && RxBuffer[2]==10)
			{MPU6050_Msg = 2;}
			if(RxBuffer[1]==5 && RxBuffer[2]==11)
			{MPU6050_Msg = 3;}
			//黑匣子页面按钮
			if(RxBuffer[1]==6 && RxBuffer[2]==3)
			{Mount_Disk();}
			if(RxBuffer[1]==6 && RxBuffer[2]==4)
			{Unmount_Disk();}
			if(RxBuffer[1]==6 && RxBuffer[2]==7)
			{
				Img_Record(Image_Convent);
				HMI_SendString("imgshot.txt=\"截取完成！\"");
			}
      //刹车页面按钮
      if(RxBuffer[1]==7 && RxBuffer[2]==10)
			{PrintBrake();}
			break;
			
		case ISO_MSG:
			OV7725_ChangeISO(RxBuffer[1]);
			break;
			
    case SPEED_MSG:
    case DUOJI_MSG:
      Ctrl_Msg = RxBuffer[0];
      Ctrl_Val = RxBuffer[2]<<8;
      Ctrl_Val += RxBuffer[1];
      Brake_Cnt = 0;
			break;
      
    case PID_MSG:
      ChangePID();
      break;
    
    case BRAKE_MSG:
      ChangeBrake();
      break;
	}
}

void PrintPID()
{
  u8 i;
  char strbuf[12];
  
  for(i=0; i<10; i++)
  {
    sprintf((int8*)strbuf, "n%d.val=%d", i*2, Image_Line[i]);
    HMI_SendString(strbuf);
    sprintf((int8*)strbuf, "n%d.val=%d", i*2+1, Image_Pval[i]);
    HMI_SendString(strbuf);
  }
}

void ChangePID()
{
  u8 i;
  
  for(i=0; i<10; i++)
  {
    if(RxBuffer[1]==(0xC0+i))
    {Image_Line[i]=RxBuffer[2];}
    if(RxBuffer[1]==(0xD0+i))
    {Image_Pval[i]=RxBuffer[2];}
  }
}

extern u8 BrakeLimit_L;
extern u8 BrakeLimit_R;
extern u8 BrakeLimit_Line;
extern u8 BrakeLimit_Delay;
extern u8 BrakeLimit_Speed;

void PrintBrake()
{
  char strbuf[15];
  
  sprintf((int8*)strbuf, "n0.val=%d", BrakeLimit_L);
  HMI_SendString(strbuf);
  sprintf((int8*)strbuf, "n1.val=%d", BrakeLimit_R);
  HMI_SendString(strbuf);
  sprintf((int8*)strbuf, "n2.val=%d", BrakeLimit_Line);
  HMI_SendString(strbuf);
  sprintf((int8*)strbuf, "n3.val=%d", BrakeLimit_Speed);
  HMI_SendString(strbuf);
  sprintf((int8*)strbuf, "n4.val=%d", BrakeLimit_Delay);
  HMI_SendString(strbuf);
}

void ChangeBrake()
{
  if(RxBuffer[1]==0xC0)
  {BrakeLimit_L=RxBuffer[2];}
  if(RxBuffer[1]==0xC1)
  {BrakeLimit_R=RxBuffer[2];}
  if(RxBuffer[1]==0xC2)
  {BrakeLimit_Line=RxBuffer[2];}
  if(RxBuffer[1]==0xC3)
  {BrakeLimit_Speed=RxBuffer[2];}
  if(RxBuffer[1]==0xC4)
  {BrakeLimit_Delay=RxBuffer[2];}
}