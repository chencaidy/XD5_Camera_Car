#include "Algorithm.h"
#include "HMI.h"
#include "OV7725.h"
#include "OLED12864.h"
#include "Comm_Master.h"
#include "MPU6050_Soft.h"
#include "Math.h"

#define CAMERA_ML		(CAMERA_W/2-1)
#define CAMERA_MR		(CAMERA_W/2)
#define CAMERA_MT		(CAMERA_W-1)
#define CAMERA_HT		(CAMERA_H-1)		//Ԥ�����ܽ�ʡCPUʱ��

/**  ���� Buffer  *************************************************************/
u8 Image_Convent[CAMERA_H][CAMERA_W];

/**  ���� Value  **************************************************************/
u8 Image_Line[10]={ 20, 25,30,35,40,50,60,0,0,0}; //�Ӿ�����
u8 Image_Pval[10]={132,100,85,57,40,23,10,0,0,0}; //�ֶ�Pֵ����
s16 Accel[3], Gyro[3], Angle[3];
s16 GoalSpeed=10000;
u16 Brake_Cnt=0, Brake_Delay=0;

u8 BrakeLimit_L=15;     //��ɲ����ֵ
u8 BrakeLimit_R=15;     //��ɲ����ֵ
u8 BrakeLimit_Line=50;    //ֱ����ֵ
u8 BrakeLimit_Delay=2;    //��ʱ��n*50ms��
u8 BrakeLimit_Speed=0;    //ɲ��Ŀ���ٶ�

char str_t[12];
/******************************************************************************/

s8 d_last;
void Algorithm_1(void)
{
	u8 h, w;		//��ǰ֡ ��,�� �ƴ�
  u8 a=0, a_max=0;		//��ǰ֡�Ӿ࣬��ǰ֡����Ӿ�
  s8 b=0, c=0, d=0;    //���߳��ȣ����߳��ȣ����Ȳ�ֵ
  s16 e=0;    //���ƫ�����
  s16 f=0;    //ת����ٱ���
  
  DMP_GetData(Accel, Gyro, Angle);
	OV7725_ImageExtract(Image_Convent);
	
	//��ȡǰ���Ӿ࣬�ṩ�ֶ� PD����
	for(w=0; w<10; w++)
	{
		for(h=0; h<CAMERA_H; h++)		//���ݲΧ
		{
			if(Image_Convent[CAMERA_HT-h][CAMERA_ML-w]==0xFF)
			{a++;}
			else
			{break;}
		}
		if(a>a_max)
		{a_max = a;}
		a = 0;
		
		for(h=0; h<CAMERA_H; h++)		//���ݲΧ
		{
			if(Image_Convent[CAMERA_HT-h][CAMERA_MR+w]==0xFF)
			{a++;}
			else
			{break;}
		}
		if(a>a_max)
		{a_max = a;}
		a = 0;
	}
	
	//��ȡ���߳���
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_ML-w]==0xFF)
		{b++;}
		else
		{break;}
	}
	//��ȡ���߳���
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_MR+w]==0xFF)
		{c++;}
		else
		{break;}
	}
  
  //�����Ӿ�У��
  if(b==0 || c==0)
  {
    for(w=0; w<CAMERA_MR; w++)
    {
      if(Image_Convent[CAMERA_HT][CAMERA_ML-w]==0xFF)
      {b++;}
      if(Image_Convent[CAMERA_HT][CAMERA_MR+w]==0xFF)
      {c++;}
    }
  }
  
	//����ƫ��
  d = b-c;
	
	//����ֶ� PD
  if(a_max < 40)
  {f = (s32)((d*150)+(d-d_last)*100);}
	else
	{f = (s32)((d*20)+(d-d_last)*100);}
  d_last = d;
  
  if(f > 5000)
  {f = 5000;}
  if(f < -5000)
  {f = -5000;}
  SD5_ChangeDuty(5000-f);
	
	//����ٶ��趨 
  e = (int)(Gyro[2] * 0.01);            // �����Ǽ���
  if(e < 0)                             //
  {e = -e;}                             //
	//Motor_ChangeSpeed(GoalSpeed-e);     //
  if(a_max < 45)                        // �Ӿ��жϼ���
  {Motor_ChangeSpeed(GoalSpeed);}   //
  else                                  //
	{Motor_ChangeSpeed(GoalSpeed);}       //
  
  //ɲ���ж�
  Brake_Scan();
	
	//������Ϣ���
	sprintf((int8*)str_t, "%5d", a_max);		//�Ӿ��ӡ
	OLED_WriteStr(11, 1, str_t, 0);
  sprintf((int8*)str_t, "%5d", b);		//��ӡ����
  OLED_WriteStr(11, 2, str_t, 0);
  sprintf((int8*)str_t, "%5d", c);		//��ӡ����
  OLED_WriteStr(11, 3, str_t, 0);
}

void Algorithm_2(void)		//�㷨2
{
	u8 h, w;		//��ǰ֡ ��,�� �ƴ�
  u8 a=0, a_max=0;		//��ǰ֡�Ӿ࣬��ǰ֡����Ӿ�
  s16 b=0, c=0, d=0;    //����ɫ�����ҿ��ɫ������ֵ
  s16 e=0;    //ת����ٱ���
  s16 f=0;    //���ƫ�����
  
  DMP_GetData(Accel, Gyro, Angle);
	OV7725_ImageExtract(Image_Convent);
	
	//��ȡǰ���Ӿ�
	for(w=0; w<10; w++)
	{
		for(h=0; h<CAMERA_H; h++)		//���ݲΧ
		{
			if(Image_Convent[CAMERA_HT-h][CAMERA_ML-w]==0xFF)
			{a++;}
			else
			{break;}
		}
		if(a>a_max)
		{a_max = a;}
		a = 0;
		
		for(h=0; h<CAMERA_H; h++)		//���ݲΧ
		{
			if(Image_Convent[CAMERA_HT-h][CAMERA_MR+w]==0xFF)
			{a++;}
			else
			{break;}
		}
		if(a>a_max)
		{a_max = a;}
		a = 0;
	}
  
  //��ȡ���߳���
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_ML-w]==0xFF)
		{b++;}
		else
		{break;}
	}
	//��ȡ���߳���
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_MR+w]==0xFF)
		{c++;}
		else
		{break;}
	}
  
  //�����Ӿ�У��
  if(b==0 || c==0)
  {
    for(w=0; w<CAMERA_MR; w++)
    {
      if(Image_Convent[CAMERA_HT][CAMERA_ML-w]==0xFF)
      {b++;}
      if(Image_Convent[CAMERA_HT][CAMERA_MR+w]==0xFF)
      {c++;}
    }
  }
  d = b-c;
  
  //���ö�����
  if(a_max<35)
  {
    e = (s32)((61-a_max)*35);
    if(d<0)
    {e = -e;}
  }
  else
  {	//����ֶ� PD
    if(a_max < Image_Line[0])
    {e = (s32)(d*Image_Pval[0]);}
    else if(a_max < Image_Line[1])
    {e = (s32)(d*Image_Pval[1]);}
    else if(a_max < Image_Line[2])
    {e = (s32)(d*Image_Pval[2]);}
    else if(a_max < Image_Line[3])
    {e = (s32)(d*Image_Pval[3]);}
    else if(a_max < Image_Line[4])
    {e = (s32)(d*Image_Pval[4]);}
    else if(a_max < Image_Line[5])
    {e = (s32)(d*Image_Pval[5]);}
    else
    {e = (s32)(d*Image_Pval[6]);}
  }
            
  if(e > 5000)
  {f = 5000;}
  if(e < -5000)
  {f = -5000;}
  SD5_ChangeDuty(5000-e);
  
  //����ٶ��趨 
  f = (int)(Gyro[2] * 0.01);            // �����Ǽ���
  if(f < 0)                             //
  {f = -f;}                             //
	//Motor_ChangeSpeed(GoalSpeed-e);     //
  if(a_max < 45)                        // �Ӿ��жϼ���
  {Motor_ChangeSpeed(GoalSpeed);}   //
  else                                  //
	{Motor_ChangeSpeed(GoalSpeed);}       //
  
  //������Ϣ���
	sprintf((int8*)str_t, "%5d", a_max);		//�Ӿ��ӡ
	OLED_WriteStr(11, 1, str_t, 0);
  sprintf((int8*)str_t, "%5d", b);		//��ӡ����
  OLED_WriteStr(11, 2, str_t, 0);
  sprintf((int8*)str_t, "%5d", c);		//��ӡ����
  OLED_WriteStr(11, 3, str_t, 0);
}

void Algorithm_Bak(void)		//�����㷨
{
	u8 h, w;		//��ǰ֡ ��,�� �ƴ�
  u8 a=0, a_max=0;    //��ǰ֡�Ӿ�
  s8 b=0, c=0, d=0;    //���߳��ȣ����߳��ȣ����Ȳ�ֵ
  s16 e=0;    //���ƫ�����
  u8 f=0;     //ɲ����ʱ�ƴ�
  
  DMP_GetData(Accel, Gyro, Angle);
	OV7725_ImageExtract(Image_Convent);
	
	//��ȡǰ���Ӿ࣬�ṩ�ֶ� PD����
	for(w=0; w<10; w++)
	{
		for(h=0; h<CAMERA_H; h++)		//���ݲΧ
		{
			if(Image_Convent[CAMERA_HT-h][CAMERA_ML-w]==0xFF)
			{a++;}
			else
			{break;}
		}
		if(a>a_max)
		{a_max = a;}
		a = 0;
		
		for(h=0; h<CAMERA_H; h++)		//���ݲΧ
		{
			if(Image_Convent[CAMERA_HT-h][CAMERA_MR+w]==0xFF)
			{a++;}
			else
			{break;}
		}
		if(a>a_max)
		{a_max = a;}
		a = 0;
	}
	
	//��ȡ���߳���
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_ML-w]==0xFF)
		{b++;}
		else
		{break;}
	}
	//��ȡ���߳���
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_MR+w]==0xFF)
		{c++;}
		else
		{break;}
	}
  
  //�����Ӿ�У��
  if(b==0 || c==0)
  {
    for(w=0; w<CAMERA_MR; w++)
    {
      if(Image_Convent[CAMERA_HT][CAMERA_ML-w]==0xFF)
      {b++;}
      if(Image_Convent[CAMERA_HT][CAMERA_MR+w]==0xFF)
      {c++;}
    }
  }
  
	//����ƫ��
  d = b-c;
	
	//����ֶ� PD
  if(a_max < Image_Line[0])
  {e = (s32)(d*Image_Pval[0]);}
  else if(a_max < Image_Line[1])
  {e = (s32)(d*Image_Pval[1]);}
  else if(a_max < Image_Line[2])
  {e = (s32)(d*Image_Pval[2]);}
	else if(a_max < Image_Line[3])
	{e = (s32)(d*Image_Pval[3]);}
	else if(a_max < Image_Line[4])
	{e = (s32)(d*Image_Pval[4]);}
	else if(a_max < Image_Line[5])
	{e = (s32)(d*Image_Pval[5]);}
	else
	{e = (s32)(d*Image_Pval[6]);}
  
  if(e > 5000)
  {e = 5000;}
  if(e < -5000)
  {e = -5000;}
  SD5_ChangeDuty(5000-e);
	
	//����ٶ��趨
  if(a_max < 45)                        // �Ӿ��жϼ���
  {Motor_ChangeSpeed(GoalSpeed-200);}   //
  else                                  //
	{Motor_ChangeSpeed(GoalSpeed);}       //
  
  //ɲ���ж�
  Brake_Scan_Bak(a_max);
  
  //ɲ��
  if(Brake_Cnt%2==0 && Brake_Cnt>0)
  {
    GoalSpeed=10000;
    for(f=0; f<BrakeLimit_Delay; f++)
    {LPLD_SYSTICK_DelayMs(50);}
    GoalSpeed = BrakeLimit_Speed;
  }
	
	//������Ϣ���
	sprintf((int8*)str_t, "%5d", a_max);		//�Ӿ��ӡ
	OLED_WriteStr(11, 1, str_t, 0);
  /*sprintf((int8*)str_t, "%5d", b);		//��ӡ����
  OLED_WriteStr(11, 2, str_t, 0);
  sprintf((int8*)str_t, "%5d", c);		//��ӡ����
  OLED_WriteStr(11, 3, str_t, 0);*/
  sprintf((int8*)str_t, "%5d", Brake_Cnt);		//��ӡɲ���߼���
  OLED_WriteStr(11, 3, str_t, 0);
}

void Brake_Scan(void)   //ͣ���ж�
{
  u8 h, w;		//��ǰ֡ ��,�� �ƴ�
  u8 a=0, b=0;    //��ڵ���, �Һڵ���
  
  //������������߱�־
  for(w=0; w<20; w++)
	{
		for(h=0; h<7; h++)
		{
      if(Image_Convent[CAMERA_HT-h][CAMERA_ML-10-w]==0)
      {a++;}
      if(Image_Convent[CAMERA_HT-h][CAMERA_MR+10+w]==0)
      {b++;}
    }
  }
  
  //������ʽ��ʱ, ��ֹ��ʱ���μ��������
  if(Brake_Delay!=0)
  {Brake_Delay--;}
  
  //����Ƿ�Ϊ�����߲�����־
  if(a>BrakeLimit_L && b>BrakeLimit_R)
  {
    if(Brake_Delay==0)
    {Brake_Cnt++;}
    
    Brake_Delay = 1000;
  }
}

void Brake_Scan_Bak(u8 line)   //ͣ���ж�
{
  u8 h, w;		//��ǰ֡ ��,�� �ƴ�
  u8 a=0, b=0;    //��ڵ���, �Һڵ���
  
  //ֱ���ж���ֵ
  if(line>BrakeLimit_Line)    
	{
    //������������߱�־
    for(w=0; w<20; w++)
    {
      for(h=0; h<10; h++)
      {
        if(Image_Convent[CAMERA_HT-h][CAMERA_ML-10-w]==0)
        {a++;}
        if(Image_Convent[CAMERA_HT-h][CAMERA_MR+10+w]==0)
        {b++;}
      }
    }

    //����Ƿ�Ϊ�����߲�����־
    if(a>BrakeLimit_L && b>BrakeLimit_R)
    {
      if(Brake_Delay==0)
      {Brake_Cnt++;}
      
      Brake_Delay = 1000;
    }
  }
  
  //������ʽ��ʱ, ��ֹ��ʱ���μ��������
  if(Brake_Delay!=0)
  {Brake_Delay--;}
  
  //sprintf((int8*)str_t, "%5d", a);
  //OLED_WriteStr(11, 2, str_t, 0);
  //sprintf((int8*)str_t, "%5d", b);
  //OLED_WriteStr(11, 3, str_t, 0);
}
