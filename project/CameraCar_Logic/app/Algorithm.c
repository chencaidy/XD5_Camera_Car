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
#define CAMERA_HT		(CAMERA_H-1)		//预编译能节省CPU时间

/**  各种 Buffer  *************************************************************/
u8 Image_Convent[CAMERA_H][CAMERA_W];

/**  各种 Value  **************************************************************/
u8 Image_Line[10]={ 20, 25,30,35,40,50,60,0,0,0}; //视距数组
u8 Image_Pval[10]={132,100,85,57,40,23,10,0,0,0}; //分段P值数组
s16 Accel[3], Gyro[3], Angle[3];
s16 GoalSpeed=10000;
u16 Brake_Cnt=0, Brake_Delay=0;

u8 BrakeLimit_L=15;     //左刹车阈值
u8 BrakeLimit_R=15;     //右刹车阈值
u8 BrakeLimit_Line=50;    //直线阈值
u8 BrakeLimit_Delay=2;    //延时（n*50ms）
u8 BrakeLimit_Speed=0;    //刹车目标速度

char str_t[12];
/******************************************************************************/

s8 d_last;
void Algorithm_1(void)
{
	u8 h, w;		//当前帧 高,宽 计次
  u8 a=0, a_max=0;		//当前帧视距，当前帧最大视距
  s8 b=0, c=0, d=0;    //左线长度，右线长度，长度差值
  s16 e=0;    //舵机偏差变量
  s16 f=0;    //转向减速变量
  
  DMP_GetData(Accel, Gyro, Angle);
	OV7725_ImageExtract(Image_Convent);
	
	//获取前方视距，提供分段 PD依据
	for(w=0; w<10; w++)
	{
		for(h=0; h<CAMERA_H; h++)		//左容差范围
		{
			if(Image_Convent[CAMERA_HT-h][CAMERA_ML-w]==0xFF)
			{a++;}
			else
			{break;}
		}
		if(a>a_max)
		{a_max = a;}
		a = 0;
		
		for(h=0; h<CAMERA_H; h++)		//右容差范围
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
	
	//获取左线长度
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_ML-w]==0xFF)
		{b++;}
		else
		{break;}
	}
	//获取右线长度
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_MR+w]==0xFF)
		{c++;}
		else
		{break;}
	}
  
  //超出视距校正
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
  
	//计算偏差
  d = b-c;
	
	//舵机分段 PD
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
	
	//电机速度设定 
  e = (int)(Gyro[2] * 0.01);            // 陀螺仪减速
  if(e < 0)                             //
  {e = -e;}                             //
	//Motor_ChangeSpeed(GoalSpeed-e);     //
  if(a_max < 45)                        // 视距判断减速
  {Motor_ChangeSpeed(GoalSpeed);}   //
  else                                  //
	{Motor_ChangeSpeed(GoalSpeed);}       //
  
  //刹车判断
  Brake_Scan();
	
	//调试信息输出
	sprintf((int8*)str_t, "%5d", a_max);		//视距打印
	OLED_WriteStr(11, 1, str_t, 0);
  sprintf((int8*)str_t, "%5d", b);		//打印左线
  OLED_WriteStr(11, 2, str_t, 0);
  sprintf((int8*)str_t, "%5d", c);		//打印右线
  OLED_WriteStr(11, 3, str_t, 0);
}

void Algorithm_2(void)		//算法2
{
	u8 h, w;		//当前帧 高,宽 计次
  u8 a=0, a_max=0;		//当前帧视距，当前帧最大视距
  s16 b=0, c=0, d=0;    //左块白色数，右块白色数，差值
  s16 e=0;    //转向减速变量
  s16 f=0;    //舵机偏差变量
  
  DMP_GetData(Accel, Gyro, Angle);
	OV7725_ImageExtract(Image_Convent);
	
	//获取前方视距
	for(w=0; w<10; w++)
	{
		for(h=0; h<CAMERA_H; h++)		//左容差范围
		{
			if(Image_Convent[CAMERA_HT-h][CAMERA_ML-w]==0xFF)
			{a++;}
			else
			{break;}
		}
		if(a>a_max)
		{a_max = a;}
		a = 0;
		
		for(h=0; h<CAMERA_H; h++)		//右容差范围
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
  
  //获取左线长度
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_ML-w]==0xFF)
		{b++;}
		else
		{break;}
	}
	//获取右线长度
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_MR+w]==0xFF)
		{c++;}
		else
		{break;}
	}
  
  //超出视距校正
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
  
  //设置舵机打角
  if(a_max<35)
  {
    e = (s32)((61-a_max)*35);
    if(d<0)
    {e = -e;}
  }
  else
  {	//舵机分段 PD
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
  
  //电机速度设定 
  f = (int)(Gyro[2] * 0.01);            // 陀螺仪减速
  if(f < 0)                             //
  {f = -f;}                             //
	//Motor_ChangeSpeed(GoalSpeed-e);     //
  if(a_max < 45)                        // 视距判断减速
  {Motor_ChangeSpeed(GoalSpeed);}   //
  else                                  //
	{Motor_ChangeSpeed(GoalSpeed);}       //
  
  //调试信息输出
	sprintf((int8*)str_t, "%5d", a_max);		//视距打印
	OLED_WriteStr(11, 1, str_t, 0);
  sprintf((int8*)str_t, "%5d", b);		//打印左线
  OLED_WriteStr(11, 2, str_t, 0);
  sprintf((int8*)str_t, "%5d", c);		//打印右线
  OLED_WriteStr(11, 3, str_t, 0);
}

void Algorithm_Bak(void)		//保留算法
{
	u8 h, w;		//当前帧 高,宽 计次
  u8 a=0, a_max=0;    //当前帧视距
  s8 b=0, c=0, d=0;    //左线长度，右线长度，长度差值
  s16 e=0;    //舵机偏差变量
  u8 f=0;     //刹车延时计次
  
  DMP_GetData(Accel, Gyro, Angle);
	OV7725_ImageExtract(Image_Convent);
	
	//获取前方视距，提供分段 PD依据
	for(w=0; w<10; w++)
	{
		for(h=0; h<CAMERA_H; h++)		//左容差范围
		{
			if(Image_Convent[CAMERA_HT-h][CAMERA_ML-w]==0xFF)
			{a++;}
			else
			{break;}
		}
		if(a>a_max)
		{a_max = a;}
		a = 0;
		
		for(h=0; h<CAMERA_H; h++)		//右容差范围
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
	
	//获取左线长度
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_ML-w]==0xFF)
		{b++;}
		else
		{break;}
	}
	//获取右线长度
	for(w=0; w<CAMERA_MR; w++)
	{
		if(Image_Convent[CAMERA_HT-w][CAMERA_MR+w]==0xFF)
		{c++;}
		else
		{break;}
	}
  
  //超出视距校正
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
  
	//计算偏差
  d = b-c;
	
	//舵机分段 PD
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
	
	//电机速度设定
  if(a_max < 45)                        // 视距判断减速
  {Motor_ChangeSpeed(GoalSpeed-200);}   //
  else                                  //
	{Motor_ChangeSpeed(GoalSpeed);}       //
  
  //刹车判断
  Brake_Scan_Bak(a_max);
  
  //刹车
  if(Brake_Cnt%2==0 && Brake_Cnt>0)
  {
    GoalSpeed=10000;
    for(f=0; f<BrakeLimit_Delay; f++)
    {LPLD_SYSTICK_DelayMs(50);}
    GoalSpeed = BrakeLimit_Speed;
  }
	
	//调试信息输出
	sprintf((int8*)str_t, "%5d", a_max);		//视距打印
	OLED_WriteStr(11, 1, str_t, 0);
  /*sprintf((int8*)str_t, "%5d", b);		//打印左线
  OLED_WriteStr(11, 2, str_t, 0);
  sprintf((int8*)str_t, "%5d", c);		//打印右线
  OLED_WriteStr(11, 3, str_t, 0);*/
  sprintf((int8*)str_t, "%5d", Brake_Cnt);		//打印刹车线计数
  OLED_WriteStr(11, 3, str_t, 0);
}

void Brake_Scan(void)   //停车判断
{
  u8 h, w;		//当前帧 高,宽 计次
  u8 a=0, b=0;    //左黑点数, 右黑点数
  
  //检测左右起跑线标志
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
  
  //非阻塞式延时, 防止短时间多次检测起跑线
  if(Brake_Delay!=0)
  {Brake_Delay--;}
  
  //检测是否为起跑线并做标志
  if(a>BrakeLimit_L && b>BrakeLimit_R)
  {
    if(Brake_Delay==0)
    {Brake_Cnt++;}
    
    Brake_Delay = 1000;
  }
}

void Brake_Scan_Bak(u8 line)   //停车判断
{
  u8 h, w;		//当前帧 高,宽 计次
  u8 a=0, b=0;    //左黑点数, 右黑点数
  
  //直线判断阈值
  if(line>BrakeLimit_Line)    
	{
    //检测左右起跑线标志
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

    //检测是否为起跑线并做标志
    if(a>BrakeLimit_L && b>BrakeLimit_R)
    {
      if(Brake_Delay==0)
      {Brake_Cnt++;}
      
      Brake_Delay = 1000;
    }
  }
  
  //非阻塞式延时, 防止短时间多次检测起跑线
  if(Brake_Delay!=0)
  {Brake_Delay--;}
  
  //sprintf((int8*)str_t, "%5d", a);
  //OLED_WriteStr(11, 2, str_t, 0);
  //sprintf((int8*)str_t, "%5d", b);
  //OLED_WriteStr(11, 3, str_t, 0);
}
