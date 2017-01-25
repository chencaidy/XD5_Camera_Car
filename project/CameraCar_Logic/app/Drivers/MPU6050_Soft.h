#include "common.h"

void MPU6050_Init(void);		//MPU6050初始化
void DMP_Init(void);				//DMP运动姿态解算器初始化
void MPU6050_Isr(void);			//中断函数传递: OV7725.c


/*********************************************************** 
函数名称：DMP_GetData
函数功能：读取DMP解算数据
入口参数：*ACCEL - 待写入加速度数组,长度至少为3 (X, Y, Z),单位 g
					*GYRO  - 待写入角速度数组,长度至少为3 (X, Y, Z),单位 °/s
					*QUAT  - 待写入欧拉角数组,长度至少为3 (Pitch, Roll, Yaw),单位 °
出口参数：无
备注：所有数据放大100倍, 如：QUAT中得到数据9000，代表旋转90°
***********************************************************/
void DMP_GetData(s16 *ACCEL, s16 *GYRO, s16 *QUAT);

/*********************************************************** 
函数名称：gyro_data_ready_cb
函数功能：数据解算完成标志（中断内用）
入口参数：无
出口参数：无
***********************************************************/
void gyro_data_ready_cb(void);

/*********************************************************** 
函数名称：MPU6050_WriteBuffer
函数功能：写一段数据至寄存器
入口参数：addr - 器件地址
					reg - 寄存器起始地址
					len - 待写入数据长度
					*data - 待写入数组
出口参数：0-成功, 其他失败
***********************************************************/
u8 MPU6050_WriteBuffer(u8 addr, u8 reg, u8 len, u8 *data);

/*********************************************************** 
函数名称：MPU6050_ReadBuffer
函数功能：读取一段数据至缓冲区
入口参数：addr - 器件地址
					reg - 寄存器起始地址
					len - 读取数据长度
					*buf - 目标数组
出口参数：0-成功, 其他失败
***********************************************************/
u8 MPU6050_ReadBuffer(u8 addr, u8 reg, u8 len, u8 *buf);

/*********************************************************** 
函数名称：MPU6050_WriteReg
函数功能：写一个寄存器
入口参数：RegisterAddress - 寄存器地址
					Data - 所需要写得内容
出口参数：无 
***********************************************************/
void MPU6050_WriteReg(u8 RegisterAddress, u8 Data);

/*********************************************************** 
函数名称：MPU6050_ReadReg
函数功能：读一个寄存器
入口参数：RegisterAddress - 寄存器地址
出口参数：所读寄存器数据
***********************************************************/
u8 MPU6050_ReadReg(u8 RegisterAddress);

/*********************************************************** 
函数名称：MPU6050_GetResult
函数功能：将寄存器高低位数据融合
入口参数：Regs_Addr - 寄存器高位地址
出口参数：轴数据
***********************************************************/
s16 MPU6050_GetResult(u8 Regs_Addr);


//MPU6050内部寄存器
#define	SMPLRT_DIV			(0x19)	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG					(0x1A)	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG			(0x1B)	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG		(0x1C)	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H		(0x3B)  //X轴加速度
#define	ACCEL_XOUT_L		(0x3C)
#define	ACCEL_YOUT_H		(0x3D)  //Y轴加速度
#define	ACCEL_YOUT_L		(0x3E)
#define	ACCEL_ZOUT_H		(0x3F)  //Z轴加速度
#define	ACCEL_ZOUT_L		(0x40)
#define	TEMP_OUT_H			(0x41)
#define	TEMP_OUT_L			(0x42)
#define	GYRO_XOUT_H			(0x43)  //X轴角速度  陀螺仪
#define	GYRO_XOUT_L			(0x44)	
#define	GYRO_YOUT_H			(0x45)  //Y轴角速度
#define	GYRO_YOUT_L			(0x46)
#define	GYRO_ZOUT_H			(0x47)  //Z轴角速度
#define	GYRO_ZOUT_L			(0x48)
#define	PWR_MGMT_1			(0x6B)	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I				(0x75)	//IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress		(0x68)	//硬件I2C地址0x68 软件I2C地址0xD0

//定义SCL Bus Speed取值，外设总线为100Mhz时的计算结果
#define MPU6050_SCL_100KHZ      (0x33)
#define MPU6050_SCL_200KHZ      (0x2A)
#define MPU6050_SCL_400KHZ      (0x23)

//模拟I2C - I/O层定义
void IIC_Start(void);
void IIC_Stop(void);
void IIC_SendByte(u8 dat);
u8 IIC_ReadByte();
void IIC_SendACK(u8 a);
void IIC_DelayUs(u8 t);
