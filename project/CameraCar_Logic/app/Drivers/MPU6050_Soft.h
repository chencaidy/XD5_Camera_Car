#include "common.h"

void MPU6050_Init(void);		//MPU6050��ʼ��
void DMP_Init(void);				//DMP�˶���̬��������ʼ��
void MPU6050_Isr(void);			//�жϺ�������: OV7725.c


/*********************************************************** 
�������ƣ�DMP_GetData
�������ܣ���ȡDMP��������
��ڲ�����*ACCEL - ��д����ٶ�����,��������Ϊ3 (X, Y, Z),��λ g
					*GYRO  - ��д����ٶ�����,��������Ϊ3 (X, Y, Z),��λ ��/s
					*QUAT  - ��д��ŷ��������,��������Ϊ3 (Pitch, Roll, Yaw),��λ ��
���ڲ�������
��ע���������ݷŴ�100��, �磺QUAT�еõ�����9000��������ת90��
***********************************************************/
void DMP_GetData(s16 *ACCEL, s16 *GYRO, s16 *QUAT);

/*********************************************************** 
�������ƣ�gyro_data_ready_cb
�������ܣ����ݽ�����ɱ�־���ж����ã�
��ڲ�������
���ڲ�������
***********************************************************/
void gyro_data_ready_cb(void);

/*********************************************************** 
�������ƣ�MPU6050_WriteBuffer
�������ܣ�дһ���������Ĵ���
��ڲ�����addr - ������ַ
					reg - �Ĵ�����ʼ��ַ
					len - ��д�����ݳ���
					*data - ��д������
���ڲ�����0-�ɹ�, ����ʧ��
***********************************************************/
u8 MPU6050_WriteBuffer(u8 addr, u8 reg, u8 len, u8 *data);

/*********************************************************** 
�������ƣ�MPU6050_ReadBuffer
�������ܣ���ȡһ��������������
��ڲ�����addr - ������ַ
					reg - �Ĵ�����ʼ��ַ
					len - ��ȡ���ݳ���
					*buf - Ŀ������
���ڲ�����0-�ɹ�, ����ʧ��
***********************************************************/
u8 MPU6050_ReadBuffer(u8 addr, u8 reg, u8 len, u8 *buf);

/*********************************************************** 
�������ƣ�MPU6050_WriteReg
�������ܣ�дһ���Ĵ���
��ڲ�����RegisterAddress - �Ĵ�����ַ
					Data - ����Ҫд������
���ڲ������� 
***********************************************************/
void MPU6050_WriteReg(u8 RegisterAddress, u8 Data);

/*********************************************************** 
�������ƣ�MPU6050_ReadReg
�������ܣ���һ���Ĵ���
��ڲ�����RegisterAddress - �Ĵ�����ַ
���ڲ����������Ĵ�������
***********************************************************/
u8 MPU6050_ReadReg(u8 RegisterAddress);

/*********************************************************** 
�������ƣ�MPU6050_GetResult
�������ܣ����Ĵ����ߵ�λ�����ں�
��ڲ�����Regs_Addr - �Ĵ�����λ��ַ
���ڲ�����������
***********************************************************/
s16 MPU6050_GetResult(u8 Regs_Addr);


//MPU6050�ڲ��Ĵ���
#define	SMPLRT_DIV			(0x19)	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG					(0x1A)	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG			(0x1B)	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG		(0x1C)	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H		(0x3B)  //X����ٶ�
#define	ACCEL_XOUT_L		(0x3C)
#define	ACCEL_YOUT_H		(0x3D)  //Y����ٶ�
#define	ACCEL_YOUT_L		(0x3E)
#define	ACCEL_ZOUT_H		(0x3F)  //Z����ٶ�
#define	ACCEL_ZOUT_L		(0x40)
#define	TEMP_OUT_H			(0x41)
#define	TEMP_OUT_L			(0x42)
#define	GYRO_XOUT_H			(0x43)  //X����ٶ�  ������
#define	GYRO_XOUT_L			(0x44)	
#define	GYRO_YOUT_H			(0x45)  //Y����ٶ�
#define	GYRO_YOUT_L			(0x46)
#define	GYRO_ZOUT_H			(0x47)  //Z����ٶ�
#define	GYRO_ZOUT_L			(0x48)
#define	PWR_MGMT_1			(0x6B)	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I				(0x75)	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	SlaveAddress		(0x68)	//Ӳ��I2C��ַ0x68 ���I2C��ַ0xD0

//����SCL Bus Speedȡֵ����������Ϊ100Mhzʱ�ļ�����
#define MPU6050_SCL_100KHZ      (0x33)
#define MPU6050_SCL_200KHZ      (0x2A)
#define MPU6050_SCL_400KHZ      (0x23)

//ģ��I2C - I/O�㶨��
void IIC_Start(void);
void IIC_Stop(void);
void IIC_SendByte(u8 dat);
u8 IIC_ReadByte();
void IIC_SendACK(u8 a);
void IIC_DelayUs(u8 t);
