#include "MPU6050_Soft.h"

#define SCL_OUT() DDRC10=1
#define SDA_OUT() DDRC11=1
#define SDA_IN() 	DDRC11=0

#define IIC_SCL(val)	PTC10_O=val
#define IIC_SDA(val)	PTC11_O=val
#define READ_SDA()		PTC11_I

u8 i=0;		//test
void MPU6050_Isr()
{
	if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin12))
  {
		gyro_data_ready_cb();
		
		i++;											//test
		if(i==100)								//test
		{PTC0_O = !PTC0_I;i=0;}		//test
		
		LPLD_GPIO_ClearIntFlag(PORTC);
	}
}

GPIO_InitTypeDef i2c_gpio_init;
void MPU6050_Init()
{
	i2c_gpio_init.GPIO_PTx = PTC;
	i2c_gpio_init.GPIO_Pins = GPIO_Pin10 | GPIO_Pin11;
	i2c_gpio_init.GPIO_Dir = DIR_OUTPUT;
	i2c_gpio_init.GPIO_Output = OUTPUT_H;
	i2c_gpio_init.GPIO_PinControl = IRQC_DIS | OUTPUT_OD_EN;
  LPLD_GPIO_Init(i2c_gpio_init);
	
	i2c_gpio_init.GPIO_PTx = PTC;
	i2c_gpio_init.GPIO_Pins = GPIO_Pin12;
	i2c_gpio_init.GPIO_Dir = DIR_INPUT;
	i2c_gpio_init.GPIO_PinControl = IRQC_FA | INPUT_PULL_UP;
  LPLD_GPIO_Init(i2c_gpio_init);
	LPLD_GPIO_EnableIrq(i2c_gpio_init);
  LPLD_SYSTICK_DelayMs(1);
  
  MPU6050_WriteReg(PWR_MGMT_1,0x00);    //�������״̬
  MPU6050_WriteReg(SMPLRT_DIV,0x07);    //�����ǲ����ʣ�����ֵ��0x07(125Hz)
  MPU6050_WriteReg(CONFIG,0x06);        //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
  MPU6050_WriteReg(GYRO_CONFIG,0x18);   //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
  MPU6050_WriteReg(ACCEL_CONFIG,0x01);  //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
  LPLD_SYSTICK_DelayMs(1);
}

u8 ACK;		//ȫ��I2C - ACK����
u8 MPU6050_WriteBuffer(u8 addr, u8 reg, u8 len, u8 *data)
{  
	u8 i;

	IIC_Start();	/*��������*/
	IIC_SendByte(addr<<1 + 0);	/*����������ַ*/
	if(ACK==0)
	{return(1);}
	
	IIC_SendByte(reg);	/*���������ӵ�ַ*/
	if(ACK==0)
	{return(1);}
	
	for(i=0; i<len; i++)
	{   
		IIC_SendByte(*data);	/*��������*/
		if(ACK==0)
		{return(1);}
		data++;
	} 
	
	IIC_Stop();	/*��������*/ 

	return(0);
}  

u8 MPU6050_ReadBuffer(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 i;

	IIC_Start();	/*��������*/
	IIC_SendByte((addr<<1) + 0);	/*����������ַ*/
	if(ACK==0)
	{return(1);}
	
	IIC_SendByte(reg);	/*���������ӵ�ַ*/
	if(ACK==0)
	{return(1);}
	
	IIC_Start();	/*������������*/
	IIC_SendByte((addr<<1) + 1);
	if(ACK==0)
	{return(1);}
	
	for(i=0;i<len-1;i++)
	{   
		*buf=IIC_ReadByte();	/*��������*/
		IIC_SendACK(0);	/*���;ʹ�λ*/  
		buf++;
	}
	*buf=IIC_ReadByte();
	IIC_SendACK(1);	/*���ͷ�Ӧλ*/
	
	IIC_Stop();	/*��������*/ 

	return(0);
}

void MPU6050_WriteReg(u8 RegisterAddress, u8 Data)
{
  IIC_Start();	/*��������*/
	IIC_SendByte(SlaveAddress<<1 + 0);	/*����������ַ*/
	if(ACK==0)
	{return;}
	
	IIC_SendByte(RegisterAddress);	/*���������ӵ�ַ*/
	if(ACK==0)
	{return;}
	
	IIC_SendByte(Data);	/*��������*/
	if(ACK==0)
	{return;}
	
	IIC_Stop();	/*��������*/ 
}

u8 MPU6050_ReadReg(u8 RegisterAddress)
{
  u8 result;

	IIC_Start();	/*��������*/
	IIC_SendByte((SlaveAddress<<1) + 0);	/*����������ַ*/
	if(ACK==0)
	{return(1);}
	
	IIC_SendByte(RegisterAddress);	/*���������ӵ�ַ*/
	if(ACK==0)
	{return(1);}
	
	IIC_Start();	/*������������*/
	IIC_SendByte((SlaveAddress<<1) + 1);
	if(ACK==0)
	{return(1);}
	
	result = IIC_ReadByte();	/*��������*/
	IIC_SendACK(1);	/*���ͷ�Ӧλ*/
	
	IIC_Stop();	/*��������*/ 

  return result;
}

/***********************************************************/
/* I2C - I/O��                                             */
/***********************************************************/
void IIC_Start()
{
	SDA_OUT();
	IIC_SDA(1);
	IIC_DelayUs(1);
	IIC_SCL(1);
	IIC_DelayUs(2);	
	IIC_SDA(0);
	IIC_DelayUs(2);
	IIC_SCL(0);
	IIC_DelayUs(1);
}

void IIC_Stop()
{
	SDA_OUT();
	IIC_SDA(0);
	IIC_DelayUs(1);
	IIC_SCL(1);
	IIC_DelayUs(2);
	IIC_SDA(1);
	IIC_DelayUs(2);
}

void IIC_SendByte(u8 dat)
{
	u8 BitCnt;

	SDA_OUT();  
	for(BitCnt=0; BitCnt<8; BitCnt++)   /*Ҫ���͵����ݳ���Ϊ8λ*/
	{
		if((dat<<BitCnt)&0x80) /*�жϷ���λ*/
		{IIC_SDA(1);} 
		else 
		{IIC_SDA(0);}
		IIC_DelayUs(1);          /*��֤ʱ�ӵ͵�ƽ���ڴ���1.3��s*/    
		IIC_SCL(1);              /*��ʱ����Ϊ�ߣ�֪ͨ��������ʼ��������λ*/
		IIC_DelayUs(1);          /*��֤ʱ�Ӹߵ�ƽ���ڴ���0.6��s*/    
		IIC_SCL(0); 
		IIC_DelayUs(1);          /*��֤ʱ�ӵ͵�ƽ���ڴ���1.3��s*/   
	}

	SDA_IN();                 /*8λ��������ͷ������ߣ�׼������Ӧ��λ*/
	IIC_DelayUs(1);  
	IIC_SCL(1);
	IIC_DelayUs(1);
	if(READ_SDA())					/*�ж��Ƿ���յ�Ӧ���ź�*/
	{ACK = 0;}   
	else
	{ACK = 1;}         
	IIC_SCL(0);
	IIC_DelayUs(2); 
}

u8 IIC_ReadByte()
{
	u8 retc;
	u8 BitCnt;

	retc=0;
	SDA_IN();	/*��������Ϊ���뷽ʽ*/
	for(BitCnt=0; BitCnt<8; BitCnt++)
	{
		IIC_DelayUs(1);   
		
		IIC_SCL(0);	/*��ʱ����Ϊ�ͣ�׼����������λ*/
		IIC_DelayUs(2);	/*ʱ�ӵ͵�ƽ���ڴ���1.3��s*/
		IIC_SCL(1);	/*��ʱ����Ϊ��ʹ��������������Ч*/
		
		IIC_DelayUs(1);
		retc=retc<<1;
		if(READ_SDA())	/*������λ,���յ�����λ����retc�� */
		{retc+=1;}  
	}
	IIC_SCL(0);   
	IIC_DelayUs(2);
	return(retc);
}

void IIC_SendACK(u8 a)
{  
  SDA_OUT();  
	if(a == 0)	/*�ڴ˷���Ӧ����Ӧ���ź� */
	{IIC_SDA(0);}          
	else
	{IIC_SDA(1);}
  IIC_DelayUs(1);      
	
  IIC_SCL(1);
  IIC_DelayUs(1);	/*ʱ�Ӹߵ�ƽ���ڴ���0.6��s*/
  IIC_SCL(0);	/*��ʱ���ߣ�ǯסI2C�����Ա��������*/
	
  IIC_DelayUs(2);    
}

void IIC_DelayUs(u8 t)
{
	while(t--)
	{asm("nop");}
}
