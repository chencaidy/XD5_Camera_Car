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
  
  MPU6050_WriteReg(PWR_MGMT_1,0x00);    //解除休眠状态
  MPU6050_WriteReg(SMPLRT_DIV,0x07);    //陀螺仪采样率，典型值：0x07(125Hz)
  MPU6050_WriteReg(CONFIG,0x06);        //低通滤波频率，典型值：0x06(5Hz)
  MPU6050_WriteReg(GYRO_CONFIG,0x18);   //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
  MPU6050_WriteReg(ACCEL_CONFIG,0x01);  //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
  LPLD_SYSTICK_DelayMs(1);
}

u8 ACK;		//全局I2C - ACK变量
u8 MPU6050_WriteBuffer(u8 addr, u8 reg, u8 len, u8 *data)
{  
	u8 i;

	IIC_Start();	/*启动总线*/
	IIC_SendByte(addr<<1 + 0);	/*发送器件地址*/
	if(ACK==0)
	{return(1);}
	
	IIC_SendByte(reg);	/*发送器件子地址*/
	if(ACK==0)
	{return(1);}
	
	for(i=0; i<len; i++)
	{   
		IIC_SendByte(*data);	/*发送数据*/
		if(ACK==0)
		{return(1);}
		data++;
	} 
	
	IIC_Stop();	/*结束总线*/ 

	return(0);
}  

u8 MPU6050_ReadBuffer(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 i;

	IIC_Start();	/*启动总线*/
	IIC_SendByte((addr<<1) + 0);	/*发送器件地址*/
	if(ACK==0)
	{return(1);}
	
	IIC_SendByte(reg);	/*发送器件子地址*/
	if(ACK==0)
	{return(1);}
	
	IIC_Start();	/*重新启动总线*/
	IIC_SendByte((addr<<1) + 1);
	if(ACK==0)
	{return(1);}
	
	for(i=0;i<len-1;i++)
	{   
		*buf=IIC_ReadByte();	/*发送数据*/
		IIC_SendACK(0);	/*发送就答位*/  
		buf++;
	}
	*buf=IIC_ReadByte();
	IIC_SendACK(1);	/*发送非应位*/
	
	IIC_Stop();	/*结束总线*/ 

	return(0);
}

void MPU6050_WriteReg(u8 RegisterAddress, u8 Data)
{
  IIC_Start();	/*启动总线*/
	IIC_SendByte(SlaveAddress<<1 + 0);	/*发送器件地址*/
	if(ACK==0)
	{return;}
	
	IIC_SendByte(RegisterAddress);	/*发送器件子地址*/
	if(ACK==0)
	{return;}
	
	IIC_SendByte(Data);	/*发送数据*/
	if(ACK==0)
	{return;}
	
	IIC_Stop();	/*结束总线*/ 
}

u8 MPU6050_ReadReg(u8 RegisterAddress)
{
  u8 result;

	IIC_Start();	/*启动总线*/
	IIC_SendByte((SlaveAddress<<1) + 0);	/*发送器件地址*/
	if(ACK==0)
	{return(1);}
	
	IIC_SendByte(RegisterAddress);	/*发送器件子地址*/
	if(ACK==0)
	{return(1);}
	
	IIC_Start();	/*重新启动总线*/
	IIC_SendByte((SlaveAddress<<1) + 1);
	if(ACK==0)
	{return(1);}
	
	result = IIC_ReadByte();	/*发送数据*/
	IIC_SendACK(1);	/*发送非应位*/
	
	IIC_Stop();	/*结束总线*/ 

  return result;
}

/***********************************************************/
/* I2C - I/O层                                             */
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
	for(BitCnt=0; BitCnt<8; BitCnt++)   /*要传送的数据长度为8位*/
	{
		if((dat<<BitCnt)&0x80) /*判断发送位*/
		{IIC_SDA(1);} 
		else 
		{IIC_SDA(0);}
		IIC_DelayUs(1);          /*保证时钟低电平周期大于1.3μs*/    
		IIC_SCL(1);              /*置时钟线为高，通知被控器开始接收数据位*/
		IIC_DelayUs(1);          /*保证时钟高电平周期大于0.6μs*/    
		IIC_SCL(0); 
		IIC_DelayUs(1);          /*保证时钟低电平周期大于1.3μs*/   
	}

	SDA_IN();                 /*8位发送完后释放数据线，准备接收应答位*/
	IIC_DelayUs(1);  
	IIC_SCL(1);
	IIC_DelayUs(1);
	if(READ_SDA())					/*判断是否接收到应答信号*/
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
	SDA_IN();	/*置数据线为输入方式*/
	for(BitCnt=0; BitCnt<8; BitCnt++)
	{
		IIC_DelayUs(1);   
		
		IIC_SCL(0);	/*置时钟线为低，准备接收数据位*/
		IIC_DelayUs(2);	/*时钟低电平周期大于1.3μs*/
		IIC_SCL(1);	/*置时钟线为高使数据线上数据有效*/
		
		IIC_DelayUs(1);
		retc=retc<<1;
		if(READ_SDA())	/*读数据位,接收的数据位放入retc中 */
		{retc+=1;}  
	}
	IIC_SCL(0);   
	IIC_DelayUs(2);
	return(retc);
}

void IIC_SendACK(u8 a)
{  
  SDA_OUT();  
	if(a == 0)	/*在此发出应答或非应答信号 */
	{IIC_SDA(0);}          
	else
	{IIC_SDA(1);}
  IIC_DelayUs(1);      
	
  IIC_SCL(1);
  IIC_DelayUs(1);	/*时钟高电平周期大于0.6μs*/
  IIC_SCL(0);	/*清时钟线，钳住I2C总线以便继续接收*/
	
  IIC_DelayUs(2);    
}

void IIC_DelayUs(u8 t)
{
	while(t--)
	{asm("nop");}
}
