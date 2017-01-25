#include "Comm_Master.h"

void Motor_ChangeSpeed(s16 val)
{
	u8 val_H, val_L;
	
	val_H = val>>8;
	val_L = val&0xFF;
	
	LPLD_SPI_Master_WriteRead(SPI1, 0xCD, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, SPEED_SET, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, val_H, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, val_L, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, 0xDC, SPI_PCS0, SPI_PCS_INACTIVE);
	LPLD_SYSTICK_DelayUs(10);
}

void SD5_ChangeDuty(u16 val)
{
	u8 val_H, val_L;
	
	val_H = val>>8;
	val_L = val&0xFF;
	
	LPLD_SPI_Master_WriteRead(SPI1, 0xCD, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, SD5_SET, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, val_H, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, val_L, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, 0xDC, SPI_PCS0, SPI_PCS_INACTIVE);
	LPLD_SYSTICK_DelayUs(10);
}

void PID_Change(u8 reg, double val)
{
	u16 tmp;
	u8 val_H, val_L;
	
	tmp = (u16)(val*1000);
	val_H = tmp>>8;
	val_L = tmp&0xFF;
	
	LPLD_SPI_Master_WriteRead(SPI1, 0xCD, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, reg, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, val_H, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, val_L, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, 0xDC, SPI_PCS0, SPI_PCS_INACTIVE);
	LPLD_SYSTICK_DelayUs(10);
}
	
u8 GetSpeed(s16 *dat)
{
	s16 speed;
	u8 RxData[5];
	
	LPLD_SPI_Master_WriteRead(SPI1, 0xCD, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, SPEED_READ, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, 0xFF, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, 0xFF, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, 0xDC, SPI_PCS0, SPI_PCS_INACTIVE);
	LPLD_SYSTICK_DelayUs(10);
	RxData[0] = LPLD_SPI_Master_WriteRead(SPI1, 0xCD, SPI_PCS0, SPI_PCS_ASSERTED);
	RxData[1] = LPLD_SPI_Master_WriteRead(SPI1, 0xFF, SPI_PCS0, SPI_PCS_ASSERTED);
	RxData[2] = LPLD_SPI_Master_WriteRead(SPI1, 0xFF, SPI_PCS0, SPI_PCS_ASSERTED);
	RxData[3] = LPLD_SPI_Master_WriteRead(SPI1, 0xFF, SPI_PCS0, SPI_PCS_ASSERTED);
	RxData[4] = LPLD_SPI_Master_WriteRead(SPI1, 0xDC, SPI_PCS0, SPI_PCS_INACTIVE);
	LPLD_SYSTICK_DelayUs(10);
	
	if(RxData[0]==0xCD && RxData[4]==0xDC)
	{
		speed = RxData[2]<<8;
		speed += RxData[3];
		*dat = speed;
		
		return 1;
	}
	else
	{return 0;}
}

u8 Get_PID(u8 reg, double *dat)
{
	u16 tmp;
	u8 RxData[5];
	
	LPLD_SPI_Master_WriteRead(SPI1, 0xCD, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, reg, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, 0xFF, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, 0xFF, SPI_PCS0, SPI_PCS_ASSERTED);
	LPLD_SPI_Master_WriteRead(SPI1, 0xDC, SPI_PCS0, SPI_PCS_INACTIVE);
	LPLD_SYSTICK_DelayUs(10);
	RxData[0] = LPLD_SPI_Master_WriteRead(SPI1, 0xCD, SPI_PCS0, SPI_PCS_ASSERTED);
	RxData[1] = LPLD_SPI_Master_WriteRead(SPI1, 0xFF, SPI_PCS0, SPI_PCS_ASSERTED);
	RxData[2] = LPLD_SPI_Master_WriteRead(SPI1, 0xFF, SPI_PCS0, SPI_PCS_ASSERTED);
	RxData[3] = LPLD_SPI_Master_WriteRead(SPI1, 0xFF, SPI_PCS0, SPI_PCS_ASSERTED);
	RxData[4] = LPLD_SPI_Master_WriteRead(SPI1, 0xDC, SPI_PCS0, SPI_PCS_INACTIVE);
	LPLD_SYSTICK_DelayUs(10);
	
	if(RxData[0]==0xCD && RxData[4]==0xDC)
	{
		tmp = RxData[2]<<8;
		tmp += RxData[3];
		*dat = ((double)tmp)/1000;
		
		return 1;
	}
	else
	{return 0;}
}

GPIO_InitTypeDef led_gpio_init;
void RunLED_Config()
{
	led_gpio_init.GPIO_PTx = PTC;
	led_gpio_init.GPIO_Pins = GPIO_Pin0;
	led_gpio_init.GPIO_Dir = DIR_OUTPUT;
	led_gpio_init.GPIO_Output = OUTPUT_H;
	led_gpio_init.GPIO_PinControl = IRQC_DIS;
	LPLD_GPIO_Init(led_gpio_init);
}

SPI_InitTypeDef comm_init_structure;
void Comm_Config()
{
	comm_init_structure.SPI_SPIx = SPI1;
	comm_init_structure.SPI_ModeSelect = SPI_MODE_MASTER;
	comm_init_structure.SPI_SckDivider = SPI_SCK_DIV_16;
	comm_init_structure.SPI_Pcs0Pin = PTB10;
	comm_init_structure.SPI_SckPin  = PTB11;
	comm_init_structure.SPI_MosiPin = PTB16;
	comm_init_structure.SPI_MisoPin = PTB17;
	LPLD_SPI_Init(comm_init_structure);
}

