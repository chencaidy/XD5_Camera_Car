#include "OLED12864.h"
#include "fonts.h"

#define OLED_SPI 	SPI0
#define OLED_PCS 	SPI_PCS0
#define OLED_RST 	PTD4_O
#define OLED_DC 	PTD5_O

GPIO_InitTypeDef oled_gpio_init;
void GPIO_Config()
{
	oled_gpio_init.GPIO_PTx = PTD;
	oled_gpio_init.GPIO_Pins = GPIO_Pin4 | GPIO_Pin5;
	oled_gpio_init.GPIO_Dir = DIR_OUTPUT;
	oled_gpio_init.GPIO_Output = OUTPUT_H;
	oled_gpio_init.GPIO_PinControl = IRQC_DIS;
	LPLD_GPIO_Init(oled_gpio_init);
}

SPI_InitTypeDef oled_init_structure;
void SPI_Config()
{
	oled_init_structure.SPI_SPIx = OLED_SPI;
	oled_init_structure.SPI_ModeSelect = SPI_MODE_MASTER;
	oled_init_structure.SPI_SckDivider = SPI_SCK_DIV_2;
	
	oled_init_structure.SPI_Pcs0Pin = PTD0;
	oled_init_structure.SPI_SckPin  = PTD1;
	oled_init_structure.SPI_MosiPin = PTD2;
	oled_init_structure.SPI_MisoPin = PTD3;
	
	LPLD_SPI_Init(oled_init_structure);
}

void OLED_WriteCMD(u8 reg)
{
	OLED_DC = 0;
	LPLD_SPI_Master_Write(OLED_SPI, reg, OLED_PCS, SPI_PCS_INACTIVE);
}

void OLED_WriteDAT(u8 dat)
{
	OLED_DC = 1;
	LPLD_SPI_Master_Write(OLED_SPI, dat, OLED_PCS, SPI_PCS_INACTIVE);
}

void OLED_Clear(u8 Color)
{
	u8 i,j;

	for (i = 0 ; i< 8; i++)
	{
		OLED_WriteCMD(0xB0 + i);	/* 设置页地址（0~7）	*/
		OLED_WriteCMD(0x00);			/* 设置列地址的低地址 */
		OLED_WriteCMD(0x10);			/* 设置列地址的高地址 */

		for (j = 0 ; j < 128; j++)
		{
			OLED_WriteDAT(Color);
		}
	}
}

void OLED_Config()
{
	SPI_Config();
	GPIO_Config();
	
	OLED_RST = 0;	/* 产生一个让LCD复位的低电平脉冲 */
	LPLD_SYSTICK_DelayMs(1);
	OLED_RST = 1;
	LPLD_SYSTICK_DelayMs(5);
	
	OLED_WriteCMD(0xAE);	/* 关闭OLED面板显示(休眠) */
	OLED_WriteCMD(0x00);	/* 设置列地址低4bit */
	OLED_WriteCMD(0x10);	/* 设置列地址高4bit */
	OLED_WriteCMD(0x40);	/* 设置起始行地址（低5bit 0-63）， 硬件相关*/
	
	OLED_WriteCMD(0x81);	/* 设置对比度命令(双字节命令），第1个字节是命令，第2个字节是对比度参数0-255 */
	OLED_WriteCMD(0xCF);	/* 设置对比度参数 */
	
	OLED_WriteCMD(0xA0);	/* A0 ：列地址0映射到SEG0; A1 ：列地址127映射到SEG0 */
	OLED_WriteCMD(0xA6);	/* A6 : 设置正常显示模式; A7 : 设置为反显模式 */
	
	OLED_WriteCMD(0xA8);	/* 设置COM路数 */
	OLED_WriteCMD(0x3F);	/* 1 ->（63+1）路 */
	
	OLED_WriteCMD(0xD3);	/* 设置显示偏移（双字节命令）*/
	OLED_WriteCMD(0x00);	/* 无偏移 */
	
	OLED_WriteCMD(0xD5);	/* 设置显示时钟分频系数/振荡频率 */
	OLED_WriteCMD(0xF0);	/* 设置分频系数,高4bit是分频系数，低4bit是振荡频率 */
	
	OLED_WriteCMD(0xD9);	/* 设置预充电周期 */
	OLED_WriteCMD(0xF1);	/* [3:0],PHASE 1; [7:4],PHASE 2; */
	
	OLED_WriteCMD(0xDA);	/* 设置COM脚硬件接线方式 */
	OLED_WriteCMD(0x12);
	
	OLED_WriteCMD(0xDB);	/* 设置 vcomh 电压倍率 */
	OLED_WriteCMD(0x40);	/* [6:4] 000 = 0.65 x VCC; 0.77 x VCC (RESET); 0.83 x VCC  */
	
	OLED_WriteCMD(0x8D);	/* 设置充电泵（和下个命令结合使用） */
	OLED_WriteCMD(0x14);	/* 0x14 使能充电泵， 0x10 是关闭 */
	OLED_WriteCMD(0xAF);	/* 打开OLED面板 */
	
	OLED_Clear(0);
}

void OLED_WriteChar(u8 x, u8 y, char ch, u8 set)
{
	u16 i;
	u8 line[]={0xB1,0xB3,0xB5,0xB7,0xB0,0xB2,0xB4,0xB6};
	
	ch -= 32;
	
	OLED_WriteCMD (line[y+4]);	/* 设置页地址 */
	OLED_WriteCMD (0x00 + (x & 0x0F));	/* 设置列地址的低地址 */
	OLED_WriteCMD (0x10 + ((x >> 4) & 0x0F));	/* 设置列地址的高地址 */
	for (i=0; i<8; i++)
	{
		if(set==0)
		{OLED_WriteDAT(ASCII_16[ch*8+i]);}
		else
		{OLED_WriteDAT(~ASCII_16[ch*8+i]);}
	}
	
	OLED_WriteCMD(line[y]);	/* 设置页地址 */
	OLED_WriteCMD (0x00 + (x & 0x0F));	/* 设置列地址的低地址 */
	OLED_WriteCMD (0x10 + ((x >> 4) & 0x0F));	/* 设置列地址的高地址 */
	for (i=760; i<768; i++)
	{
		if(set==0)
		{OLED_WriteDAT(ASCII_16[ch*8+i]);}
		else
		{OLED_WriteDAT(~ASCII_16[ch*8+i]);}
	}
}

void OLED_WriteStr(u8 x, u8 y, char *s, u8 set)
{
	u8 i=0;
	while (*s)
	{
		OLED_WriteChar((x+i)*8, y, *s, set);
		s++;
		i++;
	}
}

u8 lastBuffer, thisBuffer;
extern u8 Image_Convent[CAMERA_H][CAMERA_W];
void OLED_DrawImage()
{
	u8 i,j,k;
	u8 dat;
	
	lastBuffer = OV7725_GetReadyBuf();
	if(thisBuffer==lastBuffer)
	{return;}
	lastBuffer = thisBuffer;
  
  for(i=0; i<60; i++)
	{memcpy(&Image_Print[2+i][2], &Image_Convent[i][0], 80);}
	
	for(i=0; i<8; i++)
	{
		OLED_WriteCMD (0xB0+i);	/* 设置页地址 */
		OLED_WriteCMD (0x00);	/* 设置列地址的低地址 */
		OLED_WriteCMD (0x10);	/* 设置列地址的高地址 */
		for(j=0; j<84; j++)
		{
			dat = 0;
			for(k=0; k<8; k++)
			{
				dat = dat>>1;
				if(Image_Print[8*i+k][j]==0xFF)
				{dat += 0x80;}
			}
			OLED_WriteDAT(dat);
		}
	}
}
