#include "common.h"

u8 PIT_Flag=0;
u8 LED_Cnt=0;
void PIT_Isr()
{
	PIT_Flag = 1;
	LED_Cnt++;
	if(LED_Cnt==50)
	{
		//PTC0_O = !PTC0_I;
		LED_Cnt = 0;
	}
}

PIT_InitTypeDef pit_init_structure;
GPIO_InitTypeDef led_init;
void PIT_Config(u8 period)
{
	pit_init_structure.PIT_Pitx = PIT0;
	pit_init_structure.PIT_PeriodMs = period;
	pit_init_structure.PIT_Isr = PIT_Isr;
	
	LPLD_PIT_Init(pit_init_structure);
	LPLD_PIT_EnableIrq(pit_init_structure);
	
	led_init.GPIO_PTx = PTC;
	led_init.GPIO_Pins = GPIO_Pin0;
	led_init.GPIO_Dir = DIR_OUTPUT;
	led_init.GPIO_Output = OUTPUT_H;
	led_init.GPIO_PinControl = IRQC_DIS;
	LPLD_GPIO_Init(led_init);
}
