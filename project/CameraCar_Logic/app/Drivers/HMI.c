#include "HMI.h"
#include "Menu.h"

#define HMI_Port 				UART5		//HMI���ں�
#define HMI_Rx 					PTE9		//HMI����Rx����
#define HMI_Tx 					PTE8		//HMI����Tx����

char RxBuffer[RxBufferSize];
u8 RxCounter=0,FFCounter=0;

void HMI_RxIsr()
{
	char RxData;

	RxData = LPLD_UART_GetChar(HMI_Port);
	if(RxData==0xFF)
	{FFCounter++;}
	else if(FFCounter>0)
	{
		RxCounter = 0;
		FFCounter = 0;
	}
	
	RxBuffer[RxCounter] = RxData;
	RxCounter++;
	if(RxCounter>RxBufferSize-2 || FFCounter==3)
	{
		RxBuffer[RxCounter] = '\0';
		RxCounter = 0;
		FFCounter = 0;
		//��Ϣ���������
		HMI_MsgReceived();
	}
}

UART_InitTypeDef HMI_Init_Struct;
void HMI_Config(vu32 bps)
{
	HMI_Init_Struct.UART_Uartx = HMI_Port;
	HMI_Init_Struct.UART_BaudRate = bps;
	HMI_Init_Struct.UART_RxPin = HMI_Rx;
	HMI_Init_Struct.UART_TxPin = HMI_Tx;
	HMI_Init_Struct.UART_RxIntEnable = TRUE;
	HMI_Init_Struct.UART_TxIntEnable = TRUE;
	HMI_Init_Struct.UART_RxIsr = HMI_RxIsr;
	LPLD_UART_Init(HMI_Init_Struct);
	LPLD_UART_EnableIrq(HMI_Init_Struct);
}

void HMI_SendString(char *str)
{
	while(*str)
	{
		LPLD_UART_PutChar(HMI_Port, *str);
		str++;
	}
	LPLD_UART_PutChar(HMI_Port, 0xFF);
	LPLD_UART_PutChar(HMI_Port, 0xFF);
	LPLD_UART_PutChar(HMI_Port, 0xFF);
}

double atof(char *str)
{
	u8 i=0;
	u16 str2num=0,str2dot=0;
	double num;
	
	while(*str!='\0')
	{
		if(*str=='.')
		{i = 1;}
		if(*str>='0' && *str<='9')
		{
			if(i==0)
			{str2num = str2num*10+*str-48;}
			else
			{str2dot = *str-48; break;}
		}
		str++;
	}
	num = str2num+(str2dot*0.1);
		
	return num;
}
