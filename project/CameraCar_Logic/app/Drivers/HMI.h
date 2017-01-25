#include "common.h"

#define RxBufferSize		64			//接收缓冲区大小

void HMI_Config(vu32 bps);		//HMI串口屏端口初始化

void HMI_SendString(char *str);		//向HMI串口屏发送指令


/***********************************************************
函数名称：HMI串口屏返回数据头定义区
***********************************************************/
//串口指令执行成功或失败的通知格式 :
#define CMD_ERR					0x00
#define CMD_PASS				0x01
#define BUTTON_ERR			0x02
#define PAGE_ERR				0x03
#define IMG_ERR					0x04
#define FONT_ERR				0x05
#define BAUD_ERR				0x11
#define SCOPE_ERR				0x12
#define NAME_ERR				0x1A
#define LOGIC_ERR				0x1B
//其他数据返回格式 :
#define BUTTON_RTN			0x65
#define PAGE_RTN				0x66
#define TOUCH_RTN				0x67
#define SLEEPTOUCH_RTN	0x68
#define STRING_RTN			0x70
#define NUM_RTN					0x71
#define SLEEP_RTN				0x86
#define WAKE_RTN				0x87
#define BOOT_RTN				0x88
#define UPDATE_RTN			0x89
//自定义区
#define SPEED_MSG				0xA0
#define DUOJI_MSG				0xA1
#define ISO_MSG					0xA2
#define PID_MSG					0xA3
#define BRAKE_MSG	  		0xA4