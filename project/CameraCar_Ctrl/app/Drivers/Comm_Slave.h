#include "common.h"

void Comm_Config(void);		//SPI接收器初始化

void SPI_Config(void);
void SPI_DMARX_Config(void);
void SPI_DMATX_Config(void);
void LPLD_SPI_Cmd(SPI_Type *spix, u8 stat);
void LPLD_SPI_Restart(SPI_Type *spix);

/* 返回信息定义 ***************************************************************/
#define SPEED_SET			0xA1		//设置电机速度
#define P_SET					0xA2		//设置电机P值
#define I_SET					0xA3		//设置电机I值
#define D_SET					0xA4		//设置电机D值
#define SD5_SET				0xA5		//设置舵机打角

#define SPEED_READ		0xB1		//读电机速度
#define P_READ				0xB2		//读电机P值
#define I_READ				0xB3		//读电机I值
#define D_READ				0xB4		//读电机D值

/*******************************************************************************/
