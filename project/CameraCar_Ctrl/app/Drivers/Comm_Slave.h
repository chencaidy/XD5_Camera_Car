#include "common.h"

void Comm_Config(void);		//SPI��������ʼ��

void SPI_Config(void);
void SPI_DMARX_Config(void);
void SPI_DMATX_Config(void);
void LPLD_SPI_Cmd(SPI_Type *spix, u8 stat);
void LPLD_SPI_Restart(SPI_Type *spix);

/* ������Ϣ���� ***************************************************************/
#define SPEED_SET			0xA1		//���õ���ٶ�
#define P_SET					0xA2		//���õ��Pֵ
#define I_SET					0xA3		//���õ��Iֵ
#define D_SET					0xA4		//���õ��Dֵ
#define SD5_SET				0xA5		//���ö�����

#define SPEED_READ		0xB1		//������ٶ�
#define P_READ				0xB2		//�����Pֵ
#define I_READ				0xB3		//�����Iֵ
#define D_READ				0xB4		//�����Dֵ

/*******************************************************************************/
