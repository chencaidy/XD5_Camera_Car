#include "Comm_Slave.h"
#include "FTM_Drv.h"

u8 TxBuffer[5], RxBuffer[5];

u32 cntOK=0, cntFail=0;

extern s16 SpeedGoal;
extern s32 Speed;
extern double SpeedKP, SpeedKI, SpeedKD;

void GetService()
{
	u16 unsignedTmp=0;
	s16 signedTmp=0;
	
	TxBuffer[0] = 0xCD;
	TxBuffer[1] = RxBuffer[1];
	if(RxBuffer[1]==SPEED_READ)
	{
		signedTmp = (s16)Speed;
		TxBuffer[2] = signedTmp>>8;
		TxBuffer[3] = signedTmp&0xFF;
	}
	else if(RxBuffer[1]==P_READ)
	{
		unsignedTmp = (u16)(SpeedKP*1000);
		TxBuffer[2] = unsignedTmp>>8;
		TxBuffer[3] = unsignedTmp&0xFF;
	}
	else if(RxBuffer[1]==I_READ)
	{
		unsignedTmp = (u16)(SpeedKI*1000);
		TxBuffer[2] = unsignedTmp>>8;
		TxBuffer[3] = unsignedTmp&0xFF;
	}
	else if(RxBuffer[1]==D_READ)
	{
		unsignedTmp = (u16)(SpeedKD*1000);
		TxBuffer[2] = unsignedTmp>>8;
		TxBuffer[3] = unsignedTmp&0xFF;
	}
	TxBuffer[4] = 0xDC;
}

void SetService()
{
	u16 unsignedTmp=0;
	s16 signedTmp=0;
	
	if(RxBuffer[1]==SPEED_SET)
	{
		signedTmp = RxBuffer[2]<<8;
		signedTmp+= RxBuffer[3];
		SpeedGoal = signedTmp;
	}
	else if(RxBuffer[1]==P_SET)
	{
		unsignedTmp = RxBuffer[2]<<8;
		unsignedTmp+= RxBuffer[3];
		SpeedKP = ((double)unsignedTmp)/1000;
	}
	else if(RxBuffer[1]==I_SET)
	{
		unsignedTmp = RxBuffer[2]<<8;
		unsignedTmp+= RxBuffer[3];
		SpeedKP = ((double)unsignedTmp)/1000;
	}
	else if(RxBuffer[1]==D_SET)
	{
		unsignedTmp = RxBuffer[2]<<8;
		unsignedTmp+= RxBuffer[3];
		SpeedKP = ((double)unsignedTmp)/1000;
	}
	else if(RxBuffer[1]==SD5_SET)
	{
		unsignedTmp = RxBuffer[2]<<8;
		unsignedTmp+= RxBuffer[3];
		SD5_ChangeDuty(unsignedTmp);
	}
}

void Comm_Config()
{
	SPI_Config();
	SPI_DMATX_Config();
	SPI_DMARX_Config();
}

void SPI_DMARX_Isr()
{
	LPLD_SPI_Cmd(SPI0, FALSE);		//ֹͣ���ݴ���
	if(RxBuffer[0]==0xCD && RxBuffer[4]==0xDC)
	{
		//�жϴ�����
		switch(RxBuffer[1])
		{
			case SPEED_SET:	
			case P_SET:
			case I_SET:
			case D_SET:
			case SD5_SET:
				SetService();
				LPLD_DMA_LoadDstAddr(DMA_CH0, RxBuffer);
				LPLD_DMA_EnableReq(DMA_CH0);
				LPLD_SPI_Cmd(SPI0, TRUE);		//��ʼ���ݴ���
				break;
				
			case SPEED_READ:
			case P_READ:
			case I_READ:
			case D_READ:
				GetService();
				LPLD_DMA_LoadSrcAddr(DMA_CH1, TxBuffer);
				LPLD_DMA_EnableReq(DMA_CH1);		//ʹ�ܷ���DMA
				LPLD_SPI_Cmd(SPI0, TRUE);		//��ʼ���ݴ���
				break;
				
			default:
				LPLD_DMA_LoadDstAddr(DMA_CH0, RxBuffer);
				LPLD_DMA_EnableReq(DMA_CH0);
				LPLD_SPI_Cmd(SPI0, TRUE);		//��ʼ���ݴ���
				break;
		}
		
		cntOK++;
		PTC0_O = !PTC0_I;
	}
	else		//���ݴ���ʧ�ܴ���
	{
		SPI_Config();		//���³�ʼ��SPI�ӿ�
		while(!PTC4_I);		//�ȴ�CSƬѡ�ͷ�
		LPLD_DMA_LoadDstAddr(DMA_CH0, RxBuffer);
		LPLD_DMA_EnableReq(DMA_CH0);		//ʹ�ܽ���DMA
		LPLD_SPI_Cmd(SPI0, TRUE);		//��ʼ���ݴ���
		
		cntFail++;
	}
}

void SPI_DMATX_Isr()
{
	LPLD_SPI_Cmd(SPI0, FALSE);		//ֹͣ���ݴ���
	LPLD_SPI_Restart(SPI0);		//SPI��������
	while(!PTC4_I);		//�ȴ�CSƬѡ�ͷ�
	LPLD_DMA_LoadDstAddr(DMA_CH0, RxBuffer);
	LPLD_DMA_EnableReq(DMA_CH0);		//ʹ�ܽ���DMA
	LPLD_SPI_Cmd(SPI0, TRUE);		//��ʼ���ݴ���
	
	PTC0_O = !PTC0_I;
}

DMA_InitTypeDef dmarx_init_struct;
void SPI_DMARX_Config()
{
  dmarx_init_struct.DMA_CHx = DMA_CH0;    								//CH0ͨ��
  dmarx_init_struct.DMA_Req = SPI0_REV_DMAREQ;       			//SPI0����Ϊ����Դ
  dmarx_init_struct.DMA_MajorLoopCnt = 5; 								//��ѭ������ֵ��5
  dmarx_init_struct.DMA_MinorByteCnt = 1; 								//��ѭ���ֽڼ�����ÿ�ζ���1�ֽ�
  dmarx_init_struct.DMA_SourceAddr = (uint32)&SPI0->POPR;	//Դ��ַ��SPI0->POPR
  dmarx_init_struct.DMA_DestAddr = (uint32)RxBuffer;			//Ŀ�ĵ�ַ��RxBuffer[]
  dmarx_init_struct.DMA_DestAddrOffset = 1;       				//Ŀ�ĵ�ַƫ��1
  dmarx_init_struct.DMA_AutoDisableReq = TRUE;    				//�Զ���������
	dmarx_init_struct.DMA_MajorCompleteIntEnable = TRUE;
	dmarx_init_struct.DMA_Isr = SPI_DMARX_Isr;
	
  LPLD_DMA_Init(dmarx_init_struct);
	LPLD_DMA_EnableIrq(dmarx_init_struct);
	LPLD_DMA_EnableReq(DMA_CH0);
	LPLD_SPI_Cmd(SPI0, TRUE);		//��ʼ���ݴ���
}

DMA_InitTypeDef dmatx_init_struct;
void SPI_DMATX_Config()
{
  dmatx_init_struct.DMA_CHx = DMA_CH1;    								//CH0ͨ��
  dmatx_init_struct.DMA_Req = SPI0_TRAN_DMAREQ;      			//SPI0����Ϊ����Դ
  dmatx_init_struct.DMA_MajorLoopCnt = 5; 								//��ѭ������ֵ��5
  dmatx_init_struct.DMA_MinorByteCnt = 1; 								//��ѭ���ֽڼ�����ÿ�η���1�ֽ�
  dmatx_init_struct.DMA_SourceAddr = (uint32)TxBuffer;		//Դ��ַ��TxBuffer[]
	dmatx_init_struct.DMA_SourceAddrOffset = 1;
  dmatx_init_struct.DMA_DestAddr = (uint32)&SPI0->PUSHR_SLAVE;		//Ŀ�ĵ�ַ��SPI0->PUSHR_SLAVE
  dmatx_init_struct.DMA_AutoDisableReq = TRUE;    				//�Զ���������
	dmatx_init_struct.DMA_MajorCompleteIntEnable = TRUE;
	dmatx_init_struct.DMA_Isr = SPI_DMATX_Isr;
	
  LPLD_DMA_Init(dmatx_init_struct);
	LPLD_DMA_EnableIrq(dmatx_init_struct);
}

SPI_InitTypeDef spi_init_structure;
void SPI_Config()
{
	spi_init_structure.SPI_SPIx = SPI0;
	spi_init_structure.SPI_ModeSelect = SPI_MODE_SLAVE;
	spi_init_structure.SPI_Pcs0Pin = PTC4;
	spi_init_structure.SPI_SckPin = PTC5;
	spi_init_structure.SPI_MosiPin = PTC6;
	spi_init_structure.SPI_MisoPin = PTC7;
	//spi_init_structure.SPI_EnableRxFIFO = TRUE;
	spi_init_structure.SPI_EnableTxFIFO = TRUE;
	spi_init_structure.SPI_TxFIFO_RequestSelect = SPI_FIFO_DMAREQUEST;
	spi_init_structure.SPI_RxFIFO_RequestSelect = SPI_FIFO_DMAREQUEST;
	spi_init_structure.SPI_TxFIFO_FillIntEnable = TRUE;
	spi_init_structure.SPI_RxFIFO_DrainIntEnable = TRUE;
	
	LPLD_SPI_Init(spi_init_structure);
	LPLD_SPI_Cmd(SPI0, FALSE);
}

void LPLD_SPI_Cmd(SPI_Type *spix, u8 stat)
{
	if(stat == FALSE)
	{SPI0->MCR |= SPI_MCR_HALT_MASK;}
	else
	{spix->MCR &=~SPI_MCR_HALT_MASK;}
}

void LPLD_SPI_Restart(SPI_Type *spix)
{
	spix->SR =  SPI_SR_RFDF_MASK   
              |SPI_SR_RFOF_MASK
              |SPI_SR_TFFF_MASK
              |SPI_SR_TFUF_MASK
              |SPI_SR_TCF_MASK
              |SPI_SR_EOQF_MASK;
}
