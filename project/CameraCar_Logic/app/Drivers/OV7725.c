#include "OV7725.h"
#include "DEV_SCCB.h"
#include "CameraConf.h"
#include "MPU6050_Soft.h"

#define Data_PTx 		PTD							//���ݿڶ˿ں�
#define Data_Pins		GPIO_Pin8_15		//���ݿ�����

#define VSYNC_PTx 	PTC							//֡ͬ���˿ں�
#define VSYNC_Pins	GPIO_Pin19			//֡ͬ������
#define VSYNC_Port	PORTC						//֡ͬ���жϴ����˿�
#define VSYNC_IRQn	PORTC_IRQn			//֡ͬ���жϺ�

#define PCLK_PTx 		PTC							//����ͬ���˿ں�
#define PCLK_Pins		GPIO_Pin18			//����ͬ������

u8 NextBuffer=0;
u8 Image_Buffer1[CAMERA_BUF];						//���ݻ�����1
u8 Image_Buffer2[CAMERA_BUF];						//���ݻ�����2
u8 Image_Buffer3[CAMERA_BUF];						//���ݻ�����3
u8 Image_Buffer4[CAMERA_BUF];						//���ݻ�����4
u16 OV7725_FPS=0;		//OV7725ʵʱ����֡��

volatile IMG_STATUS OV7725_ImageStatus = IMG_FINISH;

void OV7725_Port_Isr()
{
  MPU6050_Isr();
	
	if(LPLD_GPIO_IsPinxExt(VSYNC_Port, VSYNC_Pins))
  {
		if(OV7725_ImageStatus == IMG_START)
		{
			OV7725_ImageStatus = IMG_GATHER;
			LPLD_GPIO_ClearIntFlag(VSYNC_Port);
			
			if(NextBuffer==0)
			{LPLD_DMA_LoadDstAddr(DMA_CH0, Image_Buffer1);}
			else if(NextBuffer==1)
			{LPLD_DMA_LoadDstAddr(DMA_CH0, Image_Buffer2);}
			else if(NextBuffer==2)
			{LPLD_DMA_LoadDstAddr(DMA_CH0, Image_Buffer3);}
			else if(NextBuffer==3)
			{LPLD_DMA_LoadDstAddr(DMA_CH0, Image_Buffer4);}
			
			NextBuffer++;
			if(NextBuffer==4)
			{NextBuffer = 0;}
			
			LPLD_GPIO_ClearIntFlag(VSYNC_Port);
			LPLD_DMA_EnableReq(DMA_CH0);
		}
		else																				//ͼ��ɼ�����
		{
			OV7725_ImageStatus = IMG_FAIL;						//���ͼ��ɼ�ʧ��
		}
  }
}

void OV7725_DMA_Isr()
{
	OV7725_FPS++;
	OV7725_ImageStatus = IMG_START;
	LPLD_GPIO_ClearIntFlag(VSYNC_Port);
}

u8 OV7725_SCCB_Config()
{
	u8 i;
	u8 Sensor_IDCode=0,OV7725_Eagle_Cfgnum;

	OV7725_Eagle_Cfgnum = ARR_SIZE(OV7725_Eagle_Reg); /*�ṹ�������Ա��Ŀ*/
	
	LPLD_SCCB_Init();
	
	if(LPLD_SCCB_WriteReg(OV7725_COM7, 0x80) == 0) /*��λsensor */
	{
		printf("OV7725: <Error> SCCB Data Write Failed!\n");
		return 0;
	}
	LPLD_SYSTICK_DelayMs(50);
	if(LPLD_SCCB_ReadReg(OV7725_VER, &Sensor_IDCode, 1) == 0)	/* ��ȡsensor ID��*/
	{
		printf("OV7725: <Error> Read Sensor ID Failed!\n");
		return 0;
	}
	printf("OV7725: <Info> Get ID Success, Sensor ID is 0x%x\n", Sensor_IDCode);
	printf("OV7725: <Info> Config Register Number is %d\n", OV7725_Eagle_Cfgnum);
	
	if(Sensor_IDCode==OV7725_ID)
	{
		for(i=0; i<OV7725_Eagle_Cfgnum; i++)
		{
			if(LPLD_SCCB_WriteReg(OV7725_Eagle_Reg[i].addr, OV7725_Eagle_Reg[i].val)==0)
			{
				printf("OV7725: <Error> Write Register 0x%x Failed!\n", OV7725_Eagle_Reg[i].addr);
				return 0;
			}
		}
	}
	else
	{return 0;}
	printf("OV7725: <Info> Register Config Success!\n");
	
	return 1;
}

GPIO_InitTypeDef clk_init,data_init;
void OV7725_GPIO_Config()
{
	//OV7725-���ݿڳ�ʼ��:PTD8~PTD15
  data_init.GPIO_PTx = Data_PTx;
  data_init.GPIO_Pins = Data_Pins;
	data_init.GPIO_Dir = DIR_INPUT;
  data_init.GPIO_PinControl = IRQC_DIS | INPUT_PULL_DIS;
  LPLD_GPIO_Init(data_init);
	
	//OV7725-���źŽӿڳ�ʼ��:PTE0-V
  clk_init.GPIO_PTx = VSYNC_PTx;
  clk_init.GPIO_Pins = VSYNC_Pins;
	clk_init.GPIO_Dir = DIR_INPUT;
  clk_init.GPIO_PinControl = IRQC_RI | INPUT_PULL_DOWN | INPUT_PF_EN;
  clk_init.GPIO_Isr = OV7725_Port_Isr;
  LPLD_GPIO_Init(clk_init); 
	LPLD_GPIO_EnableIrq(clk_init);
  
	//OV7725-PCLK�źŽӿڳ�ʼ��:PTE1-PCLK
  clk_init.GPIO_PTx = PCLK_PTx;
  clk_init.GPIO_Pins = PCLK_Pins;
	clk_init.GPIO_Dir = DIR_INPUT;
  clk_init.GPIO_PinControl = IRQC_DMAFA | INPUT_PULL_DOWN;
  LPLD_GPIO_Init(clk_init); 
}

DMA_InitTypeDef dma_init_struct;
void OV7725_DMA_Config()
{
  dma_init_struct.DMA_CHx = DMA_CH0;    								//CH0ͨ��
  dma_init_struct.DMA_Req = PORTC_DMAREQ;       				//PORTCΪ����Դ
  dma_init_struct.DMA_MajorLoopCnt = CAMERA_BUF; 				//��ѭ������ֵ���вɼ����������
  dma_init_struct.DMA_MinorByteCnt = 1; 								//��ѭ���ֽڼ�����ÿ�ζ���1�ֽ�
  dma_init_struct.DMA_SourceAddr = (uint32)&PTD->PDIR+1;//Դ��ַ��PTD8~15
  dma_init_struct.DMA_DestAddr = (uint32)Image_Buffer1;	//Ŀ�ĵ�ַ�����ͼ�������
  dma_init_struct.DMA_DestAddrOffset = 1;       				//Ŀ�ĵ�ַƫ�ƣ�ÿ�ζ�������1
  dma_init_struct.DMA_AutoDisableReq = TRUE;    				//�Զ���������
	dma_init_struct.DMA_MajorCompleteIntEnable = TRUE;
	dma_init_struct.DMA_Isr = OV7725_DMA_Isr;
	
  LPLD_DMA_Init(dma_init_struct);
	LPLD_DMA_EnableIrq(dma_init_struct);
}

void OV7725_Config()
{
	DisableInterrupts;
	
	OV7725_SCCB_Config();
	OV7725_GPIO_Config();
	OV7725_DMA_Config();
	
	OV7725_ImageStatus = IMG_START;
	
	EnableInterrupts;
}

void OV7725_ImageExtract(void *dst)
{
	u8 colour[2] = {255, 0}; //0 �� 1 �ֱ��Ӧ����ɫ
	u8 * mdst = dst;
	u8 * msrc;
	//ע��ɽ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
	u8 tmpsrc;
	u8 buffer;
	u16 strlen = CAMERA_BUF;
	
	buffer = OV7725_GetReadyBuf();
	if(buffer==0)
	{msrc = Image_Buffer1;}
	else if(buffer==1)
	{msrc = Image_Buffer2;}
	else if(buffer==2)
	{msrc = Image_Buffer3;}
	else if(buffer==3)
	{msrc = Image_Buffer4;}
	
	while(strlen --)
	{
		tmpsrc = *msrc++;
		*mdst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
	}
}

void OV7725_RefreshRate(u8 rate)
{
	switch(rate)
	{
		case 1:
			LPLD_SCCB_WriteReg(OV7725_COM4, 0xC1);
			LPLD_SCCB_WriteReg(OV7725_CLKRC, 0x00);
			break;
		case 2:
			LPLD_SCCB_WriteReg(OV7725_COM4, 0x81);
			LPLD_SCCB_WriteReg(OV7725_CLKRC, 0x00);
			break;
		case 3:
			LPLD_SCCB_WriteReg(OV7725_COM4, 0x41);
			LPLD_SCCB_WriteReg(OV7725_CLKRC, 0x00);
			break;
		case 4:
			LPLD_SCCB_WriteReg(OV7725_COM4, 0xC1);
			LPLD_SCCB_WriteReg(OV7725_CLKRC, 0x02);
			break;
	}
}

void OV7725_ChangeISO(u8 val)
{
	LPLD_SCCB_WriteReg(OV7725_CNST, val);
}

void OV7725_SendImage(void *imgaddr)
{
    LPLD_UART_PutChar(UART5, 0x01);    //�ȷ�������
		LPLD_UART_PutChar(UART5, 0xFE);    //�ȷ�������
    LPLD_UART_PutCharArr(UART5, (int8 *)imgaddr, CAMERA_H*CAMERA_W/8); //�ٷ���ͼ��
		LPLD_UART_PutChar(UART5, 0xFE);    //�ȷ�������
		LPLD_UART_PutChar(UART5, 0x01);    //�ȷ�������
}

u8 OV7725_GetReadyBuf()
{
	//�Ѿ���ȡ��ɵ�֡Ϊ"����д���֡"��ǰ��֡
	if(NextBuffer==0)
	{return 2;}
	else if(NextBuffer==1)
	{return 3;}
	else
	{return NextBuffer-2;}
}

u16 OV7725_GetFPS()
{
	u16 temp;
	
	temp = OV7725_FPS;
	OV7725_FPS = 0;
	
	return temp;
}

void OV7725_SendImg(void *imgaddr, uint32_t imgsize)
{
#define CMD_IMG     1
    int8_t cmdf[2] = {CMD_IMG, ~CMD_IMG};    //ɽ����λ�� ʹ�õ�����
    int8_t cmdr[2] = {~CMD_IMG, CMD_IMG};    //ɽ����λ�� ʹ�õ�����

    LPLD_UART_PutCharArr(UART5, cmdf, sizeof(cmdf));    //�ȷ�������

    LPLD_UART_PutCharArr(UART5, (int8_t *)imgaddr, imgsize); //�ٷ���ͼ��

    LPLD_UART_PutCharArr(UART5, cmdr, sizeof(cmdr));    //�ȷ�������
}
