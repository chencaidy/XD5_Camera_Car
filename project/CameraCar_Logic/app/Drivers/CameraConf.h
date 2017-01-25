#include "OV7725.h"

typedef struct
{
	u8 addr;                 /*寄存器地址*/
	u8 val;                   /*寄存器值*/
} Camera_Reg;

//定义图像采集状态
typedef enum
{
    IMG_NOTINIT=0,
    IMG_FINISH,             //图像采集完毕
    IMG_FAIL,               //图像采集失败(采集行数少了)
    IMG_GATHER,             //图像采集中
    IMG_START,              //开始采集图像
    IMG_STOP,               //禁止图像采集
} IMG_STATUS;

/*OV7725初始化配置表*/
Camera_Reg OV7725_Eagle_Reg[] =
{
	//寄存器，寄存器值次
	{OV7725_COM2         , 0x03},
	{OV7725_COM3         , 0xD0},
	{OV7725_COM4         , 0xC1},
	{OV7725_CLKRC        , 0x00},
	{OV7725_COM7         , 0x40},
	{OV7725_HSTART       , 0x3F},
	{OV7725_HSIZE        , 0x50},
	{OV7725_VSTRT        , 0x03},
	{OV7725_VSIZE        , 0x78},
	{OV7725_HREF         , 0x00},
	{OV7725_SCAL0        , 0x0A},
	{OV7725_AWB_Ctrl0    , 0xE0},
	{OV7725_DSPAuto      , 0xff},
	{OV7725_DSP_Ctrl2    , 0x0C},
	{OV7725_DSP_Ctrl3    , 0x00},
	{OV7725_DSP_Ctrl4    , 0x00},

#if (CAMERA_W == 80)
	{OV7725_HOutSize     , 0x14},
#elif (CAMERA_W == 160)
	{OV7725_HOutSize     , 0x28},
#elif (CAMERA_W == 240)
	{OV7725_HOutSize     , 0x3c},
#elif (CAMERA_W == 320)
	{OV7725_HOutSize     , 0x50},
#else
	{OV7725_HOutSize     , 0x14},
#endif

#if (CAMERA_H == 60 )
	{OV7725_VOutSize     , 0x1E},
#elif (CAMERA_H == 120 )
	{OV7725_VOutSize     , 0x3c},
#elif (CAMERA_H == 180 )
	{OV7725_VOutSize     , 0x5a},
#elif (CAMERA_H == 240 )
	{OV7725_VOutSize     , 0x78},
#else
	{OV7725_VOutSize     , 0x1E},
#endif

	{OV7725_EXHCH        , 0x00},
	{OV7725_GAM1         , 0x0c},
	{OV7725_GAM2         , 0x16},
	{OV7725_GAM3         , 0x2a},
	{OV7725_GAM4         , 0x4e},
	{OV7725_GAM5         , 0x61},
	{OV7725_GAM6         , 0x6f},
	{OV7725_GAM7         , 0x7b},
	{OV7725_GAM8         , 0x86},
	{OV7725_GAM9         , 0x8e},
	{OV7725_GAM10        , 0x97},
	{OV7725_GAM11        , 0xa4},
	{OV7725_GAM12        , 0xaf},
	{OV7725_GAM13        , 0xc5},
	{OV7725_GAM14        , 0xd7},
	{OV7725_GAM15        , 0xe8},
	{OV7725_SLOP         , 0x20},
	{OV7725_LC_RADI      , 0x00},
	{OV7725_LC_COEF      , 0x13},
	{OV7725_LC_XC        , 0x08},
	{OV7725_LC_COEFB     , 0x14},
	{OV7725_LC_COEFR     , 0x17},
	{OV7725_LC_CTR       , 0x05},
	{OV7725_BDBase       , 0x99},
	{OV7725_BDMStep      , 0x03},
	{OV7725_SDE          , 0x04},
	{OV7725_BRIGHT       , 0x00},
	{OV7725_CNST         , 0x60},
	{OV7725_SIGN         , 0x06},
	{OV7725_UVADJ0       , 0x11},
	{OV7725_UVADJ1       , 0x02},
};
