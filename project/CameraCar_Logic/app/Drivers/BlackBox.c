#include "BlackBox.h"
#include "OV7725.h"

FRESULT rc;			//结果代码 
FATFS fatfs;			// 文件系统对象 
FIL f_bin;			// 行驶数据文件对象 
FIL f_img;			// 图像文件对象 
UINT bw;

u8 BinFileCnt=0;		//轨迹文件编号
u8 ImgFileCnt=0;		//图像文件编号
u16 DataCnt=0;		//轨迹数据编号

u8 isMount=0, isBinOpen=0;		//挂载状态


void Fail(FRESULT rc)		// 打印文件返回代码
{printf("FatFS: <Error> Return=%u.\r\n", rc);}

void Mount_Disk()
{
	rc = f_mount(&fatfs, "0:", 0);
	if (rc) 
	{
		Fail(rc);
		return;
	}
	printf("BlackBox: <Info> SDCard Mount Success!\r\n");
	isMount = 1;
}

void Unmount_Disk()
{
	rc = f_mount(NULL, "0:", 0);
	if (rc) 
	{
		Fail(rc);
		return;
	}
	printf("BlackBox: <Info> SDCard Unmount Success!\r\n");
	isMount = 0;
}

void BinRecord_Start()
{
	if(isMount==0)
	{
		printf("BlackBox: <Error> SDCard Mount Failed!\r\n");
		return;
	}
	else
	{
		s8 str[15];

		// 创建一个新的文件
		sprintf(str, "0:Locus_%03d.bin", BinFileCnt);
		rc = f_open(&f_bin, (char const *)str, FA_WRITE | FA_CREATE_ALWAYS);
		if (rc) 
		{
			Fail(rc);
			return;
		}
		isBinOpen = 1;
	}
}

void BinRecord_Stop()
{
	rc = f_close(&f_bin);
	if (rc) 
	{
		Fail(rc);
		return;
	}
	
	DataCnt = 0;
	BinFileCnt++;
	isBinOpen = 0;
}

void BinRecord_Data(s16 speed, s16 *accel, s16 *gyro, s16 *quat)
{
	if(isMount==0 || isBinOpen==0)
	{
		printf("BlackBox: <Error> SDCard Mount Failed!\r\n");
		return;
	}
	else
	{
		s8 str[96];
		u8 len;
		
		len = sprintf(str, "T%dS%dP%dR%dY%dX%dY%dZ%dX%dY%dZ%d\n", 
						DataCnt, speed, accel[0], accel[1], accel[2],
						gyro[0], gyro[1], gyro[2], 
						quat[0], quat[1], quat[2]);
		rc = f_write(&f_bin, str, len, &bw);
		DataCnt++;
		if (rc) 
		{
			Fail(rc);
			return;
		}
	}	
}

void Img_Record(u8 ImgConvent[CAMERA_H][CAMERA_W])
{
	s8 str[15];
	
	if(isMount==0)
	{
		printf("BlackBox: <Error> SDCard Mount Failed!\r\n");
		return;
	}
	else
	{
		// 创建一个新的文件
		sprintf(str, "0:Image_%03d.raw", ImgFileCnt);
		rc = f_open(&f_img, (char const *)str, FA_WRITE | FA_CREATE_ALWAYS);
		if (rc) 
		{
			Fail(rc);
			return;
		}
	}
	
	//写入图片
	rc = f_write(&f_img, ImgConvent, CAMERA_H*CAMERA_W, &bw);
	if (rc) 
	{
		Fail(rc);
		return;
	}
	
	rc = f_close(&f_img);
	if (rc) 
	{
		Fail(rc);
		return;
	}
	
	printf("BlackBox: <Info> ScreenShot, Path:");
	printf(str);
	printf("\r\n");
	ImgFileCnt++;
}
