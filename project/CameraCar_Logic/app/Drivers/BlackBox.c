#include "BlackBox.h"
#include "OV7725.h"

FRESULT rc;			//������� 
FATFS fatfs;			// �ļ�ϵͳ���� 
FIL f_bin;			// ��ʻ�����ļ����� 
FIL f_img;			// ͼ���ļ����� 
UINT bw;

u8 BinFileCnt=0;		//�켣�ļ����
u8 ImgFileCnt=0;		//ͼ���ļ����
u16 DataCnt=0;		//�켣���ݱ��

u8 isMount=0, isBinOpen=0;		//����״̬


void Fail(FRESULT rc)		// ��ӡ�ļ����ش���
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

		// ����һ���µ��ļ�
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
		// ����һ���µ��ļ�
		sprintf(str, "0:Image_%03d.raw", ImgFileCnt);
		rc = f_open(&f_img, (char const *)str, FA_WRITE | FA_CREATE_ALWAYS);
		if (rc) 
		{
			Fail(rc);
			return;
		}
	}
	
	//д��ͼƬ
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
