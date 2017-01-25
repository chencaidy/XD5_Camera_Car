#include "common.h"
#include "OV7725.h"

void Mount_Disk();

void Unmount_Disk();

void BinRecord_Start();

void BinRecord_Stop();

void BinRecord_Data(s16 speed, s16 *accel, s16 *gyro, s16 *quat);

void Img_Record(u8 ImgConvent[CAMERA_H][CAMERA_W]);

