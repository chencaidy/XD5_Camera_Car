#include "HMI.h"
#include "OV7725.h"
#include "BlackBox.h"
#include "OLED12864.h"
#include "Comm_Master.h"
#include "MPU6050_Soft.h"
#include "Debug.h"
#include "Algorithm.h"

void main()
{
	RunLED_Config();
	Comm_Config();
	HMI_Config(115200);
	OLED_Config();
	OV7725_Config();
	MPU6050_Init();
	DMP_Init();

	PIT_Config();		//调试用PIT定时器，1000ms触发
	
  while(1)
  {
		//To-Do
		Algorithm_Bak();
		
		Main_Test();		//test
  }
}
