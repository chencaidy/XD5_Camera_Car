	for(k=0;k<20;k++)
	{
		for(j=30;j<60;j++)
		{
			if(Image_Convent[j][k]==0xFF)
			{a++;}
		}
	}
	for(k=0;k<20;k++)
	{
		for(j=30;j<60;j++)
		{
			if(Image_Convent[j][79-k]==0xFF)
			{b++;}
		}
	}
	c = a-b;
	GetSpeed(&d);
	if(fabs(Gyro[2])<5)
	{SD5_ChangeDuty(5000-(s32)(c*1.3));}
	else
	{SD5_ChangeDuty(5000-(s32)(c*2.7));}
	Motor_ChangeSpeed(GoalSpeed-(fabs(Gyro[2])*0.02));
	a = 0; b = 0;
	c_last = c;
	
	//舵机分段 PD
	if(a_max<30)
	{SD5_ChangeDuty(5000-(s32)(d*80));}
	else if(a_max<40)
	{SD5_ChangeDuty(5000-(s32)(d*55));}
	else if(a_max<50)
	{SD5_ChangeDuty(5000-(s32)(d*37));}
	else
	{SD5_ChangeDuty(5000-(s32)(d*30));}
	
	//电机速度设定
	if(a_max<45)
	{Motor_ChangeSpeed(GoalSpeed-200);}
	else
	{Motor_ChangeSpeed(GoalSpeed);}