//=====================================================================================================
// File Name:	quantization_layer.c
// Author:		Master Yip
// Date:		20220711
//=====================================================================================================
// Description:
//	【马达量化层】，对马达转速进行抽象，目的是将【硬件驱动层】与【马达控制层】解耦，方便库函数移植
//=====================================================================================================
#include "CtrlLib_settings.h"

/**
  * @brief  马达转速向量量化函数
  * @note   输入马达相对转速向量（转速介于[-1, 1]，便于控制和代码编写），返回马达实际转速
  * @param  double RevVector[] 马达相对转速向量
  * @retval double OutputVector[] 马达实际转速
  */
int RevVector_Quantification(double RevVector[], double OutputVector[])
{
	int i=0;
	for(i=0; i<MOTORNUMBER; i++)
	{
		OutputVector[i] = RevVector[i]*MAXSPEED;
	}
	return 0;
}
