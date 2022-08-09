//=====================================================================================================
// File Name:	quantization_layer.c
// Author:		Master Yip
// Date:		20220711
//=====================================================================================================
// Description:
//	����������㡿�������ת�ٽ��г���Ŀ���ǽ���Ӳ�������㡿�롾�����Ʋ㡿�������⺯����ֲ
//=====================================================================================================
#include "CtrlLib_settings.h"

/**
  * @brief  ���ת��������������
  * @note   ����������ת��������ת�ٽ���[-1, 1]�����ڿ��ƺʹ����д�����������ʵ��ת��
  * @param  double RevVector[] ������ת������
  * @retval double OutputVector[] ���ʵ��ת��
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
