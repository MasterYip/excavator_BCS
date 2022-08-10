//=====================================================================================================
// File Name:	drive_layer.c
// Author:		Master Yip
// Date:		20220711
//=====================================================================================================
// Description:
//	������������㣬����Ӳ���ӿڣ��ṩ������ƺ��������������ƶ�ȡ����
//=====================================================================================================
#include "CtrlLib_settings.h"

/**
  * @brief  �������ת�ٺ���
  * @note   Speed���á�������ʵ��ת�١���Speed�������Ϊ������ʽ���ٶȣ�Ҳ�������Ϊ���ת�����ٶȣ�
  * ȫ�ֱ���MaxSpeed MaxPWM��CtrlLib_settings.c�ж���
  * @param  int SerialNumber �����
  * @param  double Speed ���ʵ��ת�٣�ȡֵ��ΧΪ[-MaxSpeed, MaxSpeed]
  * @retval VOID
  */
int Motor_Control(int SerialNumber, double Speed)
{
	//������������ʵ�ʵ����������Ʒ�ʽ��д��
	if(Speed > MaxSpeed)Speed = MaxSpeed;
	if(Speed < -MaxSpeed)Speed = -MaxSpeed;
	
	if(Speed >= 0)
	{
		*(MotorGroup[SerialNumber].PWM_IN1) = (u16) (Speed/MaxSpeed*MaxPWM);
		*(MotorGroup[SerialNumber].PWM_IN2) = (u16) 0;
	}
	else
	{
		*(MotorGroup[SerialNumber].PWM_IN1) = (u16) 0;
		*(MotorGroup[SerialNumber].PWM_IN2) = (u16) (-Speed/MaxSpeed*MaxPWM);
	}
	
}

/**
  * @brief  ���ݵ����Ž�����ת���������������е�����ν���ת�ٿ���
  * @note   MOTORNUMBER��CtrlLib_settings.h�ж���
  * @param  double Vector[] ת�������������±�Ϊ������
  * @param  
  * @retval 
  */
int Vector_Control(double Vector[])
{
	int i=0;
	for(i=0; i<MOTORNUMBER; i++)
	{
		Motor_Control(i, Vector[i]);
	}
}

/**
  * @brief  ���ݵ����Ž�����ת���������������е�����ν��о���ʱ�����Ƶ�ת�ٿ��ƣ�������������
  * @note   MOTORNUMBER��CtrlLib_settings.h�ж���
  *			ע�⣡�˺���������ÿ���������ڵ���
  * @param  double Vector[] ת�������������±�Ϊ������
  * @param  double LastingTime[] ����ʱ��
  *			lastingtime>=0 ���������������µ��ת�ٺ͵���ʱ
  *			lastingtime<0 ���������ݣ�������ǰ���˶���
  * @retval 
  */
int Vector_Control_with_Time_Limited(struct TimebasedRevVector CtrlVector)
{
	int i=0;
	static double motion_duration[MOTORNUMBER] = {0};
	
	In_Motion = 0;
	for(i=0; i<MOTORNUMBER; i++)
	{
		motion_duration[i]-=TimePeriod;
		
		if(CtrlVector.LastingTime[i] >= 0)
		{
			motion_duration[i] = CtrlVector.LastingTime[i];
			Motor_Control(i, CtrlVector.RevVector[i]);
		}
		

		if(motion_duration[i]<=0)
		{
			Motor_Control(i, 0.0);
			motion_duration[i]=0.0;
		}
		
		if(motion_duration[i] > 0) In_Motion=1;
	}
	return 0;
}

/**
  * @brief  �������������
  * @note   [DEBUG]�������ƴű�����
  * @param  
  * @param  
  * @retval 
  */
int Encoder_RW(int SerialNumber)
{
	int Encoder_TIM;
	switch(SerialNumber)
	{
		case 0:  
			Encoder_TIM = (short)(TIM2 -> CNT-30000);
			TIM2 -> CNT=30000;
			break;
		case 1:
			Encoder_TIM = (short)-(TIM3 -> CNT-30000);
			TIM3 -> CNT=30000;
			break;
		case 2:
			Encoder_TIM = (short)(TIM4 -> CNT-30000);
			TIM4 -> CNT=30000;
			break;
		case 3:
			Encoder_TIM = (short)-(TIM5 -> CNT-30000);
			TIM5 -> CNT=30000;
			break;
		default:
			Encoder_TIM=0;
	}
	return Encoder_TIM;
}

/**
  * @brief  �����������ȡ
  * @note   [DEBUG]���ڶ�ȡ�ű�����
  * @param  
  * @param  
  * @retval 
  */
int Encoder_Read(int SerialNumber)
{
	int Encoder_TIM;
	switch(SerialNumber)
	{
		case 0:  
			Encoder_TIM = (short)(TIM2 -> CNT);
			break;
		case 1:
			Encoder_TIM = (short)-(TIM3 -> CNT);
			break;
		case 2:
			Encoder_TIM = (short)(TIM4 -> CNT);
			break;
		case 3:
			Encoder_TIM = (short)-(TIM5 -> CNT);
			break;
		default:
			Encoder_TIM=0;
	}
	return Encoder_TIM;
}