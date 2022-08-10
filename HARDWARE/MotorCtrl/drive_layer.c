//=====================================================================================================
// File Name:	drive_layer.c
// Author:		Master Yip
// Date:		20220711
//=====================================================================================================
// Description:
//	电机控制驱动层，处理硬件接口，提供电机控制函数、编码器控制读取函数
//=====================================================================================================
#include "CtrlLib_settings.h"

/**
  * @brief  控制马达转速函数
  * @note   Speed采用【马达广义实际转速】（Speed可以理解为车轮形式线速度，也可以理解为马达转动角速度）
  * 全局变量MaxSpeed MaxPWM在CtrlLib_settings.c中定义
  * @param  int SerialNumber 马达编号
  * @param  double Speed 马达实际转速，取值范围为[-MaxSpeed, MaxSpeed]
  * @retval VOID
  */
int Motor_Control(int SerialNumber, double Speed)
{
	//如下语句须根据实际电机驱动板控制方式编写！
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
  * @brief  根据电机编号建立【转速向量】，对所有电机依次进行转速控制
  * @note   MOTORNUMBER在CtrlLib_settings.h中定义
  * @param  double Vector[] 转速向量，数组下标为电机编号
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
  * @brief  根据电机编号建立【转速向量】，对所有电机依次进行具有时间限制的转速控制，并允许动作叠加
  * @note   MOTORNUMBER在CtrlLib_settings.h中定义
  *			注意！此函数必须在每个控制周期调用
  * @param  double Vector[] 转速向量，数组下标为电机编号
  * @param  double LastingTime[] 持续时间
  *			lastingtime>=0 根据数据正常更新电机转速和倒计时
  *			lastingtime<0 不更新数据（延续以前的运动）
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
  * @brief  电机编码器控制
  * @note   [DEBUG]用于重制磁编码器
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
  * @brief  电机编码器读取
  * @note   [DEBUG]用于读取磁编码器
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