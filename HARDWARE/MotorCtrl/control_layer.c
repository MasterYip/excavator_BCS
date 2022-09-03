//=====================================================================================================
// File Name:	control_layer.c
// Author:		Master Yip
// Date:		20220711
//=====================================================================================================
// Description:
//	�������Ʋ㡿���ԡ����ת����������һ������Ϊ�������ٶ�������
//=====================================================================================================
#include "CtrlLib_settings.h"

/**
  * @brief  �����ٶ�������ת��Ϊ�����ת��������
  * @note   ��ʱ֧�֡������ķת�򡿡���ͨ��ϵ����ת��
  * @param  double VelocityVector[] �����ٶ�����
  * @retval double RevVector[] Revolving Speed Vectorת������
  */
int RevVector_Superposition(double VelocityVector[], double RevVector[])
{
	int i=0;
	int j=0;
	double sum = 0;
	
	//�����ķ�� �ٶ�ʸ����������
	for(i=0; i<3; i++)
	{
		sum += fabs(VelocityVector[i]);
	}
	
	if(sum > 1*SafetyFactor)
	{
		for(i=0; i<3; i++)
		{
			VelocityVector[i] /= sum;
			VelocityVector[i] *= SafetyFactor;
		}
	}
	
	//�ٶ�ʸ�� ת��Ϊ ת��ʸ��
	for(i=0; i<4; i++)
	{
		RevVector[i] = 0;
		
		for(j=0; j<3; j++)
		{
			RevVector[i] += VelocityVector[j]*MecanumStdVector[j][i];
		}
	}
}


/**
  * @brief  Ŀǰ��ר��Ϊ��е����Ƶ�PID
  * @note   
  * @param  
  * @param  
  * @retval 
  */
int PID_arm_controller(char* name[], double TargetAngles[], double TargetAngVel[], double PresentAngles[])
{
	char data[200]="";
	
	static float kp = 1;
	static float kd = 0.05;
	static float ki = 0; //��ʱû�н������
	static double PreviousAngles[4] = {0};
	static struct TimebasedRevVector CtrlVector = {{0}, 
	{MOTOR_CTRL_DELAY,MOTOR_CTRL_DELAY,MOTOR_CTRL_DELAY,MOTOR_CTRL_DELAY,MOTOR_CTRL_DELAY,MOTOR_CTRL_DELAY}};
	
	for(int i = 2; i<MOTORNUMBER; i++)
	{
		CtrlVector.RevVector[i] = 
			kp*(TargetAngles[i-2]-PresentAngles[i-2]) 
			- kd*(PresentAngles[i-2]-PreviousAngles[i-2])/TimePeriod
			+ TargetAngVel[i-2];
	}
	
//	for(int i = 0; i<4; i++)
//	{
//		int k=-1;
////		HALprintf("\r\n NAME: ");
////		HALprintf(name[i]);
////		if(strcmp(name[i], "table_joint")==0)
////		{
////			k=2;
////		}
////		else if(strcmp(name[i], "link1_joint")==0)
////		{
////			k=3;
////		}
////		else if(strcmp(name[i], "link2_joint")==0)
////		{
////			k=4;
////		}
////		else if(strcmp(name[i], "ram_joint")==0)
////		{
////			k=5;
////		}
//		k=i+2;
//		
//		if(k != -1)
//		{
//			CtrlVector.RevVector[k] = 
//				kp*(TargetAngles[i]-PresentAngles[i]) 
//				- kd*(PresentAngles[i]-PreviousAngles[i])/TimePeriod
//				+ TargetAngVel[i];
//		}
//	}
	
//	if(CtrlVector.RevVector[2] < 0.1)CtrlVector.RevVector[2]=0;
	
	//�������ѵ�Ƭ�������� ���������
//	memcpy(PreviousAngles, PresentAngles, sizeof(double)*4);
	
	PreviousAngles[0] = PresentAngles[0];
	PreviousAngles[1] = PresentAngles[1];
	PreviousAngles[2] = PresentAngles[2];
	PreviousAngles[3] = PresentAngles[3];
	
	
	Vector_Control_with_Time_Limited(CtrlVector);
}

