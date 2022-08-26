//=====================================================================================================
// File Name:	CtrlLib_settings.c
// Author:		Master Yip
// Date:		20220711
//=====================================================================================================
// Description:
//	MotorCtrl����ȫ�ֱ�������
//=====================================================================================================
#include "CtrlLib_settings.h"

/* 1 Drive Layer ---------------------------------------------------------*/

//��е�ۼ��Ĵ��Ƿ����˶�״̬
int In_Motion = 0;

//������ʵ���ٶ� ���PWM����
double MaxSpeed = 1.0;
int MaxPWM = 5000;

////���������ʵ������޸ġ�

struct MotorAttrib MotorGroup[16]={
{&TIM3->CCR2, &TIM3->CCR1},//�Ĵ���
{&TIM3->CCR4, &TIM3->CCR3},//�Ĵ���
{&TIM4->CCR3, &TIM4->CCR4},//ת̨
{&TIM4->CCR2, &TIM4->CCR1},//���
{&TIM8->CCR1, &TIM8->CCR3},//С��
{&TIM8->CCR2, &TIM8->CCR4}};//���鴸


//Legacy Control Vector
//ֹͣת��
double STOPVector[MOTORNUMBER] = {0};
//����е��̧�������
double RevisionMotion[MOTORNUMBER] = {0,0,0,0.7,0.7,0.9};

//Control Vector
struct TimebasedRevVector MOTOR_Stop 		={{0},{0}};
struct TimebasedRevVector MOTOR_Calibrate	={{0,0,0,0.7,0.7,0.8},{0,0,4,4,4,4}};
struct TimebasedRevVector MOTOR_Continue 	={{0},{-1,-1,-1,-1,-1,-1}};

/* 2 Quantization Layer ---------------------------------------------------------*/


/* 3 Control Layer ---------------------------------------------------------*/

//�����ķ�� �淶״̬�����飨��ͨ���ӿ��� �ڶ�άΪ0���ɣ�
double MecanumStdVector[3][4] = {{1, 1, 1, 1}, {1 ,-1, -1, 1}, {-1, 1, -1, 1}};

//ת�ٰ�ȫϵ������
double SafetyFactor = 1;
	
//ϵͳʱ������(s)
double TimePeriod = 0.005;

//˳�� ǰ�� ��ת ����
double VelocityVector[3] = {0};



/* 4 Abstract Layer ---------------------------------------------------------*/





