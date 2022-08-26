//=====================================================================================================
// File Name:	CtrlLib_settings.c
// Author:		Master Yip
// Date:		20220711
//=====================================================================================================
// Description:
//	MotorCtrl所需全局变量定义
//=====================================================================================================
#include "CtrlLib_settings.h"

/* 1 Drive Layer ---------------------------------------------------------*/

//机械臂及履带是否在运动状态
int In_Motion = 0;

//最大广义实际速度 最大PWM限制
double MaxSpeed = 1.0;
int MaxPWM = 5000;

////【必须根据实际情况修改】

struct MotorAttrib MotorGroup[16]={
{&TIM3->CCR2, &TIM3->CCR1},//履带左
{&TIM3->CCR4, &TIM3->CCR3},//履带右
{&TIM4->CCR3, &TIM4->CCR4},//转台
{&TIM4->CCR2, &TIM4->CCR1},//大臂
{&TIM8->CCR1, &TIM8->CCR3},//小臂
{&TIM8->CCR2, &TIM8->CCR4}};//破碎锤


//Legacy Control Vector
//停止转动
double STOPVector[MOTORNUMBER] = {0};
//将机械臂抬升至最高
double RevisionMotion[MOTORNUMBER] = {0,0,0,0.7,0.7,0.9};

//Control Vector
struct TimebasedRevVector MOTOR_Stop 		={{0},{0}};
struct TimebasedRevVector MOTOR_Calibrate	={{0,0,0,0.7,0.7,0.8},{0,0,4,4,4,4}};
struct TimebasedRevVector MOTOR_Continue 	={{0},{-1,-1,-1,-1,-1,-1}};

/* 2 Quantization Layer ---------------------------------------------------------*/


/* 3 Control Layer ---------------------------------------------------------*/

//麦克纳姆轮 规范状态向量组（普通轮子可用 第二维为0即可）
double MecanumStdVector[3][4] = {{1, 1, 1, 1}, {1 ,-1, -1, 1}, {-1, 1, -1, 1}};

//转速安全系数保护
double SafetyFactor = 1;
	
//系统时间周期(s)
double TimePeriod = 0.005;

//顺序 前行 右转 右移
double VelocityVector[3] = {0};



/* 4 Abstract Layer ---------------------------------------------------------*/





