#ifndef __CTRLLIB_SETTINGS_H
#define __CTRLLIB_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* PROJECT Global includes ---------------------------------------------*/

#include "main.h"

/* General Lib includes (使可以独立使用)------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

////////////////////////////////////////////////////////
//=================== DriveLayer ===================//
////////////////////////////////////////////////////////

/* Macro Definition ----------------------------------------------------*/

#define MOTORNUMBER 6
#define MAXSPEED (double)1.0

//无控制指令的马达转动延时
#define MOTOR_CTRL_DELAY 0.4

extern double MaxSpeed;
extern int MaxPWM;

/* Variables Declaration -----------------------------------------------*/

//小车参数
//(待完善)

//电机地址结构体
struct MotorAttrib
{
	volatile uint32_t *PWM_IN1;
	volatile uint32_t *PWM_IN2;
};


struct TimebasedRevVector
{
	double RevVector[MOTORNUMBER];
	double LastingTime[MOTORNUMBER];
};


//红外传感器参数
struct InfraredSensor
{
	int InputValue;
	int LowThreshold;
	int HighThershold;
	double Position;
	double value;
};



//运动状态控制
struct ProcessControl
{
	int MotionType;//0-定长直行 1-转弯
	double parameter[10];
};


//停止转动
extern double STOPVector[];
//修正动作 抬升到最高
extern double RevisionMotion[MOTORNUMBER];

extern struct TimebasedRevVector MOTOR_Stop; 		//={{0},{0}};
extern struct TimebasedRevVector MOTOR_Calibrate;	//={{0,0,0,0.7,0.7,0.8},{0,0,4,4,4,4}};
extern struct TimebasedRevVector MOTOR_Continue; 	//={{0},{-1,-1,-1,-1,-1,-1}};



//根据实际情况修改
extern struct MotorAttrib MotorGroup[16];
//红外
extern struct InfraredSensor Infrared[2][5];
//状态管理
extern struct ProcessControl ProcessCtrl[10];

//麦克纳姆轮 规范状态向量组
extern double MecanumStdVector[3][4];

//转速安全系数保护
extern double SafetyFactor;
	
//系统时间周期(s)
extern double TimePeriod;

//速度向量
extern double VelocityVector[3];


//串口缓冲

extern u8 cmd;




/* Function Declaration ------------------------------------------------*/

/* drive_layer */
int Motor_Control(int SerialNumber, double Speed);
int Vector_Control(double Vector[]);
int Vector_Control_with_Time_Limited(struct TimebasedRevVector CtrlVector);

//读取编码器并重设数据（检测速度）
int Encoder_RW(int SerialNumber);
//仅读取编码器（检测位置）
int Encoder_Read(int SerialNumber);


/* Quantization_layer */
int RevVector_Quantification(double RevVector[], double OutputVector[]);
/* Control_layer */
int RevVector_Superposition(double VelocityVector[], double RevVector[]);

/**
  * @brief  目前是专门为机械臂设计的PID
  * @note   
  * @param  
  * @param  
  * @retval 
  */
int PID_arm_controller(double TargetAngles[], double TargetAngVel[], double PresentAngles[]);

/* Abstract_layer */
void Velocity_Set(double VelocityVector[]);

void Motor_Debug(void);
void key_teleop_ctrl(u8 *cmd);
void chassis_vel_ctrl_time(double OutputVector[], u8 mode);


#ifdef __cplusplus
}
#endif
#endif
