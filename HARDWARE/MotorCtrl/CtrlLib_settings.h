#ifndef __CTRLLIB_SETTINGS_H
#define __CTRLLIB_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* PROJECT Global includes ---------------------------------------------*/

#include "main.h"

/* General Lib includes (ʹ���Զ���ʹ��)------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

////////////////////////////////////////////////////////
//=================== DriveLayer ===================//
////////////////////////////////////////////////////////

/* Macro Definition ----------------------------------------------------*/

#define MOTORNUMBER 6
#define MAXSPEED (double)1.0

//�޿���ָ������ת����ʱ
#define MOTOR_CTRL_DELAY 0.4

extern double MaxSpeed;
extern int MaxPWM;

/* Variables Declaration -----------------------------------------------*/

//С������
//(������)

//�����ַ�ṹ��
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


//���⴫��������
struct InfraredSensor
{
	int InputValue;
	int LowThreshold;
	int HighThershold;
	double Position;
	double value;
};



//�˶�״̬����
struct ProcessControl
{
	int MotionType;//0-����ֱ�� 1-ת��
	double parameter[10];
};


//ֹͣת��
extern double STOPVector[];
//�������� ̧�������
extern double RevisionMotion[MOTORNUMBER];

extern struct TimebasedRevVector MOTOR_Stop; 		//={{0},{0}};
extern struct TimebasedRevVector MOTOR_Calibrate;	//={{0,0,0,0.7,0.7,0.8},{0,0,4,4,4,4}};
extern struct TimebasedRevVector MOTOR_Continue; 	//={{0},{-1,-1,-1,-1,-1,-1}};



//����ʵ������޸�
extern struct MotorAttrib MotorGroup[16];
//����
extern struct InfraredSensor Infrared[2][5];
//״̬����
extern struct ProcessControl ProcessCtrl[10];

//�����ķ�� �淶״̬������
extern double MecanumStdVector[3][4];

//ת�ٰ�ȫϵ������
extern double SafetyFactor;
	
//ϵͳʱ������(s)
extern double TimePeriod;

//�ٶ�����
extern double VelocityVector[3];


//���ڻ���

extern u8 cmd;




/* Function Declaration ------------------------------------------------*/

/* drive_layer */
int Motor_Control(int SerialNumber, double Speed);
int Vector_Control(double Vector[]);
int Vector_Control_with_Time_Limited(struct TimebasedRevVector CtrlVector);

//��ȡ���������������ݣ�����ٶȣ�
int Encoder_RW(int SerialNumber);
//����ȡ�����������λ�ã�
int Encoder_Read(int SerialNumber);


/* Quantization_layer */
int RevVector_Quantification(double RevVector[], double OutputVector[]);
/* Control_layer */
int RevVector_Superposition(double VelocityVector[], double RevVector[]);

/**
  * @brief  Ŀǰ��ר��Ϊ��е����Ƶ�PID
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
