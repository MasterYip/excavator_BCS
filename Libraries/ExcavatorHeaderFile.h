#ifndef EXCAVATORHEADERFILE_H
#define EXCAVATORHEADERFILE_H

/* Headers ---------------------------------------------------------*/
//System Header
#include <math.h>
#include <string.h>
#include <stdbool.h>
//User Header
#include "delay.h"
#include "sys.h"

//ICM20602
#include "mahony_filter.h"

#include "CtrlLib_settings.h"

#include "ano_client.h"
#include "RobotArm.h"

//ROS
#include "uart_node.hpp"

//Ӳ��IIC���� ����ע�͵�MDK���Զ����Һ���
//#include "MPU_6050.h"

#ifdef __cplusplus
extern "C" {
#endif
/* Macros ---------------------------------------------------------*/


/* Structs ---------------------------------------------------------*/
//struct Attitude_Msg
//{
//	float angles[4];	//ת̨����е����̬(�ھ������ϵ)
//};

struct Ctrl_Msg
{
	/*ctrl_mode=0 | Control robotarm using destinate angles
	 *ctrl_mode=1 | Control robotarm using expected angle velocity*/
	u8 ctrl_mode;
	float chassis_vel[2];	//��ctrl_lib�ĳ�����Ʋ�Խӣ������ھ���ƶ���ת�䣨ʵ�����������͹�����������ctrl_lib�ļ����ԣ�
	float angles[4];	//ctrl_mode=0ʱ��Ч��������ת̨����е����̬(�ھ������ϵ)
	float angle_vel[4];	//ctrl_mode=1ʱ��Ч�������Ļ�е�۹ؽڽ��ٶ�
};


/* Extern Variables ---------------------------------------------------------*/
//���ڻ���
extern u8 buff[];
extern u8 sendbuff[];
extern u8 cmd;
extern float systime;
extern float Angles[4];
extern double urdfAngles[4];
extern float AngleforRevision[4];
/* Init Functions ---------------------------------------------------------*/
void IMU_Data_Init(struct IMU_data imu[4]);


/* Utility ---------------------------------------------------------*/
void HALprintf(char *data);
#ifdef __cplusplus
}
#endif

#endif
