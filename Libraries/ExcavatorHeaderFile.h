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

//硬件IIC测试 不过注释掉MDK会自动查找函数
//#include "MPU_6050.h"

#ifdef __cplusplus
extern "C" {
#endif
/* Macros ---------------------------------------------------------*/


/* Structs ---------------------------------------------------------*/
//struct Attitude_Msg
//{
//	float angles[4];	//转台、机械臂姿态(挖掘机本体系)
//};

struct Ctrl_Msg
{
	/*ctrl_mode=0 | Control robotarm using destinate angles
	 *ctrl_mode=1 | Control robotarm using expected angle velocity*/
	u8 ctrl_mode;
	float chassis_vel[2];	//与ctrl_lib的抽象控制层对接，控制挖掘机移动、转弯（实际上两个数就够，但考虑了ctrl_lib的兼容性）
	float angles[4];	//ctrl_mode=0时生效，期望的转台、机械臂姿态(挖掘机本体系)
	float angle_vel[4];	//ctrl_mode=1时生效，期望的机械臂关节角速度
};


/* Extern Variables ---------------------------------------------------------*/
//串口缓冲
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
