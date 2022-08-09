//=====================================================================================================
// File Name:	RobotArm.c
// Author:		Master Yip
// Date:		20220711
//=====================================================================================================
// Description:
//	Including RobotArm resolver and controller
//=====================================================================================================
#ifndef ROBOTARM_H
#define ROBOTARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SEEKFREE_ICM20602.h"

 /**
 * @brief  给定IMU数据与机械臂角度，计算修正四元数
   * @note   
   * @param  float IMUs_Quaternion[4][4]
   * @param  float Angles[4]
   * @retval float IMUs_QuaRevision[4][4]
   */
void QuaternionReviser(struct IMU_data imu[4], float Angles[4]);
	
/**
  * @brief  Resolve joints' angles given Four IMU Attitudes and its revision quaternion
  * @note   
  * @param  float IMUs_Quaternion[4][4]  IMU data in the form of Quaternion
  * @param  float IMUs_QuaRevision[4][4]  IMU Revision matrix
  * @retval float Angles[4]
  */
void RobotArm_Resolver(struct IMU_data imu[4], float Angles[4]);

/**
  * @brief  计算相对四元数u*Conj(v)
  * @note   
  * @param  
  * @param  
  * @retval 
  */
void xiangdui(float u[4],float v[4],float k[4]);

#define M_PI 3.14159

#ifdef __cplusplus
}
#endif
#endif

