//=====================================================================================================
// IMU.h
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
//
// See IMU.c file for description.
// 
//=====================================================================================================
#ifndef __MAHONY_FILTER_H
#define __MAHONY_FILTER_H
#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "filter.h"
#include "SEEKFREE_ICM20602.h"	

/**
  * @brief  ����imu
  * @note   ��û���Զ�get_data!
  * @param  
  * @param  
  * @retval 
  */
void IMUupdate(struct ICM_CS_POINTER Pointer, struct IMU_data *imu, Filter6axisTypeDef *filter);

/**
  * @brief  ����ȫ��imu
  * @note   �Զ�get_data
  * @param  
  * @param  
  * @retval 
  */
void IMUs_update(struct ICM_CS_POINTER Pointer[], struct IMU_data imu[], Filter6axisTypeDef filter[]);
	
#ifdef __cplusplus
}
#endif

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
