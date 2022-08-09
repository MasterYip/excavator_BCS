//=====================================================================================================
// IMU.h
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
//
// See IMU.c file for description.
// 
//=====================================================================================================
#ifndef IMU_h
#define IMU_h
#ifdef __cplusplus
extern "C" {
#endif
//---------------------------------------------------------------------------------------------------
// Function declaration

void IMUupdate_1(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw);
void IMUupdate_2(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw);
void IMUupdate_3(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw);
void IMUupdate_4(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw);

#ifdef __cplusplus
}
#endif
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
