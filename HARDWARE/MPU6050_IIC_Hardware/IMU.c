//=====================================================================================================
// IMU.c
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer 
// units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

#include "IMU.h"
#include <math.h>
#include "mpu6050.h"
//----------------------------------------------------------------------------------------------------
// Definitions

float Kp 	=	10.0f;//10.0f					// proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki	=	0.01f;//0.01f				// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025		// half the sample period

#define q30  1073741824.0f  //q30¸ñÊ½,long×ªfloatÊ±µÄ³ýÊý.
//---------------------------------------------------------------------------------------------------

//====================================================================================================
// Function
//====================================================================================================

void IMUupdate_1(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;         
	
	static float q0=1;
	static float q1=0;
	static float q2=0;
	static float q3=0;
	static float exInt=0;
	static float eyInt=0;
	static float ezInt=0;
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);   
	if(norm>=0.01){	
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;  
	}	
	else
	{
		ax=0;ay=0;az=0;
	}
	
	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	// integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	

	//¼ÆËãµÃµ½¸©Ñö½Ç/ºá¹ö½Ç/º½Ïò½Ç
	*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
	*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
	*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
		
}

void IMUupdate_2(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;         
	
	static float q0=1;
	static float q1=0;
	static float q2=0;
	static float q3=0;
	static float exInt=0;
	static float eyInt=0;
	static float ezInt=0;
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);   
	if(norm>=0.01){	
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;  
	}	
	else
	{
		ax=0;ay=0;az=0;
	}
	
	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	// integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	

	//¼ÆËãµÃµ½¸©Ñö½Ç/ºá¹ö½Ç/º½Ïò½Ç
	*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
	*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
	*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	
}

void IMUupdate_3(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;         
	
	static float q0=1;
	static float q1=0;
	static float q2=0;
	static float q3=0;
	static float exInt=0;
	static float eyInt=0;
	static float ezInt=0;
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);   
	if(norm>=0.01){	
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;  
	}	
	else
	{
		ax=0;ay=0;az=0;
	}
	
	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	// integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	

	//¼ÆËãµÃµ½¸©Ñö½Ç/ºá¹ö½Ç/º½Ïò½Ç
	*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
	*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
	*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	
}

void IMUupdate_4(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;         
	
	static float q0=1;
	static float q1=0;
	static float q2=0;
	static float q3=0;
	static float exInt=0;
	static float eyInt=0;
	static float ezInt=0;
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);   
	if(norm>=0.01){	
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;  
	}	
	else
	{
		ax=0;ay=0;az=0;
	}
	
	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	// integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	

	//¼ÆËãµÃµ½¸©Ñö½Ç/ºá¹ö½Ç/º½Ïò½Ç
	*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
	*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
	*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	
}


//====================================================================================================
// END OF CODE
//====================================================================================================
