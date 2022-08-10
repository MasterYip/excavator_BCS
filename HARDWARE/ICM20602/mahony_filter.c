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

#include "mahony_filter.h"

/* Definitions ---------------------------------------------------------*/
float Kp 	=	10.0f;//10.0f					// proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki	=	0.01f;//0.01f				// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025		// half the sample period
#define q30  1073741824.0f  //q30格式,long转float时的除数.
#define alpha 0.9999 //待调整

void Attitude_Computation(Filter6axisTypeDef filter,struct IMU_data *imu) 
{
	float gx = filter.GyroxLPF.output;
	float gy = filter.GyroyLPF.output;
	float gz = filter.GyrozLPF.output;
	float ax = filter.AccxLPF.output;
	float ay = filter.AccyLPF.output;
	float az = filter.AcczLPF.output;
		
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;         
	
	float q0=1;
	float q1=0;
	float q2=0;
	float q3=0;
	
	//检验Quaternion数组是否初始化
	if(imu->Quaternion[0]*imu->Quaternion[0]+
		imu->Quaternion[1]*imu->Quaternion[1]+
		imu->Quaternion[2]*imu->Quaternion[2]+
		imu->Quaternion[3]*imu->Quaternion[3]> 0.5)
	{
		q0=imu->Quaternion[0];
		q1=imu->Quaternion[1];
		q2=imu->Quaternion[2];
		q3=imu->Quaternion[3];
	}
	

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
	imu->ErrorIntegrate[0] = imu->ErrorIntegrate[0] + ex*Ki;
	imu->ErrorIntegrate[1] = imu->ErrorIntegrate[1] + ey*Ki;
	imu->ErrorIntegrate[2] = imu->ErrorIntegrate[2] + ez*Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + imu->ErrorIntegrate[0];
	gy = gy + Kp*ey + imu->ErrorIntegrate[1];
	gz = gz + Kp*ez + imu->ErrorIntegrate[2];
	
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
	
	imu->Quaternion[0] = q0;
	imu->Quaternion[1] = q1;
	imu->Quaternion[2] = q2;
	imu->Quaternion[3] = q3;

	//计算得到俯仰角/横滚角/航向角
	imu->EulerAngles[0]  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
	imu->EulerAngles[1] = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
	imu->EulerAngles[2]   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
		
}

extern int In_Motion;

void IMUupdate(struct ICM_CS_POINTER Pointer, struct IMU_data *imu, Filter6axisTypeDef *filter)
{
	float calibrated_gyro[3];
//	get_icm20602_data(Pointer, imu);//放在IMUs_update中了
	
	calibrated_gyro[0] = imu->AngularVelocity[0] - imu->GyroAverageCalibration[0];
	calibrated_gyro[1] = imu->AngularVelocity[1] - imu->GyroAverageCalibration[1];
	calibrated_gyro[2] = imu->AngularVelocity[2] - imu->GyroAverageCalibration[2];
	LPFUpdate6axis(calibrated_gyro, imu->Acceleration, filter);
	Attitude_Computation(*filter, imu);
}

void IMUs_update(struct ICM_CS_POINTER Pointer[], struct IMU_data imu[], Filter6axisTypeDef filter[])
{
	for(int i=0; i<4; i++)
	{
		if(i==3)
		{
			get_icm20602_imu4_remap_data_simspi(Pointer[i], &imu[i]);
		}
		else
		{
			get_icm20602_data(Pointer[i], &imu[i]);
		}
		
		//初代IMU防漂移措施 亟待改进 很多时候z轴并不朝上
		if(!In_Motion) imu[i].GyroAverageCalibration[2] = alpha*imu[i].GyroAverageCalibration[2] + (1-alpha)*imu[i].AngularVelocity[2];
		
		IMUupdate(Pointer[i], &imu[i], &filter[i]);
	}
}
//====================================================================================================
// END OF CODE
//====================================================================================================
