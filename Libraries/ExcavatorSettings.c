#include "ExcavatorHeaderFile.h"



/* Global Variables ---------------------------------------------------------*/
u8 buff[256] = {0};
u8 sendbuff[512] = {0};
u8 cmd = 0;
float systime = 0;
float Angles[4] = {0};
double urdfAngles[4] = {0};
float AngleforRevision[4] = {0, 0.9948, 2.5133, 4.7124};
/* Init Functions ---------------------------------------------------------*/
void IMU_Data_Init(struct IMU_data imu[4])
{
	int i=0;
	for(i=0; i<4; i++)
	{
		imu[i].Quaternion[0] = 1;
	}
	
	//四元数初始化（使IMU共地！）
	imu[0].Quaternion[0] = 1;
	imu[0].Quaternion[1] = 0;
	imu[0].Quaternion[2] = 0;
	imu[0].Quaternion[3] = 0;

	imu[1].Quaternion[0] = 0.707107;
	imu[1].Quaternion[1] = 0;
	imu[1].Quaternion[2] = 0;
	imu[1].Quaternion[3] = -0.707107;
	
	imu[2].Quaternion[0] = 0.707107;
	imu[2].Quaternion[1] = 0;
	imu[2].Quaternion[2] = 0;
	imu[2].Quaternion[3] = -0.707107;
	
	imu[3].Quaternion[0] = 1;
	imu[3].Quaternion[1] = 0;
	imu[3].Quaternion[2] = 0;
	imu[3].Quaternion[3] = 0;
	
	
	//修正四元数初始化
	imu[0].QuaRevision[0] = 1;
	imu[0].QuaRevision[1] = 0;
	imu[0].QuaRevision[2] = 0;
	imu[0].QuaRevision[3] = 0;

	imu[1].QuaRevision[0] = 1;
	imu[1].QuaRevision[1] = 0;
	imu[1].QuaRevision[2] = 0;
	imu[1].QuaRevision[3] = 0;
	
	imu[2].QuaRevision[0] = 1;
	imu[2].QuaRevision[1] = 0;
	imu[2].QuaRevision[2] = 0;
	imu[2].QuaRevision[3] = 0;
	
	imu[3].QuaRevision[0] = 1;
	imu[3].QuaRevision[1] = 0;
	imu[3].QuaRevision[2] = 0;
	imu[3].QuaRevision[3] = 0;
}

/* Utility ---------------------------------------------------------*/

void HALprintf(char *data)
{
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data, strlen(data));
}
