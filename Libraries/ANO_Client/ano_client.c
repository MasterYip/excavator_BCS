//=====================================================================================================
// ano_client.c
// 	Master Yip
//	20220708
//=====================================================================================================
// Description:
//	Driver for ANO Client (Using USART)
// Useage:
//	Set_USART_Send_String(roll, pitch, yaw, str_USART);//Prepare the string
//
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)str_USART,TX_BUF_LEN);//Transmit the string
//	<Or>
//	HAL_UART_Transmit_IT(&huart1, (uint8_t*)str_USART, TX_BUF_LEN);
//=====================================================================================================
#include "ano_client.h"
#include "RobotArm.h"

char str_USART[TX_BUF_LEN];

/** 
 * @brief Prepare the string to USART for ANO client
 * @param
 * @param
 *
 * @return
 *     -<em>false</em> fail
 *     -<em>true</em> succeed
 */
void Set_USART_Send_String(float roll, float pitch, float yaw, char str_USART[])
{
	unsigned char i=0;
	unsigned char sum = 0;
	unsigned int _temp;
	
	unsigned char _cnt = 0;

	str_USART[_cnt++]=0xAA;
	str_USART[_cnt++]=0xAA; 
	str_USART[_cnt++]=0x01; 
	str_USART[_cnt++]=0; 
	
	_temp = 0-(int)(roll*100);
	str_USART[_cnt++]=_temp>>8;			
	str_USART[_cnt++]=_temp;
	
	_temp = (int)(pitch*100);
	str_USART[_cnt++]=_temp>>8;
	str_USART[_cnt++]=_temp;
	
	_temp = 0-(int)(yaw*100);
	str_USART[_cnt++]=_temp>>8;
	str_USART[_cnt++]=_temp;
	


	_temp = 0;
	str_USART[_cnt++]=_temp>>24;
	str_USART[_cnt++]=_temp>>16;
	str_USART[_cnt++]=_temp>>8;
	str_USART[_cnt++]=_temp;
	
	str_USART[_cnt++]=0xA0;
	
	str_USART[3] = 11;
	
	//和校验
	for(i=0;i<_cnt;i++)
	{
		sum+= str_USART[i];
	}
	str_USART[_cnt++]=sum;
}

/** 
 * @brief Send Attitude info for ANOTC Client to DISPALY.
 * @note  It's parameters are defined in other files!
 * @param UART_HandleTypeDef *huart  HAL UART Handle.
 * @param MPU6050 imu  IMU data struct defined in @mpu6050_1.c
 * @return void
 */
void ANOTC_Attitude_Display(UART_HandleTypeDef *huart, struct IMU_data imu)
{
	Set_USART_Send_String(imu.EulerAngles[0], imu.EulerAngles[1], imu.EulerAngles[2], str_USART);
//	HAL_UART_Transmit_DMA(huart,(uint8_t*)str_USART,TX_BUF_LEN);
	HAL_UART_Transmit_IT(huart, (uint8_t*)str_USART, TX_BUF_LEN);
}

/** 
 * @brief Send Attitude info for ANOTC Client to DISPALY in DMA Mode.
 * @note  It's parameters are defined in other files!
 * @param UART_HandleTypeDef *huart  HAL UART Handle.
 * @param MPU6050 imu  IMU data struct defined in @mpu6050_1.c
 * @return void
 */
void ANOTC_Attitude_Display_DMA(UART_HandleTypeDef *huart, struct IMU_data imu)
{
	Set_USART_Send_String(imu.EulerAngles[0], imu.EulerAngles[1], imu.EulerAngles[2], str_USART);
	HAL_UART_Transmit_DMA(huart,(uint8_t*)str_USART,TX_BUF_LEN);
}


/** 
 * @brief Send Attitude info for ANOTC Client to DISPALY in DMA Mode.
 * @note  It's parameters are defined in other files!
 * @param UART_HandleTypeDef *huart  HAL UART Handle.
 * @param MPU6050 imu  IMU data struct defined in @mpu6050_1.c
 * @return void
 */
void ANOTC_RevisedAttitude_Display_DMA(UART_HandleTypeDef *huart, struct IMU_data *imu)
{
	float qua[4]={0};
	xiangdui(imu->Quaternion, imu->QuaRevision, qua);
	
	//计算得到俯仰角/横滚角/航向角
	imu->RevisedEuler[0] = atan2(2 * qua[2] * qua[3] + 2 * qua[0] * qua[1], -2 * qua[1] * qua[1] - 2 * qua[2]* qua[2] + 1)* 57.3;	// roll
	imu->RevisedEuler[1] = asin(-2 * qua[1] * qua[3] + 2 * qua[0]* qua[2])* 57.3;	// pitch
	imu->RevisedEuler[2] = atan2(2*(qua[1]*qua[2] + qua[0]*qua[3]),qua[0]*qua[0]+qua[1]*qua[1]-qua[2]*qua[2]-qua[3]*qua[3]) * 57.3;	//yaw
	
	Set_USART_Send_String(imu->RevisedEuler[0], imu->RevisedEuler[1], imu->RevisedEuler[2], str_USART);
	HAL_UART_Transmit_DMA(huart,(uint8_t*)str_USART,TX_BUF_LEN);
}


