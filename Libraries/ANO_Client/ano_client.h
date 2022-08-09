//=====================================================================================================
// ano_client.h
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
#ifndef ANO_CLIENT_H
#define ANO_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif
	
#define START_NUM	1
#define TX_BUF_LEN	16

//#include "main.h"
#include "SEEKFREE_ICM20602.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
extern char str_USART[TX_BUF_LEN];

void Set_USART_Send_String(float roll, float pitch, float yaw, char str_USART[]);
void ANOTC_Attitude_Display(UART_HandleTypeDef *huart,struct IMU_data imu);
void ANOTC_Attitude_Display_DMA(UART_HandleTypeDef *huart,struct IMU_data imu);
	
void ANOTC_RevisedAttitude_Display_DMA(UART_HandleTypeDef *huart, struct IMU_data *imu);
#ifdef __cplusplus
}
#endif

#endif
