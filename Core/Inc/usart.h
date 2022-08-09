/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
typedef unsigned char u8;

#define USART1_BAUD				115200  //921600
#define USART1_BUF_LEN		200  	//定义最大缓冲字节数 

#define USART1_RX_LEN			18
#define DMA_RX_EN(DMA_CH,LEN);          DMA_Cmd(DMA_CH, DISABLE );\
																					DMA_CH->CNDTR =LEN;\
																					DMA_Cmd(DMA_CH, ENABLE);
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern u8  USART1_RX_BUF[USART1_BUF_LEN];//接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u8	 USART1_TX_BUF[USART1_BUF_LEN]; 

void Usart1_Init(u8 prio);
void USART1_DMA_TX_Enable(unsigned int buf_bytes,char* str_USART) ;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

