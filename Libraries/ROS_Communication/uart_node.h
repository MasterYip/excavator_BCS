#ifndef __UART_NODE_H
#define __UART_NODE_H

#include "stm32f1xx_hal.h"
#include "mpu6050.h"

extern ros::NodeHandle nh;
extern std_msgs::String str_msg;
extern sensor_msgs::Imu imu_data;

#ifdef __cplusplus
 extern "C" {
#endif

void USART_RxCplt_ROSCallback(void);
void USART_TxCplt_ROSCallback(void); 
void setup(void);
void loop(void);

#ifdef __cplusplus
}
#endif

#endif