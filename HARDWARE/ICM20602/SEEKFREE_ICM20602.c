/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		ICM20602
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		IAR 8.3 or MDK 5.26
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 * @note		
					接线定义：
					------------------------------------ 
						SCL                 查看SEEKFREE_IIC文件内的SEEKFREE_SCL宏定义
						SDA                 查看SEEKFREE_IIC文件内的SEEKFREE_SDA宏定义
					------------------------------------ 
 ********************************************************************************************************************/


#include "SEEKFREE_ICM20602.h"



//多设备片选信号引脚指针
//Change when needed.
struct ICM_CS_POINTER ICM20602_CS_Pointer[6]={
	{GPIOB, GPIO_PIN_4},
	{GPIOD, GPIO_PIN_2},
	{GPIOC, GPIO_PIN_10},
	{GPIOA, GPIO_PIN_15}};

//陀螺仪数据
struct IMU_data ICM20602_Data[6];

int16_t icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16_t icm_acc_x,icm_acc_y,icm_acc_z;

#define ICM20602_CSNs(Port, Pin, x)  	HAL_GPIO_WritePin(Port,Pin,(GPIO_PinState)x)

//////////////////////////////////////////////////////////////////////////////////
/* SoftwareSPI声明 ---------------------------------------------------------*/
#if SOFTWARE_SPI_ENABLED
//需要实时更新
#define ICM20602_SCK_Pin GPIO_PIN_12
#define ICM20602_SCK_GPIO_Port GPIOC
#define ICM20602_MOSI_Pin GPIO_PIN_3
#define ICM20602_MOSI_GPIO_Port GPIOB
#define ICM20602_MISO_Pin GPIO_PIN_5
#define ICM20602_MISO_GPIO_Port GPIOB

#define ICM20602_SCK(x)		HAL_GPIO_WritePin(ICM20602_SCK_GPIO_Port,ICM20602_SCK_Pin,(GPIO_PinState)x)
#define ICM20602_MOSI(x) 	HAL_GPIO_WritePin(ICM20602_MOSI_GPIO_Port,ICM20602_MOSI_Pin,(GPIO_PinState)x)
#define ICM20602_MISO    	(uint8_t)HAL_GPIO_ReadPin(ICM20602_MISO_GPIO_Port,ICM20602_MISO_Pin)	

/* 基本读写 ---------------------------------------------------------*/
uint8_t icm_simspi_wr_byte(uint8_t byte);

void icm_simspi_w_reg_byte(uint8_t cmd, uint8_t val, GPIO_TypeDef * Port, uint16_t Pin);
void icm_simspi_r_reg_byte(uint8_t cmd, uint8_t *val, GPIO_TypeDef * Port, uint16_t Pin);
void icm_simspi_r_reg_bytes(uint8_t cmd, uint8_t *val, uint8_t num, GPIO_TypeDef * Port, uint16_t Pin);

/* 初始化与数据读取 ---------------------------------------------------------*/
void icm20602_init_simspi(struct ICM_CS_POINTER Pointer);
void get_icm20602_accdata_simspi(GPIO_TypeDef * Port, uint16_t Pin);
void get_icm20602_gyro_simspi(GPIO_TypeDef * Port, uint16_t Pin);
//直接结构体读取
void get_icm20602_data_simspi(
	struct ICM_CS_POINTER Pointer,
	struct IMU_data *data);

/* 数据处理 ---------------------------------------------------------*/
void icm20602_data_change(void);
#endif
//////////////////////////////////////////////////////////////////////////////////
/* HardwareSPI声明 ---------------------------------------------------------*/
#if HARDWARE_SPI_ENABLED
extern SPI_HandleTypeDef hspi2;
#define SPI_DEVICE hspi2

/* 基本读写 ---------------------------------------------------------*/
void icm_spi_w_reg_byte(uint8_t cmd, uint8_t val, GPIO_TypeDef * Port, uint16_t Pin);
void icm_spi_r_reg_byte(uint8_t cmd, uint8_t *val, GPIO_TypeDef * Port, uint16_t Pin);
void icm_spi_r_reg_bytes(uint8_t cmd, uint8_t *val, uint8_t num, GPIO_TypeDef * Port, uint16_t Pin);

/* 初始化与数据读取 ---------------------------------------------------------*/
void icm20602_init_spi(struct ICM_CS_POINTER Pointer);
void get_icm20602_accdata_spi(GPIO_TypeDef * Port, uint16_t Pin);
void get_icm20602_gyro_spi(GPIO_TypeDef * Port, uint16_t Pin);
//直接结构体读取
void get_icm20602_data_spi(
	struct ICM_CS_POINTER Pointer,
	struct IMU_data *data);
#endif

////////////////////////////////////////////////////////
//=================== General Func ===================//
////////////////////////////////////////////////////////
	
void icm20602_init(struct ICM_CS_POINTER Pointer)
{
#if USE_SOFT_SPI
	icm20602_init_simspi(Pointer);
#else
	icm20602_init_spi(Pointer);
#endif
}


void get_icm20602_data(struct ICM_CS_POINTER Pointer, struct IMU_data *data)
{
#if USE_SOFT_SPI
	get_icm20602_data_simspi(Pointer, data);
#else
	get_icm20602_data_spi(Pointer, data);
#endif
}

void Get_Gyro_Static_Error(
	struct ICM_CS_POINTER Pointer,
	struct IMU_data *ICM20602_Data)
{
	int i=0;
	float gyrox_add=0;
	float gyroy_add=0;
	float gyroz_add=0;
	struct IMU_data imudata;
	for(i=0;i<100;i++)
	{
		#if USE_SOFT_SPI
			get_icm20602_data_simspi(Pointer, &imudata);
		#else
			get_icm20602_data_spi(Pointer, &imudata);
		#endif
		
		HAL_Delay(5);
		gyrox_add+=imudata.AngularVelocity[0];
		gyroy_add+=imudata.AngularVelocity[1];
		gyroz_add+=imudata.AngularVelocity[2];
	}
	ICM20602_Data->GyroAverageCalibration[0] = gyrox_add/100.0;
	ICM20602_Data->GyroAverageCalibration[1] = gyroy_add/100.0;
	ICM20602_Data->GyroAverageCalibration[2] = gyroz_add/100.0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=================== 不需要对下方函数进行修改（宏定义需要） ===================//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////
//=================== Software SPI ===================//
////////////////////////////////////////////////////////

#if SOFTWARE_SPI_ENABLED
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602自检函数
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self4_check(GPIO_TypeDef * Port, uint16_t Pin)
{
    uint8_t val;
    do
    {
        icm_simspi_r_reg_byte(ICM20602_WHO_AM_I,&val, Port, Pin);
        //卡在这里原因有以下几点
        //1 MPU6050坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V
    }while(0x12 != val);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      软件SPI，初始化ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_simspi(struct ICM_CS_POINTER Pointer)
{
    uint8_t val;
	
    icm20602_self4_check(Pointer.Port, Pointer.Pin);//检测
    
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80, Pointer.Port, Pointer.Pin);//复位设备
    HAL_Delay(2);
    do
    {//等待复位成功
        icm_simspi_r_reg_byte(ICM20602_PWR_MGMT_1,&val, Pointer.Port, Pointer.Pin);
    }while(0x41 != val);
    
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01, Pointer.Port, Pointer.Pin);            //时钟设置
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00, Pointer.Port, Pointer.Pin);            //开启陀螺仪和加速度计
    icm_simspi_w_reg_byte(ICM20602_CONFIG,         0x01, Pointer.Port, Pointer.Pin);            //176HZ 1KHZ
    icm_simspi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x07, Pointer.Port, Pointer.Pin);            //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    icm_simspi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18, Pointer.Port, Pointer.Pin);            //±2000 dps
    icm_simspi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x10, Pointer.Port, Pointer.Pin);            //±8g
    icm_simspi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x03, Pointer.Port, Pointer.Pin);            //Average 8 samples   44.8HZ
	//ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据， 单位为：°/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
    
}




/**
  * @brief  获取ICM20602加速度、角速度数据
  * @note   
  * @param  struct ICM_CS_POINTER Pointer 指定IMU进行数据读取
  * @retval struct IMU_data *data 对IMU_data写入加速度数据
  */

void get_icm20602_data_simspi(
	struct ICM_CS_POINTER Pointer,
	struct IMU_data *data)
{
	uint8_t dat[6];
	int16_t Acceleration[3];
	int16_t AngularVelocity[3];
	//Acceleration
    icm_simspi_r_reg_bytes(ICM20602_ACCEL_XOUT_H, dat, 6, Pointer.Port, Pointer.Pin);
    Acceleration[0] = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    Acceleration[1] = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    Acceleration[2] = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
	
	//Angular Velocity
	icm_simspi_r_reg_bytes(ICM20602_GYRO_XOUT_H, dat, 6, Pointer.Port, Pointer.Pin);
    AngularVelocity[0] = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    AngularVelocity[1] = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    AngularVelocity[2] = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
	
	//单位转换
    data->Acceleration[0] = (float)Acceleration[0]/4096;
    data->Acceleration[1] = (float)Acceleration[1]/4096;
    data->Acceleration[2] = (float)Acceleration[2]/4096;
	
	data->AngularVelocity[0] = (float)AngularVelocity[0]/57.3/16.384;
    data->AngularVelocity[1] = (float)AngularVelocity[1]/57.3/16.384;
    data->AngularVelocity[2] = (float)AngularVelocity[2]/57.3/16.384;
	//ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：rad/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：rad/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：rad/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：rad/s

    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
}

/**
  * @brief  获取ICM20602 imu4加速度、角速度数据 并重新映射
  * @note   可以重新指定加速度计的xyz轴(划掉)  目前针对imu4的特殊情况重新映射坐标轴
  * 		x=-Z  y=X z=-Y 大写为原IMU默认坐标架
  * @param  struct ICM_CS_POINTER Pointer 指定IMU进行数据读取
  * @retval struct IMU_data *data 对IMU_data写入加速度数据
  */
void get_icm20602_imu4_remap_data_simspi(
	struct ICM_CS_POINTER Pointer,
	struct IMU_data *data)
{
	uint8_t dat[6];
	int16_t Acceleration[3];
	int16_t AngularVelocity[3];
	//Acceleration
    icm_simspi_r_reg_bytes(ICM20602_ACCEL_XOUT_H, dat, 6, Pointer.Port, Pointer.Pin);
    Acceleration[0] = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    Acceleration[1] = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    Acceleration[2] = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
	
	//Angular Velocity
	icm_simspi_r_reg_bytes(ICM20602_GYRO_XOUT_H, dat, 6, Pointer.Port, Pointer.Pin);
    AngularVelocity[0] = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    AngularVelocity[1] = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    AngularVelocity[2] = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
	
	//单位转换
    data->Acceleration[0] = -(float)Acceleration[2]/4096;
    data->Acceleration[1] = (float)Acceleration[0]/4096;
    data->Acceleration[2] = -(float)Acceleration[1]/4096;
	
	data->AngularVelocity[0] = -(float)AngularVelocity[2]/57.3/16.384;
    data->AngularVelocity[1] = (float)AngularVelocity[0]/57.3/16.384;
    data->AngularVelocity[2] = -(float)AngularVelocity[1]/57.3/16.384;
	//ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：rad/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：rad/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：rad/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：rad/s

    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_simspi(GPIO_TypeDef * Port, uint16_t Pin)
{
    uint8_t dat[6];
    
    icm_simspi_r_reg_bytes(ICM20602_ACCEL_XOUT_H, dat, 6, Port, Pin);
    icm_acc_x = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    icm_acc_y = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    icm_acc_z = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_simspi(GPIO_TypeDef * Port, uint16_t Pin)
{
    uint8_t dat[6];
    
    icm_simspi_r_reg_bytes(ICM20602_GYRO_XOUT_H, dat, 6, Port, Pin);
    icm_gyro_x = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    icm_gyro_y = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    icm_gyro_z = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过SPI写一个byte,同时读取一个byte
//  @param      byte        发送的数据    
//  @return     uint8_t       return 返回status状态
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8_t icm_simspi_wr_byte(uint8_t byte)
{
    uint8_t i;
    for(i=0; i<8; i++)
    {
        ICM20602_MOSI((GPIO_PinState)(byte&0x80));
        byte <<= 1;
        ICM20602_SCK (0);    
        byte |= ICM20602_MISO;        
        ICM20602_SCK (1);
    }	
    return(byte);                                      		
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      将val写入cmd对应的寄存器地址,同时返回status字节
//  @param      cmd         命令字
//  @param      val         待写入寄存器的数值
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm_simspi_w_reg_byte(uint8_t cmd, uint8_t val, GPIO_TypeDef * Port, uint16_t Pin)
{
	ICM20602_CSNs(Port, Pin, 0);
    cmd |= ICM20602_SPI_W;
    icm_simspi_wr_byte(cmd);                      	
    icm_simspi_wr_byte(val);                               	
	ICM20602_CSNs(Port, Pin, 1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm_simspi_r_reg_byte(uint8_t cmd, uint8_t *val, GPIO_TypeDef * Port, uint16_t Pin)
{
	ICM20602_CSNs(Port, Pin, 0);
    cmd |= ICM20602_SPI_R;
    icm_simspi_wr_byte(cmd);                               	
    *val = icm_simspi_wr_byte(0);                           	
	ICM20602_CSNs(Port, Pin, 1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @param      num         读取的数量
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm_simspi_r_reg_bytes(uint8_t cmd, uint8_t *val, uint8_t num, GPIO_TypeDef * Port, uint16_t Pin)
{
    uint16_t i;
	ICM20602_CSNs(Port, Pin, 0);
    cmd |= ICM20602_SPI_R;
    icm_simspi_wr_byte(cmd);                      	            
    for(i=0; i<num; i++)	
        val[i] = icm_simspi_wr_byte(0);
	ICM20602_CSNs(Port, Pin, 1);
}






//-------------------------------------------------------------------------------------------------------------------
//  @brief      将ICM20602数据转化为带有物理单位的数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
float unit_icm_gyro_x;
float unit_icm_gyro_y;
float unit_icm_gyro_z;

float unit_icm_acc_x;
float unit_icm_acc_y;
float unit_icm_acc_z;

void icm20602_data_change(void)
{
    //ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据， 单位为：°/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
    unit_icm_gyro_x = (float)icm_gyro_x/131;
    unit_icm_gyro_y = (float)icm_gyro_y/131;
    unit_icm_gyro_z = (float)icm_gyro_z/131;

    unit_icm_acc_x = (float)icm_acc_x/4096;
    unit_icm_acc_y = (float)icm_acc_y/4096;
    unit_icm_acc_z = (float)icm_acc_z/4096;

}






#endif



////////////////////////////////////////////////////////
//=================== Hardware SPI ===================//
////////////////////////////////////////////////////////

#if HARDWARE_SPI_ENABLED
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI写寄存器
//  @param      cmd     寄存器地址
//  @param      val     需要写入的数据
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_w_reg_byte(uint8_t cmd, uint8_t val, GPIO_TypeDef * Port, uint16_t Pin)
{
    uint8_t dat[2];
	
	ICM20602_CSNs(Port, Pin, 0);
    dat[0] = cmd | ICM20602_SPI_W;
    dat[1] = val;
    HAL_SPI_Transmit(&SPI_DEVICE,dat,2,500);                            	
	ICM20602_CSNs(Port, Pin, 1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI读寄存器
//  @param      cmd     寄存器地址
//  @param      *val    接收数据的地址
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------

void icm_spi_r_reg_byte(uint8_t cmd, uint8_t *val, GPIO_TypeDef * Port, uint16_t Pin)
{
    uint8_t dat[2];

	ICM20602_CSNs(Port, Pin, 0);
    dat[0] = cmd | ICM20602_SPI_R;
    dat[1] = *val;
	HAL_SPI_TransmitReceive(&SPI_DEVICE,dat,dat,2,500);
	ICM20602_CSNs(Port, Pin, 1);

    *val = dat[1];
}
  
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI多字节读寄存器
//  @param      cmd     寄存器地址
//  @param      *val    接收数据的地址
//  @param      num     读取数量
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_r_reg_bytes(uint8_t * val, uint8_t num, GPIO_TypeDef * Port, uint16_t Pin)
{
	ICM20602_CSNs(Port, Pin, 0);
	HAL_SPI_TransmitReceive(&SPI_DEVICE,val,val,num,500);
	ICM20602_CSNs(Port, Pin, 1);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602自检函数
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self3_check(GPIO_TypeDef * Port, uint16_t Pin)
{
    uint8_t dat=0;
    while(0x12 != dat)   //读取ICM20602 ID
    {
        icm_spi_r_reg_byte(ICM20602_WHO_AM_I,&dat, Port, Pin);
        HAL_Delay(10);
        //卡在这里原因有以下几点
        //1 MPU6050坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V
    }

}
     
//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_spi(struct ICM_CS_POINTER Pointer)
{
    uint8_t val = 0x0;

    HAL_Delay(10);  //上电延时
    

    icm20602_self3_check(Pointer.Port, Pointer.Pin);//检测
    
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80, Pointer.Port, Pointer.Pin);//复位设备
    HAL_Delay(2);
    do
    {//等待复位成功
        icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val, Pointer.Port, Pointer.Pin);
    }while(0x41 != val);
    
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01, Pointer.Port, Pointer.Pin);            //时钟设置
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00, Pointer.Port, Pointer.Pin);            //开启陀螺仪和加速度计
    icm_spi_w_reg_byte(ICM20602_CONFIG,         0x01, Pointer.Port, Pointer.Pin);            //176HZ 1KHZ
    icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x07, Pointer.Port, Pointer.Pin);            //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18, Pointer.Port, Pointer.Pin);            //±2000 dps
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x10, Pointer.Port, Pointer.Pin);            //±8g
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x03, Pointer.Port, Pointer.Pin);            //Average 4 samples   44.8HZ   //0x23 Average 16 samples
	//ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据， 单位为：°/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_spi(GPIO_TypeDef * Port, uint16_t Pin)
{
    struct
    {
        uint8_t reg;
        uint8_t dat[6];
    }buf;

    buf.reg = ICM20602_ACCEL_XOUT_H | ICM20602_SPI_R;
    
    icm_spi_r_reg_bytes(&buf.reg, 7, Port, Pin);
    icm_acc_x = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    icm_acc_y = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    icm_acc_z = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM20602陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_spi(GPIO_TypeDef * Port, uint16_t Pin)
{
    struct
    {
        uint8_t reg;
        uint8_t dat[6];
    }buf;

    buf.reg = ICM20602_GYRO_XOUT_H | ICM20602_SPI_R;
    
    icm_spi_r_reg_bytes(&buf.reg, 7, Port, Pin);
    icm_gyro_x = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    icm_gyro_y = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    icm_gyro_z = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
}


/**
  * @brief  获取ICM20602加速度、角速度数据
  * @note   
  * @param  struct ICM_CS_POINTER Pointer 指定IMU进行数据读取
  * @retval struct IMU_data *data 对IMU_data写入加速度数据
  */

void get_icm20602_data_spi(
	struct ICM_CS_POINTER Pointer,
	struct IMU_data *data)
{
	uint8_t dat[6];
	int16_t Acceleration[3];
	int16_t AngularVelocity[3];
	
	struct
    {
        uint8_t reg;
        uint8_t dat[6];
    }buf;
	
	//Acceleration
	buf.reg = ICM20602_ACCEL_XOUT_H | ICM20602_SPI_R;
	icm_spi_r_reg_bytes(&buf.reg, 7, Pointer.Port, Pointer.Pin);
	Acceleration[0] = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    Acceleration[1] = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    Acceleration[2] = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
	
	//Angular Velocity
	buf.reg = ICM20602_GYRO_XOUT_H | ICM20602_SPI_R;
	icm_spi_r_reg_bytes(&buf.reg, 7, Pointer.Port, Pointer.Pin);
	AngularVelocity[0] = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    AngularVelocity[1] = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    AngularVelocity[2] = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
	
	//单位转换
    data->Acceleration[0] = (float)Acceleration[0]/4096;
    data->Acceleration[1] = (float)Acceleration[1]/4096;
    data->Acceleration[2] = (float)Acceleration[2]/4096;
	
	data->AngularVelocity[0] = (float)AngularVelocity[0]/57.3/16.384;
    data->AngularVelocity[1] = (float)AngularVelocity[1]/57.3/16.384;
    data->AngularVelocity[2] = (float)AngularVelocity[2]/57.3/16.384;
	//ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：rad/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：rad/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：rad/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：rad/s

    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
}

#endif
