/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		ICM20602
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		IAR 8.3 or MDK 5.26
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 * @note		
					���߶��壺
					------------------------------------ 
						SCL                 �鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SCL�궨��
						SDA                 �鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SDA�궨��
					------------------------------------ 
 ********************************************************************************************************************/


#include "SEEKFREE_ICM20602.h"



//���豸Ƭѡ�ź�����ָ��
//Change when needed.
struct ICM_CS_POINTER ICM20602_CS_Pointer[6]={
	{GPIOB, GPIO_PIN_4},
	{GPIOD, GPIO_PIN_2},
	{GPIOC, GPIO_PIN_10},
	{GPIOA, GPIO_PIN_15}};

//����������
struct IMU_data ICM20602_Data[6];

int16_t icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16_t icm_acc_x,icm_acc_y,icm_acc_z;

#define ICM20602_CSNs(Port, Pin, x)  	HAL_GPIO_WritePin(Port,Pin,(GPIO_PinState)x)

//////////////////////////////////////////////////////////////////////////////////
/* SoftwareSPI���� ---------------------------------------------------------*/
#if SOFTWARE_SPI_ENABLED
//��Ҫʵʱ����
#define ICM20602_SCK_Pin GPIO_PIN_12
#define ICM20602_SCK_GPIO_Port GPIOC
#define ICM20602_MOSI_Pin GPIO_PIN_3
#define ICM20602_MOSI_GPIO_Port GPIOB
#define ICM20602_MISO_Pin GPIO_PIN_5
#define ICM20602_MISO_GPIO_Port GPIOB

#define ICM20602_SCK(x)		HAL_GPIO_WritePin(ICM20602_SCK_GPIO_Port,ICM20602_SCK_Pin,(GPIO_PinState)x)
#define ICM20602_MOSI(x) 	HAL_GPIO_WritePin(ICM20602_MOSI_GPIO_Port,ICM20602_MOSI_Pin,(GPIO_PinState)x)
#define ICM20602_MISO    	(uint8_t)HAL_GPIO_ReadPin(ICM20602_MISO_GPIO_Port,ICM20602_MISO_Pin)	

/* ������д ---------------------------------------------------------*/
uint8_t icm_simspi_wr_byte(uint8_t byte);

void icm_simspi_w_reg_byte(uint8_t cmd, uint8_t val, GPIO_TypeDef * Port, uint16_t Pin);
void icm_simspi_r_reg_byte(uint8_t cmd, uint8_t *val, GPIO_TypeDef * Port, uint16_t Pin);
void icm_simspi_r_reg_bytes(uint8_t cmd, uint8_t *val, uint8_t num, GPIO_TypeDef * Port, uint16_t Pin);

/* ��ʼ�������ݶ�ȡ ---------------------------------------------------------*/
void icm20602_init_simspi(struct ICM_CS_POINTER Pointer);
void get_icm20602_accdata_simspi(GPIO_TypeDef * Port, uint16_t Pin);
void get_icm20602_gyro_simspi(GPIO_TypeDef * Port, uint16_t Pin);
//ֱ�ӽṹ���ȡ
void get_icm20602_data_simspi(
	struct ICM_CS_POINTER Pointer,
	struct IMU_data *data);

/* ���ݴ��� ---------------------------------------------------------*/
void icm20602_data_change(void);
#endif
//////////////////////////////////////////////////////////////////////////////////
/* HardwareSPI���� ---------------------------------------------------------*/
#if HARDWARE_SPI_ENABLED
extern SPI_HandleTypeDef hspi2;
#define SPI_DEVICE hspi2

/* ������д ---------------------------------------------------------*/
void icm_spi_w_reg_byte(uint8_t cmd, uint8_t val, GPIO_TypeDef * Port, uint16_t Pin);
void icm_spi_r_reg_byte(uint8_t cmd, uint8_t *val, GPIO_TypeDef * Port, uint16_t Pin);
void icm_spi_r_reg_bytes(uint8_t cmd, uint8_t *val, uint8_t num, GPIO_TypeDef * Port, uint16_t Pin);

/* ��ʼ�������ݶ�ȡ ---------------------------------------------------------*/
void icm20602_init_spi(struct ICM_CS_POINTER Pointer);
void get_icm20602_accdata_spi(GPIO_TypeDef * Port, uint16_t Pin);
void get_icm20602_gyro_spi(GPIO_TypeDef * Port, uint16_t Pin);
//ֱ�ӽṹ���ȡ
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
//=================== ����Ҫ���·����������޸ģ��궨����Ҫ�� ===================//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////
//=================== Software SPI ===================//
////////////////////////////////////////////////////////

#if SOFTWARE_SPI_ENABLED
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602�Լ캯��
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
        //��������ԭ�������¼���
        //1 MPU6050���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
    }while(0x12 != val);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���SPI����ʼ��ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_simspi(struct ICM_CS_POINTER Pointer)
{
    uint8_t val;
	
    icm20602_self4_check(Pointer.Port, Pointer.Pin);//���
    
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80, Pointer.Port, Pointer.Pin);//��λ�豸
    HAL_Delay(2);
    do
    {//�ȴ���λ�ɹ�
        icm_simspi_r_reg_byte(ICM20602_PWR_MGMT_1,&val, Pointer.Port, Pointer.Pin);
    }while(0x41 != val);
    
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01, Pointer.Port, Pointer.Pin);            //ʱ������
    icm_simspi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00, Pointer.Port, Pointer.Pin);            //���������Ǻͼ��ٶȼ�
    icm_simspi_w_reg_byte(ICM20602_CONFIG,         0x01, Pointer.Port, Pointer.Pin);            //176HZ 1KHZ
    icm_simspi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x07, Pointer.Port, Pointer.Pin);            //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    icm_simspi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18, Pointer.Port, Pointer.Pin);            //��2000 dps
    icm_simspi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x10, Pointer.Port, Pointer.Pin);            //��8g
    icm_simspi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x03, Pointer.Port, Pointer.Pin);            //Average 8 samples   44.8HZ
	//ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ� ��λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    
}




/**
  * @brief  ��ȡICM20602���ٶȡ����ٶ�����
  * @note   
  * @param  struct ICM_CS_POINTER Pointer ָ��IMU�������ݶ�ȡ
  * @retval struct IMU_data *data ��IMU_dataд����ٶ�����
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
	
	//��λת��
    data->Acceleration[0] = (float)Acceleration[0]/4096;
    data->Acceleration[1] = (float)Acceleration[1]/4096;
    data->Acceleration[2] = (float)Acceleration[2]/4096;
	
	data->AngularVelocity[0] = (float)AngularVelocity[0]/57.3/16.384;
    data->AngularVelocity[1] = (float)AngularVelocity[1]/57.3/16.384;
    data->AngularVelocity[2] = (float)AngularVelocity[2]/57.3/16.384;
	//ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ���λΪ��rad/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ��rad/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ��rad/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ��rad/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
}

/**
  * @brief  ��ȡICM20602 imu4���ٶȡ����ٶ����� ������ӳ��
  * @note   ��������ָ�����ٶȼƵ�xyz��(����)  Ŀǰ���imu4�������������ӳ��������
  * 		x=-Z  y=X z=-Y ��дΪԭIMUĬ�������
  * @param  struct ICM_CS_POINTER Pointer ָ��IMU�������ݶ�ȡ
  * @retval struct IMU_data *data ��IMU_dataд����ٶ�����
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
	
	//��λת��
    data->Acceleration[0] = -(float)Acceleration[2]/4096;
    data->Acceleration[1] = (float)Acceleration[0]/4096;
    data->Acceleration[2] = -(float)Acceleration[1]/4096;
	
	data->AngularVelocity[0] = -(float)AngularVelocity[2]/57.3/16.384;
    data->AngularVelocity[1] = (float)AngularVelocity[0]/57.3/16.384;
    data->AngularVelocity[2] = -(float)AngularVelocity[1]/57.3/16.384;
	//ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ���λΪ��rad/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ��rad/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ��rad/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ��rad/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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
//  @brief      ��ȡICM20602����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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
//  @brief      ͨ��SPIдһ��byte,ͬʱ��ȡһ��byte
//  @param      byte        ���͵�����    
//  @return     uint8_t       return ����status״̬
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
//  @brief      ��valд��cmd��Ӧ�ļĴ�����ַ,ͬʱ����status�ֽ�
//  @param      cmd         ������
//  @param      val         ��д��Ĵ�������ֵ
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
//  @brief      ��ȡcmd����Ӧ�ļĴ�����ַ
//  @param      cmd         ������
//  @param      *val        �洢��ȡ�����ݵ�ַ
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
//  @brief      ��ȡcmd����Ӧ�ļĴ�����ַ
//  @param      cmd         ������
//  @param      *val        �洢��ȡ�����ݵ�ַ
//  @param      num         ��ȡ������
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
//  @brief      ��ICM20602����ת��Ϊ��������λ������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
float unit_icm_gyro_x;
float unit_icm_gyro_y;
float unit_icm_gyro_z;

float unit_icm_acc_x;
float unit_icm_acc_y;
float unit_icm_acc_z;

void icm20602_data_change(void)
{
    //ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ� ��λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
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
//  @brief      ICM20602 SPIд�Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      val     ��Ҫд�������
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
//  @brief      ICM20602 SPI���Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      *val    �������ݵĵ�ַ
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
//  @brief      ICM20602 SPI���ֽڶ��Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      *val    �������ݵĵ�ַ
//  @param      num     ��ȡ����
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
//  @brief      ICM20602�Լ캯��
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self3_check(GPIO_TypeDef * Port, uint16_t Pin)
{
    uint8_t dat=0;
    while(0x12 != dat)   //��ȡICM20602 ID
    {
        icm_spi_r_reg_byte(ICM20602_WHO_AM_I,&dat, Port, Pin);
        HAL_Delay(10);
        //��������ԭ�������¼���
        //1 MPU6050���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
    }

}
     
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_spi(struct ICM_CS_POINTER Pointer)
{
    uint8_t val = 0x0;

    HAL_Delay(10);  //�ϵ���ʱ
    

    icm20602_self3_check(Pointer.Port, Pointer.Pin);//���
    
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80, Pointer.Port, Pointer.Pin);//��λ�豸
    HAL_Delay(2);
    do
    {//�ȴ���λ�ɹ�
        icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val, Pointer.Port, Pointer.Pin);
    }while(0x41 != val);
    
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01, Pointer.Port, Pointer.Pin);            //ʱ������
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00, Pointer.Port, Pointer.Pin);            //���������Ǻͼ��ٶȼ�
    icm_spi_w_reg_byte(ICM20602_CONFIG,         0x01, Pointer.Port, Pointer.Pin);            //176HZ 1KHZ
    icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x07, Pointer.Port, Pointer.Pin);            //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18, Pointer.Port, Pointer.Pin);            //��2000 dps
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x10, Pointer.Port, Pointer.Pin);            //��8g
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x03, Pointer.Port, Pointer.Pin);            //Average 4 samples   44.8HZ   //0x23 Average 16 samples
	//ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ� ��λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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
//  @brief      ��ȡICM20602����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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
  * @brief  ��ȡICM20602���ٶȡ����ٶ�����
  * @note   
  * @param  struct ICM_CS_POINTER Pointer ָ��IMU�������ݶ�ȡ
  * @retval struct IMU_data *data ��IMU_dataд����ٶ�����
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
	
	//��λת��
    data->Acceleration[0] = (float)Acceleration[0]/4096;
    data->Acceleration[1] = (float)Acceleration[1]/4096;
    data->Acceleration[2] = (float)Acceleration[2]/4096;
	
	data->AngularVelocity[0] = (float)AngularVelocity[0]/57.3/16.384;
    data->AngularVelocity[1] = (float)AngularVelocity[1]/57.3/16.384;
    data->AngularVelocity[2] = (float)AngularVelocity[2]/57.3/16.384;
	//ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ���λΪ��rad/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ��rad/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ��rad/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ��rad/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
}

#endif
