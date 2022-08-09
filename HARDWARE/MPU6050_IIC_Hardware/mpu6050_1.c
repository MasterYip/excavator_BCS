#include "mpu6050.h"
#include "sys.h"
#include "gpio.h"
#include "delay.h"
  
/************************MPU6050�ṹ�����鶨��*************************/
MPU6050 mpu6050[4]={0};

float alpha_1=0.9999;	//��Ȩƽ������

//Ӳ��IIC������
void MPU6050_Update(s16 *data)
{

	static short ax=0;
	static short ay=0;
	static short az=0;
	static short gx=0;
	static short gy=0;
	static short gz=0;
	static float gyroGx=0;
	static float gyroGy=0;
	static float gyroGz=0;
	static float gyroz_Average=0;
	//////////////////////////////////////////////////
	// Acquire MPU6050 gyro & acc raw data
		
//		MPU_1_Get_Accelerometer(&ax,&ay,&az);
//		MPU_1_Get_Gyroscope(&gx,&gy,&gz);
	
//		gyroz_Average=gyroz_Average*alpha_1+gz*(1-alpha_1);
	/////////////////////////////////////////////////////////////////////////////
	// Add gyro data static error & range process code below here
	// to get the real gyroscope data
	// gyroGx = .....   gyroGy = .....  gyroGz = .....
	// Note:  static error variables are f_gx_error, f_gy_error, f_gz_error
	ax = data[0]/16384;
	ay = data[1]/16384;
	az = data[2]/16384;
	
	
	gyroGx = (data[3] - mpu6050[0].f_gx_error)/131.f;
	gyroGy = (data[4] - mpu6050[0].f_gy_error)/131.f;
	gyroGz = (data[5] - mpu6050[0].f_gz_error)/131.f;
//	gyroGz = (gz - gyroz_Average)/16.384f;
		
	
		
	//////////////////////////////////////////////////
	// Lowpass filtering
		LPFUpdate6axis_1(gyroGx,gyroGy,gyroGz,ax,ay,az);

	//////////////////////////////////////////////////
	// Mahony algorithm
	// Euler angles are stored by pitch, roll, yaw
		IMUupdate_1(Filters_1.GyroxLPF.output/57.3, 
							Filters_1.GyroyLPF.output/57.3, 
							Filters_1.GyrozLPF.output/57.3,
							Filters_1.AccxLPF.output, 
							Filters_1.AccyLPF.output, 
							Filters_1.AcczLPF.output,
							&mpu6050[0].pitch,&mpu6050[0].roll,&mpu6050[0].yaw);
}




u8 MPU_1_Init(void)
{ 
	u8 res;
	
	if(MPU_1_Write_Byte(MPU_PWR_MGMT1_REG,0X80) && MPU1_TEST_ENABLE)PAout(8)=0;
	HAL_Delay(100);
	MPU_1_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_1_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_1_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_1_Set_Rate(500);						//���ò�����50Hz
	MPU_1_Write_Byte(MPU_INT_EN_REG,0X00);	
	MPU_1_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_1_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_1_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_1_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_1_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_1_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_1_Set_Rate(500);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}

//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_1_Write_Byte(u8 reg,u8 data) 				 
{ 
  MPU_1_IIC_Start(); 
	MPU_1_IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	if(MPU_1_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MPU_1_IIC_Stop();		 
		return 1;		
	}

	
  MPU_1_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
  MPU_1_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	MPU_1_IIC_Send_Byte(data);//��������
	if(MPU_1_IIC_Wait_Ack())	//�ȴ�ACK
	{
		MPU_1_IIC_Stop();	 
		return 1;		 
	}		 
  MPU_1_IIC_Stop();	 
	return 0;
}

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_1_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_1_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}

//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_1_Set_Accel_Fsr(u8 fsr)
{
	return MPU_1_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_1_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_1_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_1_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_1_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_1_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_1_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_1_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}

//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_1_Get_Gyroscope(short *gx,short *gy,short *gz)
{
  u8 buf[6],res;  
	res=MPU_1_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
  return res;
}

//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_1_Get_Accelerometer(short *ax,short *ay,short *az)
{
  u8 buf[6],res;  
	res=MPU_1_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
  return res;;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_1_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_1_IIC_Start(); 
	MPU_1_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(MPU_1_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MPU_1_IIC_Stop();		 
		return 1;		
	}
    MPU_1_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_1_IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		MPU_1_IIC_Send_Byte(buf[i]);	//��������
		if(MPU_1_IIC_Wait_Ack())		//�ȴ�ACK
		{
			MPU_1_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_1_IIC_Stop();	 
	return 0;	
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_1_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_1_IIC_Start(); 
	MPU_1_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(MPU_1_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MPU_1_IIC_Stop();		 
		return 1;		
	}
  MPU_1_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
  MPU_1_IIC_Wait_Ack();		//�ȴ�Ӧ��
  MPU_1_IIC_Start();
	MPU_1_IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
  MPU_1_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=MPU_1_IIC_Read_Byte(0);//������,����nACK 
		else *buf=MPU_1_IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
  MPU_1_IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_1_Read_Byte(u8 reg)
{
	u8 res;
  MPU_1_IIC_Start(); 
	MPU_1_IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	MPU_1_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
  MPU_1_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
  MPU_1_IIC_Wait_Ack();		//�ȴ�Ӧ��
  MPU_1_IIC_Start();
	MPU_1_IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������	
  MPU_1_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	res=MPU_1_IIC_Read_Byte(0);//��ȡ����,����nACK 
  MPU_1_IIC_Stop();			//����һ��ֹͣ���� 
	return res;		
}

//MPU IIC ��ʱ����
void MPU_1_IIC_Delay(void)
{
	delay_us(DELAY_T);
}

//����IIC��ʼ�ź�
void MPU_1_IIC_Start(void)
{
	MPU_1_SDA_OUT();     //sda�����
	MPU_1_IIC_SDA=1;	  	  
	MPU_1_IIC_SCL=1;
	MPU_1_IIC_Delay();
 	MPU_1_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	MPU_1_IIC_Delay();
	MPU_1_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  

//����IICֹͣ�ź�
void MPU_1_IIC_Stop(void)
{
	MPU_1_SDA_OUT();//sda�����
	MPU_1_IIC_SCL=0;
	MPU_1_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	MPU_1_IIC_Delay();
	MPU_1_IIC_SCL=1; 
	MPU_1_IIC_SDA=1;//����I2C���߽����ź�
	MPU_1_IIC_Delay();							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 MPU_1_IIC_Wait_Ack(void)
{
	u16 ucErrTime=0;
	MPU_1_SDA_IN();      //SDA����Ϊ����  
	MPU_1_IIC_SDA=1;MPU_1_IIC_Delay();	   
	MPU_1_IIC_SCL=1;MPU_1_IIC_Delay();	 
	while(MPU_1_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>ACK_T)
		{
			MPU_1_IIC_Stop();
			return 1;
		}
	}
	MPU_1_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
void MPU_1_IIC_Ack(void)
{
	MPU_1_IIC_SCL=0;
	MPU_1_SDA_OUT();
	MPU_1_IIC_SDA=0;
	MPU_1_IIC_Delay();
	MPU_1_IIC_SCL=1;
	MPU_1_IIC_Delay();
	MPU_1_IIC_SCL=0;
}

//������ACKӦ��		    
void MPU_1_IIC_NAck(void)
{
	MPU_1_IIC_SCL=0;
	MPU_1_SDA_OUT();
	MPU_1_IIC_SDA=1;
	MPU_1_IIC_Delay();
	MPU_1_IIC_SCL=1;
	MPU_1_IIC_Delay();
	MPU_1_IIC_SCL=0;
}					

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_1_IIC_Send_Byte(u8 txd)
{                        
  u8 t;   
	MPU_1_SDA_OUT(); 	    
  MPU_1_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
  for(t=0;t<8;t++)
  {              
    MPU_1_IIC_SDA=(txd&0x80)>>7;
    txd<<=1; 	  
		MPU_1_IIC_SCL=1;
		MPU_1_IIC_Delay(); 
		MPU_1_IIC_SCL=0;	
		MPU_1_IIC_Delay();
  }	 
} 	   

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 MPU_1_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_1_SDA_IN();//SDA����Ϊ����
  for(i=0;i<8;i++ )
	{
    MPU_1_IIC_SCL=0; 
    MPU_1_IIC_Delay();
		MPU_1_IIC_SCL=1;
    receive<<=1;
    if(MPU_1_READ_SDA)receive++;   
		MPU_1_IIC_Delay(); 
  }					 
  if (!ack)
		MPU_1_IIC_NAck();//����nACK
  else
    MPU_1_IIC_Ack(); //����ACK   
  return receive;
}


// ��ȡ�ӵ�ʱ���ٶȵľ�̬���
void Get_Gyro_Static_Error_1(void)
{
	int i=0;
	int gyrox_add=0;
	int gyroy_add=0;
	int gyroz_add=0;
	short gx, gy, gz;
	
	for(i=0;i<100;i++)
	{
		MPU_1_Get_Gyroscope(&gx,&gy,&gz);
		
		HAL_Delay(5);
		gyrox_add+=gx;
		gyroy_add+=gy;
		gyroz_add+=gz;
	}
	mpu6050[0].f_gx_error = (float)(gyrox_add/100.0);
	mpu6050[0].f_gy_error = (float)(gyroy_add/100.0);
	mpu6050[0].f_gz_error = (float)(gyroz_add/100.0);
	mpu6050[0].gyroz_Average = (float)(gyroz_add/100.0);
}

void MPU_1_Update(void)
{
	static short ax=0;
	static short ay=0;
	static short az=0;
	static short gx=0;
	static short gy=0;
	static short gz=0;
	static float gyroGx=0;
	static float gyroGy=0;
	static float gyroGz=0;
	static float gyroz_Average=0;
	//////////////////////////////////////////////////
	// Acquire MPU6050 gyro & acc raw data
		MPU_1_Get_Accelerometer(&ax,&ay,&az);
		MPU_1_Get_Gyroscope(&gx,&gy,&gz);
	
		gyroz_Average=gyroz_Average*alpha_1+gz*(1-alpha_1);
	/////////////////////////////////////////////////////////////////////////////
	// Add gyro data static error & range process code below here
	// to get the real gyroscope data
	// gyroGx = .....   gyroGy = .....  gyroGz = .....
	// Note:  static error variables are f_gx_error, f_gy_error, f_gz_error
		
	gyroGx = (gx - mpu6050[0].f_gx_error)/16.384f;
	gyroGy = (gy - mpu6050[0].f_gy_error)/16.384f;
	//gyroGz = (gz - f_gz_error)/16.384f;
	gyroGz = (gz - gyroz_Average)/16.384f;
		
	
		
	//////////////////////////////////////////////////
	// Lowpass filtering
		LPFUpdate6axis_1(gyroGx,gyroGy,gyroGz,ax,ay,az);

	//////////////////////////////////////////////////
	// Mahony algorithm
	// Euler angles are stored by pitch, roll, yaw
		IMUupdate_1(Filters_1.GyroxLPF.output/57.3, 
							Filters_1.GyroyLPF.output/57.3, 
							Filters_1.GyrozLPF.output/57.3,
							Filters_1.AccxLPF.output, 
							Filters_1.AccyLPF.output, 
							Filters_1.AcczLPF.output,
							&mpu6050[0].pitch,&mpu6050[0].roll,&mpu6050[0].yaw);
}
