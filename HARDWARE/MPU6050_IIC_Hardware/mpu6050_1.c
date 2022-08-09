#include "mpu6050.h"
#include "sys.h"
#include "gpio.h"
#include "delay.h"
  
/************************MPU6050结构体数组定义*************************/
MPU6050 mpu6050[4]={0};

float alpha_1=0.9999;	//加权平均因子

//硬件IIC测试用
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
	MPU_1_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_1_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_1_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_1_Set_Rate(500);						//设置采样率50Hz
	MPU_1_Write_Byte(MPU_INT_EN_REG,0X00);	
	MPU_1_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_1_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_1_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_1_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_1_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_1_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_1_Set_Rate(500);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}

//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_1_Write_Byte(u8 reg,u8 data) 				 
{ 
  MPU_1_IIC_Start(); 
	MPU_1_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(MPU_1_IIC_Wait_Ack())	//等待应答
	{
		MPU_1_IIC_Stop();		 
		return 1;		
	}

	
  MPU_1_IIC_Send_Byte(reg);	//写寄存器地址
  MPU_1_IIC_Wait_Ack();		//等待应答 
	MPU_1_IIC_Send_Byte(data);//发送数据
	if(MPU_1_IIC_Wait_Ack())	//等待ACK
	{
		MPU_1_IIC_Stop();	 
		return 1;		 
	}		 
  MPU_1_IIC_Stop();	 
	return 0;
}

//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_1_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_1_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}

//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_1_Set_Accel_Fsr(u8 fsr)
{
	return MPU_1_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_1_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_1_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}

//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_1_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_1_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_1_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
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

//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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

//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_1_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_1_IIC_Start(); 
	MPU_1_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_1_IIC_Wait_Ack())	//等待应答
	{
		MPU_1_IIC_Stop();		 
		return 1;		
	}
    MPU_1_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_1_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_1_IIC_Send_Byte(buf[i]);	//发送数据
		if(MPU_1_IIC_Wait_Ack())		//等待ACK
		{
			MPU_1_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_1_IIC_Stop();	 
	return 0;	
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_1_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_1_IIC_Start(); 
	MPU_1_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_1_IIC_Wait_Ack())	//等待应答
	{
		MPU_1_IIC_Stop();		 
		return 1;		
	}
  MPU_1_IIC_Send_Byte(reg);	//写寄存器地址
  MPU_1_IIC_Wait_Ack();		//等待应答
  MPU_1_IIC_Start();
	MPU_1_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
  MPU_1_IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=MPU_1_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=MPU_1_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
  MPU_1_IIC_Stop();	//产生一个停止条件 
	return 0;	
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_1_Read_Byte(u8 reg)
{
	u8 res;
  MPU_1_IIC_Start(); 
	MPU_1_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	MPU_1_IIC_Wait_Ack();		//等待应答 
  MPU_1_IIC_Send_Byte(reg);	//写寄存器地址
  MPU_1_IIC_Wait_Ack();		//等待应答
  MPU_1_IIC_Start();
	MPU_1_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
  MPU_1_IIC_Wait_Ack();		//等待应答 
	res=MPU_1_IIC_Read_Byte(0);//读取数据,发送nACK 
  MPU_1_IIC_Stop();			//产生一个停止条件 
	return res;		
}

//MPU IIC 延时函数
void MPU_1_IIC_Delay(void)
{
	delay_us(DELAY_T);
}

//产生IIC起始信号
void MPU_1_IIC_Start(void)
{
	MPU_1_SDA_OUT();     //sda线输出
	MPU_1_IIC_SDA=1;	  	  
	MPU_1_IIC_SCL=1;
	MPU_1_IIC_Delay();
 	MPU_1_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	MPU_1_IIC_Delay();
	MPU_1_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  

//产生IIC停止信号
void MPU_1_IIC_Stop(void)
{
	MPU_1_SDA_OUT();//sda线输出
	MPU_1_IIC_SCL=0;
	MPU_1_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	MPU_1_IIC_Delay();
	MPU_1_IIC_SCL=1; 
	MPU_1_IIC_SDA=1;//发送I2C总线结束信号
	MPU_1_IIC_Delay();							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MPU_1_IIC_Wait_Ack(void)
{
	u16 ucErrTime=0;
	MPU_1_SDA_IN();      //SDA设置为输入  
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
	MPU_1_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
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

//不产生ACK应答		    
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

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_1_IIC_Send_Byte(u8 txd)
{                        
  u8 t;   
	MPU_1_SDA_OUT(); 	    
  MPU_1_IIC_SCL=0;//拉低时钟开始数据传输
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

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MPU_1_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_1_SDA_IN();//SDA设置为输入
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
		MPU_1_IIC_NAck();//发送nACK
  else
    MPU_1_IIC_Ack(); //发送ACK   
  return receive;
}


// 获取加电时角速度的静态误差
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
