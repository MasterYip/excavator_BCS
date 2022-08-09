#include "mpu6050.h"
#include "sys.h"
#include "gpio.h"
#include "delay.h"
 
//³õÊ¼»¯MPU6050_4   //·µ»ØÖµ:0,³É¹¦    ÆäËû,´íÎó´úÂë
float alpha_4=0.9999;	//¼ÓÈ¨Æ½¾ùÒò×

u8 MPU_4_Init(void)
{ 
	u8 res;
	
//	MPU_4_Write_Byte(MPU_PWR_MGMT1_REG,0X80);
	if(MPU_4_Write_Byte(MPU_PWR_MGMT1_REG,0X80) && MPU4_TEST_ENABLE)PAout(8)=0;
	HAL_Delay(100);
	MPU_4_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//»½ÐÑMPU6050 
	MPU_4_Set_Gyro_Fsr(3);					//ÍÓÂÝÒÇ´«¸ÐÆ÷,¡À2000dps
	MPU_4_Set_Accel_Fsr(0);					//¼ÓËÙ¶È´«¸ÐÆ÷,¡À2g
	MPU_4_Set_Rate(500);						//ÉèÖÃ²ÉÑùÂÊ50Hz
	MPU_4_Write_Byte(MPU_INT_EN_REG,0X00);	
	MPU_4_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2CÖ÷Ä£Ê½¹Ø±Õ
	MPU_4_Write_Byte(MPU_FIFO_EN_REG,0X00);	//¹Ø±ÕFIFO
	MPU_4_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INTÒý½ÅµÍµçÆ½ÓÐÐ§
	res=MPU_4_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//Æ÷¼þIDÕýÈ·
	{
		MPU_4_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//ÉèÖÃCLKSEL,PLL XÖáÎª²Î¿¼
		MPU_4_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//¼ÓËÙ¶ÈÓëÍÓÂÝÒÇ¶¼¹¤×÷
		MPU_4_Set_Rate(500);						//ÉèÖÃ²ÉÑùÂÊÎª50Hz
 	}else return 1;
	return 0;
}

//IICÐ´Ò»¸ö×Ö½Ú 
//reg:¼Ä´æÆ÷µØÖ·
//data:Êý¾Ý
//·µ»ØÖµ:0,Õý³£
//    ÆäËû,´íÎó´úÂë
u8 MPU_4_Write_Byte(u8 reg,u8 data) 				 
{ 
  MPU_4_IIC_Start(); 
	MPU_4_IIC_Send_Byte((MPU_ADDR<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî	
	if(MPU_4_IIC_Wait_Ack())	//µÈ´ýÓ¦´ð
	{
		MPU_4_IIC_Stop();		 
		return 1;		
	}

	
  MPU_4_IIC_Send_Byte(reg);	//Ð´¼Ä´æÆ÷µØÖ·
  MPU_4_IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
	MPU_4_IIC_Send_Byte(data);//·¢ËÍÊý¾Ý
	if(MPU_4_IIC_Wait_Ack())	//µÈ´ýACK
	{
		MPU_4_IIC_Stop();	 
		return 1;		 
	}		 
  MPU_4_IIC_Stop();	 
	return 0;
}

//ÉèÖÃMPU6050ÍÓÂÝÒÇ´«¸ÐÆ÷ÂúÁ¿³Ì·¶Î§
//fsr:0,¡À250dps;1,¡À500dps;2,¡À1000dps;3,¡À2000dps
//·µ»ØÖµ:0,ÉèÖÃ³É¹¦
//    ÆäËû,ÉèÖÃÊ§°Ü 
u8 MPU_4_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_4_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//ÉèÖÃÍÓÂÝÒÇÂúÁ¿³Ì·¶Î§  
}

//ÉèÖÃMPU6050¼ÓËÙ¶È´«¸ÐÆ÷ÂúÁ¿³Ì·¶Î§
//fsr:0,¡À2g;1,¡À4g;2,¡À8g;3,¡À16g
//·µ»ØÖµ:0,ÉèÖÃ³É¹¦
//    ÆäËû,ÉèÖÃÊ§°Ü 
u8 MPU_4_Set_Accel_Fsr(u8 fsr)
{
	return MPU_4_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//ÉèÖÃ¼ÓËÙ¶È´«¸ÐÆ÷ÂúÁ¿³Ì·¶Î§  
}

//ÉèÖÃMPU6050µÄÊý×ÖµÍÍ¨ÂË²¨Æ÷
//lpf:Êý×ÖµÍÍ¨ÂË²¨ÆµÂÊ(Hz)
//·µ»ØÖµ:0,ÉèÖÃ³É¹¦
//    ÆäËû,ÉèÖÃÊ§°Ü 
u8 MPU_4_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_4_Write_Byte(MPU_CFG_REG,data);//ÉèÖÃÊý×ÖµÍÍ¨ÂË²¨Æ÷  
}

//ÉèÖÃMPU6050µÄ²ÉÑùÂÊ(¼Ù¶¨Fs=1KHz)
//rate:4~1000(Hz)
//·µ»ØÖµ:0,ÉèÖÃ³É¹¦
//    ÆäËû,ÉèÖÃÊ§°Ü 
u8 MPU_4_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_4_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//ÉèÖÃÊý×ÖµÍÍ¨ÂË²¨Æ÷
 	return MPU_4_Set_LPF(rate/2);	//×Ô¶¯ÉèÖÃLPFÎª²ÉÑùÂÊµÄÒ»°ë
}

//µÃµ½ÎÂ¶ÈÖµ
//·µ»ØÖµ:ÎÂ¶ÈÖµ(À©´óÁË100±¶)
short MPU_4_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_4_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}

//µÃµ½ÍÓÂÝÒÇÖµ(Ô­Ê¼Öµ)
//gx,gy,gz:ÍÓÂÝÒÇx,y,zÖáµÄÔ­Ê¼¶ÁÊý(´ø·ûºÅ)
//·µ»ØÖµ:0,³É¹¦
//    ÆäËû,´íÎó´úÂë
u8 MPU_4_Get_Gyroscope(short *gx,short *gy,short *gz)
{
  u8 buf[6],res;  
	res=MPU_4_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
  return res;
}

//µÃµ½¼ÓËÙ¶ÈÖµ(Ô­Ê¼Öµ)
//gx,gy,gz:ÍÓÂÝÒÇx,y,zÖáµÄÔ­Ê¼¶ÁÊý(´ø·ûºÅ)
//·µ»ØÖµ:0,³É¹¦
//    ÆäËû,´íÎó´úÂë
u8 MPU_4_Get_Accelerometer(short *ax,short *ay,short *az)
{
  u8 buf[6],res;  
	res=MPU_4_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
  return res;;
}

//IICÁ¬ÐøÐ´
//addr:Æ÷¼þµØÖ· 
//reg:¼Ä´æÆ÷µØÖ·
//len:Ð´Èë³¤¶È
//buf:Êý¾ÝÇø
//·µ»ØÖµ:0,Õý³£
//    ÆäËû,´íÎó´úÂë
u8 MPU_4_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_4_IIC_Start(); 
	MPU_4_IIC_Send_Byte((addr<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî	
	if(MPU_4_IIC_Wait_Ack())	//µÈ´ýÓ¦´ð
	{
		MPU_4_IIC_Stop();		 
		return 1;		
	}
    MPU_4_IIC_Send_Byte(reg);	//Ð´¼Ä´æÆ÷µØÖ·
    MPU_4_IIC_Wait_Ack();		//µÈ´ýÓ¦´ð
	for(i=0;i<len;i++)
	{
		MPU_4_IIC_Send_Byte(buf[i]);	//·¢ËÍÊý¾Ý
		if(MPU_4_IIC_Wait_Ack())		//µÈ´ýACK
		{
			MPU_4_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_4_IIC_Stop();	 
	return 0;	
} 

//IICÁ¬Ðø¶Á
//addr:Æ÷¼þµØÖ·
//reg:Òª¶ÁÈ¡µÄ¼Ä´æÆ÷µØÖ·
//len:Òª¶ÁÈ¡µÄ³¤¶È
//buf:¶ÁÈ¡µ½µÄÊý¾Ý´æ´¢Çø
//·µ»ØÖµ:0,Õý³£
//    ÆäËû,´íÎó´úÂë
u8 MPU_4_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_4_IIC_Start(); 
	MPU_4_IIC_Send_Byte((addr<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî	
	if(MPU_4_IIC_Wait_Ack())	//µÈ´ýÓ¦´ð
	{
		MPU_4_IIC_Stop();		 
		return 1;		
	}
  MPU_4_IIC_Send_Byte(reg);	//Ð´¼Ä´æÆ÷µØÖ·
  MPU_4_IIC_Wait_Ack();		//µÈ´ýÓ¦´ð
  MPU_4_IIC_Start();
	MPU_4_IIC_Send_Byte((addr<<1)|1);//·¢ËÍÆ÷¼þµØÖ·+¶ÁÃüÁî	
  MPU_4_IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
	while(len)
	{
		if(len==1)*buf=MPU_4_IIC_Read_Byte(0);//¶ÁÊý¾Ý,·¢ËÍnACK 
		else *buf=MPU_4_IIC_Read_Byte(1);		//¶ÁÊý¾Ý,·¢ËÍACK  
		len--;
		buf++; 
	}    
  MPU_4_IIC_Stop();	//²úÉúÒ»¸öÍ£Ö¹Ìõ¼þ 
	return 0;	
}

//IIC¶ÁÒ»¸ö×Ö½Ú 
//reg:¼Ä´æÆ÷µØÖ· 
//·µ»ØÖµ:¶Áµ½µÄÊý¾Ý
u8 MPU_4_Read_Byte(u8 reg)
{
	u8 res;
  MPU_4_IIC_Start(); 
	MPU_4_IIC_Send_Byte((MPU_ADDR<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî	
	MPU_4_IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
  MPU_4_IIC_Send_Byte(reg);	//Ð´¼Ä´æÆ÷µØÖ·
  MPU_4_IIC_Wait_Ack();		//µÈ´ýÓ¦´ð
  MPU_4_IIC_Start();
	MPU_4_IIC_Send_Byte((MPU_ADDR<<1)|1);//·¢ËÍÆ÷¼þµØÖ·+¶ÁÃüÁî	
  MPU_4_IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
	res=MPU_4_IIC_Read_Byte(0);//¶ÁÈ¡Êý¾Ý,·¢ËÍnACK 
  MPU_4_IIC_Stop();			//²úÉúÒ»¸öÍ£Ö¹Ìõ¼þ 
	return res;		
}

//MPU IIC ÑÓÊ±º¯Êý
void MPU_4_IIC_Delay(void)
{
	delay_us(DELAY_T);
}

//²úÉúIICÆðÊ¼ÐÅºÅ
void MPU_4_IIC_Start(void)
{
	MPU_4_SDA_OUT();     //sdaÏßÊä³ö
	MPU_4_IIC_SDA=1;	  	  
	MPU_4_IIC_SCL=1;
	MPU_4_IIC_Delay();
 	MPU_4_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	MPU_4_IIC_Delay();
	MPU_4_IIC_SCL=0;//Ç¯×¡I2C×ÜÏß£¬×¼±¸·¢ËÍ»ò½ÓÊÕÊý¾Ý 
}	  

//²úÉúIICÍ£Ö¹ÐÅºÅ
void MPU_4_IIC_Stop(void)
{
	MPU_4_SDA_OUT();//sdaÏßÊä³ö
	MPU_4_IIC_SCL=0;
	MPU_4_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	MPU_4_IIC_Delay();
	MPU_4_IIC_SCL=1; 
	MPU_4_IIC_SDA=1;//·¢ËÍI2C×ÜÏß½áÊøÐÅºÅ
	MPU_4_IIC_Delay();							   	
}

//µÈ´ýÓ¦´ðÐÅºÅµ½À´
//·µ»ØÖµ£º1£¬½ÓÊÕÓ¦´ðÊ§°Ü
//        0£¬½ÓÊÕÓ¦´ð³É¹¦
u8 MPU_4_IIC_Wait_Ack(void)
{
	u16 ucErrTime=0;
	MPU_4_SDA_IN();      //SDAÉèÖÃÎªÊäÈë  
	MPU_4_IIC_SDA=1;MPU_4_IIC_Delay();	   
	MPU_4_IIC_SCL=1;MPU_4_IIC_Delay();	 
	while(MPU_4_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>ACK_T)
		{
			MPU_4_IIC_Stop();
			return 1;
		}
	}
	MPU_4_IIC_SCL=0;//Ê±ÖÓÊä³ö0 	   
	return 0;  
} 

//²úÉúACKÓ¦´ð
void MPU_4_IIC_Ack(void)
{
	MPU_4_IIC_SCL=0;
	MPU_4_SDA_OUT();
	MPU_4_IIC_SDA=0;
	MPU_4_IIC_Delay();
	MPU_4_IIC_SCL=1;
	MPU_4_IIC_Delay();
	MPU_4_IIC_SCL=0;
}

//²»²úÉúACKÓ¦´ð		    
void MPU_4_IIC_NAck(void)
{
	MPU_4_IIC_SCL=0;
	MPU_4_SDA_OUT();
	MPU_4_IIC_SDA=1;
	MPU_4_IIC_Delay();
	MPU_4_IIC_SCL=1;
	MPU_4_IIC_Delay();
	MPU_4_IIC_SCL=0;
}					

//IIC·¢ËÍÒ»¸ö×Ö½Ú
//·µ»Ø´Ó»úÓÐÎÞÓ¦´ð
//1£¬ÓÐÓ¦´ð
//0£¬ÎÞÓ¦´ð			  
void MPU_4_IIC_Send_Byte(u8 txd)
{                        
  u8 t;   
	MPU_4_SDA_OUT(); 	    
  MPU_4_IIC_SCL=0;//À­µÍÊ±ÖÓ¿ªÊ¼Êý¾Ý´«Êä
  for(t=0;t<8;t++)
  {              
    MPU_4_IIC_SDA=(txd&0x80)>>7;
    txd<<=1; 	  
		MPU_4_IIC_SCL=1;
		MPU_4_IIC_Delay(); 
		MPU_4_IIC_SCL=0;	
		MPU_4_IIC_Delay();
  }	 
} 	   

//¶Á1¸ö×Ö½Ú£¬ack=1Ê±£¬·¢ËÍACK£¬ack=0£¬·¢ËÍnACK   
u8 MPU_4_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_4_SDA_IN();//SDAÉèÖÃÎªÊäÈë
  for(i=0;i<8;i++ )
	{
    MPU_4_IIC_SCL=0; 
    MPU_4_IIC_Delay();
		MPU_4_IIC_SCL=1;
    receive<<=1;
    if(MPU_4_READ_SDA)receive++;   
		MPU_4_IIC_Delay(); 
  }					 
  if (!ack)
		MPU_4_IIC_NAck();//·¢ËÍnACK
  else
    MPU_4_IIC_Ack(); //·¢ËÍACK   
  return receive;
}


// »ñÈ¡¼ÓµçÊ±½ÇËÙ¶ÈµÄ¾²Ì¬Îó²î
void Get_Gyro_Static_Error_4(void)
{
	int i=0;
	int gyrox_add=0;
	int gyroy_add=0;
	int gyroz_add=0;
	short gx, gy, gz;
	
	for(i=0;i<100;i++)
	{
		MPU_4_Get_Gyroscope(&gx,&gy,&gz);
		
		HAL_Delay(5);
		gyrox_add+=gx;
		gyroy_add+=gy;
		gyroz_add+=gz;
	}
	mpu6050[3].f_gx_error = (float)(gyrox_add/100.0);
	mpu6050[3].f_gy_error = (float)(gyroy_add/100.0);
	mpu6050[3].f_gz_error = (float)(gyroz_add/100.0);
	mpu6050[3].gyroz_Average = (float)(gyroz_add/100.0);
}


void MPU_4_Update(void)
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
		MPU_4_Get_Accelerometer(&ax,&ay,&az);
		MPU_4_Get_Gyroscope(&gx,&gy,&gz);
	

		gyroz_Average=gyroz_Average*alpha_4+gz*(1-alpha_4);
	/////////////////////////////////////////////////////////////////////////////
	// Add gyro data static error & range process code below here
	// to get the real gyroscope data
	// gyroGx = .....   gyroGy = .....  gyroGz = .....
	// Note:  static error variables are f_gx_error, f_gy_error, f_gz_error
		
	gyroGx = (gx - mpu6050[3].f_gx_error)/16.384f;
	gyroGy = (gy - mpu6050[3].f_gy_error)/16.384f;
	//gyroGz = (gz - f_gz_error)/16.384f;
	gyroGz = (gz - gyroz_Average)/16.384f;
		
	
		
	//////////////////////////////////////////////////
	// Lowpass filtering
		LPFUpdate6axis_4(gyroGx,gyroGy,gyroGz,ax,ay,az);

	//////////////////////////////////////////////////
	// Mahony algorithm
	// Euler angles are stored by pitch, roll, yaw
		IMUupdate_4(Filters_4.GyroxLPF.output/57.3, 
							Filters_4.GyroyLPF.output/57.3, 
							Filters_4.GyrozLPF.output/57.3,
							Filters_4.AccxLPF.output, 
							Filters_4.AccyLPF.output, 
							Filters_4.AcczLPF.output,
							&mpu6050[3].pitch,&mpu6050[3].roll,&mpu6050[3].yaw);
}
