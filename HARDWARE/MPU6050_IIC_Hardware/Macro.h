#ifndef __Macro_H
#define __Macro_H


#ifdef __cplusplus
extern "C" {
#endif

/* Connectivity Test ---------------------------------------------------------*/
//用于测试IMU是否成功初始化
#define MPU1_TEST_ENABLE 1
#define MPU2_TEST_ENABLE 0
#define MPU3_TEST_ENABLE 0
#define MPU4_TEST_ENABLE 0

//IIC 延时函数长度 单位微秒 default2
#define DELAY_T 8
//Ack应答等待循环数 default250 errtime改为u16
#define ACK_T 700

/**********IIC_1的定义**********/
/*
	以PB10为SCL，PB11为SDA
*/
//IO方向设置
#define MPU_1_SDA_IN()  {GPIOB->CRH&=(unsigned int)0XFFFF0FFF;GPIOB->CRH|=(unsigned int)8<<12;}
#define MPU_1_SDA_OUT() {GPIOB->CRH&=(unsigned int)0XFFFF0FFF;GPIOB->CRH|=(unsigned int)3<<12;}

//IO操作函数	 
#define MPU_1_IIC_SCL    PBout(10) 		//SCL
#define MPU_1_IIC_SDA    PBout(11) 		//SDA	 
#define MPU_1_READ_SDA   PBin(11) 		//输入SDA 


/**********IIC_2的定义**********/
/*
	以PB12为SCL，PB13为SDA
*/

//IO方向设置
#define MPU_2_SDA_IN()  {GPIOC->CRL&=(unsigned int)0XFFFFF0FF;GPIOC->CRL|=(unsigned int)8<<8;}
#define MPU_2_SDA_OUT() {GPIOC->CRL&=(unsigned int)0XFFFFF0FF;GPIOC->CRL|=(unsigned int)3<<8;}

//IO操作函数	 
#define MPU_2_IIC_SCL    PCout(1) 		//SCL
#define MPU_2_IIC_SDA    PCout(2) 		//SDA	 
#define MPU_2_READ_SDA   PCin(2) 		//输入SDA 


/**********IIC_3的定义**********/
/*
	以PB14为SCL，PB15为SDA
*/

//IO方向设置
#define MPU_3_SDA_IN()  {GPIOB->CRH&=(unsigned int)0X0FFFFFFF;GPIOB->CRH|=(unsigned int)8<<28;}
#define MPU_3_SDA_OUT() {GPIOB->CRH&=(unsigned int)0X0FFFFFFF;GPIOB->CRH|=(unsigned int)3<<28;}

//IO操作函数	 
#define MPU_3_IIC_SCL    PBout(14) 		//SCL
#define MPU_3_IIC_SDA    PBout(15) 		//SDA	 
#define MPU_3_READ_SDA   PBin(15) 		//输入SDA 


/**********IIC_4的定义**********/
/*
	以PE1为SCL，PE2为SDA
*/

//IO方向设置
#define MPU_4_SDA_IN()  {GPIOC->CRH&=(unsigned int)0XFFF0FFFF;GPIOC->CRH|=(unsigned int)8<<16;}
#define MPU_4_SDA_OUT() {GPIOC->CRH&=(unsigned int)0XFFF0FFFF;GPIOC->CRH|=(unsigned int)3<<16;}

//IO操作函数	 
#define MPU_4_IIC_SCL    PCout(11) 		//SCL
#define MPU_4_IIC_SDA    PCout(12) 		//SDA	 
#define MPU_4_READ_SDA   PCin(12) 		//输入SDA 

#ifdef __cplusplus
}
#endif

#endif
