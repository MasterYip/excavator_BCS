//=====================================================================================================
// File Name:	RobotArm.c
// Author:		Master Yip
// Date:		20220711
//=====================================================================================================
// Description:
//	Including RobotArm resolver and controller
//=====================================================================================================

#include "RobotArm.h"
#include <stdio.h>
#include <math.h> 
#include <string.h>

float k[4]={1,0,0,0};//4个IMU计算对称面向量权重

float jiajiao(float a[3],float b[3])
{
	float k=(a[0]*b[0]+a[1]*b[1]+a[2]*b[2]);
	float norm1=sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
	float norm2=sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);
	float v=k/(norm1*norm2);
	float j;
	if(v>=1)
	{
		j=0;
	}
	else if(v<=-1)
	{
		j=M_PI;
	}
	else
	{
		j=acos(v);
	}
	return j;
 } 

 void Cross(float a[3],float b[3],float c[3])
//叉乘 
{
    c[0] = a[1]*b[2] - a[2]*b[1] ;
    c[1] = a[2]*b[0] - a[0]*b[2] ;
    c[2] = a[0]*b[1] - a[1]*b[0] ;
}

/**
  * @brief  四元数乘法 ,名称本应为Mcl，懒得改了
  * @note   r=p×q 先作用q再作用p
  * @param  
  * @param  
  * @retval 
  */
void Dot(float p[4],float q[4], float r[4])
{
	r[0]=p[0]*q[0]-p[1]*q[1]-p[2]*q[2]-p[3]*q[3];
	r[1]=p[1]*q[0]+p[0]*q[1]+p[2]*q[3]-p[3]*q[2];
	r[2]=p[2]*q[0]+p[0]*q[2]+p[3]*q[1]-p[1]*q[3];
	r[3]=p[3]*q[0]+p[0]*q[3]+p[1]*q[2]-p[2]*q[1];
}

/**
  * @brief  四元数的逆 
  * @note   
  * @param  
  * @param  
  * @retval 
  */
void Inv(float p[4], float r[4])
{
	float mod=p[0]*p[0]+p[1]*p[1]+p[2]*p[2]+p[3]*p[3];
	r[0]=p[0]/mod;
	r[1]=-p[1]/mod;
	r[2]=-p[2]/mod;
	r[3]=-p[3]/mod;
}

/**
  * @brief  取共轭
  * @note   
  * @param  
  * @param  
  * @retval 
  */
void Conj(float p[4],float q[4])
{
	q[0]=p[0];
	q[1]=-p[1];
	q[2]=-p[2];
	q[3]=-p[3];
}

/**
  * @brief  求点绕axis轴旋转theta后的位置
  * @note   
  * @param  float theta 单位rad
  * @param  
  * @retval 
  */
void Rot(float u[3], float axis[3],float theta, float N[3])
//旋转角单位rad 
{
	float q[4];
	q[0]=0;
	q[1]=u[0];q[2]=u[1];q[3]=u[2];
	float p[4];
	//theta=180/M_PI;//角度弧度互化	
	p[0]=cos(theta*0.5);
	float R=sqrt((axis[0])*(axis[0])+(axis[1])*(axis[1])+(axis[2])*(axis[2]));
	p[1]=sin(theta*0.5)*axis[0]/R;
	p[2]=sin(theta*0.5)*axis[1]/R;
	p[3]=sin(theta*0.5)*axis[2]/R;
	float m[4];
	Dot(p,q,m);
	float v[4];
	Inv(p,v);
    float w[4];
	Dot(m,v,w);
	N[0]=w[1];
	N[1]=w[2];
	N[2]=w[3];
}

/**
  * @brief  直接四元数进行旋转，N[3]为输出的坐标，默认p[4]已经归一化
  * @note   
  * @param  
  * @param  
  * @retval 
  */
void Qrot(float p[4],float r[3],float N[3])
//直接四元数进行旋转，N[3]为输出的坐标，默认p[4]已经归一化。 
{
	float q[4];
	q[0]=0;
	q[1]=r[0];q[2]=r[1];q[3]=r[2];
	float m[4];
	Dot(p,q,m);
	float v[4];
	Conj(p,v);
    float w[4];
	Dot(m,v,w);
	N[0]=w[1];
	N[1]=w[2];
	N[2]=w[3];
	//printf("%f,%f,%f\n",N[0],N[1],N[2]);
} 

/**
  * @brief  计算相对四元数u*Conj(v)
  * @note   
  * @param  
  * @param  
  * @retval 
  */
void xiangdui(float u[4],float v[4],float k[4])
//计算u*Conj(v) 
{
	float m[4];
	Conj(v,m);
	Dot(u,m,k);	
	float norm;
	norm=sqrt(k[0]*k[0]+k[1]*k[1]+k[2]*k[2]+k[3]*k[3]);
	int i;
	for(i=0;i<4;i++)
	{
		k[i]=k[i]/norm;
	}
}


void lxiangdui(float u[4],float v[4],float k[4])
//计算Conj(u)*v 
{
	float m[4];
	Conj(u,m);
	Dot(m,v,k);	
	float norm;
	norm=sqrt(k[0]*k[0]+k[1]*k[1]+k[2]*k[2]+k[3]*k[3]);
	int i;
	for(i=0;i<4;i++)
	{
		k[i]=k[i]/norm;
	}
}


void Q(float Angle1,float Angle2,float q[4])
//计算校正四元数的中间量。 
{
	float r1[4]={cos(Angle1*0.5),0,0,sin(Angle1*0.5)};
	float r2[4]={cos(Angle2*0.5),sin(Angle2*0.5),0,0}; 
	Dot(r1,r2,q);
 } 

 /**
 * @brief  给定IMU数据与机械臂角度，计算修正四元数
   * @note   
   * @param  float IMUs_Quaternion[4][4]
   * @param  float Angles[4]
   * @retval float IMUs_QuaRevision[4][4]
   */
void QuaternionReviser(struct IMU_data imu[4], float Angles[4])
{
	float r[4][4];
	Q(Angles[0],0.0,r[0]);
	Q(Angles[0],Angles[1],r[1]);
	Q(Angles[0],Angles[1]+Angles[2]-M_PI,r[2]);
	/* 貌似需要-2pi ---------------------------------------------------------*/
	Q(Angles[0],Angles[1]+Angles[2]+Angles[3]-2*M_PI,r[3]);//Q为(理想，地面)的四元数
	//q(实，理)=Inv q(理，地）*q(实，地） 
	int i;
	for(i=0;i<4;i++)
	{
		lxiangdui(r[i],imu[i].Quaternion,imu[i].QuaRevision);
	} 
}

/**
  * @brief  Resolve joints' angles given Four IMU Attitudes and its revision quaternion
  * @note   
  * @param  float IMUs_Quaternion[4][4]  IMU data in the form of Quaternion
  * @param  float IMUs_QuaRevision[4][4]  IMU Revision matrix
  * @retval float Angles[0]
  */
void RobotArm_Resolver(struct IMU_data imu[4], float Angles[4])
{
  int i;
  float x[3]={1.0,0.0,0.0};
  float y[3]={0.0,1.0,0.0};
  float z[3]={0.0,0.0,1.0};
  float qr[4][4];//(姿态修正为图中位置后的四元数)
  for(i=0;i<4;i++)
  {
	xiangdui(imu[i].Quaternion, imu[i].QuaRevision, qr[i]);//q(理，地）=q(实，地）*Inv q(实，理) 
  }
  
  float x1[3],x2[3],x3[3],x4[3];
  Qrot(qr[0],x,x1);
  Qrot(qr[1],x,x2);
  Qrot(qr[2],x,x3);
  Qrot(qr[3],x,x4);
  float x_bar[3];
  x_bar[0]=k[0]*x1[0]+k[1]*x2[0]+k[2]*x3[0]+k[3]*x4[0];//计算平均的对称面法向量 
  x_bar[1]=k[0]*x1[1]+k[1]*x2[1]+k[2]*x3[1]+k[3]*x4[1];
  x_bar[2]=k[0]*x1[2]+k[1]*x2[2]+k[2]*x3[2]+k[3]*x4[2];
 // printf("%f,%f,%f\n",x_bar[0],x_bar[1],x_bar[2]); 
  float e[4][3];
  Cross(x1,x_bar,e[0]);
  Cross(x2,x_bar,e[1]);
  Cross(x3,x_bar,e[2]);//IMU的x与平均值叉乘得到误差向量;
  Cross(x4,x_bar,e[3]);
  float theta[4];  
  theta[0]=jiajiao(x1,x_bar);
  theta[1]=jiajiao(x2,x_bar);
  theta[2]=jiajiao(x3,x_bar);
  theta[3]=jiajiao(x4,x_bar);
//printf("%f,%f,%f,%f\n",theta[0],theta[1],theta[2],theta[3]); 
  float eq[4][4];//由误差夹角，轴得到的四元数；
  float norm;
  for(i=0;i<4;i++)
  {
  	  norm=sqrt(e[i][0]*e[i][0]+e[i][1]*e[i][1]+e[i][2]*e[i][2]);
	  if(norm<=1e-4)
	  {
	  	eq[i][0]=1;
	  	eq[i][1]=0;
	  	eq[i][2]=0;
	  	eq[i][3]=0;
	  }
	  else
	  {
	  	eq[i][0]=cos(theta[i]*0.5);
	  	eq[i][1]=sin(theta[i]*0.5)*e[i][0]/norm;
	  	eq[i][2]=sin(theta[i]*0.5)*e[i][1]/norm;
	  	eq[i][3]=sin(theta[i]*0.5)*e[i][2]/norm;	  	
	  }
  		//printf("%f,%f,%f,%f\n",eq[i][0],eq[i][1],eq[i][2],eq[i][3]); 
  }
  float Mq[4][4];//对称面矫正后的四元数

	for(i=0;i<4;i++)
	{
		Dot(eq[i],qr[i],Mq[i]);
	 //printf("%f,%f,%f,%f\n",Mq[i][0],Mq[i][1],Mq[i][2],Mq[i][3]);    
	}
  float n[3];
  Qrot(Mq[0],x,n);
 // printf("%f %f %f\n",n[0],n[1],n[2]);
  float y1[3];//校正后IMU2的y基底在IMU1系的表示
  float y2[3],z3[3];
  float qr21[4];
  float qr32[4],qr43[4];
  float F[4][4];
  for(i=0;i<4;i++)
  {
  	lxiangdui(Mq[0],Mq[i],F[i]);
  	//printf("%f,%f,%f,%f\n",F[i][0],F[i][1],F[i][2],F[i][3]);  
  }
  xiangdui(F[1],F[0],qr21);
// printf("qr21 %f %f %f %f\n",qr21[0],qr21[1],qr21[2],qr21[3]);
  xiangdui(F[2],F[1],qr32);
// printf("qr32%f %f %f %f\n",qr32[0],qr32[1],qr32[2],qr32[3]);
  xiangdui(F[3],F[2],qr43);
//printf("qr43%f %f %f %f\n",qr43[0],qr43[1],qr43[2],qr43[3]);
  Angles[1]=2*asin(qr21[1]);
  Angles[2]=M_PI+2*asin(qr32[1]);
  Angles[3]=M_PI+2*asin(qr43[1]);//IMU4z的IMU3的y分量是sin gamma 
  Angles[0]=atan2(n[1],n[0]);//转台转角

  
  //更新四元数值(修正之前IMU的数据)
	for(i=0;i<4;i++)
	{
		//更新IMU姿态
		Dot(Mq[i], imu[i].QuaRevision, imu[i].Quaternion);
		//更新机械臂姿态
		memcpy(imu[i].LinkQuaternion, Mq[i], sizeof(float)*4);
	}

}

