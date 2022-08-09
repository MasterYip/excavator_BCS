//=====================================================================================================
// File Name:	abstract_layer.c
// Author:		Master Yip
// Date:		20220711
//=====================================================================================================
// Description:
//	【马达控制抽象层】，主要内容是电赛送药小车的运动控制、状态机、决策函数
//=====================================================================================================
#include "CtrlLib_settings.h"

/**
  * @brief  设置广义速度示例函数，执行函数可使电机运动
  * @note   
  * @param  double VelocityVector[] 给定的广义速度
  * @retval 
  */
void Velocity_Set(double VelocityVector[])
{
	double RevVector[MOTORNUMBER] = {0};
	double OutputVector[MOTORNUMBER] = {0};
	RevVector_Superposition(VelocityVector, RevVector);
	RevVector_Quantification(RevVector, OutputVector);
	Vector_Control(OutputVector);
}


//串口接收cmd字符对电机进行调试
void key_teleop_ctrl(u8 *cmd)
{
	float t = MOTOR_CTRL_DELAY;
	struct TimebasedRevVector CtrlVector={{0},{t,t,t,t,t,t}};
	double cmd_vel[6] = {0};
	double velocity = 0.8;
	
	if(*cmd != 0)
	{
		switch(*cmd)
		{
			//履带左
			case '0':
				CtrlVector.RevVector[0] = velocity;
				break;
			case '1':
				CtrlVector.RevVector[0] = -velocity;
				break;
			//履带右
			case '2':
				CtrlVector.RevVector[1] = velocity;
				break;
			case '3':
				CtrlVector.RevVector[1] = -velocity;
				break;
			//转台
			case '4':
				CtrlVector.RevVector[2] = velocity;
				break;
			case '5':
				CtrlVector.RevVector[2] = -velocity;
				break;
			//大臂
			case '6':
				CtrlVector.RevVector[3] = velocity;
				break;
			case '7':
				CtrlVector.RevVector[3] = -velocity;
				break;
			//小臂
			case '8':
				CtrlVector.RevVector[4] = velocity;
				break;
			case '9':
				CtrlVector.RevVector[4] = -velocity;
				break;
			//破碎头
			case 'a':
				CtrlVector.RevVector[5] = velocity;
				break;
			case 's':
				CtrlVector.RevVector[5] = -velocity;
				break;
			default:
				break;
		}
		Vector_Control_with_Time_Limited(CtrlVector);
	}

	*cmd = 0;
}








/* Legacy Functions 仅供参考 ---------------------------------------------------------*/
//=================================================================================
/*
//控制速度过程：赋值VelocityVector数组，Velocity_Set函数传参数
void test3(void)
{
	char buf[100] = {0};
	static double time = 0; 
	time += TimePeriod;
	servo();
	
//	if(cmd[0] == 0)
//	{
//		cmd[0]=48;
//	}
//	
//	if(PAin(15) == 0)
//	{
//		time = 0;
////		sprintf(buf, "ASCII:%d\r\n",cmd[0]);
////		HAL_UART_Transmit(&huart1, buf, strlen(buf), 50);
////		PAout(8) = 0;
//	}
	

	
//	if(time >2)
//	{
//		VelocityVector[0]=0;
//		VelocityVector[2]=0;
//		Velocity_Set(VelocityVector);
//	}
//	else
//	{
//		VelocityVector[0] = 0.5;
//		VelocityVector[2] = 0;
//		Velocity_Set(VelocityVector);
////		Motor_Control(1, 0.9);
////		Motor_Control(0, 0.9);
//	}
	
	
	

	
//	sprintf(buff, "%f,%f,%f,%f,%f,%f,0,", Infrared[0][0].value,Infrared[0][1].value,Infrared[0][2].value,Infrared[0][3].value, Infrared[0][4].value, x1);
//	HAL_UART_Transmit(&huart1, buff, strlen(buff), 50);
//	sprintf(buff, "%f,%f,%f,%f,%f,%f\r\n", Infrared[1][0].value,Infrared[1][1].value,Infrared[1][2].value,Infrared[1][3].value, Infrared[1][4].value, x2);
//	HAL_UART_Transmit(&huart1, buff, strlen(buff), 50);
	double an = (ADC_Values[10]+ADC_Values[21]+ADC_Values[32])/3.0;
	sprintf(buff, "%f\r\n", an);
	HAL_UART_Transmit(&huart1, buff, strlen(buff), 50);

//	sprintf(buff, "%f,%f\r\n", x1, x2);
//	sprintf(buff, "%d\r\n", cmd[0]);
//	HAL_UART_Transmit(&huart1, cmd, strlen(cmd), 50);
//	

	
}


//若要左转 全部取反
int rotation_motion(double NormalVelocity, double angle)
{
	char buf[100] = {0};
	static double time = 0; 
	time += TimePeriod;


	double t_out = angle*13.5/(Velocity_Calc(NormalVelocity));
	
	if(time > t_out)
	{
		VelocityVector[0]=0;
		VelocityVector[2]=0;
		Velocity_Set(VelocityVector);
		time = 0;
		return -1;
	}
	else
	{
		VelocityVector[0] = 0;
		VelocityVector[2] = NormalVelocity;
		Velocity_Set(VelocityVector);
		return 0;
	}
}


int forward_motion(double NormalVelocity, double distance)
{
	char buf[100] = {0};
	static double time = 0; 
	time += TimePeriod;
	servo();
	double t_out = distance/Velocity_Calc(NormalVelocity);
	double Velocity = Velocity_Calc(NormalVelocity);
	
	if(time > t_out)
	{
		VelocityVector[0]=0;
		VelocityVector[2]=0;
		Velocity_Set(VelocityVector);
		time = 0;
		return -1;
	}
	else
	{
		VelocityVector[0] = NormalVelocity;
		VelocityVector[2] = tracking_controller(x1, x2, 13.5, 1.3, Velocity);
		Velocity_Set(VelocityVector);
		return 0;
	}
}

int forward_motion_notracking(double NormalVelocity, double distance)
{
	char buf[100] = {0};
	static double time = 0; 
	time += TimePeriod;
	servo();
	double t_out = distance/Velocity_Calc(NormalVelocity);
	double Velocity = Velocity_Calc(NormalVelocity);
	
	if(time > t_out)
	{
		VelocityVector[0]=0;
		VelocityVector[2]=0;
		Velocity_Set(VelocityVector);
		time = 0;
		return -1;
	}
	else
	{
		VelocityVector[0] = NormalVelocity;
		VelocityVector[2] = 0;
		Velocity_Set(VelocityVector);
		return 0;
	}
}

int forward_CrossingDetect(double NormalVelocity, double distance, double DetectSpeed, double DistanceCompensation)
{
	char buf[100] = {0};
	static double time = 0; 
	static int CorssDetectFlag = 0;
	time += TimePeriod;
	servo();
	double t_out = distance/Velocity_Calc(NormalVelocity);
	double Velocity = Velocity_Calc(NormalVelocity);
	
	if(CorssDetectFlag == 0)
	{
		if(time > t_out)
		{
			
			VelocityVector[0]=DetectSpeed;
			VelocityVector[2]=tracking_controller(x1, x2, 13.5, 1.3, Velocity_Calc(DetectSpeed));
			Velocity_Set(VelocityVector);
			
			if(SumCalc(Infrared[0], 5) > 0.5)//计算前方
			{
				CorssDetectFlag = 1;
			}
			return 0;
		}
		else
		{
			VelocityVector[0] = NormalVelocity;
			VelocityVector[2] = tracking_controller(x1, x2, 13.5, 1.3, Velocity);
			Velocity_Set(VelocityVector);
			return 0;
		}
	}
	else
	{
		int re = forward_motion_notracking(0.25, DistanceCompensation);//半车长
		if(re == -1)
		{
			time = 0;
			CorssDetectFlag = 0;
		}
		return re;
	}
}

//NormalVelocity distance DetectSpeed DistanceCompensation 均取负数
int backward_CrossingDetect(double NormalVelocity, double distance, double DetectSpeed, double DistanceCompensation)
{
	char buf[100] = {0};
	static double time = 0; 
	static int CorssDetectFlag = 0;
	time += TimePeriod;
	servo();
	double t_out = distance/Velocity_Calc(NormalVelocity);
	double Velocity = Velocity_Calc(NormalVelocity);
	
	if(CorssDetectFlag == 0)
	{
		if(time > t_out)
		{
			VelocityVector[0]=DetectSpeed;
			VelocityVector[2]=tracking_controller(x1, x2, 13.5, 1.3, Velocity_Calc(DetectSpeed));
			Velocity_Set(VelocityVector);
			
			if(SumCalc(Infrared[1], 5) > 0.5)//计算后方
			{
				CorssDetectFlag = 1;
			}
			return 0;
		}
		else
		{
			VelocityVector[0] = NormalVelocity;
			VelocityVector[2] = tracking_controller(x1, x2, 13.5, 1.3, Velocity);
			Velocity_Set(VelocityVector);
			return 0;
		}
	}
	else
	{
		int re = forward_motion_notracking(0.25, DistanceCompensation);//半车长
		if(re == -1)
		{
			time = 0;
			CorssDetectFlag = 0;
		}
		return re;
	}
}

int Medicine_Detect(void)
{
	double an = (ADC_Values[10]+ADC_Values[21]+ADC_Values[32])/3.0;
	if(an > 2500)
	{
		PAout(8) = 0;
		return 0;
	}
	else
	{
		PAout(8) = 1;
		return -1;
	}
}

int Process_Manager(void)
{
	static int ExecutionState = 0;
	static int ForecastState = 2;//指向空域
	
	//过程结束
	if (ExecutionState == ForecastState)
	{
		return 0;
	}
	
	//运动种类
	int type = ProcessCtrl[ExecutionState].MotionType;
	//状态结束标记 -1为结束
	int StateDisableFlag = 0;
	
	if(type == 0)
	{
		//定长巡线直行
		StateDisableFlag = forward_motion(ProcessCtrl[ExecutionState].parameter[0], ProcessCtrl[ExecutionState].parameter[1]);
	}
	else if (type == 1)
	{
		//顶角变向转弯
		StateDisableFlag = rotation_motion(ProcessCtrl[ExecutionState].parameter[0], ProcessCtrl[ExecutionState].parameter[1]);
	}
	else if (type == 2)
	{
		//巡线直行寻交叉口
		StateDisableFlag = forward_CrossingDetect(ProcessCtrl[ExecutionState].parameter[0], ProcessCtrl[ExecutionState].parameter[1], ProcessCtrl[ExecutionState].parameter[2], ProcessCtrl[ExecutionState].parameter[3]);
	}
	
	if(StateDisableFlag == -1)
	{
		ExecutionState ++;
	}
}

int Process_Manager2(int command[])
{
	static int ExecutionState = 1;
	
	//状态结束标记 -1为结束
	int StateDisableFlag = 0;
	
	
	if(command[ExecutionState] == 0)
	{
		PDout(2) = 0;
	}
	else if(command[ExecutionState] == 1)
	{
		//左转90
		StateDisableFlag = rotation_motion(-0.5, -1.57);
	}
	else if(command[ExecutionState] == 2)
	{
		//右转90
		StateDisableFlag = rotation_motion(0.5, 1.57);
	}
	else if(command[ExecutionState] == 3)
	{
		//直行到交叉口
		StateDisableFlag = forward_CrossingDetect(0.6, ForwardLength, 0.3, 11);
	}
	else if(command[ExecutionState] == 4)
	{
		//后退到交叉口
		StateDisableFlag = backward_CrossingDetect(-0.6, -ForwardLength, -0.3, -11);
	}
	else if(command[ExecutionState] == 5)
	{
		//后退到药房
		StateDisableFlag = backward_CrossingDetect(-0.6, -ForwardLength, -0.3, -5);
	}
	else if(command[ExecutionState] == 6)
	{
		//等待取药
		StateDisableFlag = Medicine_Detect();
	}
	
	if(StateDisableFlag == -1)
	{
		ExecutionState ++;
//		sprintf(cmd, "%d\r\n", command[ExecutionState]);
//		HAL_UART_Transmit(&huart1, cmd, strlen(cmd), 1);
	}
}

int Decision_Making(char command)
{
	if(maxx==1) return 0;
	if(command=='0')
	{
		steep[++cnt]=1;
		steep[++cnt]=3;
		steep[++cnt]=2;
		steep[++cnt]=5;
		steep[++cnt]=6;
		maxx=cnt;
	}
	if(maxx==0)
	{

		if(command=='1')
		{
			steep[++cnt]=1;
			steep[++cnt]=3;
		}
		if(command=='2')
		{
			steep[++cnt]=2;
			steep[++cnt]=3;
		}
		if(command=='3')
		{
			steep[++cnt]=3;
		}
		if(command=='4')
		{
			steep[++cnt]=2;
			steep[++cnt]=5;
			steep[++cnt]=6;
			maxx=cnt;
		}
		if(command=='5')
		{
			steep[++cnt]=1;
			steep[++cnt]=5;
			steep[++cnt]=6;
			maxx=cnt;
		}
	}
	if(maxx!=0)
	{
		while(maxx!=0)
		{
			if(maxx==1)
			{
				steep[++cnt]=5;
				break;
			}
			if(steep[maxx]==5)steep[++cnt]=3;
			if(steep[maxx]==3)steep[++cnt]=4;
			if(steep[maxx]==1)steep[++cnt]=2;
			if(steep[maxx]==2)steep[++cnt]=1;
			maxx--;
		}
	}
}
*/

