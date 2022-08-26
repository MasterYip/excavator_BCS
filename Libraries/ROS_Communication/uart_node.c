//#include "uart_node.h"
#include "uart_node.hpp"

#include "stm32f1xx_hal.h"

#include "ros.h"
#include "ros/time.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/IMU.h"
#include "sensor_msgs/JointState.h"

#include "CtrlLib_settings.h"
ros::NodeHandle nh;
sensor_msgs::Imu imu_data;
sensor_msgs::JointState present_joint_state;
char *joint_name[8] = {
	(char *)"wheel1_joint", (char *)"wheel2_joint", (char *)"wheel3_joint", (char *)"wheel4_joint",
	(char *)"table_joint", (char *)"link1_joint", (char *)"link2_joint", (char *)"ram_joint"};

void motor_cmd_Callback(const geometry_msgs::Twist& cmd_msg)
{
	double VelocityVector[3] = {cmd_msg.linear.x, 0, cmd_msg.angular.z};
	double RevVector[MOTORNUMBER] = {0};
	struct TimebasedRevVector CtrlVector = {{0},{MOTOR_CTRL_DELAY,MOTOR_CTRL_DELAY,-1,-1,-1,-1}};
	
	
	//实际速度向量 叠加运算转化为 实际转速向量
	RevVector_Superposition(VelocityVector, RevVector);
	//实际转速向量 量化为 广义转速向量
	RevVector_Quantification(RevVector, CtrlVector.RevVector);
	//时基伺服函数
	Vector_Control_with_Time_Limited(CtrlVector);

}

void joint_state_cmd_Callback(const sensor_msgs::JointState& target_joint_state)
{
//	char data[200]="";
	PID_arm_controller(target_joint_state.name, target_joint_state.position, target_joint_state.velocity, urdfAngles);
//	sprintf(data, "\r\n%s %s %s %s", target_joint_state.name[0], target_joint_state.name[1],target_joint_state.name[2],target_joint_state.name[3]);
//	HALprintf(data);
}

ros::Publisher joint_state_publisher("/joint_states", &present_joint_state);
ros::Publisher imu_data_publisher("/imu/data_raw", &imu_data);
ros::Subscriber<sensor_msgs::JointState> target_joint_state_listener("/target_joint_state", &joint_state_cmd_Callback);
ros::Subscriber<geometry_msgs::Twist> motor_cmd_listener("/cmd_vel", &motor_cmd_Callback);


//放入HAL_UART_TxCpltCallback回调函数
void USART_TxCplt_ROSCallback(void)
{
	nh.getHardware()->flush();
}

//放入HAL_UART_RxCpltCallback回调函数
void USART_RxCplt_ROSCallback(void)
{
	nh.getHardware()->reset_rbuf();
}

//发布IMU_Data信息
void IMU_Data_pub(void)
{
	imu_data.header.stamp = nh.now();
	imu_data.angular_velocity.x = ICM20602_Data[0].AngularVelocity[0]-ICM20602_Data[0].GyroAverageCalibration[0];
	imu_data.angular_velocity.y = ICM20602_Data[0].AngularVelocity[1]-ICM20602_Data[0].GyroAverageCalibration[1];
	imu_data.angular_velocity.z = ICM20602_Data[0].AngularVelocity[2]-ICM20602_Data[0].GyroAverageCalibration[2];
	imu_data.linear_acceleration.x = ICM20602_Data[0].Acceleration[0];
	imu_data.linear_acceleration.y = ICM20602_Data[0].Acceleration[1];
	imu_data.linear_acceleration.z = ICM20602_Data[0].Acceleration[2];
	imu_data.orientation.w = ICM20602_Data[0].LinkQuaternion[0];
	imu_data.orientation.x = ICM20602_Data[0].LinkQuaternion[1];
	imu_data.orientation.y = ICM20602_Data[0].LinkQuaternion[2];
	imu_data.orientation.z = ICM20602_Data[0].LinkQuaternion[3];
	
	imu_data_publisher.publish(&imu_data);
}


void Joint_State_pub(void)
{
	static int seq_count = 0;
	double joint_position[8] = {0, 0, 0, 0, urdfAngles[0], urdfAngles[1], urdfAngles[2], urdfAngles[3]};
	present_joint_state.header.stamp = nh.now();
	present_joint_state.header.seq = seq_count++;
	
	present_joint_state.name_length = 8;
	present_joint_state.position_length = 8;
	present_joint_state.velocity_length = 8;
	present_joint_state.effort_length = 8;
	
	present_joint_state.name = joint_name;
	present_joint_state.position = joint_position;
	present_joint_state.position = joint_position;
	//有待完善
//	present_joint_state.velocity = joint_velocity;
	
	joint_state_publisher.publish(&present_joint_state);
	
}


//ROS 初始化函数
void setup(void)
{
	nh.initNode();
	nh.advertise(imu_data_publisher);
	nh.advertise(joint_state_publisher);
	nh.subscribe(motor_cmd_listener);
	nh.subscribe(target_joint_state_listener);
}

//回环函数
void loop(void)
{
	static float toggle_time=0;
	toggle_time+=TimePeriod;
	if(toggle_time > 0.1)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		toggle_time = 0;
	}
	
	Joint_State_pub();
	IMU_Data_pub();

	nh.spinOnce();
	HAL_Delay(100);
	
}