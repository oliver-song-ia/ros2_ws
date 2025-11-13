
#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h> 
#include <stdlib.h>    
#include <unistd.h> 

#include "rclcpp/rclcpp.hpp"
#include <rcl/types.h>
#include <sys/stat.h>
#include <fcntl.h>          
#include <stdbool.h>
      
#include <sys/types.h>

#include <serial/serial.h>
#include <stm_driver/stm_driver.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "wheeltec_robot_msg/msg/data.hpp" 
#include "wheeltec_robot_msg/msg/supersonic.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/range.hpp>

//回充相关新增
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <turtlesim/srv/spawn.hpp>

#include "stm_msg/msg/tcpdata.hpp"
using namespace std;


//Macro definition
//宏定义
#define PI 				  3.1415926f //PI //圆周率


extern sensor_msgs::msg::Imu Mpu6050;  //External variables, IMU topic data //外部变量，IMU话题数据

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;

//Data structure for speed and position
//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;

#define TCP_DATA_LEN 1024
typedef struct
{
	int length;
	char buf[TCP_DATA_LEN];
}Tcp_Recv_Data_t;
/****************轮毂定义*****************/
#define MOTOR_HEAD_VALUE 0x55		//发送头
#define MOTOR_WHEEL_CMD_LEN	5			//单个速度指令长度
#define MOTOR_JOINT_CMD_LEN	6			//单个位置指令长度
#define MOTOR_WHEEL_LEN	4				//速度指令长度

#define IA_NEW_ROBOT
#ifdef IA_NEW_ROBOT

#define MOTOR_JOINT_LEN	14			//位置指令长度

#define WHEEL_FRONT_LEFT -1.0
#define WHEEL_REAR_LEFT -1.0
#define WHEEL_FRONT_RIGHT 1.0
#define WHEEL_REAR_RIGTH 1.0

const int Pos_init_data[MOTOR_JOINT_LEN] = {
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
};

const float WHEEL_POSITION[4] = {
	-1.0,
	-1.0,
	1.0,
	1.0,
};

//基于方位对应的驱动器位置
const float WHEEL_SPEED_ID[4][2] = {
	{0,0},
	{1,2},
	{2,1},
	{3,3},
};

#define JOINT_DATA_PUB 0x000f3fff		//高16位为四个轮子 低16位为关节
#else

#define MOTOR_JOINT_LEN	4			//位置指令长度

#define WHEEL_FRONT_LEFT -1.0
#define WHEEL_REAR_LEFT -1.0
#define WHEEL_FRONT_RIGHT 1.0
#define WHEEL_REAR_RIGTH 1.0

const int Pos_init_data[MOTOR_JOINT_LEN] = {
	0x15800,
	0x13000,
	0x10800,
	0x1e800,
};

const float WHEEL_POSITION[4] = {
	-1.0,
	-1.0,
	1.0,
	1.0,
};

//基于方位对应的驱动器位置
const float WHEEL_SPEED_ID[4][2] = {
	{0,0},
	{1,2},
	{2,1},
	{3,3},
};

#define JOINT_DATA_PUB 0x000f003f
#endif

//#define IA_USE_TCP
//当前指令
typedef enum {
    CMD_TYPE_HEART,
    CMD_TYPE_JOINT_MOTOR = 1,
    CMD_TYPE_WHEEL_MOTOR = 2,
    CMD_TYPE_SOUND_RANGE = 3,
    CMD_TYPE_TOF_RANGE = 4,
    CMD_TYPE_IMU_DATA = 5,
    CMD_TYPE_RS485_MOTOR = 6,
    CMD_TYPE_CONTROL_BOARD = 0xf0,
    CMD_TYPE_END = 0xff,
} cmd_t;


//关节定义
typedef enum{
    JOINT_PAN_MOTOR_1 = 2,
    JOINT_PAN_MOTOR_2 = 3,
    JOINT_PAN_MOTOR_3 = 4,
    JOINT_PAN_MOTOR_4 = 5,
    JOINT_PAN_MOTOR_5 = 6,
    JOINT_PAN_MOTOR_6 = 7,

    JOINT_JOINT_MOTOR_1 = 8,
    JOINT_JOINT_MOTOR_2 = 9,
    JOINT_JOINT_MOTOR_3 = 10,
    JOINT_JOINT_MOTOR_4 = 11,
    JOINT_JOINT_MOTOR_5 = 12,
    JOINT_JOINT_MOTOR_6 = 13,
    JOINT_JOINT_MOTOR_7 = 14,
    JOINT_JOINT_MOTOR_8 = 15,
}Joint_motor_id_t;



//字节发送结构
typedef struct {
    uint8_t cmd_head;
    uint8_t cmd_type;
    uint8_t cmd_command[2];     //当前的命令字 暂时没有用
    uint8_t cmd_len[2];
    uint8_t cmd_data[128];		//data写大一点
} cmd_structure_t;

/*
//小车轮子速度角度
typedef struct __Vel_Pos_Data_
{
	float F_L_VEL;
	float F_L_VEL;
	float F_L_VEL;
	float F_L_VEL;
	float F_L_VEL;
	float F_L_VEL
}Vel_Pos_Data;*/

//IMU data structure
//IMU数据结构体
typedef struct __MPU6050_DATA_
{
	short accele_x_data; 
	short accele_y_data; 	
	short accele_z_data; 
    short gyros_x_data; 
	short gyros_y_data; 	
	short gyros_z_data; 

}MPU6050_DATA;

//The robot chassis class uses constructors to initialize data, publish topics, etc
//机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot : public rclcpp::Node
{
	public:
		turn_on_robot();  //Constructor //构造函数
		~turn_on_robot(); //Destructor //析构函数
		void Control();   //Loop control code //循环控制代码
		serial::Serial Stm32_Serial; //Declare a serial object //声明串口对象 
		stm_driver stm_all;
		data_handle data_handle_p;

		void func1Thread();		//订阅相关处理
		void func2Thread();		//网络相关处理
		std::shared_ptr<std::thread> difop_thread1_;  
		std::shared_ptr<std::thread> difop_thread2_;
	private:
		//ros::NodeHandle n;           //Create a ROS node handle //创建ROS节点句柄
		FILE *ia_fp; 
		rclcpp::Time _Now, _Last_Time;  //Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
		float Sampling_Time;         //Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)


		rclcpp::TimerBase::SharedPtr heart_timer_;		//定时器 用来给心跳

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;//Initialize the topic subscriber //初始化话题订阅者
		
		rclcpp::Subscription<stm_msg::msg::Tcpdata>::SharedPtr Cmd_Vel_New;
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Cmd_Motor_Status;
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Ia_Motor_Status;

        //Initialize the topic publisher //初始化话题发布者
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;       
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;        
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
        rclcpp::Publisher<wheeltec_robot_msg::msg::Supersonic>::SharedPtr distance_publisher;         
		//小车状态发布者
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_speed_publisher;
		sensor_msgs::msg::JointState robot_status;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_wheel_left_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_wheel_right_publisher;
		sensor_msgs::msg::JointState robot_wheel_status;

		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robot_joint_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_2_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_3_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_4_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_5_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_6_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_7_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_8_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_9_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_10_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_11_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_12_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_13_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_14_publisher;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_15_publisher;

		//小车状态统一发布
		//rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Robot_joint_publisher;

		sensor_msgs::msg::JointState robot_joint_status;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robot_state_publisher;
		//回充相关发布者
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Charging_publisher;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr Charging_current_publisher;
		rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr RED_publisher;
		//超声波相关发布者
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_front_left_forward_topic;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_front_left_side_topic;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_front_right_forward_topic;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_front_right_side_topic;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_rear_left_forward_topic;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_rear_left_side_topic;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_rear_right_forward_topic;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_rear_right_side_topic;

		//回充相关订阅者
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Red_Vel_Sub;
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr Recharge_Flag_Sub;
		//回充相关服务
		rclcpp::Service<turtlesim::srv::Spawn>::SharedPtr SetCharge_Service;

		//The speed topic subscribes to the callback function
		//速度话题订阅回调函数
        void Cmd_Vel_Callback(const stm_msg::msg::Tcpdata::SharedPtr twist_aux);
		void Cmd_Vel_Callback_Status(const sensor_msgs::msg::JointState::SharedPtr staus);
		void Ia_Joint_Commands_Callback_Status(const sensor_msgs::msg::JointState::SharedPtr staus);
		void Ia_Joint_Commands_Callback_Status_New(const sensor_msgs::msg::JointState::SharedPtr staus);

	
		//ia话题发布
		void Ia_Write_Log_To_File(char* buf,int len);
		void Ia_Publish_Odom();
		void Publish_Odom();      //Pub the speedometer topic //发布里程计话题
		void Publish_ImuSensor(); //Pub the IMU sensor topic //发布IMU传感器话题
		void Publish_distance();//发布超声波距离

		void Publish_Robot_Status(char* Rceive_Tcp_Data);
        //从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
        //Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
        bool Get_Sensor_Data();   
		bool Get_Sensor_Data_New();
		bool Get_Low_Computer_Data();
		void Tcp_Recv_Data_Deal(char* tcp_data,int len);
		void Tcp_Cmd_Data_Deal(char* tcp_data,int len);
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数
        short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);  //IMU data conversion read //IMU数据转化读取
		float Odom_Trans(uint8_t Data_High,uint8_t Data_Low); //Odometer data is converted to read //里程计数据转化读取

		void Sound_Data_Deal(char* data,int len);
		void Wheel_Data_Deal(char* data,int len);
		void Joint_Data_Deal(char* data,int len);
		void Robot_Data_Deal(char* data,int len);
		void heart_timer_callback();

        string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id; //Define the related variables //定义相关变量
        int serial_baud_rate;      //Serial communication baud rate //串口通信波特率
       
        Vel_Pos_Data Robot_Pos;    //The position of the robot //机器人的位置
        Vel_Pos_Data Robot_Vel;    //The speed of the robot //机器人的速度
        float odom_x_scale,odom_y_scale,odom_z_scale_positive,odom_z_scale_negative; //Odometer correction parameters //里程计修正参数
};
#endif
