#include "turn_on_wheeltec_robot/wheeltec_tcp_robot.h"
#include "turn_on_wheeltec_robot/Quaternion_Solution.h"
#include "wheeltec_robot_msg/msg/data.hpp" 
#include "turn_on_wheeltec_robot/crc.h"

sensor_msgs::msg::Imu Mpu6050;//Instantiate an IMU object //实例化IMU对象 

using std::placeholders::_1;
using namespace std;
rclcpp::Node::SharedPtr node_handle = nullptr;

//自动回充使用相关变量
bool check_AutoCharge_data = false;
bool charge_set_state = false;
/**************************************
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); //ROS initializes and sets the node name //ROS初始化 并设置节点名称 
  //turn_on_robot Robot_Control;//Instantiate an object //实例化一个对象
  //Robot_Control.Control();//Loop through data collection and publish the topic //循环执行数据采集和发布话题等操作
  rclcpp::spin(std::make_shared<turn_on_robot>());
  rclcpp::shutdown();

  while (rclcpp::ok()){
  }  
  return 0;  
} 


/**************************************
Function: Data conversion function
功能: 数据转换函数
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;   
  transition_16 |=  Data_Low;
  return transition_16;     
}
float turn_on_robot::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;  //Get the high 8 bits of data   //获取数据的高8位
  transition_16 |=  Data_Low;      //Get the lowest 8 bits of data //获取数据的低8位
  data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}
/**************************************
Date: January 28, 2021
Function: 速度话题订阅回调函数，将ROS标准Twist速度指令转换为下位机TCP控制帧并发送
          处理机器人线速度(x/y轴)和角速度(z轴)指令，完成数据打包与校验后通过TCP下发
Input: twist_aux - ROS标准Twist消息指针，包含机器人目标运动速度
  - linear.x: x轴目标线速度(m/s)
  - linear.y: y轴目标线速度(m/s)
  - angular.z: z轴目标角速度(rad/s)
Output: 无返回值，通过TCP发送11字节控制帧到下位机
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  short  transition;  //intermediate variable //中间变量，用于暂存速度转换后的整数型值

  // 构建TCP控制帧头部 (共11字节: 帧头1字节 + 控制位2字节 + 速度数据6字节 + 校验位1字节 + 帧尾1字节)
  Send_Data.tx[0] = FRAME_HEADER;       //frame head 0x7B //帧头0X7B，固定起始标识
  Send_Data.tx[1] = AutoRecharge;       //set aside //预留位，用于自动回充状态标识
  Send_Data.tx[2] = 0;                  //set aside //预留位，未使用

  //The target velocity of the X-axis of the robot
  //机器人x轴的目标线速度 (前进/后退方向)
  transition = 0;
  transition = twist_aux->linear.x * 1000;  //将浮点数放大一千倍，将m/s转换为mm/s，避免小数传输误差
  Send_Data.tx[4] = transition;             //取数据的低8位 (little-endian字节序)
  Send_Data.tx[3] = transition >> 8;        //取数据的高8位 (16位整数拆分存储)

  //The target velocity of the Y-axis of the robot
  //机器人y轴的目标线速度 (左右平移方向，仅全向底盘有效)
  transition = 0;
  transition = twist_aux->linear.y * 1000;  //将浮点数放大一千倍，单位转换为mm/s
  Send_Data.tx[6] = transition;             //低8位存储
  Send_Data.tx[5] = transition >> 8;        //高8位存储

  //The target angular velocity of the robot's Z axis
  //机器人z轴的目标角速度 (旋转方向)
  transition = 0;
  transition = twist_aux->angular.z * 1000; //将浮点数放大一千倍，单位转换为mrad/s
  Send_Data.tx[8] = transition;             //低8位存储
  Send_Data.tx[7] = transition >> 8;        //高8位存储

  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);  //For the BCC check bits, see the Check_Sum function //BCC校验位，对前9字节按位异或计算
  Send_Data.tx[10] = FRAME_TAIL;                    //frame tail 0x7D //帧尾0X7D，固定结束标识
 
  #ifdef IA_USE_TCP
  // 通过TCP发送完整数据帧到下位机控制器
  //stm_all.tcp_truct_t.tcp_write((char*)Send_Data.tx, sizeof(Send_Data.tx));
  #else
  //stm_all.udp_truct_t.udp_write((char*)Send_Data.tx, sizeof(Send_Data.tx));
  #endif
}


#define STM_PI 3.1415926
#define STM_DIAM 160    //mm
#define STM_ONE_CYCLE 0x40000
//新话题接收
void turn_on_robot::Cmd_Vel_Callback_New(const stm_msg::msg::Tcpdata::SharedPtr twist_aux){
  RCLCPP_INFO(this->get_logger(),"recv: %f %f",twist_aux->speed1d,twist_aux->pos1d);

  uint8_t send_data[100];

  //头
  send_data[0] = 0x55;
  //发送轮毂指令
  send_data[1] = 0x02;
  //二级指令
  send_data[2] = 0x00;
  send_data[3] = 0x00;
  //长度
  send_data[4] = 0x00;
  send_data[5] = 0x1c;
  //数据
  //float round = PI*STM_DIAM/1000; //改为m
  float round = PI*2;
  float speed = (twist_aux->speed1d)*60/round;  //转换为m/s
  send_data[6] = 0x01;    //使能
  send_data[7] = 0x01;
  send_data[8] = (uint8_t)(speed/256);
  send_data[9] = (uint8_t)(speed);
  send_data[10] = 0x64;   //加速度
  speed = -(twist_aux->speed2d)*60/round;  //转换为m/s
  send_data[11] = 0x01;    //使能
  send_data[12] = 0x01;
  send_data[13] = (uint8_t)(speed/256);
  send_data[14] = (uint8_t)(speed);
  send_data[15] = 0x64;   //加速度
  speed = (twist_aux->speed3d)*60/round;  //转换为m/s
  send_data[16] = 0x01;    //使能
  send_data[17] = 0x01;
  send_data[18] = (uint8_t)(speed/256);
  send_data[19] = (uint8_t)(speed);
  send_data[20] = 0x64;   //加速度
  speed = -(twist_aux->speed4d)*60/round;  //转换为m/s
  send_data[21] = 0x01;    //使能
  send_data[22] = 0x01;
  send_data[23] = (uint8_t)(speed/256);
  send_data[24] = (uint8_t)(speed);
  send_data[25] = 0x64;   //加速度

  uint16_t crc = com_crc(send_data,26);
  send_data[26] = (crc>>8)&0xff;
  send_data[27] = crc&0xff;   //尾

  #ifdef IA_USE_TCP
  stm_all.tcp_truct_t.tcp_write((char*)send_data,28); //发送tcp
  #else
  stm_all.udp_truct_t.udp_write((char*)send_data,28);
  #endif
  //发送关节 TODO
  //发送轮毂指令
  send_data[1] = 0x01;
  //二级指令
  send_data[2] = 0x00;
  send_data[3] = 0x00;
  //长度
  send_data[4] = 0x00;
  send_data[5] = 0x20;
  //数据
  uint32_t pos_init_data = 0x015800;
  float positon_step = (twist_aux->pos1d)/PI/2;  //弧度计算
  uint32_t position = (uint32_t)(positon_step*STM_ONE_CYCLE+pos_init_data);
  send_data[6] = 0x02;
  send_data[7] = 0x1e;
  send_data[8] = (uint8_t)(position&0xff);
  send_data[9] = (uint8_t)((position>>8)&0xff);
  send_data[10] = (uint8_t)((position>>16)&0xff);
  send_data[11] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x013000;
  positon_step = (twist_aux->pos2d)/PI/2;
  position = (uint32_t)(positon_step*STM_ONE_CYCLE+pos_init_data);
  send_data[12] = 0x02;
  send_data[13] = 0x1e;
  send_data[14] = (uint8_t)(position&0xff);
  send_data[15] = (uint8_t)((position>>8)&0xff);
  send_data[16] = (uint8_t)((position>>16)&0xff);
  send_data[17] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x010800;
  positon_step = (twist_aux->pos3d)/PI/2;
  position = (uint32_t)(positon_step*STM_ONE_CYCLE+pos_init_data);
  send_data[18] = 0x02;
  send_data[19] = 0x1e;
  send_data[20] = (uint8_t)(position&0xff);
  send_data[21] = (uint8_t)((position>>8)&0xff);
  send_data[22] = (uint8_t)((position>>16)&0xff);
  send_data[23] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x01e800;
  positon_step = (twist_aux->pos4d)/PI/2;
  position = (uint32_t)(positon_step*STM_ONE_CYCLE+pos_init_data);
  send_data[24] = 0x02;
  send_data[25] = 0x1e;
  send_data[26] = (uint8_t)(position&0xff);
  send_data[27] = (uint8_t)((position>>8)&0xff);
  send_data[28] = (uint8_t)((position>>16)&0xff);
  send_data[29] = (uint8_t)((position>>24)&0xff);
  //crc
  crc = com_crc(send_data,30);
  send_data[30] = (crc>>8)&0xff;
  send_data[31] = crc&0xff;   //尾
  #ifdef IA_USE_TCP
    stm_all.tcp_truct_t.tcp_write((char*)send_data,32); //发送tcp
  #else
    stm_all.udp_truct_t.udp_write((char*)send_data,32);
  #endif
}

//接收机器人状态信息处理
void turn_on_robot::Cmd_Vel_Callback_Status(const sensor_msgs::msg::JointState::SharedPtr staus){
  //std::array<std::string, 4> wheel_joint_names{};
  //std::array<std::string, 4> axle_joint_names{};

  //auto wheel_joint_names = staus->name;
  //int size = wheel_joint_names.size();
  //RCLCPP_INFO(this->get_logger(),"Cmd_Vel_Callback_Status: %s %s %s %s ",wheel_joint_names[0],wheel_joint_names[1],wheel_joint_names[2],wheel_joint_names[3]);
  //RCLCPP_INFO(this->get_logger(),"Cmd_Vel_Callback_Status: %f %f %f %f ",staus->position[0],staus->position[1],staus->position[2],staus->position[3]);
  //RCLCPP_INFO(this->get_logger(),"Cmd_Vel: %f %f %f %f ",staus->velocity[0],staus->velocity[1],staus->velocity[2],staus->velocity[3]);
  uint16_t crc; 
  uint8_t send_data[100];
  uint16_t speed_data;
  //头
  send_data[0] = 0x55;
  //发送轮毂指令
  send_data[1] = 0x02;
  //二级指令
  send_data[2] = 0x00;
  send_data[3] = 0x00;
  //长度
  send_data[4] = 0x00;
  send_data[5] = 0x1c;
  //数据
  //float round = PI*STM_DIAM/1000; //改为m
  float round = PI*2;
  float speed = WHEEL_FRONT_LEFT*(staus->velocity[0])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[6] = 0x01;    //使能
  send_data[7] = 0x01;
  send_data[8] = (uint8_t)(speed_data/256);
  send_data[9] = (uint8_t)(speed_data);
  send_data[10] = 0x64;   //加速度
  speed = WHEEL_FRONT_RIGHT*(staus->velocity[2])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[11] = 0x01;    //使能
  send_data[12] = 0x01;
  send_data[13] = (uint8_t)(speed_data/256);
  send_data[14] = (uint8_t)(speed_data);
  send_data[15] = 0x64;   //加速度
  speed = WHEEL_REAR_LEFT*(staus->velocity[1])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[16] = 0x01;    //使能
  send_data[17] = 0x01;
  send_data[18] = (uint8_t)(speed_data/256);
  send_data[19] = (uint8_t)(speed_data);
  send_data[20] = 0x64;   //加速度
  speed = WHEEL_REAR_RIGTH*(staus->velocity[3])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[21] = 0x01;    //使能
  send_data[22] = 0x01;
  send_data[23] = (uint8_t)(speed_data/256);
  send_data[24] = (uint8_t)(speed_data);
  send_data[25] = 0x64;   //加速度

  crc = com_crc(send_data,26);
  send_data[26] = (crc>>8)&0xff;
  send_data[27] = crc&0xff;   //尾

  /*for(int i=0;i<28;i++){
    printf("send_data[%d]:%x\n",i,send_data[i]);
  }*/
  #ifdef IA_USE_TCP
    stm_all.tcp_truct_t.tcp_write((char*)send_data,28); //发送tcp
  #else
    stm_all.udp_truct_t.udp_write((char*)send_data,28); //发送tcp
    Ia_Write_Log_To_File((char*)send_data,28);
  #endif  
  //
  //发送关节 TODO
  send_data[1] = 0x01;
  //二级指令
  send_data[2] = 0x00;
  send_data[3] = 0x00;
  //长度
  send_data[4] = 0x00;
  send_data[5] = 0x20;
  //数据
  #ifdef IA_NEW_ROBOT
  uint32_t pos_init_data = 0;
  float positon_step = (staus->position[0])/PI/2;  //弧度计算
  uint32_t position; //= static_cast<int>(positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[6] = 0x02;
  send_data[7] = 0x1e;
  send_data[8] = (uint8_t)(position&0xff);
  send_data[9] = (uint8_t)((position>>8)&0xff);
  send_data[10] = (uint8_t)((position>>16)&0xff);
  send_data[11] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0;
  positon_step = (staus->position[1])/PI/2;
  //position = static_cast<int>(-positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[12] = 0x03;
  send_data[13] = 0x1e;
  send_data[14] = (uint8_t)(position&0xff);
  send_data[15] = (uint8_t)((position>>8)&0xff);
  send_data[16] = (uint8_t)((position>>16)&0xff);
  send_data[17] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0;
  positon_step = (staus->position[2])/PI/2;    
  //position = static_cast<int>(positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[18] = 0x04;
  send_data[19] = 0x1e;
  send_data[20] = (uint8_t)(position&0xff);
  send_data[21] = (uint8_t)((position>>8)&0xff);
  send_data[22] = (uint8_t)((position>>16)&0xff);
  send_data[23] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0;
  positon_step = (staus->position[3])/PI/2;  
  //position = static_cast<int>(-positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[24] = 0x05;
  send_data[25] = 0x1e;
  send_data[26] = (uint8_t)(position&0xff);
  send_data[27] = (uint8_t)((position>>8)&0xff);
  send_data[28] = (uint8_t)((position>>16)&0xff);
  send_data[29] = (uint8_t)((position>>24)&0xff);
  //长度
  crc = com_crc(send_data,30);
  send_data[30] = (crc>>8)&0xff;
  send_data[31] = crc&0xff;   //尾
  #ifdef IA_USE_TCP
    stm_all.tcp_truct_t.tcp_write((char*)send_data,32); //发送tcp
  #else
    stm_all.udp_truct_t.udp_write((char*)send_data,32); //发送tcp
    Ia_Write_Log_To_File((char*)send_data,32);
  #endif
  #else
  uint32_t pos_init_data = 0x015800;
  float positon_step = (staus->position[0])/PI/2;  //弧度计算
  uint32_t position; //= static_cast<int>(positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[6] = 0x02;
  send_data[7] = 0x1e;
  send_data[8] = (uint8_t)(position&0xff);
  send_data[9] = (uint8_t)((position>>8)&0xff);
  send_data[10] = (uint8_t)((position>>16)&0xff);
  send_data[11] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x013000;
  positon_step = (staus->position[1])/PI/2;
  //position = static_cast<int>(-positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[12] = 0x03;
  send_data[13] = 0x1e;
  send_data[14] = (uint8_t)(position&0xff);
  send_data[15] = (uint8_t)((position>>8)&0xff);
  send_data[16] = (uint8_t)((position>>16)&0xff);
  send_data[17] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x010800;
  positon_step = (staus->position[2])/PI/2;    
  //position = static_cast<int>(positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[18] = 0x04;
  send_data[19] = 0x1e;
  send_data[20] = (uint8_t)(position&0xff);
  send_data[21] = (uint8_t)((position>>8)&0xff);
  send_data[22] = (uint8_t)((position>>16)&0xff);
  send_data[23] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x01e800;
  positon_step = (staus->position[3])/PI/2;  
  //position = static_cast<int>(-positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[24] = 0x05;
  send_data[25] = 0x1e;
  send_data[26] = (uint8_t)(position&0xff);
  send_data[27] = (uint8_t)((position>>8)&0xff);
  send_data[28] = (uint8_t)((position>>16)&0xff);
  send_data[29] = (uint8_t)((position>>24)&0xff);
  //长度
  crc = com_crc(send_data,30);
  send_data[30] = (crc>>8)&0xff;
  send_data[31] = crc&0xff;   //尾
  #ifdef IA_USE_TCP
    stm_all.tcp_truct_t.tcp_write((char*)send_data,32); //发送tcp
  #else
    stm_all.udp_truct_t.udp_write((char*)send_data,32); //发送tc
    Ia_Write_Log_To_File((char*)send_data,32);
  #endif
  #endif
  
}

void turn_on_robot::Ia_Joint_Commands_Callback_Status(const sensor_msgs::msg::JointState::SharedPtr staus){
  uint16_t crc; 
  uint8_t send_data[100];
  uint16_t speed_data;
  //头
  send_data[0] = 0x55;
  //发送轮毂指令
  send_data[1] = 0x02;
  //二级指令
  send_data[2] = 0x00;
  send_data[3] = 0x00;
  //长度
  send_data[4] = 0x00;
  send_data[5] = 0x1c;
  //数据
  //float round = PI*STM_DIAM/1000; //改为m
  float round = PI*2;
  float speed = WHEEL_FRONT_LEFT*(staus->velocity[0])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[6] = 0x01;    //使能
  send_data[7] = 0x01;
  send_data[8] = (uint8_t)(speed_data/256);
  send_data[9] = (uint8_t)(speed_data);
  send_data[10] = 0x64;   //加速度
  speed = WHEEL_REAR_LEFT*(staus->velocity[2])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[11] = 0x01;    //使能
  send_data[12] = 0x01;
  send_data[13] = (uint8_t)(speed_data/256);
  send_data[14] = (uint8_t)(speed_data);
  send_data[15] = 0x64;   //加速度
  speed = WHEEL_FRONT_RIGHT*(staus->velocity[1])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[16] = 0x01;    //使能
  send_data[17] = 0x01;
  send_data[18] = (uint8_t)(speed_data/256);
  send_data[19] = (uint8_t)(speed_data);
  send_data[20] = 0x64;   //加速度
  speed = WHEEL_REAR_RIGTH*(staus->velocity[3])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[21] = 0x01;    //使能
  send_data[22] = 0x01;
  send_data[23] = (uint8_t)(speed_data/256);
  send_data[24] = (uint8_t)(speed_data);
  send_data[25] = 0x64;   //加速度

  crc = com_crc(send_data,26);
  send_data[26] = (crc>>8)&0xff;
  send_data[27] = crc&0xff;   //尾

  /*for(int i=0;i<28;i++){
    printf("send_data[%d]:%x\n",i,send_data[i]);
  }*/
  #ifdef IA_USE_TCP
    stm_all.tcp_truct_t.tcp_write((char*)send_data,28); //发送tcp
  #else
    stm_all.udp_truct_t.udp_write((char*)send_data,28); //发送tcp
    Ia_Write_Log_To_File((char*)send_data,28);
  #endif  
  //
  //发送关节 TODO
  send_data[1] = 0x01;
  //二级指令
  send_data[2] = 0x00;
  send_data[3] = 0x00;
  //长度
  send_data[4] = 0x00;
  send_data[5] = 0x20;
  //数据
  #ifdef IA_NEW_ROBOT
  uint32_t pos_init_data = 0;
  float positon_step = (staus->position[0])/PI/2;  //弧度计算
  uint32_t position; //= static_cast<int>(positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[6] = 0x02;
  send_data[7] = 0x1e;
  send_data[8] = (uint8_t)(position&0xff);
  send_data[9] = (uint8_t)((position>>8)&0xff);
  send_data[10] = (uint8_t)((position>>16)&0xff);
  send_data[11] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0;
  positon_step = (staus->position[1])/PI/2;
  //position = static_cast<int>(-positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[12] = 0x03;
  send_data[13] = 0x1e;
  send_data[14] = (uint8_t)(position&0xff);
  send_data[15] = (uint8_t)((position>>8)&0xff);
  send_data[16] = (uint8_t)((position>>16)&0xff);
  send_data[17] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0;
  positon_step = (staus->position[2])/PI/2;    
  //position = static_cast<int>(positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[18] = 0x04;
  send_data[19] = 0x1e;
  send_data[20] = (uint8_t)(position&0xff);
  send_data[21] = (uint8_t)((position>>8)&0xff);
  send_data[22] = (uint8_t)((position>>16)&0xff);
  send_data[23] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0;
  positon_step = (staus->position[3])/PI/2;  
  //position = static_cast<int>(-positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[24] = 0x05;
  send_data[25] = 0x1e;
  send_data[26] = (uint8_t)(position&0xff);
  send_data[27] = (uint8_t)((position>>8)&0xff);
  send_data[28] = (uint8_t)((position>>16)&0xff);
  send_data[29] = (uint8_t)((position>>24)&0xff);
  //长度
  crc = com_crc(send_data,30);
  send_data[30] = (crc>>8)&0xff;
  send_data[31] = crc&0xff;   //尾
  #ifdef IA_USE_TCP
    stm_all.tcp_truct_t.tcp_write((char*)send_data,32); //发送tcp
  #else
    stm_all.udp_truct_t.udp_write((char*)send_data,32); //发送tcp
    Ia_Write_Log_To_File((char*)send_data,32);
  #endif
  #else
  uint32_t pos_init_data = 0x015800;
  float positon_step = (staus->position[0])/PI/2;  //弧度计算
  uint32_t position; //= static_cast<int>(positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[6] = 0x02;
  send_data[7] = 0x1e;
  send_data[8] = (uint8_t)(position&0xff);
  send_data[9] = (uint8_t)((position>>8)&0xff);
  send_data[10] = (uint8_t)((position>>16)&0xff);
  send_data[11] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x013000;
  positon_step = (staus->position[1])/PI/2;
  //position = static_cast<int>(-positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[12] = 0x03;
  send_data[13] = 0x1e;
  send_data[14] = (uint8_t)(position&0xff);
  send_data[15] = (uint8_t)((position>>8)&0xff);
  send_data[16] = (uint8_t)((position>>16)&0xff);
  send_data[17] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x010800;
  positon_step = (staus->position[2])/PI/2;    
  //position = static_cast<int>(positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[18] = 0x04;
  send_data[19] = 0x1e;
  send_data[20] = (uint8_t)(position&0xff);
  send_data[21] = (uint8_t)((position>>8)&0xff);
  send_data[22] = (uint8_t)((position>>16)&0xff);
  send_data[23] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x01e800;
  positon_step = (staus->position[3])/PI/2;  
  //position = static_cast<int>(-positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[24] = 0x05;
  send_data[25] = 0x1e;
  send_data[26] = (uint8_t)(position&0xff);
  send_data[27] = (uint8_t)((position>>8)&0xff);
  send_data[28] = (uint8_t)((position>>16)&0xff);
  send_data[29] = (uint8_t)((position>>24)&0xff);
  //长度
  crc = com_crc(send_data,30);
  send_data[30] = (crc>>8)&0xff;
  send_data[31] = crc&0xff;   //尾
  #ifdef IA_USE_TCP
    stm_all.tcp_truct_t.tcp_write((char*)send_data,32); //发送tcp
  #else
    stm_all.udp_truct_t.udp_write((char*)send_data,32); //发送tc
    Ia_Write_Log_To_File((char*)send_data,32);
  #endif
  #endif
}
#if 0
//接收机器人状态信息处理
void turn_on_robot::Cmd_Vel_Callback_Status(const sensor_msgs::msg::JointState::SharedPtr staus){
  //std::array<std::string, 4> wheel_joint_names{};
  //std::array<std::string, 4> axle_joint_names{};

  //auto wheel_joint_names = staus->name;
  //int size = wheel_joint_names.size();
  //RCLCPP_INFO(this->get_logger(),"Cmd_Vel_Callback_Status: %s %s %s %s ",wheel_joint_names[0],wheel_joint_names[1],wheel_joint_names[2],wheel_joint_names[3]);
  //RCLCPP_INFO(this->get_logger(),"Cmd_Vel_Callback_Status: %f %f %f %f ",staus->position[0],staus->position[1],staus->position[2],staus->position[3]);
  //RCLCPP_INFO(this->get_logger(),"Cmd_Vel: %f %f %f %f ",staus->velocity[0],staus->velocity[1],staus->velocity[2],staus->velocity[3]);

  uint8_t send_data[100];
  uint16_t speed_data;
  //头
  send_data[0] = 0x55;
  //发送轮毂指令
  send_data[1] = 0x02;
  //二级指令
  send_data[2] = 0x00;
  send_data[3] = 0x00;
  //长度
  send_data[4] = 0x00;
  send_data[5] = 0x1c;
  //数据
  //float round = PI*STM_DIAM/1000; //改为m
  float round = PI*2;
  float speed = -(staus->velocity[0])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[6] = 0x01;    //使能
  send_data[7] = 0x01;
  send_data[8] = (uint8_t)(speed_data/256);
  send_data[9] = (uint8_t)(speed_data);
  send_data[10] = 0x64;   //加速度
  speed = (staus->velocity[1])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[11] = 0x01;    //使能
  send_data[12] = 0x01;
  send_data[13] = (uint8_t)(speed_data/256);
  send_data[14] = (uint8_t)(speed_data);
  send_data[15] = 0x64;   //加速度
  speed = (staus->velocity[2])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[16] = 0x01;    //使能
  send_data[17] = 0x01;
  send_data[18] = (uint8_t)(speed_data/256);
  send_data[19] = (uint8_t)(speed_data);
  send_data[20] = 0x64;   //加速度
  speed = -(staus->velocity[3])*60/round;  //转换为m/s
  speed_data = static_cast<int>(speed);
  send_data[21] = 0x01;    //使能
  send_data[22] = 0x01;
  send_data[23] = (uint8_t)(speed_data/256);
  send_data[24] = (uint8_t)(speed_data);
  send_data[25] = 0x64;   //加速度

  send_data[26] = 0x00;
  send_data[27] = 0x00;   //尾
  for(int i=0;i<28;i++){
    printf("send_data[%d]:%x\n",i,send_data[i]);
  }
  stm_all.tcp_truct_t.tcp_write((char*)send_data,28); //发送tcp
  //
  //发送关节 TODO
  //发送轮毂指令
  send_data[1] = 0x01;
  //二级指令
  send_data[2] = 0x00;
  send_data[3] = 0x00;
  //长度
  send_data[4] = 0x00;
  send_data[5] = 0x1c;
  //数据
  uint32_t pos_init_data = 0x015800;
  float positon_step = (staus->position[0])/PI/2;  //弧度计算
  uint32_t position; //= static_cast<int>(positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[6] = 0x02;
  send_data[7] = 0x1e;
  send_data[8] = (uint8_t)(position&0xff);
  send_data[9] = (uint8_t)((position>>8)&0xff);
  send_data[10] = (uint8_t)((position>>16)&0xff);
  send_data[11] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x013000;
  positon_step = (PI-(staus->position[1]))/PI/2;
  //position = static_cast<int>(-positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data+positon_step*STM_ONE_CYCLE);
  send_data[12] = 0x03;
  send_data[13] = 0x1e;
  send_data[14] = (uint8_t)(position&0xff);
  send_data[15] = (uint8_t)((position>>8)&0xff);
  send_data[16] = (uint8_t)((position>>16)&0xff);
  send_data[17] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x010800;
  positon_step = (staus->position[2])/PI/2;    
  //position = static_cast<int>(positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[18] = 0x04;
  send_data[19] = 0x1e;
  send_data[20] = (uint8_t)(position&0xff);
  send_data[21] = (uint8_t)((position>>8)&0xff);
  send_data[22] = (uint8_t)((position>>16)&0xff);
  send_data[23] = (uint8_t)((position>>24)&0xff);
  pos_init_data = 0x01e800;
  positon_step = (PI+(staus->position[3]))/PI/2;  
  //position = static_cast<int>(-positon_step*STM_ONE_CYCLE+pos_init_data);
  position = static_cast<int>(pos_init_data-positon_step*STM_ONE_CYCLE);
  send_data[24] = 0x05;
  send_data[25] = 0x1e;
  send_data[26] = (uint8_t)(position&0xff);
  send_data[27] = (uint8_t)((position>>8)&0xff);
  send_data[28] = (uint8_t)((position>>16)&0xff);
  send_data[29] = (uint8_t)((position>>24)&0xff);
  //长度
  send_data[30] = 0x00;
  send_data[31] = 0x1c;
  stm_all.tcp_truct_t.tcp_write((char*)send_data,32); //发送tcp
}
#endif
//55 01 00 00 00 2c 02 1e 00 58 01 00 03 1e 00 30 01 00 04 1e 00 08 01 00 05 1e 00 e8 01 00 06 1e 00 6c 01 00 07 1e 00 8f 00 00 00 00
/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
功能: 发布IMU数据话题
***************************************/

void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub; //Instantiate IMU topic data //实例化IMU话题数据
  Imu_Data_Pub.header.stamp = rclcpp::Node::now(); 
  Imu_Data_Pub.header.frame_id = gyro_frame_id; //IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack 
                                                //IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; //A quaternion represents a three-axis attitude //四元数表达三轴姿态
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y; 
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
  Imu_Data_Pub.orientation_covariance[0] = 1e6; //Three-axis attitude covariance matrix //三轴姿态协方差矩阵
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; //Triaxial angular velocity //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; //Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; //Triaxial acceleration //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y; 
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;  
  imu_publisher->publish(Imu_Data_Pub); //Pub IMU topic //发布IMU话题
}

/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void turn_on_robot::Publish_Odom()
{
    nav_msgs::msg::Odometry odom; //Instance the odometer topic data //实例化里程计话题数据
    //Convert the Z-axis rotation Angle into a quaternion for expression 
    //把Z轴转角转换为四元数进行表达
    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);
    
    odom.header.stamp = rclcpp::Node::now(); ; 
    odom.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
    odom.pose.pose.position.x = Robot_Pos.X; //Position //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat; //Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

    odom.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标
    odom.twist.twist.linear.x =  Robot_Vel.X; //Speed in the X direction //X方向速度
    odom.twist.twist.linear.y =  Robot_Vel.Y; //Speed in the Y direction //Y方向速度
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.z = Robot_Vel.Z; //Angular velocity around the Z axis //绕Z轴角速度 

    //There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    if(Robot_Vel.X== 0&&Robot_Vel.Y== 0&&Robot_Vel.Z== 0)
      //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
      //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
      memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else
      //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
      //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
      memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));       
    odom_publisher->publish(odom); //Pub odometer topic //发布里程计话题
}

//发布机器人状态
void turn_on_robot::Publish_Robot_Status(char* Receive_Tcp_Data)
{
  sensor_msgs::msg::JointState robot_status;

  robot_status.velocity[0] = Odom_Trans(Receive_Tcp_Data[6],Receive_Tcp_Data[7]);
  robot_status.position[0] = Odom_Trans(Receive_Tcp_Data[8],Receive_Tcp_Data[9]);
  robot_status.velocity[1] = Odom_Trans(Receive_Tcp_Data[10],Receive_Tcp_Data[11]);
  robot_status.position[1] = Odom_Trans(Receive_Tcp_Data[12],Receive_Tcp_Data[13]);
  robot_status.velocity[2] = Odom_Trans(Receive_Tcp_Data[14],Receive_Tcp_Data[15]);
  robot_status.position[2] = Odom_Trans(Receive_Tcp_Data[16],Receive_Tcp_Data[17]);
  robot_status.velocity[3] = Odom_Trans(Receive_Tcp_Data[18],Receive_Tcp_Data[19]);
  robot_status.position[3] = Odom_Trans(Receive_Tcp_Data[20],Receive_Tcp_Data[21]);

  RCLCPP_INFO(this->get_logger(),"robot_status.velocity[0]: %f",robot_status.velocity[0]);
  Robot_speed_publisher->publish(robot_status);
}
/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
功能: 发布电压相关信息
***************************************/
void turn_on_robot::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs; //Define the data type of the power supply voltage publishing topic //定义电源电压发布话题的数据类型
    static float Count_Voltage_Pub=0;
    if(Count_Voltage_Pub++>10)
      {
        Count_Voltage_Pub=0;  
        voltage_msgs.data = Power_voltage; //The power supply voltage is obtained //电源供电的电压获取
        voltage_publisher->publish(voltage_msgs); //Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
      }
}

////////// 回充发布与回调 ////////
/**************************************
Date: January 17, 2022
Function: Pub the topic whether the robot finds the infrared signal (charging station)
功能: 发布机器人是否寻找到红外信号(充电桩)的话题
***************************************/
void turn_on_robot::Publish_RED()
{
    std_msgs::msg::UInt8 msg;
    msg.data=Red;
    RED_publisher->publish(msg); 

}
/**************************************
Date: January 14, 2022
Function: Publish a topic about whether the robot is charging
功能: 发布机器人是否在充电的话题
***************************************/
void turn_on_robot::Publish_Charging()
{
    static bool last_charging;
    std_msgs::msg::Bool msg;
    msg.data=Charging;
    Charging_publisher->publish(msg); 
    if(last_charging==false && Charging==true)cout<<GREEN<<"Robot is charging."<<endl<<RESET;
    if(last_charging==true && Charging==false)cout<<RED  <<"Robot charging has disconnected."<<endl<<RESET;
    last_charging=Charging;
}
/**************************************
Date: January 28, 2021
Function: Publish charging current information
功能: 发布充电电流信息
***************************************/
void turn_on_robot::Publish_ChargingCurrent()
{
    std_msgs::msg::Float32 msg; 
    msg.data=Charging_Current;
    Charging_current_publisher->publish(msg);
}

/**************************************
Date: March 1, 2022
Function: Infrared connection speed topic subscription Callback function, according to the subscription command through the serial port to set the infrared connection speed
功能: 红外对接速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令设置红外对接速度
***************************************/
void turn_on_robot::Red_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  short  transition;  //intermediate variable //中间变量

  Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B

  Send_Data.tx[1] = 3; //Infrared docking speed setting flag bit = 3 //红外对接速度设置标志位=3
  Send_Data.tx[2] = 0; //set aside //预留位

  //The target velocity of the X-axis of the robot
  //机器人x轴的目标线速度
  transition=0;
  transition = twist_aux->linear.x*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  //The target velocity of the Y-axis of the robot
  //机器人y轴的目标线速度
  transition=0;
  transition = twist_aux->linear.y*1000;
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition>>8;

  //The target angular velocity of the robot's Z axis
  //机器人z轴的目标角速度
  transition=0;
  transition = twist_aux->angular.z*1000;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition>>8;

  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);  //BCC check //BCC校验
  Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
 
  #ifdef IA_USE_TCP
    //stm_all.tcp_truct_t.tcp_write((char*)Send_Data.tx,sizeof (Send_Data.tx));
  #else
    //stm_all.udp_truct_t.udp_write((char*)Send_Data.tx,sizeof (Send_Data.tx));
  #endif 
}

/**************************************
Date: January 14, 2022
Function: Subscription robot recharge flag bit topic, used to tell the lower machine speed command is normal command or recharge command
功能: 订阅机器人是否回充标志位话题，用于告诉下位机速度命令是正常命令还是回充命令
***************************************/
void turn_on_robot::Recharge_Flag_Callback(const std_msgs::msg::Int8::SharedPtr Recharge_Flag)
{
  AutoRecharge=Recharge_Flag->data;
}

//服务
void turn_on_robot::Set_Charge_Callback(const shared_ptr<turtlesim::srv::Spawn::Request> req,shared_ptr<turtlesim::srv::Spawn::Response> res)
{
    Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //֡ͷ0X7B

    if(round(req->x)==1)
      Send_Data.tx[1] = 1;
    else if(round(req->x)==2)
      Send_Data.tx[1] = 2; 
    else if(round(req->x)==0)
      Send_Data.tx[1] = 0,AutoRecharge=0;

    Send_Data.tx[2] = 0; 
    Send_Data.tx[3] = 0;  
    Send_Data.tx[4] = 0;   
    Send_Data.tx[5] = 0;
    Send_Data.tx[6] = 0;
    Send_Data.tx[7] = 0;
    Send_Data.tx[8] = 0;
    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCCУ  λ      μ Check_Sum    
    Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //֡β0X7D
    
    #ifdef IA_USE_TCP
      //stm_all.tcp_truct_t.tcp_write((char*)Send_Data.tx,sizeof (Send_Data.tx));
    #else
      //stm_all.udp_truct_t.udp_write((char*)Send_Data.tx,sizeof (Send_Data.tx));
    #endif  

    if( Send_Data.tx[1]==0 )
    {
      if(charge_set_state==0)
        AutoRecharge=0,res->name = "true";
      else
        res->name = "false";
    }
    else
    {
      if(charge_set_state==1)
        res->name = "true";
      else
        res->name = "false";
    }
    return;
}
////////// 回充发布与回调 ////////

/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
     }
  }
  if(mode==1) //Send data mode //发送数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
     }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}

//自动回充专用校验位
unsigned char turn_on_robot::Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_AutoCharge_Data.rx[k]; //By bit or by bit //按位异或
    }
  }

  return check_sum;
}

//轮子数据处理
void turn_on_robot::Wheel_Data_Deal(char* data,int len){
  // RCLCPP_INFO(this->get_logger(),"CMD_TYPE_WHEEL_MOTOR:%d",len);

  /*for(int i=0;i<(len+8);i++){
    printf("data[%d]:%x\n",i,data[i]);
  }*/

  
  robot_wheel_status.velocity[0] = Odom_Trans(data[9],data[10])*160.0/60.0;
  robot_wheel_status.position[0] = 0;
  robot_wheel_status.velocity[1] = Odom_Trans(data[11],data[12])*160.0/60.0;
  robot_wheel_status.position[1] = 0;

  //RCLCPP_INFO(this->get_logger(),"robot_status.velocity[0]: %f",robot_status.velocity[0]);

  if(data[6]==0x11){
    Robot_wheel_left_publisher->publish(robot_wheel_status);
  }else{
    Robot_wheel_right_publisher->publish(robot_wheel_status);
  }
}

//关节数据处理
void turn_on_robot::Joint_Data_Deal(char* data,int len){
  // RCLCPP_INFO(this->get_logger(),"CMD_TYPE_JOINT_MOTOR",len);
  /*for(int i=0;i<(len+8);i++){
    printf("data[%d]:%x\n",i,data[i]);
  }
  */
  robot_joint_status.velocity[0] = 0.0;
  if(data[7]==0x08){
    robot_joint_status.position[0] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
  }else{
    robot_joint_status.position[0] = 0.0;
  }

// - Leg_front_left_3     前左三（舵轮）
// - Leg_front_right_3    前右三（舵轮）
// - Leg_back_left_2     后左二（舵轮）
// - Leg_back_right_2    后右二（舵轮）
// - CHEST1              胸
// - Leg_front_left_2    前左二（舵轮）
// - Leg_front_right_2   前右二（舵轮）
// - Leg_back_left_1    后左（轮子）
// - Leg_back_right_1   后右（轮子）
// - ARM0_LEFT          
// - ARM0_RIGHT
// - Leg_front_left_1    前左一（轮子）
// - Leg_front_right_1   前右一（轮子）
// - ARM1_LEFT
// - ARM1_RIGHT
// - ARM2_LEFT
// - ARM2_RIGHT
// - ARM3_LEFT
// - ARM3_RIGHT
/*
  robot_joint_status.velocity[data[6]-2] = 0.0;
  if(data[7]==0x08){
    robot_joint_status.position[data[6]-2] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
    joint_publish_flag |= 1<<(data[6]-2);
  }else{
    robot_joint_status.position[data[6]-] = 0.0;
  }
  
  if(robot_joint_status==JOINT_DATA_PUB){
    Robot_joint_2_publisher->publish(robot_joint_status);
    robot_joint_status = 0;
  }*/

  switch(data[6]){
    case 2:
      Robot_joint_2_publisher->publish(robot_joint_status);   //前左一
      break;
    case 3:
      Robot_joint_3_publisher->publish(robot_joint_status);   //前右一
      break;
    case 4:
      Robot_joint_4_publisher->publish(robot_joint_status);    //后左
      break;
    case 5:
      Robot_joint_5_publisher->publish(robot_joint_status);     //后右
      break;
    case 6:
      Robot_joint_6_publisher->publish(robot_joint_status);    //前左二
      break;
    case 7:
      Robot_joint_7_publisher->publish(robot_joint_status);    //前右二
      break;
    case 8:
      Robot_joint_8_publisher->publish(robot_joint_status);    //腰左
      break;
    case 9:
      Robot_joint_9_publisher->publish(robot_joint_status);    //腰右
      break;
    case 10:
      Robot_joint_10_publisher->publish(robot_joint_status);    //肩左
      break;
    case 11:
      Robot_joint_11_publisher->publish(robot_joint_status);    //肩右
      break; 
    case 12:
      Robot_joint_12_publisher->publish(robot_joint_status);    //肘左
      break;
    case 13:
      Robot_joint_13_publisher->publish(robot_joint_status);    //肘右
      break;
    case 14:
      Robot_joint_14_publisher->publish(robot_joint_status);     //腕左
      break;
    case 15:
      Robot_joint_15_publisher->publish(robot_joint_status);     //腕右
      break;         
  }
/*
  switch(data[6]){
    case 2:
      Robot_joint_2_publisher->publish(robot_joint_status);   //前左一
      break;
    case 3:
      Robot_joint_3_publisher->publish(robot_joint_status);   //前右一
      break;
    case 4:
      Robot_joint_4_publisher->publish(robot_joint_status);    //后左
      break;
    case 5:
      Robot_joint_5_publisher->publish(robot_joint_status);     //后右
      break;
    case 6:
      Robot_joint_6_publisher->publish(robot_joint_status);    //前左二
      break;
    case 7:
      Robot_joint_7_publisher->publish(robot_joint_status);    //前右二
      break;
    case 8:
      Robot_joint_8_publisher->publish(robot_joint_status);    //腰左
      break;
    case 9:
      Robot_joint_9_publisher->publish(robot_joint_status);    //腰右
      break;
    case 10:
      Robot_joint_10_publisher->publish(robot_joint_status);    //肩左
      break;
    case 11:
      Robot_joint_11_publisher->publish(robot_joint_status);    //肩右
      break; 
    case 12:
      Robot_joint_12_publisher->publish(robot_joint_status);    //肘左
      break;
    case 13:
      Robot_joint_13_publisher->publish(robot_joint_status);    //肘右
      break;
    case 14:
      Robot_joint_14_publisher->publish(robot_joint_status);     //腕左
      break;
    case 15:
      Robot_joint_15_publisher->publish(robot_joint_status);     //腕右
      break;         
  }*/
}

//超声数据处理
void turn_on_robot::Sound_Data_Deal(char* data,int len){
  // 定义传感器信息结构体，存储数据偏移量、发布者和对应frame_id
  struct SensorInfo {
    int data_offset;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher;
    std::string frame_id;  // 添加frame_id字段
  };

  // 创建传感器信息数组，为每个传感器指定对应的frame_id
  std::vector<SensorInfo> sensors = {
    {6, ultrasonic_front_right_forward_topic, "ultrasonic_front_right_forward_link"},  // 前右前
    {8, ultrasonic_front_right_side_topic, "ultrasonic_front_right_side_link"},        // 前右侧
    {10, ultrasonic_rear_right_forward_topic, "ultrasonic_rear_right_forward_link"},   // 后右前
    {12, ultrasonic_rear_right_side_topic, "ultrasonic_rear_right_side_link"},         // 后右侧
    {14, ultrasonic_rear_left_forward_topic, "ultrasonic_rear_left_forward_link"},     // 后左前
    {16, ultrasonic_rear_left_side_topic, "ultrasonic_rear_left_side_link"},           // 后左侧
    {18, ultrasonic_front_left_side_topic, "ultrasonic_front_left_side_link"},         // 前左侧
    {20, ultrasonic_front_left_forward_topic, "ultrasonic_front_left_forward_link"}    // 前左前
  };

  sensor_msgs::msg::Range send_range;
  // 设置通用参数
  send_range.header.stamp = this->get_clock()->now();
  send_range.min_range = 0.05;
  send_range.max_range = 3.0;

  // 遍历所有传感器，设置frame_id并发布数据
  for (const auto& sensor : sensors) {
    send_range.header.frame_id = sensor.frame_id;  // 设置当前传感器的frame_id
    send_range.range = Odom_Trans(data[sensor.data_offset], data[sensor.data_offset + 1]);
    sensor.publisher->publish(send_range);
  }

  /*RCLCPP_INFO(this->get_logger(),"all_data:%dmm ; %dmm ; %dmm ; %dmm ; %dmm ; %dmm ; %dmm ; %dmm",
              (data[6]<<8)+data[7],(data[8]<<8)+data[9],(data[10]<<8)+data[11],(data[12]<<8)+data[13],
               (data[14]<<8)+data[15],(data[16]<<8)+data[17],(data[18]<<8)+data[19],(data[20]<<8)+data[21]);
  */
}



/**************************************
功能: 信息处理
********************************************/
Tcp_Recv_Data_t Tcp_Recv_Data;
void turn_on_robot::Tcp_Recv_Data_Deal(char* tcp_data,int len){
  int index = 0;
  /*if((Tcp_Recv_Data.length+len)<=TCP_DATA_LEN){      //在范围内
    memcpy(&Tcp_Recv_Data.buf[Tcp_Recv_Data.length],tcp_data,len);
    Tcp_Recv_Data.length+=len;
  }else{
    Tcp_Recv_Data.length = 0;
  }*/
  while((index+8)<=len){   //偏移判断
    if(tcp_data[index]==0xa5){
      Tcp_Cmd_Data_Deal(&tcp_data[index],tcp_data[index+5]);
      index+=(tcp_data[index+5]+8);   //下一条指令
    }else{
      index++;                        //找下一个指令
    }
  }
  /*
  while(res){
    if((Tcp_Recv_Data.buf[index]==0xa5)&&((Tcp_Recv_Data.buf[index+5]+index+8)<=Tcp_Recv_Data.length)){ //检测头部 和长度

    }else{
      Tcp_Recv_Data.length = 0;
    }
  }*/
  
}
int joint_publish_flag = 0;
void turn_on_robot::Robot_Data_Deal(char* data,int len){
 
  if(data[1]==CMD_TYPE_JOINT_MOTOR){
    //2- Leg_front_left_2    前左二（舵轮）
    //3- Leg_front_right_2   前右二（舵轮）
    //4- Leg_back_left_2     后左二（舵轮）
    //5- Leg_back_right_2    后右二（舵轮
    //6- Leg_front_left_3     前左三（舵轮）
    //7- Leg_front_right_3    前右三（舵轮）
    //8.9- CHEST1               胸
    // 10- ARM0_LEFT          
    // 11- ARM0_RIGHT
    // 12- ARM1_LEFT
    // 13- ARM1_RIGHT
    // 14- ARM2_LEFT
    // 15- ARM2_RIGHT
    // 16- ARM3_LEFT
    // 17- ARM3_RIGHT

    // 18- Leg_front_left_2    前左二（舵轮）
    // 19- Leg_front_right_2   前右二（舵轮）
    // 20- Leg_back_left_2     后左二（舵轮）
    // 21- Leg_back_right_2    后右二（舵轮）

    
      /*robot_joint_status.velocity[data[6]-2] = 0.0;
      if(data[7]==0x08){
        robot_joint_status.position[data[6]-2] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
        joint_publish_flag |= 1<<(data[6]-2);
      }else{
        robot_joint_status.position[data[6]-] = 0.0;
      }*/
      
      switch(data[6]){
        case 2:                                                 //前左一
          robot_joint_status.velocity[0] = 0.0;
          robot_joint_status.position[0] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x01;
          break;
        case 3:
          robot_joint_status.velocity[1] = 0.0;
          robot_joint_status.position[1] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x02;
          break;
        case 4:
          robot_joint_status.velocity[2] = 0.0;
          robot_joint_status.position[2] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x04;
          break;
        case 5:
          robot_joint_status.velocity[3] = 0.0;
          robot_joint_status.position[3] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x08;
          break;
        case 6:
          robot_joint_status.velocity[4] = 0.0;
          robot_joint_status.position[4] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x10;
          break;
        case 7:
          robot_joint_status.velocity[5] = 0.0;
          robot_joint_status.position[5] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x20;
          break;
        case 8:
          robot_joint_status.velocity[6] = 0.0;
          robot_joint_status.position[6] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x40;
          break;
        case 9:
          joint_publish_flag |=0x80;
          break;
        case 10:
          robot_joint_status.velocity[7] = 0.0;
          robot_joint_status.position[7] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x100;
          break;
        case 11:
          robot_joint_status.velocity[8] = 0.0;
          robot_joint_status.position[8] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x200;
          break; 
        case 12:
          robot_joint_status.velocity[9] = 0.0;
          robot_joint_status.position[9] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x400;
          break;
        case 13:
          robot_joint_status.velocity[10] = 0.0;
          robot_joint_status.position[10] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x800;
          break;
        case 14:
          robot_joint_status.velocity[11] = 0.0;
          robot_joint_status.position[11] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x1000;
          break;
        case 15:
          robot_joint_status.velocity[12] = 0.0;
          robot_joint_status.position[12] = (float)((data[11]<<24)+(data[10]<<16)+(data[9]<<8)+data[8])/262144.0*3.1415*2;
          joint_publish_flag |=0x2000;
          break;         
      }
  }else{
    
    if(data[6]==0x11){
      robot_joint_status.velocity[15] = WHEEL_FRONT_LEFT*Odom_Trans(data[9],data[10])*160.0/60.0;
      robot_joint_status.position[15] = 0;
      robot_joint_status.velocity[17] = WHEEL_REAR_LEFT*Odom_Trans(data[9],data[10])*160.0/60.0;
      robot_joint_status.position[17] = 0;
      
      joint_publish_flag |=0x30000;
    }else{
      robot_joint_status.velocity[16] = WHEEL_FRONT_RIGHT*Odom_Trans(data[11],data[12])*160.0/60.0;
      robot_joint_status.position[16] = 0;
      robot_joint_status.velocity[18] = WHEEL_REAR_RIGTH*Odom_Trans(data[11],data[12])*160.0/60.0;
      robot_joint_status.position[18] = 0;
      joint_publish_flag |=0xc0000;
    }
  }

  //RCLCPP_INFO(this->get_logger(),"joint_publish_flag:%x\n",joint_publish_flag);
  if(joint_publish_flag==JOINT_DATA_PUB){
      robot_state_publisher->publish(robot_joint_status);
      joint_publish_flag = 0;
  }
}
/**************************************
功能: 信息处理
********************************************/
void turn_on_robot::Tcp_Cmd_Data_Deal(char* Receive_Tcp_Data,int num){
  switch(Receive_Tcp_Data[1]){
      /*case CMD_TYPE_JOINT_MOTOR:
        //RCLCPP_INFO(this->get_logger(),"CMD_TYPE_JOINT_MOTOR");
        Joint_Data_Deal(Receive_Tcp_Data,num);
        break;
      case CMD_TYPE_WHEEL_MOTOR:
        //RCLCPP_INFO(this->get_logger(),"CMD_TYPE_WHEEL_MOTOR");
        Wheel_Data_Deal(Receive_Tcp_Data,num);
        break;*/
      case CMD_TYPE_JOINT_MOTOR: 
      case CMD_TYPE_WHEEL_MOTOR:
        Robot_Data_Deal(Receive_Tcp_Data,num);
        break; 
      case CMD_TYPE_SOUND_RANGE:
        //RCLCPP_INFO(this->get_logger(),"CMD_TYPE_SOUND_RANGE");
        Sound_Data_Deal(Receive_Tcp_Data,num);
        break;  
      case CMD_TYPE_IMU_DATA: //imu指令
        //sub_cmd = (Receive_Tcp_Data[2]<<8 + Receive_Tcp_Data[3]);//二级指令解析
        if(Receive_Tcp_Data[3]){
          //Robot_speed_publisher
          //Publish_Robot_Status(Receive_Tcp_Data);
          
          robot_status.velocity[0] = Odom_Trans(Receive_Tcp_Data[6],Receive_Tcp_Data[7]);
          robot_status.position[0] = Odom_Trans(Receive_Tcp_Data[8],Receive_Tcp_Data[9])/180*PI;
          robot_status.velocity[1] = Odom_Trans(Receive_Tcp_Data[10],Receive_Tcp_Data[11]);
          robot_status.position[1] = Odom_Trans(Receive_Tcp_Data[12],Receive_Tcp_Data[13])/180*PI;
          robot_status.velocity[2] = Odom_Trans(Receive_Tcp_Data[14],Receive_Tcp_Data[15]);
          robot_status.position[2] = Odom_Trans(Receive_Tcp_Data[16],Receive_Tcp_Data[17])/180*PI;
          robot_status.velocity[3] = Odom_Trans(Receive_Tcp_Data[18],Receive_Tcp_Data[19]);
          robot_status.position[3] = Odom_Trans(Receive_Tcp_Data[20],Receive_Tcp_Data[21])/180*PI;

          //RCLCPP_INFO(this->get_logger(),"robot_status.velocity[0]: %f",robot_status.velocity[0]);
          Robot_speed_publisher->publish(robot_status);
        }else{
          Receive_Data.Flag_Stop= 0x00; //set aside //预留位
          Robot_Vel.X = Odom_Trans(Receive_Tcp_Data[6],Receive_Tcp_Data[7]); //Get the speed of the moving chassis in the X direction //获取运动底盘X方向速度
            
          Robot_Vel.Y = Odom_Trans(Receive_Tcp_Data[8],Receive_Tcp_Data[9]); //Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the omnidirectional mobile robot chassis
                                                                            //获取运动底盘Y方向速度，Y速度仅在全向移动机器人底盘有效
          Robot_Vel.Z = Odom_Trans(Receive_Tcp_Data[10],Receive_Tcp_Data[11]); //Get the speed of the moving chassis in the Z direction //获取运动底盘Z方向速度   
            
          //MPU6050 stands for IMU only and does not refer to a specific model. It can be either MPU6050 or MPU9250
          //Mpu6050仅代表IMU，不指代特定型号，既可以是MPU6050也可以是MPU9250
          Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Tcp_Data[12],Receive_Tcp_Data[13]);   //Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度  
          Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Tcp_Data[14],Receive_Tcp_Data[15]); //Get the Y-axis acceleration of the IMU     //获取IMU的Y轴加速度
          Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Tcp_Data[16],Receive_Tcp_Data[17]); //Get the Z-axis acceleration of the IMU     //获取IMU的Z轴加速度
          Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Tcp_Data[18],Receive_Tcp_Data[19]);  //Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度  
          Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Tcp_Data[20],Receive_Tcp_Data[21]);  //Get the Y-axis angular velocity of the IMU //获取IMU的Y轴角速度  
          Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Tcp_Data[22],Receive_Tcp_Data[23]);  //Get the Z-axis angular velocity of the IMU //获取IMU的Z轴角速度  
          //Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2
          //线性加速度单位转化，和STM32的IMU初始化的时候的量程有关,这里量程±2g=19.6m/s^2
          Mpu6050.linear_acceleration.x = (float)((float)(Mpu6050_Data.accele_x_data)/32768.0*16*9.8);
          Mpu6050.linear_acceleration.y = (float)((float)(Mpu6050_Data.accele_y_data)/32768.0*16*9.8);
          Mpu6050.linear_acceleration.z = (float)((float)(Mpu6050_Data.accele_z_data)/32768.0*16*9.8);
          //The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s
          //Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
          //陀螺仪单位转化，和STM32的IMU初始化的时候的量程有关，这里IMU的陀螺仪的量程是±500°/s
          //因为机器人一般Z轴速度不快，降低量程可以提高精度
          Mpu6050.angular_velocity.x =  (float)((float)(Mpu6050_Data.gyros_x_data)/32768.0*2000.0*3.14159/180.0);
          Mpu6050.angular_velocity.y =  (float)((float)(Mpu6050_Data.gyros_y_data)/32768.0*2000.0*3.14159/180.0);
          Mpu6050.angular_velocity.z =  (float)((float)(Mpu6050_Data.gyros_z_data)/32768.0*2000.0*3.14159/180.0);
          
          if(Mpu6050.linear_acceleration.x<0.02){
            Mpu6050.linear_acceleration.x=0.0;
          }
          if(Mpu6050.linear_acceleration.y<0.02){
            Mpu6050.linear_acceleration.y=0.0;
          }
          if(Mpu6050.linear_acceleration.z<0.02){
            Mpu6050.linear_acceleration.z=0.0;
          }

          if(Mpu6050.angular_velocity.x<0.02){
            Mpu6050.angular_velocity.x=0.0;
          }
          if(Mpu6050.angular_velocity.y<0.02){
            Mpu6050.angular_velocity.y=0.0;
          }
          if(Mpu6050.angular_velocity.z<0.02){
            Mpu6050.angular_velocity.z=0.0;
          }
          
          //RCLCPP_INFO(this->get_logger(),"Receive_Tcp_Data: %x and %x",Receive_Tcp_Data[12],Receive_Tcp_Data[13]);
          //RCLCPP_INFO(this->get_logger(),"Mpu6050_Data.accele_x_data: %x",Mpu6050_Data.accele_x_data);

          RCLCPP_INFO(this->get_logger(),"Mpu6050.linear_acceleration.x: %f",Mpu6050.linear_acceleration.x);
          RCLCPP_INFO(this->get_logger(),"Mpu6050.linear_acceleration.y: %f",Mpu6050.linear_acceleration.y);
          RCLCPP_INFO(this->get_logger(),"Mpu6050.linear_acceleration.z: %f",Mpu6050.linear_acceleration.z);
          RCLCPP_INFO(this->get_logger(),"Mpu6050.angular_velocity.x: %f",Mpu6050.angular_velocity.x);
          RCLCPP_INFO(this->get_logger(),"Mpu6050.angular_velocity.y: %f",Mpu6050.angular_velocity.y);
          RCLCPP_INFO(this->get_logger(),"Mpu6050.angular_velocity.z : %f",Mpu6050.angular_velocity.z);
          //Get the battery voltage
          //获取电池电压 TODO 电池电压放在两外一条指令里面
          /*transition_16 = 0;
          transition_16 |=  Receive_Data.rx[20]<<8;
          transition_16 |=  Receive_Data.rx[21];  
          Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001; //Unit conversion millivolt(mv)->volt(v) //单位转换毫伏(mv)->伏(v)
          */ 
        } 
        break;
      default:
        break;  

    }
}

/**************************************
功能: 读取tcp信息并处理 songkan
********************************************/
//sensor_msgs::msg::JointState::SharedPtr robot_status;
bool turn_on_robot::Get_Low_Computer_Data()
{
  //char ring_buffer[1024];   //缓存区
  char Receive_Tcp_Data[256];
  #ifdef IA_USE_TCP
    int num = stm_all.tcp_truct_t.tcp_read((char*)Receive_Tcp_Data,sizeof(Receive_Tcp_Data));
  #else
    int num = stm_all.udp_truct_t.udp_read((char*)Receive_Tcp_Data,sizeof(Receive_Tcp_Data));
  #endif  
  // RCLCPP_INFO(this->get_logger(),"recv: %d",num);
  int sub_cmd = 0;

  Tcp_Recv_Data_Deal(Receive_Tcp_Data,num);
  #if 0
  if(num>8){
    switch(Receive_Tcp_Data[1]){
      case CMD_TYPE_JOINT_MOTOR:
        //RCLCPP_INFO(this->get_logger(),"CMD_TYPE_JOINT_MOTOR");
        
        break;
      case CMD_TYPE_WHEEL_MOTOR:
        //RCLCPP_INFO(this->get_logger(),"CMD_TYPE_WHEEL_MOTOR");
        break;
      case CMD_TYPE_SOUND_RANGE:
        //RCLCPP_INFO(this->get_logger(),"CMD_TYPE_SOUND_RANGE");
        Sound_Data_Deal(Receive_Tcp_Data,num);
        break;  
      case CMD_TYPE_IMU_DATA: //imu指令
        //sub_cmd = (Receive_Tcp_Data[2]<<8 + Receive_Tcp_Data[3]);//二级指令解析
        if(Receive_Tcp_Data[3]){
          //Robot_speed_publisher
          //Publish_Robot_Status(Receive_Tcp_Data);
          
          robot_status.velocity[0] = Odom_Trans(Receive_Tcp_Data[6],Receive_Tcp_Data[7]);
          robot_status.position[0] = Odom_Trans(Receive_Tcp_Data[8],Receive_Tcp_Data[9])/180*PI;
          robot_status.velocity[1] = Odom_Trans(Receive_Tcp_Data[10],Receive_Tcp_Data[11]);
          robot_status.position[1] = Odom_Trans(Receive_Tcp_Data[12],Receive_Tcp_Data[13])/180*PI;
          robot_status.velocity[2] = Odom_Trans(Receive_Tcp_Data[14],Receive_Tcp_Data[15]);
          robot_status.position[2] = Odom_Trans(Receive_Tcp_Data[16],Receive_Tcp_Data[17])/180*PI;
          robot_status.velocity[3] = Odom_Trans(Receive_Tcp_Data[18],Receive_Tcp_Data[19]);
          robot_status.position[3] = Odom_Trans(Receive_Tcp_Data[20],Receive_Tcp_Data[21])/180*PI;

          //RCLCPP_INFO(this->get_logger(),"robot_status.velocity[0]: %f",robot_status.velocity[0]);
          Robot_speed_publisher->publish(robot_status);
        }else{
          Receive_Data.Flag_Stop= 0x00; //set aside //预留位
          Robot_Vel.X = Odom_Trans(Receive_Tcp_Data[6],Receive_Tcp_Data[7]); //Get the speed of the moving chassis in the X direction //获取运动底盘X方向速度
            
          Robot_Vel.Y = Odom_Trans(Receive_Tcp_Data[8],Receive_Tcp_Data[9]); //Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the omnidirectional mobile robot chassis
                                                                            //获取运动底盘Y方向速度，Y速度仅在全向移动机器人底盘有效
          Robot_Vel.Z = Odom_Trans(Receive_Tcp_Data[10],Receive_Tcp_Data[11]); //Get the speed of the moving chassis in the Z direction //获取运动底盘Z方向速度   
            
          //MPU6050 stands for IMU only and does not refer to a specific model. It can be either MPU6050 or MPU9250
          //Mpu6050仅代表IMU，不指代特定型号，既可以是MPU6050也可以是MPU9250
          Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Tcp_Data[12],Receive_Tcp_Data[13]);   //Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度  
          Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Tcp_Data[14],Receive_Tcp_Data[15]); //Get the Y-axis acceleration of the IMU     //获取IMU的Y轴加速度
          Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Tcp_Data[16],Receive_Tcp_Data[17]); //Get the Z-axis acceleration of the IMU     //获取IMU的Z轴加速度
          Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Tcp_Data[18],Receive_Tcp_Data[19]);  //Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度  
          Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Tcp_Data[20],Receive_Tcp_Data[21]);  //Get the Y-axis angular velocity of the IMU //获取IMU的Y轴角速度  
          Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Tcp_Data[22],Receive_Tcp_Data[23]);  //Get the Z-axis angular velocity of the IMU //获取IMU的Z轴角速度  
          //Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2
          //线性加速度单位转化，和STM32的IMU初始化的时候的量程有关,这里量程±2g=19.6m/s^2
          Mpu6050.linear_acceleration.x = (float)((float)(Mpu6050_Data.accele_x_data)/32768.0*16*9.8);
          Mpu6050.linear_acceleration.y = (float)((float)(Mpu6050_Data.accele_y_data)/32768.0*16*9.8);
          Mpu6050.linear_acceleration.z = (float)((float)(Mpu6050_Data.accele_z_data)/32768.0*16*9.8);
          //The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s
          //Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
          //陀螺仪单位转化，和STM32的IMU初始化的时候的量程有关，这里IMU的陀螺仪的量程是±500°/s
          //因为机器人一般Z轴速度不快，降低量程可以提高精度
          Mpu6050.angular_velocity.x =  (float)((float)(Mpu6050_Data.gyros_x_data)/32768.0*2000.0*3.14159/180.0);
          Mpu6050.angular_velocity.y =  (float)((float)(Mpu6050_Data.gyros_y_data)/32768.0*2000.0*3.14159/180.0);
          Mpu6050.angular_velocity.z =  (float)((float)(Mpu6050_Data.gyros_z_data)/32768.0*2000.0*3.14159/180.0);
          
          if(Mpu6050.linear_acceleration.x<0.02){
            Mpu6050.linear_acceleration.x=0.0;
          }
          if(Mpu6050.linear_acceleration.y<0.02){
            Mpu6050.linear_acceleration.y=0.0;
          }
          if(Mpu6050.linear_acceleration.z<0.02){
            Mpu6050.linear_acceleration.z=0.0;
          }

          if(Mpu6050.angular_velocity.x<0.02){
            Mpu6050.angular_velocity.x=0.0;
          }
          if(Mpu6050.angular_velocity.y<0.02){
            Mpu6050.angular_velocity.y=0.0;
          }
          if(Mpu6050.angular_velocity.z<0.02){
            Mpu6050.angular_velocity.z=0.0;
          }
          
          //RCLCPP_INFO(this->get_logger(),"Receive_Tcp_Data: %x and %x",Receive_Tcp_Data[12],Receive_Tcp_Data[13]);
          //RCLCPP_INFO(this->get_logger(),"Mpu6050_Data.accele_x_data: %x",Mpu6050_Data.accele_x_data);

          // RCLCPP_INFO(this->get_logger(),"Mpu6050.linear_acceleration.x: %f",Mpu6050.linear_acceleration.x);
          // RCLCPP_INFO(this->get_logger(),"Mpu6050.linear_acceleration.y: %f",Mpu6050.linear_acceleration.y);
          // RCLCPP_INFO(this->get_logger(),"Mpu6050.linear_acceleration.z: %f",Mpu6050.linear_acceleration.z);
          // RCLCPP_INFO(this->get_logger(),"Mpu6050.angular_velocity.x: %f",Mpu6050.angular_velocity.x);
          // RCLCPP_INFO(this->get_logger(),"Mpu6050.angular_velocity.y: %f",Mpu6050.angular_velocity.y);
          // RCLCPP_INFO(this->get_logger(),"Mpu6050.angular_velocity.z : %f",Mpu6050.angular_velocity.z);
          //Get the battery voltage
          //获取电池电压 TODO 电池电压放在两外一条指令里面
          /*transition_16 = 0;
          transition_16 |=  Receive_Data.rx[20]<<8;
          transition_16 |=  Receive_Data.rx[21];  
          Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001; //Unit conversion millivolt(mv)->volt(v) //单位转换毫伏(mv)->伏(v)
          */ 
        } 
        break;
      default:
        break;  

    }
    return true;
  }
  #endif
  return false;
}

/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void turn_on_robot::Control()
{
  //_Last_Time = ros::Time::now();
  _Last_Time = rclcpp::Node::now();
  while(rclcpp::ok())
  {
    try
    {
      //_Now = ros::Time::now();
      _Now = rclcpp::Node::now();
      Sampling_Time = (_Now - _Last_Time).seconds();  //Retrieves time interval, which is used to integrate velocity to obtain displacement (mileage) 
                                                  //获取时间间隔，用于积分速度获得位移(里程) 
      if (true == Get_Low_Computer_Data()) //The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
                                    //通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
      {
        //Odometer error correction //里程计误差修正
        Robot_Vel.X = Robot_Vel.X*odom_x_scale;
        Robot_Vel.Y = Robot_Vel.Y*odom_y_scale;
        if( Robot_Vel.Z>=0 )
          Robot_Vel.Z = Robot_Vel.Z*odom_z_scale_positive;
        else
          Robot_Vel.Z = Robot_Vel.Z*odom_z_scale_negative;

        Robot_Pos.X+=(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time; //Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
        Robot_Pos.Y+=(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time; //Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m
        Robot_Pos.Z+=Robot_Vel.Z * Sampling_Time; //The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad 

        //Calculate the three-axis attitude from the IMU with the angular velocity around the three-axis and the three-axis acceleration
        //通过IMU绕三轴角速度与三轴加速度计算三轴姿态
        Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,\
                  Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);

        Publish_Odom();      //Pub the speedometer topic //发布里程计话题
        Publish_ImuSensor(); //Pub the IMU topic //发布IMU话题    TODO 会导致段错误
        //Publish_Voltage();   //Pub the topic of power supply voltage //发布电源电压话题

        _Last_Time = _Now; //Record the time and use it to calculate the time interval //记录时间，用于计算时间间隔
    }
    
    //自动回充数据话题
    if(check_AutoCharge_data)
    {
      Publish_Charging();  //Pub a topic about whether the robot is charging //发布机器人是否在充电的话题
      Publish_RED();       //Pub the topic whether the robot finds the infrared signal (charging station) //发布机器人是否寻找到红外信号(充电桩)的话题
      Publish_ChargingCurrent(); //Pub the charging current topic //发布充电电流话题
      check_AutoCharge_data = false;
    }

    //rclcpp::spin_some(this->get_node_base_interface());   //The loop waits for the callback function //循环等待回调函数多线程中不能打开
  }
  catch (const rclcpp::exceptions::RCLError & e )
  {
    RCLCPP_ERROR(this->get_logger(),"unexpectedly failed whith %s",e.what()); 
  }
}
}

void turn_on_robot::func1Thread(){
  Sampling_Time=0;
  Power_voltage=0;
  //Clear the data
  //清空数据
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  //ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
  //The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  //private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  
  this->declare_parameter<int>("serial_baud_rate");
  this->declare_parameter<std::string>("usart_port_name", "/dev/wheeltec_controller");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
  this->declare_parameter<double>("odom_x_scale");
  this->declare_parameter<double>("odom_y_scale");
  this->declare_parameter<double>("odom_z_scale_positive");
  this->declare_parameter<double>("odom_z_scale_negative");

  this->get_parameter("serial_baud_rate", serial_baud_rate);//Communicate baud rate 115200 to the lower machine //和下位机通信波特率115200
  this->get_parameter("usart_port_name", usart_port_name);//Fixed serial port number //固定串口号
  this->get_parameter("odom_frame_id", odom_frame_id);//The odometer topic corresponds to the parent TF coordinate //里程计话题对应父TF坐标
  this->get_parameter("robot_frame_id", robot_frame_id);//The odometer topic corresponds to sub-TF coordinates //里程计话题对应子TF坐标
  this->get_parameter("gyro_frame_id", gyro_frame_id);//IMU topics correspond to TF coordinates //IMU话题对应TF坐标
  this->get_parameter("odom_x_scale", odom_x_scale);
  this->get_parameter("odom_y_scale", odom_y_scale);
  this->get_parameter("odom_z_scale_positive", odom_z_scale_positive);
  this->get_parameter("odom_z_scale_negative", odom_z_scale_negative);

  //odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);//Create the odometer topic publisher //创建里程计话题发布者
  //imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10); //Create an IMU topic publisher //创建IMU话题发布者
 // voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);//Create a battery-voltage topic publisher //创建电池电压话题发布者

  //回充发布者
  Charging_publisher = create_publisher<std_msgs::msg::Bool>("robot_charging_flag", 10);
  Charging_current_publisher = create_publisher<std_msgs::msg::Float32>("robot_charging_current", 10);
  RED_publisher = create_publisher<std_msgs::msg::UInt8>("robot_red_flag", 10);

  //回充订阅者
  Red_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "red_vel", 10, std::bind(&turn_on_robot::Red_Vel_Callback, this, std::placeholders::_1));
  Recharge_Flag_Sub = create_subscription<std_msgs::msg::Int8>(
      "robot_recharge_flag", 10, std::bind(&turn_on_robot::Recharge_Flag_Callback, this,std::placeholders::_1));
  //回充服务提供
  SetCharge_Service=this->create_service<turtlesim::srv::Spawn>\
  ("/set_charge",std::bind(&turn_on_robot::Set_Charge_Callback,this, \ 
    std::placeholders::_1 ,std::placeholders::_2));
    
  //Set the velocity control command callback function
  //速度控制命令订阅回调函数设置
  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 2, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));

  Cmd_Vel_New = create_subscription<stm_msg::msg::Tcpdata>(
      "cmd_vel_new", 2, std::bind(&turn_on_robot::Cmd_Vel_Callback_New, this, _1));

  Cmd_Motor_Status = create_subscription<sensor_msgs::msg::JointState>(
       "robot/joint_command", 2, std::bind(&turn_on_robot::Cmd_Vel_Callback_Status, this, _1));

  Ia_Motor_Status = create_subscription<sensor_msgs::msg::JointState>(
       "joint_commands", 2, std::bind(&turn_on_robot::Ia_Joint_Commands_Callback_Status, this, _1));

  RCLCPP_INFO(this->get_logger(),"wheeltec_robot Data ready"); //Prompt message //提示信息

  while (rclcpp::ok()){
  }
}

void turn_on_robot::func2Thread(){
  RCLCPP_INFO(this->get_logger(),"func2Thready");

  //TODO 不能放在第一个线程 会导致发布段错误 因为定义在另一个线程中
  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom_orig", 10);//Create the odometer topic publisher //创建里程计话题发布者
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10); //Create an IMU topic publisher //创建IMU话题发布者
  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);//Create a battery-voltage topic publisher //创建电池电压话题发布者
  
  ultrasonic_front_left_forward_topic = create_publisher<sensor_msgs::msg::Range>("/ultrasonic/front_left_forward", 10);
  ultrasonic_front_left_side_topic = create_publisher<sensor_msgs::msg::Range>("/ultrasonic/front_left_side", 10);
  ultrasonic_front_right_forward_topic = create_publisher<sensor_msgs::msg::Range>("/ultrasonic/front_right_forward", 10);
  ultrasonic_front_right_side_topic = create_publisher<sensor_msgs::msg::Range>("/ultrasonic/front_right_side", 10);
  ultrasonic_rear_left_forward_topic = create_publisher<sensor_msgs::msg::Range>("/ultrasonic/rear_left_forward", 10);
  ultrasonic_rear_left_side_topic = create_publisher<sensor_msgs::msg::Range>("/ultrasonic/rear_left_side", 10);
  ultrasonic_rear_right_forward_topic = create_publisher<sensor_msgs::msg::Range>("/ultrasonic/rear_right_forward", 10);
  ultrasonic_rear_right_side_topic = create_publisher<sensor_msgs::msg::Range>("/ultrasonic/rear_right_side", 10);
  
  //小车状态发布者
  Robot_speed_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_speed_angle_status", 10);
  robot_status.header.stamp = rclcpp::Node::now();
  robot_status.name = {"joint1","joint2","joint3","joint4"};
  robot_status.position = {0.0, 0.0, 0.0, 0.0}; 
  robot_status.velocity = {0.0, 0.0, 0.0, 0.0}; 

  Robot_wheel_left_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_wheel_left_status", 10);
  Robot_wheel_right_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_wheel_right_status", 10);
  robot_wheel_status.header.stamp = rclcpp::Node::now();
  robot_wheel_status.name = {"front","rear"};
  robot_wheel_status.position = {0.0, 0.0}; 
  robot_wheel_status.velocity = {0.0, 0.0};

  robot_joint_publisher = create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
  Robot_joint_2_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_2_status", 10);
  Robot_joint_3_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_3_status", 10);
  Robot_joint_4_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_4_status", 10);
  Robot_joint_5_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_5_status", 10);
  Robot_joint_6_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_6_status", 10);
  Robot_joint_7_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_7_status", 10);
  Robot_joint_8_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_8_status", 10);
  Robot_joint_9_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_9_status", 10);
  Robot_joint_10_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_10_status", 10);
  Robot_joint_11_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_11_status", 10);
  Robot_joint_12_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_12_status", 10);
  Robot_joint_13_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_13_status", 10);
  Robot_joint_14_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_14_status", 10);
  Robot_joint_15_publisher = create_publisher<sensor_msgs::msg::JointState>("robot_joint_15_status", 10);

  //单独发送所有的状态
  robot_state_publisher = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  
  robot_joint_status.header.stamp = rclcpp::Node::now();
  robot_joint_status.name = {"Leg_front_left_2","Leg_front_right_2","Leg_back_left_2","Leg_back_right_2",
                             "Leg_front_left_3","Leg_front_right_3",
                             "CHEST1",
                             "ARM0_LEFT","ARM0_RIGHT","ARM1_LEFT","ARM1_RIGHT","ARM2_LEFT","ARM2_RIGHT","ARM3_LEFT","ARM3_RIGHT",
                             "Leg_front_left_1","Leg_front_right_1","Leg_back_left_1","Leg_back_right_1"};
  robot_joint_status.velocity = {0.0,0.0,0.0,0.0,
                                 0.0,0.0,0.0,0.0,
                                 0.0,0.0,0.0,0.0,
                                 0.0,0.0,0.0,0.0,
                                 0.0,0.0,0.0 };
  robot_joint_status.position = {0.0,0.0,0.0,0.0,
                                 0.0,0.0,0.0,0.0,
                                 0.0,0.0,0.0,0.0,
                                 0.0,0.0,0.0,0.0,
                                 0.0,0.0,0.0 };


  heart_timer_ = this->create_wall_timer(5000ms, std::bind(&turn_on_robot::heart_timer_callback, this));

  Control();
  uint8_t data[100];
  while (rclcpp::ok()){
    #ifdef IA_USE_TCP
      stm_all.tcp_truct_t.tcp_read((char*)data,sizeof(data));
    #else
      stm_all.udp_truct_t.udp_read((char*)data,sizeof(data));
    #endif  
    // RCLCPP_INFO(this->get_logger(),"recv: %s",data);
  }
}

/**************************************
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
//心跳 55 00 00 00 00 09 00 00 00 
turn_on_robot::turn_on_robot():rclcpp::Node ("wheeltec_robot")
{
  this->difop_thread1_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&turn_on_robot::func1Thread, this)));
  this->difop_thread2_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&turn_on_robot::func2Thread, this)));
  
  //开发板ip为192.168.2.30
  //电脑ip为192.168.2.50
  #ifdef IA_USE_TCP
    stm_all.tcp_truct_t.tcp_init("192.168.2.30",8080);
  #else  
    stm_all.udp_truct_t.udp_init("192.168.2.30",8080);
    //stm_all.udp_truct_t.udp_write("hello",5);
  #endif 

  this->difop_thread1_->detach();
  this->difop_thread2_->detach();
}


/**************************************
Date: January 28, 20
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot()
{
  RCLCPP_INFO(this->get_logger(),"Shutting down"); //Prompt message //提示信息
}

/**************************************
Date: 20251103
功能: 心跳函数
心跳 55 00 00 00 00 09 00 00 00 
***************************************/
void turn_on_robot::heart_timer_callback(){
  RCLCPP_INFO(this->get_logger(),"heart_timer_callback");
  char Heart_Send_Data[9] = {0x55,0x00,0x00,0x00,0x00,0x09,0x00,0x55,0x18}; //小端
  uint16_t crc = com_crc((uint8_t*)Heart_Send_Data,7);
  Heart_Send_Data[7] = (crc>>8)&0xff;
  Heart_Send_Data[8] = crc&0xff;   //尾
  stm_all.udp_truct_t.udp_write(Heart_Send_Data,sizeof(Heart_Send_Data));
}

/**************************************
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void turn_on_robot::Ia_Publish_Odom()
{
    nav_msgs::msg::Odometry odom; //Instance the odometer topic data //实例化里程计话题数据
    //Convert the Z-axis rotation Angle into a quaternion for expression 
    //把Z轴转角转换为四元数进行表达
    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);
    
    odom.header.stamp = rclcpp::Node::now(); ; 
    odom.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
    odom.pose.pose.position.x = Robot_Pos.X; //Position //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat; //Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

    odom.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标
    odom.twist.twist.linear.x =  Robot_Vel.X; //Speed in the X direction //X方向速度
    odom.twist.twist.linear.y =  Robot_Vel.Y; //Speed in the Y direction //Y方向速度
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.z = Robot_Vel.Z; //Angular velocity around the Z axis //绕Z轴角速度 

    //There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    if(Robot_Vel.X== 0&&Robot_Vel.Y== 0&&Robot_Vel.Z== 0)
      //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
      //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
      memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else
      //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
      //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
      memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));       
    odom_publisher->publish(odom); //Pub odometer topic //发布里程计话题
}

/**************************************
功能: 发送信息写入到文件
***************************************/
void turn_on_robot::Ia_Write_Log_To_File(char* buf,int len){
  auto now = rclcpp::Node::now();
  ia_fp = fopen("1.log", "wb+");
  time_t curTime = time(NULL);
  struct tm *curTm = localtime(&curTime);
  char bufTime[30] = {0};
  sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
          curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);

  fprintf(ia_fp, bufTime);
  //RCLCPP_WARN(this->get_logger(), "%s ",bufTime);

  for(int i=0;i<len;i++){
    //printf(" %x",buf[i]);
    //fprintf(ia_fp, buf[i]);
  }

  fclose(ia_fp);

}
