#include "stm_driver/dataparse.hpp"
//#include "stm_msg/msg/tcpdata.hpp"

//55 00 00 00 00 09 00 00 00 
data_handle::data_handle(){
    this->cmd_head = 0x55;
    this->cmd_type = 0x00;
    this->cmd_command[0] = 0x00;
    this->cmd_command[1] = 0x00;
    this->cmd_len[0] = 0x00;
    this->cmd_len[1] = 0x09;
    this->cmd_data.push_back(0x00);
    this->cmd_data.push_back(0x00);
    this->cmd_data.push_back(0x00);
}

void data_handle::tcp_data_parse(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    uint8_t send_data[100];

    /* 
  short  transition;  //intermediate variable //中间变量

  Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
  Send_Data.tx[1] = AutoRecharge; //set aside //预留位
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

  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
 
  stm_all.tcp_truct_t.tcp_write((char*)Send_Data.tx,sizeof (Send_Data.tx));*/
}

char* data_handle::data_init(){
    /*(auto message = tcpsendtype();

    message.data.push_back(this->cmd_head);
    message.data.push_back(this->cmd_type);
    //message.data.push_back(this->cmd_command);
    // message.data.push_back(this->cmd_len);
    //message.data.push_back(this->cmd_data);
    message.data.insert(message.data.end(), std::begin(this->cmd_command), std::end(this->cmd_command));
    message.data.insert(message.data.end(), std::begin(this->cmd_len), std::end(this->cmd_len));
    message.data.insert(message.data.end(), std::begin(this->cmd_data), std::end(this->cmd_data));
    
    return message;*/
    return nullptr;
}