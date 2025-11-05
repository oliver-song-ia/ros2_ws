#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
//#include "stm_msg/msg/tcpsend.hpp"

//定义msg类型 可直接使用
//using tcpsendtype = stm_msg::msg::Tcpsend;

class data_handle{
    public:
        data_handle();
        char* data_init();
        void tcp_data_parse(const geometry_msgs::msg::Twist::SharedPtr twist_aux);

    private:
        uint8_t cmd_head;
        uint8_t cmd_type;
        uint8_t cmd_command[2];     //当前的命令字 暂时没有用
        uint8_t cmd_len[2];
        std::vector<uint8_t> cmd_data;  //uint8_t cmd_data[]; 替换会比较好
};  