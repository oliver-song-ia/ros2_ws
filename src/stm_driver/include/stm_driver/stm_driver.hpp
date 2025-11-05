#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "tcp.hpp"
#include "udp.hpp"
#include "dataparse.hpp"
#include <thread>
#include "std_msgs/msg/string.hpp"

//namespace stm_driver {
    class stm_driver{
        public:
            stm_driver();
            void func1Thread();
            void func2Thread();
            void create_thread();
            tcp_struct tcp_truct_t;         //定义网络操作相关结构
            udp_struct udp_truct_t;         //定义网络操作相关结构
            data_handle data_hadle_t;       //定义数据解析相关结构
        private:
            void topic_callback(const std_msgs::msg::String & msg);  //const 可修改为非const
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
            
};
//}