#include "stm_driver/stm_driver.hpp"


stm_driver::stm_driver() {
      //auto node = rclcpp::Node::make_shared("stm_driver");

      //RCLCPP_INFO(node->get_logger(), "stm driver start");
      //network = std::make_shared<tcp_struct>();
      /*network = tcp_struct();
      network.tcp_init("192.168.1.219",8080);

      //auto message = senddata->data_init();
      //while (rclcpp::ok()){
        //network->tcp_write((char*)(message.data.data()),message.data.size());  //vector的data方法可以将vector转换为char*
      //}
      create_thread();
      network.tcp_thread_start();  //一共创建了两个线程*/

      /*while(1){
        RCLCPP_INFO(node->get_logger(), "node");
      }*/
      
      /*subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&node::topic_callback, this, _1));
      while (rclcpp::ok()){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "func1Thread");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }*/
}


/*
void stm_driver::func1Thread(){
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&node::topic_callback, this, _1));
    while (rclcpp::ok()){
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "func1Thread");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void node::func2Thread(){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "func2Thread");
}

void node::create_thread(){
    std::thread Thread1(&node::func1Thread, this);
    Thread1.detach();
}

void node::topic_callback(const std_msgs::msg::String & msg)
    {
      //订阅话题回调
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      auto senddata = std::make_shared<data_handle>();
      auto message = senddata->data_init();
      network.tcp_write((char*)(message.data.data()),message.data.size());
    }


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<node>());

  rclcpp::shutdown();
  return 0;
}*/
