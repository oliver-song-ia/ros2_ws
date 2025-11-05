#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdio>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "stm_driver/tcp.hpp"
#include "stm_driver/dataparse.hpp"

int tcp_struct::tcp_init(std::string address,int port){
    struct sockaddr_in serverAddr;
    //char buf[1024];
    int err = 0;

    // 创建套接字
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Socket creation faileds");
        return 1;
    }

    // 设置服务器地址信息
    serverAddr.sin_family = AF_INET;
    inet_pton(AF_INET, address.c_str(), &serverAddr.sin_addr);
    serverAddr.sin_port = htons(port);

    // 连接服务器
    //connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    if ((err=connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr))) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Connection failed:%d",err);
        return 1;
    }

    //this->difop_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&tcp_struct::DataPoll, this)));           
/*
    // 数据收发循环
    while (1) {
        int num = read(sockfd, buf, sizeof(buf)); // 接收数据
        write(sockfd, buf, num); // 发送数据
    }*/

    return 0;
}

int tcp_struct::tcp_write(char* buf,int sendnum){
    int num = write(sockfd, buf, sendnum); // 发送数据
    return num;
}

int tcp_struct::tcp_read(char* buf,int sendnum){
    int num = read(sockfd, buf, sendnum); // 接收数据
    return num;
}

int tcp_struct::tcp_close(){
    // 关闭套接字
    close(sockfd);
    return 0;
}
void tcp_struct::tcp_thread_start(){
    this->difop_thread_->detach();  //不能用join 不然线程无法切换
}

void tcp_struct::DataPoll(){
    while(1){
        int num = read(sockfd, buf, sizeof(buf)); // 接收数据
        if(num>0){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recv:%d",num);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    //return 0;
}