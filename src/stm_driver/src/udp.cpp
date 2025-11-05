#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include "stm_driver/udp.hpp"

#define N 65507

using namespace std;

/*初始化udp*/
int udp_struct::udp_init(string addr, int port){
    //第一步：创建套接字
    if((sockfd = socket(AF_INET,SOCK_DGRAM,0)) == -1)
    {
        perror("fail to socket");
        exit(1);
    }

    //设置广播属性
    int opt  = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));

    //第二步：将服务器的网络信息结构体绑定前填充
    struct sockaddr_in clientaddr;
    clientaddr.sin_family = AF_INET;
    //clientaddr.sin_addr.s_addr = inet_addr(INADDR_ANY);
    inet_pton(AF_INET, "0.0.0.0", &clientaddr.sin_addr);
    clientaddr.sin_port = htons(8080);

    //第三步：将网络信息结构体与套接字绑定
    if(bind(sockfd,(struct sockaddr*)&clientaddr,sizeof(clientaddr)) == -1)
    {
        perror("fail to bind");
        exit(1);
    }

    printf("socket init done\n");
    printf("ip:%s,port:%d and %d\n",inet_ntoa(clientaddr.sin_addr),ntohs(clientaddr.sin_port),port);
    
    addrlen = sizeof(serveraddr);
    serveraddr.sin_family = AF_INET; //协议族，AF_INET:ipv4协议
    //serveraddr.sin_addr.s_addr = inet_addr(addr.c_str()); //IP地址
    inet_pton(AF_INET, addr.c_str(), &serveraddr.sin_addr);
    serveraddr.sin_port = htons(port);

    return sockfd;
}

int udp_struct::udp_write(char* buf,int sendnum){

    int num = sendto(sockfd,buf,sendnum,0,(struct sockaddr *)&serveraddr,addrlen);
    
    return num;
}

int udp_struct::udp_read(char* buf,int sendnum){

    int num = recvfrom(sockfd,buf,N,0,(struct sockaddr *)&serveraddr,&addrlen);
        

    return num;
}

int udp_struct::udp_close(){
    // 关闭套接字
    close(sockfd);
    return 0;
}