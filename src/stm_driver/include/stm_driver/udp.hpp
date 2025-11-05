#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>

class udp_struct{
    public:
        int udp_init(std::string address,int port);
        int udp_write(char* buf,int sendnum);
        int udp_read(char* buf,int sendnum);
        int udp_close();

    private:
        int sockfd;
        //设置服务端地址
        struct sockaddr_in serveraddr;
        socklen_t addrlen;
};