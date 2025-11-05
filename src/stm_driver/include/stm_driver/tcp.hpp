#include <string>

class tcp_struct{
    public:
        int tcp_init(std::string address,int port);
        int tcp_write(char* buf,int sendnum);
        int tcp_read(char* buf,int sendnum);
        int tcp_close();
        void tcp_thread_start();
        
        void DataPoll();        //线程回调函数
        std::shared_ptr<std::thread> difop_thread_;   

    private:
        int sockfd;
        int port;  
        std::string address;  
        char buf[1024];
        int len;
};