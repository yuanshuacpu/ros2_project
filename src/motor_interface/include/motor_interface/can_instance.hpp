#ifndef _CAN_INSTANCE_H_P_P_

#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <memory>

#include <vector>

#include "motor_instance.hpp"

#define Can_Amount_Max 8
namespace can_instance
{

    class Can_Instance
    {
    private:
        std::string name_;

        struct sockaddr_in serverAddr_;

        char write_buf_[13];
        char recv_buf_[1300];

        std::vector<std::shared_ptr<motor_instance::Motor_Instance>> motor_instance_ptr_vector_;

        int socket_fd_;

        std::uint8_t active_flag_;

        bool is_success;

    public:
        Can_Instance(std::string name, const char serverIP[], uint16_t serverPort);
        void Read();
        void Write();
        void Active();
        void Deactive();
        void Add_Motor_Instance(std::shared_ptr<motor_instance::Motor_Instance> motor_instance_ptr);
        bool Get_Is_Success_Flag()
        {
            return is_success;
        }
        ~Can_Instance();
    };
}

#endif // !_CAN_INSTANCE_H_P_P
