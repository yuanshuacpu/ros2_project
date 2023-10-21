#include "can_instance.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
using namespace can_instance;

Can_Instance::Can_Instance(std::string name, const char serverIP[], uint16_t serverPort)
{
    is_success = true;

    active_flag_ = 0;

    name_ = name;

    socket_fd_ = socket(PF_INET, SOCK_STREAM, 0); // 获取socket_fd
    if (socket_fd_ < 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("motor_interface"), "Create socket fail!\n");
        is_success = false;
    }

    serverAddr_ = {0};

    serverAddr_.sin_family = AF_INET; // tcp
    serverAddr_.sin_addr.s_addr = inet_addr(serverIP);
    serverAddr_.sin_port = htons(serverPort);

    if (connect(socket_fd_, (struct sockaddr *)&serverAddr_, sizeof(serverAddr_)) == -1)
    {
        RCLCPP_FATAL(rclcpp::get_logger("motor_interface"), "Connect server failed!\n");
        close(socket_fd_);
        is_success = false;
    }

    RCLCPP_INFO(rclcpp::get_logger("motor_interface"), "%sConnect server [%s:%d] success\n",
                name_.c_str(), inet_ntoa(serverAddr_.sin_addr), ntohs(serverAddr_.sin_port));
}

void Can_Instance::Add_Motor_Instance(std::shared_ptr<motor_instance::Motor_Instance> motor_instance_ptr)
{

    if (motor_instance_ptr_vector_.size() <= Can_Amount_Max)
    {
        motor_instance_ptr_vector_.emplace_back(motor_instance_ptr);
        RCLCPP_INFO(rclcpp::get_logger("motor_interface"), "%s add 0X%lX \n",
                    name_.c_str(), motor_instance_ptr->Get_Id());
    }
    else
    {
        RCLCPP_FATAL(rclcpp::get_logger("motor_interface"), "can:%s,too much devices \n ", name_.c_str());
    }
}

void Can_Instance::Read()
{

    if (active_flag_)
    {
        ssize_t get_data_length;
        memset(recv_buf_, 0, sizeof(recv_buf_));

        get_data_length = recv(socket_fd_, recv_buf_, sizeof(recv_buf_), 0);

        size_t vol = (size_t)(get_data_length / 13);
        for (size_t i = 0; i < vol; i++)
        {
            uint64_t id = recv_buf_[3 + i * 13] << 8 | recv_buf_[4 + i * 13];
            uint8_t data[8];
            memset(data, 0, sizeof(data));
            memcpy(data, &recv_buf_[5 + i * 13], 8);
            for (auto &motor_instance_ptr_item : motor_instance_ptr_vector_)
            {
                if (motor_instance_ptr_item->Get_Id() == id)
                {

                    motor_instance_ptr_item->Read_Data(data, 8);
                    break;
                }
            }
        }
    }
    else
    {
        RCLCPP_FATAL(rclcpp::get_logger("motor_interface"), "can:%s is not active \n ", name_.c_str());
    }
}

void Can_Instance::Write()
{
    if (active_flag_)
    {
        uint8_t flag_200 = 0;
        uint8_t flag_1ff = 0;
        uint8_t data200[8] = {0};
        uint8_t data1ff[8] = {0};
        for (auto &motor_instance_ptr_item : motor_instance_ptr_vector_)
        {
            if ((motor_instance_ptr_item->Get_Id() >= 0x201) && (motor_instance_ptr_item->Get_Id() <= 0x204)) // group1
            {

                flag_200 = 1;

                motor_instance_ptr_item->Send_Data(data200);
            }

            if ((motor_instance_ptr_item->Get_Id() >= 0x205) && (motor_instance_ptr_item->Get_Id() <= 0x208)) // group1
            {

                flag_1ff = 1;

                motor_instance_ptr_item->Send_Data(data1ff);
            }
        }

        if (flag_200)
        {
            memset(write_buf_, 0, sizeof(write_buf_));
            write_buf_[0] = 0x08;
            write_buf_[3] = (0x200 >> 8) & 0xff;
            write_buf_[4] = 0x00;
            memcpy(&write_buf_[5], data200, 8);
            send(socket_fd_, write_buf_, sizeof(write_buf_), 0);
        }

        if (flag_1ff)
        {
            memset(write_buf_, 0, sizeof(write_buf_));
            write_buf_[0] = 0x08;
            write_buf_[3] = (0x1ff >> 8) & 0xff;
            write_buf_[4] = 0xff;
            memcpy(&write_buf_[5], data1ff, 8);
            send(socket_fd_, write_buf_, sizeof(write_buf_), 0);
        }
    }
    else
    {
        RCLCPP_FATAL(rclcpp::get_logger("motor_interface"), "can:%s is not active \n ", name_.c_str());
    }
}

Can_Instance::~Can_Instance()
{
    if (close(socket_fd_) < 0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("motor_interface"), "can:%s close socket failed!", name_.c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("motor_interface"), " can:%s close socket successed \n", name_.c_str());
}

void Can_Instance::Active()
{
    active_flag_ = 1;
}

void Can_Instance::Deactive()
{
    active_flag_ = 1;
}