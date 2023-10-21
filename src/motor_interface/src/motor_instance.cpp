#include "motor_instance.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string.h>

using namespace motor_instance;

Motor_Instance::Motor_Instance(Motor_Type type, uint64_t id)
{
    type_ = type;
    id_ = id;

    memset(&motor_data_real_, 0, sizeof(motor_data_real_));
    memset(&motor_data_relate_, 0, sizeof(motor_data_relate_));
    memset(&motor_data_set_, 0, sizeof(motor_data_set_));

    switch (type_)
    {
    case M2006:
    {
        motor_data_real_.ratio = 1.0f / 36.0f;
        break;
    }
    }
}

uint64_t Motor_Instance::Get_Id()
{
    return id_;
}

void Motor_Instance::Read_Data(uint8_t data[], size_t length)
{
    if (length == 8)
    {
        motor_data_relate_.mechanical_angle = data[0] << 8 | data[1];
        motor_data_relate_.speed = data[2] << 8 | data[3];
        motor_data_relate_.current = data[4] << 8 | data[5];
        // motor_data_real_.total_mechanical_angle

        motor_data_real_.now_mechanical_angle = (float)motor_data_relate_.mechanical_angle / 8191.0f * motor_data_real_.ratio * 360.0f;
        motor_data_real_.current_speed = (float)motor_data_relate_.speed * motor_data_real_.ratio * 60.0f * 360.0f;
        motor_data_real_.effort = motor_data_relate_.current;
    }
}

void Motor_Instance::Send_Data(uint8_t data[])
{
    if ((0x201 <= id_) && (id_ <= 0x204))
    {
        data[(id_ - 0x201) * 2] = (motor_data_set_.Set_Current >> 8) & 0xff;
        data[(id_ - 0x201) * 2 + 1] = (motor_data_set_.Set_Current) & 0xff;
    }
    else if (0x205 <= id_ && id_ <= 0x208)
    {
        data[(id_ - 0x205) * 2] = (motor_data_set_.Set_Current >> 8) & 0xff;
        data[(id_ - 0x205) * 2 + 1] = (motor_data_set_.Set_Current) & 0xff;
    }
}

Motor_Instance::~Motor_Instance()
{
}