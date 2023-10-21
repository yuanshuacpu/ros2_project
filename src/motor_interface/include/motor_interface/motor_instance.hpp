#ifndef _MOTOR_INSTANCE_CPP

#include <string>
#include <stdio.h>

namespace motor_instance
{
    constexpr char M2006_Char[] = "M2006";

    constexpr char Set_Current_Char[] = "set_current";

    enum Motor_Type
    {
        M2006 = 1,
    };

    struct Motor_Data_Real
    {
        float now_mechanical_angle;   // 单圈角度/ 度
        float total_mechanical_angle; // 总角度/ 度
        float current_speed;          // 速度/ 度每秒
        float effort;                 // 扭矩/
        float ratio;                  // 减速比/ 1：？
    };

    struct Motor_Data_Relate
    {
        int16_t mechanical_angle;
        int16_t current;
        int16_t speed;
    };

    struct Motor_Data_Set
    {
        int16_t Set_Current;
    };

    class Motor_Instance
    {
    private:
        Motor_Type type_;
        std::uint64_t id_;

    public:
        Motor_Data_Set motor_data_set_;
        Motor_Data_Real motor_data_real_;
        Motor_Data_Relate motor_data_relate_;

        Motor_Instance(Motor_Type type, std::uint64_t id);
        std::uint64_t Get_Id();
        void Read_Data(std::uint8_t data[], std::size_t length);
        void Send_Data(std::uint8_t data[]);

        ~Motor_Instance();
    };
}

#endif // !_CAN_PROCESS_H_P_P_
