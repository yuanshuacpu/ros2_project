#ifndef _PID_HPP_
#include <stdio.h>
#include <stdint.h>

#include "pid_type.hpp"
namespace pid
{
    // PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
    enum PID_Improvement_e
    {
        PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
        PID_Integral_Limit = 0b00000001,              // 0000 0001
        PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
        PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
        PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
        PID_OutputFilter = 0b00010000,                // 0001 0000
        PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
        PID_DerivativeFilter = 0b01000000,            // 0100 0000
        PID_ErrorHandle = 0b10000000,                 // 1000 0000
    };

    /* PID 报错类型枚举*/
    enum ErrorType_e
    {
        PID_ERROR_NONE = 0x00U,
        PID_MOTOR_BLOCKED_ERROR = 0x01U
    };

    struct PID_ErrorHandler_t
    {
        uint64_t ERRORCount;
        ErrorType_e ERRORType;
    };

    /* PID结构体 */
    struct PIDInstance
    {
        //---------------------------------- init config block

        // config parameter
        float Kp;
        float Ki;
        float Kd;
        float MaxOut;
        float DeadBand;

        // improve parameter
        PID_Improvement_e Improve;
        float IntegralLimit;     // 积分限幅
        float CoefA;             // 变速积分 For Changing Integral
        float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
        float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
        float Derivative_LPF_RC; // 微分滤波器系数

        //-----------------------------------
        // for calculating
        float Measure;
        float Last_Measure;
        float Err;
        float Last_Err;
        float Last_ITerm;

        float Pout;
        float Iout;
        float Dout;
        float ITerm;

        float Output;
        float Last_Output;
        float Last_Dout;

        float Ref;

        float dt;

        PID_ErrorHandler_t ERRORHandler;
    };

    /* 用于PID初始化的结构体*/
    struct PID_Init_Config_s // config parameter
    {
        // basic parameter
        float Kp;
        float Ki;
        float Kd;
        float MaxOut;   // 输出限幅
        float DeadBand; // 死区

        // improve parameter
        PID_Improvement_e Improve;
        float IntegralLimit; // 积分限幅
        float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
        float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
        float Output_LPF_RC; // RC = 1/omegac
        float Derivative_LPF_RC;

        Pid_Feedback_Type pid_feedback_type;
    };
    class pid_instance
    {
    private:
        PIDInstance pid_setting_;
        /* data */
    public:
        pid_instance(PID_Init_Config_s &config_data);

        double PIDCalculate(double measure, double ref, float duration);

        ~pid_instance();
    };
}
#endif // ! _PID_HPP_