#include "pid.hpp"
#include <memory.h>
#include <memory>
#include <math.h>
namespace
{
    using namespace pid;
    // 梯形积分
    static void
    f_Trapezoid_Intergral(PIDInstance *pid)
    {
        // 计算梯形的面积,(上底+下底)*高/2
        pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
    }

    // 变速积分(误差小时积分作用更强)
    static void f_Changing_Integration_Rate(PIDInstance *pid)
    {
        if (pid->Err * pid->Iout > 0)
        {
            // 积分呈累积趋势
            if (abs(pid->Err) <= pid->CoefB)
                return; // Full integral
            if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
                pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
            else // 最大阈值,不使用积分
                pid->ITerm = 0;
        }
    }

    static void f_Integral_Limit(PIDInstance *pid)
    {
        static float temp_Output, temp_Iout;
        temp_Iout = pid->Iout + pid->ITerm;
        temp_Output = pid->Pout + pid->Iout + pid->Dout;
        if (abs(temp_Output) > pid->MaxOut)
        {
            if (pid->Err * pid->Iout > 0) // 积分却还在累积
            {
                pid->ITerm = 0; // 当前积分项置零
            }
        }

        if (temp_Iout > pid->IntegralLimit)
        {
            pid->ITerm = 0;
            pid->Iout = pid->IntegralLimit;
        }
        if (temp_Iout < -pid->IntegralLimit)
        {
            pid->ITerm = 0;
            pid->Iout = -pid->IntegralLimit;
        }
    }

    // 微分先行(仅使用反馈值而不计参考输入的微分)
    static void f_Derivative_On_Measurement(PIDInstance *pid)
    {
        pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
    }

    // 微分滤波(采集微分时,滤除高频噪声)
    static void f_Derivative_Filter(PIDInstance *pid)
    {
        pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                    pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
    }

    // 输出滤波
    static void f_Output_Filter(PIDInstance *pid)
    {
        pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                      pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
    }

    // 输出限幅
    static void f_Output_Limit(PIDInstance *pid)
    {
        if (pid->Output > pid->MaxOut)
        {
            pid->Output = pid->MaxOut;
        }
        if (pid->Output < -(pid->MaxOut))
        {
            pid->Output = -(pid->MaxOut);
        }
    }

    // 电机堵转检测
    static void f_PID_ErrorHandle(PIDInstance *pid)
    {
        /*Motor Blocked Handle*/
        if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
            return;

        if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
        {
            // Motor blocked counting
            pid->ERRORHandler.ERRORCount++;
        }
        else
        {
            pid->ERRORHandler.ERRORCount = 0;
        }

        if (pid->ERRORHandler.ERRORCount > 500)
        {
            // Motor blocked over 1000times
            pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
        }
    }
}
namespace pid
{

    pid_instance::pid_instance(PID_Init_Config_s &config_data)
    {
        memset(&pid_setting_, 0, sizeof(pid_setting_));

        pid_setting_.Kp = config_data.Kp;
        pid_setting_.Ki = config_data.Ki;
        pid_setting_.Kd = config_data.Kd;

        pid_setting_.MaxOut = config_data.MaxOut;
        pid_setting_.DeadBand = config_data.DeadBand;

        pid_setting_.Improve = config_data.Improve;
        pid_setting_.IntegralLimit = config_data.IntegralLimit;

        pid_setting_.CoefA = config_data.CoefA;
        pid_setting_.CoefB = config_data.CoefB;

        pid_setting_.Output_LPF_RC = config_data.Output_LPF_RC;
        pid_setting_.Derivative_LPF_RC = config_data.Derivative_LPF_RC;
    }

    pid_instance::~pid_instance()
    {
    }

    double pid_instance::PIDCalculate(double measure, double ref, float duration)
    {
        // 堵转检测
        if (pid_setting_.Improve & PID_ErrorHandle)
            f_PID_ErrorHandle(&pid_setting_);

        pid_setting_.dt = duration; // 获取两次pid计算的时间间隔,用于积分和微分

        // 保存上次的测量值和误差,计算当前error
        pid_setting_.Measure = measure;
        pid_setting_.Ref = ref;
        pid_setting_.Err = pid_setting_.Ref - pid_setting_.Measure;

        // 如果在死区外,则计算PID
        if (abs(pid_setting_.Err) > pid_setting_.DeadBand)
        {
            // 基本的pid计算,使用位置式
            pid_setting_.Pout = pid_setting_.Kp * pid_setting_.Err;
            pid_setting_.ITerm = pid_setting_.Ki * pid_setting_.Err * pid_setting_.dt;
            pid_setting_.Dout = pid_setting_.Kd * (pid_setting_.Err - pid_setting_.Last_Err) / pid_setting_.dt;

            // 梯形积分
            if (pid_setting_.Improve & PID_Trapezoid_Intergral)
                f_Trapezoid_Intergral(&pid_setting_);
            // 变速积分
            if (pid_setting_.Improve & PID_ChangingIntegrationRate)
                f_Changing_Integration_Rate(&pid_setting_);
            // 微分先行
            if (pid_setting_.Improve & PID_Derivative_On_Measurement)
                f_Derivative_On_Measurement(&pid_setting_);
            // 微分滤波器
            if (pid_setting_.Improve & PID_DerivativeFilter)
                f_Derivative_Filter(&pid_setting_);
            // 积分限幅
            if (pid_setting_.Improve & PID_Integral_Limit)
                f_Integral_Limit(&pid_setting_);

            pid_setting_.Iout += pid_setting_.ITerm;                                         // 累加积分
            pid_setting_.Output = pid_setting_.Pout + pid_setting_.Iout + pid_setting_.Dout; // 计算输出

            // 输出滤波
            if (pid_setting_.Improve & PID_OutputFilter)
                f_Output_Filter(&pid_setting_);

            // 输出限幅
            f_Output_Limit(&pid_setting_);
        }
        else // 进入死区, 则清空积分和输出
        {
            pid_setting_.Output = 0;
            pid_setting_.ITerm = 0;
        }

        // 保存当前数据,用于下次计算
        pid_setting_.Last_Measure = pid_setting_.Measure;
        pid_setting_.Last_Output = pid_setting_.Output;
        pid_setting_.Last_Dout = pid_setting_.Dout;
        pid_setting_.Last_Err = pid_setting_.Err;
        pid_setting_.Last_ITerm = pid_setting_.ITerm;

        return pid_setting_.Output;
    }
}