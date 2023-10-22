#ifndef _PID_TYPE_HPP_

namespace pid
{
    /* PID 类型枚举*/
    enum Pid_Feedback_Type
    {
        PID_Position_Type = 0b00000001,
        PID_Current_Type = 0b00000010,
        PID_Speed_Type = 0b00000100,
    };
}

#endif // !_PID_TYPE_HPP_
