#ifndef _PID_CONTROL_HPP_
#include <memory>
#include <initializer_list>

#include "pid.hpp"
#include "controller_interface/controller_interface.hpp"

namespace pid
{

    struct Pid_Feedback_Data
    {
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> &feedback_data;
        enum Pid_Feedback_Type type;
    };

    class pid_control_instance
    {
    private:
        std::unique_ptr<pid_instance> current_pid_;
        std::unique_ptr<pid_instance> position_pid_;
        std::unique_ptr<pid_instance> speed_pid_;

        struct Feedback_Data
        {
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> *feedback_velocity;
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> *feedback_position;
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> *feedback_current;
        };

        Feedback_Data feedback_data_;

        uint8_t type_;

        std::string name_;
        /* data */
    public:
        pid_control_instance(std::string &name);

        uint8_t pid_init(std::initializer_list<PID_Init_Config_s> pid_settings, std::initializer_list<Pid_Feedback_Data> pid_feedbacks);

        double pid_control_calculate(double ref, float duration);

        ~pid_control_instance();
    };

}

#endif // !_PID_CONTROL_HPP_