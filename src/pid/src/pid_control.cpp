#include <string>
#include <iostream>
#include <stdexcept>
#include <limits>

#include "pid_control.hpp"

namespace pid
{

    pid_control_instance::pid_control_instance(std::string &name)
    {
        type_ = 0;
        name_ = name;
        // feedback_data_.resize(1);
    }

    uint8_t pid_control_instance::
        pid_init(std::initializer_list<PID_Init_Config_s> pid_settings, std::initializer_list<Pid_Feedback_Data> pid_feedbacks)
    {
        try
        {

            for (auto item : pid_settings)
            {

                switch (item.pid_feedback_type)
                {
                case PID_Position_Type:
                {
                    if ((type_ & PID_Position_Type))
                    {

                        char *error;
                        sprintf(error, "%s 's pid_positiion is too much", name_.c_str());
                        throw std::invalid_argument(error);
                    }
                    else
                    {
                        type_ |= PID_Position_Type;
                        position_pid_ = std::make_unique<pid_instance>(item);
                    }
                    break;
                }
                case PID_Current_Type:
                {
                    if ((type_ & PID_Current_Type))
                    {

                        char *error;
                        sprintf(error, "%s 's pid_current is too much", name_.c_str());
                        throw std::invalid_argument(error);
                    }
                    else
                    {
                        type_ |= PID_Current_Type;
                        current_pid_ = std::make_unique<pid_instance>(item);
                    }
                    break;
                }
                case PID_Speed_Type:
                {
                    if ((type_ & PID_Speed_Type))
                    {

                        char *error;
                        sprintf(error, "%s 's pid_speed is too much", name_.c_str());
                        throw std::invalid_argument(error);
                    }
                    else
                    {
                        type_ |= PID_Speed_Type;
                        speed_pid_ = std::make_unique<pid_instance>(item);
                    }
                    break;
                }
                default:
                    break;
                }
            }
            for (auto item : pid_feedbacks)
            {
                switch (item.type)
                {
                case PID_Position_Type:
                {
                    feedback_data_.feedback_position = &item.feedback_data;
                    break;
                }
                case PID_Current_Type:
                {
                    feedback_data_.feedback_current = &item.feedback_data;
                }
                case PID_Speed_Type:
                {
                    feedback_data_.feedback_velocity = &item.feedback_data;
                    break;
                }
                default:
                    break;
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        return 1;
    }

    double pid_control_instance::
        pid_control_calculate(
            double ref,
            float duration)
    {
        double result_value = 0;
        try
        {
            if ((type_ & PID_Position_Type))
            {
                if (position_pid_ == nullptr)
                {
                    char *error;
                    sprintf(error, "%s 's pid_positiion is not initial", name_.c_str());
                    throw std::invalid_argument(error);
                }
                result_value = position_pid_->PIDCalculate(feedback_data_.feedback_position->get().get_value(),
                                                           ref,
                                                           duration);
            }

            if ((type_ & PID_Speed_Type))
            {
                if (speed_pid_ == nullptr)
                {
                    char *error;
                    sprintf(error, "%s 's pid_speed is not initial", name_.c_str());
                    throw std::invalid_argument(error);
                }
                result_value = speed_pid_->PIDCalculate(feedback_data_.feedback_velocity->get().get_value(),
                                                        result_value,
                                                        duration);
            }

            if ((type_ & PID_Current_Type))
            {
                if (current_pid_ == nullptr)
                {
                    char *error;
                    sprintf(error, "%s 's pid_current is not initial", name_.c_str());
                    throw std::invalid_argument(error);
                }
                result_value = current_pid_->PIDCalculate(feedback_data_.feedback_current->get().get_value(),
                                                          result_value,
                                                          duration);
            }
            /* code */
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        return result_value;
    }

    pid_control_instance::
        ~pid_control_instance()
    {
    }
}