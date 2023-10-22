// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gimbal_controller/gimbal_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

#include "pid/pid.hpp"
namespace
{ // utility

  // TODO(destogl): remove this when merged upstream
  // Changed services history QoS to keep all so we don't lose any client service calls
  static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1, // message queue depth
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false};

  using ControllerReferenceMsg = gimbal_controller::GimbalController::ControllerReferenceMsg;

  constexpr auto DEFAULT_DR16_COMMAND_TOPIC = "~/dr16_cmd_vel";
  constexpr auto DEFAULT_KEYBOARD_COMMAND_TOPIC = "~/keyboard_cmd_vel";
  /// Constant defining position interface
  constexpr char HW_IF_POSITION[] = "position";
  /// Constant defining velocity interface
  constexpr char HW_IF_VELOCITY[] = "velocity";
  /// Constant defining acceleration interface
  constexpr char HW_IF_ACCELERATION[] = "acceleration";
  /// Constant defining effort interface
  constexpr char HW_IF_EFFORT[] = "effort";
} // namespace

namespace gimbal_controller
{

  GimbalController::GimbalController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn GimbalController::on_init()
  {

    try
    {
      param_listener_ = std::make_shared<gimbal_controller::ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn GimbalController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    // update parameters if they have changed
    if (param_listener_->is_old(params_))
    {
      params_ = param_listener_->get_params();
      RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");
    }

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    const Command_Twist empty_twist;
    cmd_ptr_.set(std::make_shared<Command_Twist>(empty_twist));

    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    // Reference Subscriber
    cmd_subscriber_from_keyboard_ = get_node()->create_subscription<Command_Twist>(
        DEFAULT_KEYBOARD_COMMAND_TOPIC, subscribers_qos,
        std::bind(&GimbalController::Cmd_Subscriber_From_Keyboard_Func, this, std::placeholders::_1));

    if (cmd_subscriber_from_keyboard_.get() == nullptr)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "cmd_subscriber_from_keyboard failed");
      return controller_interface::CallbackReturn::ERROR;
    }
    previous_commands_.emplace(empty_twist);
    previous_commands_.emplace(empty_twist);

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration GimbalController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &joint_name : params_.joints)
    {
      for (size_t i = 0; i < params_.cmd_interface_name.size(); i++)
      {

        /* code */
        command_interfaces_config.names.emplace_back(joint_name + "/" + params_.cmd_interface_name[i]);

        RCLCPP_INFO(get_node()->get_logger(), "joint %s 's cmd_interface_name is %s this size is %lu,i = %lu\n", joint_name.c_str(), params_.cmd_interface_name[i].c_str(), params_.cmd_interface_name.size(), command_interfaces_config.names.size());
      }
    }

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration GimbalController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_conf;
    state_conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &joint_name : params_.joints)
    {
      for (size_t i = 0; i < params_.state_interface_name.size(); i++)
      {
        RCLCPP_INFO(get_node()->get_logger(), "joint %s 's state_interface_name is %s this size is %lu \n", joint_name.c_str(), params_.state_interface_name[i].c_str(), params_.state_interface_name.size());
        /* code */
        state_conf.names.emplace_back(joint_name + "/" + params_.state_interface_name[i]);
      }
    }
    return state_conf;
  }

  controller_interface::CallbackReturn GimbalController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    is_halt = false;

    cmd_subscriber_is_active_ = true;

    auto gimbal_config_result = configure_gimbal(params_.joints, gimbal_state_cmd_data_);

    if (gimbal_config_result == controller_interface::CallbackReturn::ERROR)
    {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Either left wheel interfaces, right wheel interfaces are non existent");
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
    // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
    // `on_activate` method in `JointTrajectoryController` for examplary use of
    // `controller_interface::get_ordered_interfaces` helper function

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn GimbalController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
    // instead of a loop

    cmd_subscriber_is_active_ = false;
    is_halt = true;

    gimbal_state_cmd_data_.clear();

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type GimbalController::update(
      const rclcpp::Time &time, const rclcpp::Duration &period /*period*/)
  {
    if (is_halt == true)
    {
      RCLCPP_WARN(
          get_node()->get_logger(), "%s update is not active\n", "gimbal");
      return controller_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < params_.joints.size(); i++)
    {

      /* code */
    }

    if (cmd_subscriber_is_active_)
    {
      for (size_t i = 0; i < params_.joints.size(); i++)
      {
        gimbal_state_cmd_data_[i].command_effort.get().set_value((double)1000);
      }
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn GimbalController::on_cleanup(
      const rclcpp_lifecycle::State &previous_state)
  {
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn GimbalController::on_error(
      const rclcpp_lifecycle::State &previous_state)
  {
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn GimbalController::on_shutdown(
      const rclcpp_lifecycle::State &previous_state)
  {
    // for (size_t i = 0; i < params_.joints.size(); i++)
    // {
    //   gimbal_state_cmd_data_[i].command_effort.get().set_value((double)0);
    // }

    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool GimbalController::reset()
  {

    std::queue<Command_Twist> empty;
    std::swap(previous_commands_, empty);

    // gimbal_state_cmd_data_.clear();

    cmd_subscriber_is_active_ = false;

    cmd_subscriber_from_keyboard_.reset();

    cmd_ptr_.set(std::make_shared<Command_Twist>());

    is_halt = true;

    return true;
  }
  void GimbalController::Cmd_Subscriber_From_Keyboard_Func(const std::shared_ptr<Command_Twist> Cmd_Subscriber_From_Keyboard_Msg)
  {
    if (!cmd_subscriber_is_active_)
    {
      RCLCPP_WARN(
          get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
      return;
    }
    if ((Cmd_Subscriber_From_Keyboard_Msg->header.stamp.sec == 0) && (Cmd_Subscriber_From_Keyboard_Msg->header.stamp.nanosec == 0))
    {
      RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received TwistStamped with zero timestamp, setting it to current "
          "time, this message will only be shown once");
      Cmd_Subscriber_From_Keyboard_Msg->header.stamp = get_node()->get_clock()->now();
    }

    cmd_ptr_.set(std::move(Cmd_Subscriber_From_Keyboard_Msg));
  }

  controller_interface::CallbackReturn GimbalController::configure_gimbal(
      const std::vector<std::string> &gimbal_names,
      std::vector<GimbalHandle> &registered_handles)
  {
    auto logger = get_node()->get_logger();

    if (gimbal_names.empty())
    {
      RCLCPP_ERROR(logger, "Gimbal is empty\n");
      return controller_interface::CallbackReturn::ERROR;
    }

    // register handles
    registered_handles.reserve(gimbal_names.size());
    for (const auto &gimbal_name : gimbal_names)
    {

      const auto velocity_state_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&gimbal_name](const auto &interface)
          {
            return interface.get_prefix_name() == gimbal_name &&
                   interface.get_interface_name() == HW_IF_VELOCITY;
          });

      RCLCPP_INFO(logger, "%s 's velocity state name is %s name is %s ",
                  gimbal_name.c_str(),
                  velocity_state_handle->get_interface_name().c_str(),
                  velocity_state_handle->get_full_name().c_str());

      if (velocity_state_handle == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "Unable to obtain velocity joint state handle for %s", gimbal_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      const auto position_state_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&gimbal_name](const auto &interface)
          {
            return interface.get_prefix_name() == gimbal_name &&
                   interface.get_interface_name() == HW_IF_POSITION;
          });

      RCLCPP_INFO(logger, "%s 's position state name is %s name is %s ", gimbal_name.c_str(), position_state_handle->get_interface_name().c_str(), position_state_handle->get_full_name().c_str());

      if (position_state_handle == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "Unable to obtain position joint state handle for %s", gimbal_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      const auto effort_command_handle = std::find_if(
          command_interfaces_.begin(), command_interfaces_.end(),
          [&gimbal_name](const auto &interface)
          {
            return interface.get_prefix_name() == gimbal_name &&
                   interface.get_interface_name() == HW_IF_EFFORT;
          });

      RCLCPP_INFO(logger, "%s 's effort cmd name is %s name is %s ",
                  gimbal_name.c_str(),
                  effort_command_handle->get_interface_name().c_str(),
                  effort_command_handle->get_full_name().c_str());

      if (effort_command_handle == command_interfaces_.end())
      {
        RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", gimbal_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      registered_handles.emplace_back(
          GimbalHandle{std::ref(*velocity_state_handle), std::ref(*position_state_handle), std::ref(*effort_command_handle)});
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

} // namespace gimbal_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    gimbal_controller::GimbalController, controller_interface::ControllerInterface)
