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

#ifndef GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_HPP_
#define GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <queue>

#include "controller_interface/controller_interface.hpp"
#include "gimbal_controller_parameters.hpp"
#include "gimbal_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace gimbal_controller
{
  // name constants for state interfaces
  static constexpr size_t STATE_MY_ITFS = 0;

  // name constants for command interfaces
  static constexpr size_t CMD_MY_ITFS = 0;

  // TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
  enum class control_mode_type : std::uint8_t
  {
    FAST = 0,
    SLOW = 1,
  };

  class GimbalController : public controller_interface::ControllerInterface
  {
  public:
    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    GimbalController();

    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State &previous_state) override;

    GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state) override;

    // TODO(anyone): replace the state and command message types
    using Command_Twist = geometry_msgs::msg::TwistStamped;
    using ControllerReferenceMsg = control_msgs::msg::JointJog;
    using ControllerModeSrvType = std_srvs::srv::SetBool;
    using ControllerStateMsg = control_msgs::msg::JointControllerState;

  protected:
    struct GimbalHandle
    {
      std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_velocity;
      std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_position;

      std::reference_wrapper<hardware_interface::LoanedCommandInterface> command_effort;
    };

    std::shared_ptr<gimbal_controller::ParamListener>
        param_listener_;
    gimbal_controller::Params params_;

    // Command subscribers and Controller State publisher

    rclcpp::Subscription<Command_Twist>::SharedPtr cmd_subscriber_from_keyboard_ = nullptr;

    bool cmd_subscriber_is_active_;

    realtime_tools::RealtimeBox<std::shared_ptr<Command_Twist>> cmd_ptr_{nullptr};

    std::queue<Command_Twist> previous_commands_;

    std::vector<GimbalHandle> gimbal_state_cmd_data_;

    rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
    realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

    using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

    rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
    std::unique_ptr<ControllerStatePublisher> state_publisher_;

    bool is_halt;

  private:
    // callback for topic interface
    GIMBAL_CONTROLLER__VISIBILITY_LOCAL
    bool reset();

    GIMBAL_CONTROLLER__VISIBILITY_LOCAL
    void Cmd_Subscriber_From_Keyboard_Func(const std::shared_ptr<Command_Twist> Cmd_Subscriber_From_Keyboard_Msg);

    controller_interface::CallbackReturn configure_gimbal(
        const std::vector<std::string> &gimbal_names,
        std::vector<GimbalHandle> &registered_handles);

  }; // namespace gimbal_controller
}

#endif // GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_HPP_
