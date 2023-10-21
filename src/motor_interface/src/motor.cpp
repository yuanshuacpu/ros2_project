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

#include <limits>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include "motor_interface/motor.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

struct ServerAddress
{
  uint16_t serverPort;
  std::string serverIP;
};

std::uint64_t st2uint64(std::string data)
{
  std::uint64_t id = std::stoul(data);
  std::vector<uint8_t> id_data;
  std::uint64_t result = 0;
  while (1)
  {
    id_data.emplace_back(id % 10);

    id /= 10;

    if (id == 0)
      break;
    /* code */
  }
  for (auto it = id_data.crbegin(); it != id_data.crend(); ++it)
  {
    result = result | *it;
    result = result << 4;
  }
  result = result >> 4;
  return result;
}

namespace motor_interface
{
  hardware_interface::CallbackReturn Motor::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    hw_commands_currents_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Motor::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
    name_ = info_.hardware_parameters["can_name"];

    RCLCPP_INFO(rclcpp::get_logger("motor_interface"), "joint`s name=%s \n", info_.hardware_parameters["serverPort"].c_str());

    ServerAddress ip1_can1 = {
        .serverPort = (uint16_t)std::stoul(info_.hardware_parameters["serverPort"]),
        .serverIP = info_.hardware_parameters["serverIP"]};

    can_instance_.reset(new can_instance::Can_Instance(name_, ip1_can1.serverIP.c_str(), ip1_can1.serverPort));

    if (can_instance_->Get_Is_Success_Flag() == false)
    {
      return CallbackReturn::ERROR;
    }
    motor_instance_vector_.resize(info_.joints.size(), std::numeric_limits<std::shared_ptr<motor_instance::Motor_Instance>>::quiet_NaN());

    // TODO(anyone): read parameters and initialize the hardware

    // hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    // hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      static uint16_t i = 0;
      std::vector<std::string>
          tar_name;
      boost::split(tar_name, joint.name, boost::is_any_of("_"), boost::token_compress_on);

      if (tar_name[0] == motor_instance::M2006_Char)
      {
        motor_instance_vector_[i].reset(new motor_instance::Motor_Instance(motor_instance::M2006, st2uint64(tar_name[1])));
      }

      can_instance_->Add_Motor_Instance(motor_instance_vector_[i]);

      RCLCPP_INFO(rclcpp::get_logger("motor_interface"), "joint`s name=%s \n", joint.name.c_str());
      RCLCPP_INFO(rclcpp::get_logger("motor_interface"), "joint.command_interfaces.size=%ld \n", joint.command_interfaces.size());
      RCLCPP_INFO(rclcpp::get_logger("motor_interface"), "joint.command_interfaces[0].name=%s\n", joint.command_interfaces[0].name.c_str());

      RCLCPP_INFO(rclcpp::get_logger("motor_interface"), "joint.state_interfaces.size=%ld \n", joint.state_interfaces.size());
      RCLCPP_INFO(rclcpp::get_logger("motor_interface"), "joint.state_interfaces[0].name=%s,joint.state_interfaces[1].name=%s \n", joint.state_interfaces[0].name.c_str(), joint.state_interfaces[1].name.c_str());

      ++i;
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> Motor::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          // TODO(anyone): insert correct interfaces
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          // TODO(anyone): insert correct interfaces
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> Motor::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          // TODO(anyone): insert correct interfaces
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_currents_[i]));

      // command_interfaces.emplace_back(hardware_interface::CommandInterface(
      //     // TODO(anyone): insert correct interfaces
      //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
      // command_interfaces.emplace_back(hardware_interface::CommandInterface(
      //     // TODO(anyone): insert correct interfaces
      //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn Motor::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      hw_states_positions_[i] = 0;
      hw_states_velocities_[i] = 0;

      hw_commands_currents_[i] = 0;

      /* code */
    }

    // TODO(anyone): prepare the robot to receive commands
    can_instance_->Active();

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Motor::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): prepare the robot to stop receiving commands
    can_instance_->Deactive();
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Motor::read(
      const rclcpp::Time &time /*time*/, const rclcpp::Duration &period /*period*/)
  {
    // TODO(anyone): read robot states

    can_instance_->Read();

    for (std::size_t i = 0; i < motor_instance_vector_.size(); i++)
    {
      // hw_states_velocities_[i] = motor_instance_vector_[i]->motor_data_relate_.speed;
      // hw_states_positions_[i] = motor_instance_vector_[i]->motor_data_real_.now_mechanical_angle;
      hw_states_velocities_.at(i) = (double)motor_instance_vector_.at(i)->motor_data_real_.current_speed;
      hw_states_positions_.at(i) = (double)motor_instance_vector_.at(i)->motor_data_real_.now_mechanical_angle;

      // RCLCPP_INFO(rclcpp::get_logger("motor_interface"), " get velocity is %f \n", hw_states_velocities_[i]);
      // RCLCPP_INFO(rclcpp::get_logger("motor_interface"), " get position is %llf \n", hw_states_positions_[i]);
    }

    // RCLCPP_INFO(rclcpp::get_logger("motor_interface"), "period_update_rate write is %lld \n", period.nanoseconds());

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Motor::write(
      const rclcpp::Time &time /*time*/, const rclcpp::Duration &period /*period*/)
  {

    // TODO(anyone): write robot's commands'
    for (std::size_t i = 0; i < motor_instance_vector_.size(); i++)
    {
      motor_instance_vector_.at(i)->motor_data_set_.Set_Current = (int16_t)(hw_commands_currents_.at(i));
    }
    // RCLCPP_INFO(rclcpp::get_logger("motor_interface"), "update_rate write is %lld \n", period.nanoseconds());

    can_instance_->Write();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn Motor::on_shutdown(
      const rclcpp_lifecycle::State &previous_state)
  {
    can_instance_->~Can_Instance();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

} // namespace motor_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    motor_interface::Motor, hardware_interface::SystemInterface)
