// Copyright (c) 2026 Franka Robotics GmbH
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

#pragma once

#include <array>
#include <mutex>
#include <string>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

class JointTorqueController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void torque_command_callback(const std_msgs::msg::Float64MultiArray& msg);
  void set_zero_torques();

  std::string robot_type_;
  std::string arm_prefix_;
  std::string command_topic_;
  static constexpr size_t kNumJoints = 7;
  std::array<double, kNumJoints> commanded_torques_{};
  bool has_received_command_{false};
  std::mutex command_mutex_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration command_timeout_{0, 10000000};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_command_subscriber_;
};

}  // namespace franka_example_controllers
