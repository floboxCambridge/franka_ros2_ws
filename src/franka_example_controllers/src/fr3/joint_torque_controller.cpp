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

#include <franka_example_controllers/fr3/joint_torque_controller.hpp>

#include <algorithm>
#include <exception>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointTorqueController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointTorqueController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::return_type JointTorqueController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
  std::array<double, kNumJoints> torques{};
  bool command_is_stale = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_is_stale = has_received_command_ && ((time - last_command_time_) > command_timeout_);
    torques = command_is_stale ? std::array<double, kNumJoints>{} : commanded_torques_;
  }

  for (size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(torques[i]);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointTorqueController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::string>("command_topic", "~/torques");
    auto_declare<double>("command_timeout", 0.01);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  set_zero_torques();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  command_timeout_ = rclcpp::Duration::from_seconds(
      get_node()->get_parameter("command_timeout").as_double());

  torque_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray& msg) { torque_command_callback(msg); });

  set_zero_torques();
  RCLCPP_INFO(get_node()->get_logger(),
              "Listening for joint torques on topic '%s' with a %.3f s timeout.",
              torque_command_subscriber_->get_topic_name(), command_timeout_.seconds());
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTorqueController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  set_zero_torques();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTorqueController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  set_zero_torques();
  return CallbackReturn::SUCCESS;
}

void JointTorqueController::torque_command_callback(
    const std_msgs::msg::Float64MultiArray& msg) {
  if (msg.data.size() != kNumJoints) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                          "Expected %zu torque values but received %zu. Command ignored.",
                          kNumJoints, msg.data.size());
    return;
  }

  std::lock_guard<std::mutex> lock(command_mutex_);
  std::copy_n(msg.data.cbegin(), kNumJoints, commanded_torques_.begin());
  last_command_time_ = get_node()->now();
  has_received_command_ = true;
}

void JointTorqueController::set_zero_torques() {
  std::lock_guard<std::mutex> lock(command_mutex_);
  commanded_torques_.fill(0.0);
  has_received_command_ = false;
  if (get_node()) {
    last_command_time_ = get_node()->now();
  }
}

}  // namespace franka_example_controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointTorqueController,
                       controller_interface::ControllerInterface)
