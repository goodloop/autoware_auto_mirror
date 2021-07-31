// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "vesc_interface/vesc_interface.hpp"

#include <iostream>

namespace autoware
{
namespace vesc_interface
{
  VESCInterface::VESCInterface(
    rclcpp::Node &node
  )
  : m_logger{node.get_logger()}
  {
    /// \todo : Write up the initializer.

    // get conversion parameters
    speed_to_erpm_gain_ = node.declare_parameter("speed_to_erpm_gain").get<double>();
    speed_to_erpm_offset_ = node.declare_parameter("speed_to_erpm_offset").get<double>();
    steering_to_servo_gain_ = node.declare_parameter("steering_angle_to_servo_gain").get<double>();
    steering_to_servo_offset_ = node.declare_parameter("steering_angle_to_servo_offset").get<double>();

    // create publishers to vesc electric-RPM (speed) and servo commands
    erpm_pub_ = node.create_publisher<Float64>("commands/motor/speed", 10);
    servo_pub_ = node.create_publisher<Float64>("commands/servo/position", 10);
  }

  bool8_t VESCInterface::send_control_command(const VehicleControlCommand &msg)
  {
    // calc vesc electric RPM (speed)
    Float64 erpm_msg;
    erpm_msg.data = speed_to_erpm_gain_ * msg.velocity_mps + speed_to_erpm_offset_;

    // calc steering angle (servo)
    Float64 servo_msg;
    servo_msg.data = steering_to_servo_gain_ * msg.front_wheel_angle_rad + steering_to_servo_offset_;

    if (rclcpp::ok())
    {
      erpm_pub_->publish(erpm_msg);
      servo_pub_->publish(servo_msg);
      return true;
    }
    return false;
  }

  bool8_t VESCInterface::update(std::chrono::nanoseconds timeout)
  {
    return true;
  }

  bool8_t VESCInterface::send_control_command(const RawControlCommand &msg)
  {
    /// \todo: Log Error, Not Implemented.
    RCLCPP_WARN(m_logger, "Cannot control the VESC using RawControlCommand");
  }
}  // namespace vesc_interface
}  // namespace autoware
