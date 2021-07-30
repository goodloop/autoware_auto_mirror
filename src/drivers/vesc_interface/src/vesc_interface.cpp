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
