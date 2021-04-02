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

#include "ne_raptor_interface/test_ne_raptor_interface_talker.hpp"
#include <memory>

namespace autoware
{
namespace ne_raptor_interface
{
NERaptorInterfaceTalker::NERaptorInterfaceTalker(
  rclcpp::Node & node
)
{
  // Publishers (from Raptor DBW)
  t_brake_rpt_pub = node.create_publisher<BrakeReport>("brake_report", 20);
  t_gear_rpt_pub = node.create_publisher<GearReport>("gear_report", 20);
  t_misc_rpt_pub = node.create_publisher<MiscReport>("misc_report", 2);
  t_other_acts_rpt_pub = node.create_publisher<DriverInputReport>("driver_input_report", 2);
  t_steering_rpt_pub = node.create_publisher<SteeringReport>("steering_report", 20);
  t_wheel_spd_rpt_pub = node.create_publisher<WheelSpeedReport>("wheel_speed_report", 20);
  t_dbw_state_rpt_pub = node.create_publisher<std_msgs::msg::Bool>("dbw_enabled", 1);
}

bool8_t NERaptorInterfaceTalker::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  return true;
}

bool8_t NERaptorInterfaceTalker::send_report(const BrakeReport & msg)
{
  t_brake_rpt_pub->publish(msg);
  return true;
}

bool8_t NERaptorInterfaceTalker::send_report(const GearReport & msg)
{
  t_gear_rpt_pub->publish(msg);
  return true;
}

bool8_t NERaptorInterfaceTalker::send_report(const MiscReport & msg)
{
  t_misc_rpt_pub->publish(msg);
  return true;
}

bool8_t NERaptorInterfaceTalker::send_report(const DriverInputReport & msg)
{
  t_other_acts_rpt_pub->publish(msg);
  return true;
}

bool8_t NERaptorInterfaceTalker::send_report(const SteeringReport & msg)
{
  t_steering_rpt_pub->publish(msg);
  return true;
}

bool8_t NERaptorInterfaceTalker::send_report(const WheelSpeedReport & msg)
{
  t_wheel_spd_rpt_pub->publish(msg);
  return true;
}

bool8_t NERaptorInterfaceTalker::send_report(const std_msgs::msg::Bool & msg)
{
  t_dbw_state_rpt_pub->publish(msg);
  return true;
}

}  // namespace ne_raptor_interface
}  // namespace autoware
