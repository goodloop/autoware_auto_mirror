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

#include "vesc_interface/vesc_interface_node.hpp"

#include <memory>
#include <unordered_set>

namespace autoware
{
namespace vesc_interface
{

using autoware::drivers::vehicle_interface::ViFeature;

VESCInterfaceNode::VESCInterfaceNode(const rclcpp::NodeOptions & options)
:  VehicleInterfaceNode{"vesc_interface", std::unordered_set<ViFeature>{}, options}
{
  // Set up interface
  set_interface(
    std::make_unique<VESCInterface>(
      *this,
      declare_parameter<float64_t>("vesc.speed_to_erpm_gain"),
      declare_parameter<float64_t>("vesc.speed_to_erpm_offset"),
      declare_parameter<float64_t>("vesc.steering_angle_to_servo_gain"),
      declare_parameter<float64_t>("vesc.steering_angle_to_servo_offset")
    )
  );
}

}  // namespace vesc_interface
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::vesc_interface::VESCInterfaceNode)
