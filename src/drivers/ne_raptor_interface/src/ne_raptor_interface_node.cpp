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

#include "ne_raptor_interface/ne_raptor_interface_node.hpp"
#include "ne_raptor_interface/ne_raptor_interface.hpp"

#include <common/types.hpp>

#include <memory>
#include <string>
#include <unordered_set>

namespace autoware
{
namespace ne_raptor_interface
{

using autoware::common::types::float32_t;
using autoware::drivers::vehicle_interface::ViFeature;

NERaptorInterfaceNode::NERaptorInterfaceNode(const rclcpp::NodeOptions & options)
: VehicleInterfaceNode{
    "ne_raptor_interface",
    std::unordered_set<ViFeature> {
        ViFeature::HEADLIGHTS,
        ViFeature::HORN,
        ViFeature::WIPERS,
      },
    options
}
{
  set_interface(
    std::make_unique<NERaptorInterface>(
      *this,
      declare_parameter<int>("ne_raptor.ecu_build_num"),
      declare_parameter<double>("ne_raptor.front_axle_to_cog"),
      declare_parameter<double>("ne_raptor.rear_axle_to_cog"),
      declare_parameter<double>("ne_raptor.steer_to_tire_ratio"),
      declare_parameter<double>("ne_raptor.max_steer_angle"),
      get_state_machine().get_config().accel_limits().max(),
      get_state_machine().get_config().accel_limits().min(),
      declare_parameter<double>("ne_raptor.acceleration_positive_jerk_limit"),
      declare_parameter<double>("ne_raptor.deceleration_negative_jerk_limit"),
      declare_parameter<int>("ne_raptor.pub_period")
  ));
}

}  // namespace ne_raptor_interface
}  // namespace autoware
#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ne_raptor_interface::NERaptorInterfaceNode)
