// Copyright 2021 The Autoware Foundation
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

#include "vehicle_constants_manager_nodes/vehicle_constants_manager_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <common/types.hpp>

#include <map>
#include <string>
#include <utility>

namespace autoware
{
namespace common
{
namespace vehicle_constants_manager_node
{

using common::types::float64_t;

VehicleConstantsManagerNode::VehicleConstantsManagerNode(const rclcpp::NodeOptions & options)
:  Node("vehicle_constants_manager_node", options)
{
  // Declare params from yaml/inputs to ROS2 parameter server
  using ParamsPrimary = vehicle_constants_manager::VehicleConstants::ParamsPrimary;
  std::map<ParamsPrimary, float64_t> map_params_primary;
  for (ParamsPrimary param = ParamsPrimary::wheel_radius;
    param != ParamsPrimary::last;
    param = static_cast<ParamsPrimary>(static_cast<int>(param) + 1))
  {
    map_params_primary.insert(
      std::make_pair(
        param,
        this->declare_parameter(
          vehicle_constants_manager::VehicleConstants::map_names_primary.at(
            param)).get<float64_t>()));
  }
  // Build the VehicleConstants object
  auto vc = vehicle_constants_manager::VehicleConstants(map_params_primary);


  // Publish the derived parameters to the parameter server
  using ParamsDerived = vehicle_constants_manager::VehicleConstants::ParamsDerived;
  for (ParamsDerived param = ParamsDerived::cg_to_front;
    param != ParamsDerived::last;
    param = static_cast<ParamsDerived>(static_cast<int>(param) + 1))
  {
    this->declare_parameter<float64_t>(
      vehicle_constants_manager::VehicleConstants::map_names_derived.at(param),
      vc.map_params_derived.at(param),
      rcl_interfaces::msg::ParameterDescriptor(),
      true);
  }

  this->declare_parameter<types::bool8_t>(
    "published_all",
    true,
    rcl_interfaces::msg::ParameterDescriptor(),
    true);

  // Pretty print
  RCLCPP_INFO_STREAM(get_logger(), "vehicle constants: \n" << vc.str_pretty());
}

}  // namespace vehicle_constants_manager_node
}  // namespace common
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::common::vehicle_constants_manager_node::VehicleConstantsManagerNode)
