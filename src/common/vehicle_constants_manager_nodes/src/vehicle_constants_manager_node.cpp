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
#include <common/types.hpp>
#include <string>
#include <vector>

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
  // Get primary parameters to construct the VehicleConstants object
  std::vector<std::string> names_parameters{
    "wheel_radius",
    "wheel_width",
    "wheel_base",
    "wheel_tread",
    "overhang_front",
    "overhang_rear",
    "overhang_left",
    "overhang_right",
    "vehicle_height",
    "cg_to_rear",
    "tire_cornering_stiffness_front_N_per_deg",
    "tire_cornering_stiffness_rear_N_per_deg",
    "mass_vehicle",
    "inertia_yaw_kg_m2"
  };
  std::vector<float64_t> parameters(names_parameters.size());
  assert(parameters.size() == names_parameters.size());

  for (size_t i = 0; i < names_parameters.size(); ++i) {
    const auto & name_parameter = names_parameters.at(i);
    parameters.at(i) = this->declare_parameter(name_parameter).get<float64_t>();
  }

  vehicle_constants_manager::VehicleConstants vc(
    parameters.at(0),
    parameters.at(1),
    parameters.at(2),
    parameters.at(3),
    parameters.at(4),
    parameters.at(5),
    parameters.at(6),
    parameters.at(7),
    parameters.at(8),
    parameters.at(9),
    parameters.at(10),
    parameters.at(11),
    parameters.at(12),
    parameters.at(13));

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
