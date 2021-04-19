// Copyright 2021 the Autoware Foundation
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <vehicle_parameters_node/vehicle_parameters_node.hpp>

namespace autoware
{
namespace vehicle_parameters_node
{

VehicleParametersNode::VehicleParametersNode(const rclcpp::NodeOptions & options)
: Node("vehicle_parameters_node", options) {}

} // namespace vehicle_parameters_node
} // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::vehicle_parameters_node::VehicleParametersNode)
