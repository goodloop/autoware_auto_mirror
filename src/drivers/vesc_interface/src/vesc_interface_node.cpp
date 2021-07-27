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

namespace autoware
{
namespace vesc_interface
{

VescInterfaceNode::VescInterfaceNode(const rclcpp::NodeOptions & options)
:  Node("vesc_interface", options),
  verbose(true)
{
  print_hello();
}

int32_t VescInterfaceNode::print_hello() const
{
  return vesc_interface::print_hello();
}

}  // namespace vesc_interface
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::vesc_interface::VescInterfaceNode)
