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

#include "outlier_filter_nodes/outlier_filter_node.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter_nodes
{

OutlierFilterNode::OutlierFilterNode(const rclcpp::NodeOptions & options)
:  Node("outlier_filter_node", options),
  verbose(true)
{
  print_hello();
}

int32_t OutlierFilterNode::print_hello() const
{
  return outlier_filter::print_hello();
}

}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::perception::filters::outlier_filter_nodes::OutlierFilterNode)
