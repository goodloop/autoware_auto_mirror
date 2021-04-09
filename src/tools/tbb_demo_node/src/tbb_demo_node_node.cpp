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

#include "tbb_demo_node/tbb_demo_node_node.hpp"

namespace autoware
{
namespace tbb_demo_node
{

TbbDemoNodeNode::TbbDemoNodeNode(const rclcpp::NodeOptions & options)
:  Node("tbb_demo_node", options),
  verbose(true)
{
  print_hello();
}

int32_t TbbDemoNodeNode::print_hello() const
{
  return tbb_demo_node::print_hello();
}

}  // namespace tbb_demo_node
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tbb_demo_node::TbbDemoNodeNode)
