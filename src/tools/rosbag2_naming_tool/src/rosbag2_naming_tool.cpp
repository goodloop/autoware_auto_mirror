// Copyright 2020 TierIV, Inc.
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

#include "rosbag2_naming_tool/rosbag2_naming_tool.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cctype>
#include <memory>
#include <regex>
#include <string>
#include <vector>

rosbag2_naming_tool::NamingToolNode::NamingToolNode(
  const rclcpp::NodeOptions & options)
: Node("rosbag2_naming_tool", options)
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(rosbag2_naming_tool::NamingToolNode)
