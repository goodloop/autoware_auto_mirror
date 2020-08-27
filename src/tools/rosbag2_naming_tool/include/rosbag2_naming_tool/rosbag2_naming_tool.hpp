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

#ifndef ROSBAG2_NAMING_TOOL__ROSBAG2_NAMING_TOOL_HPP_
#define ROSBAG2_NAMING_TOOL__ROSBAG2_NAMING_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>

namespace rosbag2_naming_tool
{
class NamingToolNode : public rclcpp::Node
{
public:
  explicit NamingToolNode(const rclcpp::NodeOptions & options);
};

}  // namespace rosbag2_moriyama_converter

#endif  // ROSBAG2_NAMING_TOOL__ROSBAG2_NAMING_TOOL_HPP_
