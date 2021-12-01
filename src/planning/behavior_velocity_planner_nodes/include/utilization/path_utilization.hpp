// Copyright 2020 Tier IV, Inc.
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

#ifndef UTILIZATION__PATH_UTILIZATION_HPP_
#define UTILIZATION__PATH_UTILIZATION_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_msgs/msg/path.hpp"

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{
autoware_auto_msgs::msg::Path interpolatePath(
  const autoware_auto_msgs::msg::Path & path, const double length,
  const rclcpp::Logger & logger);
autoware_auto_msgs::msg::Path filterLitterPathPoint(
  const autoware_auto_msgs::msg::Path & path);
autoware_auto_msgs::msg::Path filterStopPathPoint(
  const autoware_auto_msgs::msg::Path & path);
}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware

#endif  // UTILIZATION__PATH_UTILIZATION_HPP_
