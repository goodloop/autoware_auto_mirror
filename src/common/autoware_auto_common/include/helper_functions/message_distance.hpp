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
/// \file
/// \brief This file includes common distance functions for ROS messages

#ifndef HELPER_FUNCTIONS__MESSAGE_DISTANCE_HPP_
#define HELPER_FUNCTIONS__MESSAGE_DISTANCE_HPP_

#include "common/visibility_control.hpp"
#include "common/types.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace autoware
{
namespace common
{
namespace helper_functions
{
using autoware::common::types::float64_t;
/**
 * @brief calculate the squared 2d distance between the two given Point
 * @param [in] p0 input Point message
 * @param [in] p1 input Point message
 * @return distance between the two points
 */
COMMON_PUBLIC inline float64_t calcSquaredDist2d(
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1)
{
  const float64_t dx = p1.x - p0.x;
  const float64_t dy = p1.y - p0.y;
  return dx * dx + dy * dy;
}
/**
 * @brief calculate the 2d distance between the given points
 * @tparam T0 a point type with .x and .y members
 * @tparam T1 a point type with .x and .y members
 * @param [in] p0 input point
 * @param [in] p1 input point
 * @return distance between the two points
 */
template<typename T0, typename T1>
COMMON_PUBLIC inline float64_t calcDist2d(const T0 & p0, const T1 & p1)
{
  return std::hypot(p0.x - p1.x, p0.y - p1.y);
}
/**
 * @brief calculate the 3d distance between the two given Point
 * @param [in] p0 input Point message
 * @param [in] p1 input Point message
 * @return distance between the two points
 */
COMMON_PUBLIC inline float64_t calcDist3d(
  const geometry_msgs::msg::Point & p0,
  const geometry_msgs::msg::Point & p1)
{
  const float64_t dx = p1.x - p0.x;
  const float64_t dy = p1.y - p0.y;
  const float64_t dz = p1.z - p0.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
/**
 * @brief calculate the 2d distance between the two given Pose
 * @param [in] p0 input Pose message
 * @param [in] p1 input Pose message
 * @return distance between the two pose
 */
COMMON_PUBLIC inline float64_t calcDist2d(
  const geometry_msgs::msg::Pose & p0,
  const geometry_msgs::msg::Pose & p1)
{
  return calcDist2d(p0.position, p1.position);
}
/**
 * @brief calculate the 2d distance between the two given PoseStamped
 * @param [in] p0 input PoseStamped message
 * @param [in] p1 input PoseStamped message
 * @return distance between the two pose
 */
COMMON_PUBLIC inline float64_t calcDist2d(
  const geometry_msgs::msg::PoseStamped & p0, const geometry_msgs::msg::PoseStamped & p1)
{
  return calcDist2d(p0.pose.position, p1.pose.position);
}
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__MESSAGE_DISTANCE_HPP_
