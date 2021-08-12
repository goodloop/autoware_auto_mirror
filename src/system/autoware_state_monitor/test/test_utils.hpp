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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"

#include "autoware_auto_msgs/msg/complex32.hpp"
#include "autoware_auto_msgs/msg/engage.hpp"
#include "autoware_auto_msgs/msg/had_map_route.hpp"
#include "autoware_auto_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_msgs/msg/vehicle_state_report.hpp"

using autoware_auto_msgs::msg::Complex32;
using autoware_auto_msgs::msg::Engage;
using autoware_auto_msgs::msg::RoutePoint;
using autoware_auto_msgs::msg::VehicleOdometry;
using autoware_auto_msgs::msg::VehicleStateReport;

using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Quaternion;

inline double distance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  const auto dx = p1.x - p2.x;
  const auto dy = p1.y - p2.y;
  const auto dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline int64_t nanosec(double sec)
{
  return static_cast<int64_t>(sec * 1'000'000'000.0);
}

inline rclcpp::Time toTime(double sec)
{
  return rclcpp::Time(nanosec(sec));
}

inline Engage::SharedPtr prepareEngageMsg(bool engage, rclcpp::Time stamp = {0, 0})
{
  auto msg = std::make_shared<Engage>();
  msg->engage = engage;
  msg->stamp = stamp;
  return msg;
}

inline VehicleStateReport::SharedPtr prepareVehicleStateReportMsg(bool autonomous_mode)
{
  auto msg = std::make_shared<VehicleStateReport>();
  if (autonomous_mode) {
    msg->mode = VehicleStateReport::MODE_AUTONOMOUS;
  } else {
    msg->mode = VehicleStateReport::MODE_MANUAL;
  }
  return msg;
}

inline Point getPoint(float x = 0, float y = 0, float z = 0)
{
  Point pt;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  return pt;
}

inline PoseStamped::SharedPtr preparePoseStampedMsg(
  const Point & position, const Quaternion & orientation = Quaternion())
{
  auto msg = std::make_shared<PoseStamped>();
  msg->pose.position = position;
  msg->pose.orientation = orientation;
  return msg;
}

inline RoutePoint::SharedPtr prepareRoutePointMsg(
  const Point & position, const Complex32 & heading = Complex32())
{
  auto msg = std::make_shared<RoutePoint>();
  msg->position = position;
  msg->heading = heading;
  return msg;
}

inline VehicleOdometry::SharedPtr prepareVehicleOdometryMsg(
  const float velocity_mps = 0, const float front_wheel_angle_rad = 0,
  const float rear_wheel_angle_rad = 0,
  const rclcpp::Time & stamp = {0, 0})
{
  auto msg = std::make_shared<VehicleOdometry>();
  msg->velocity_mps = velocity_mps;
  msg->front_wheel_angle_rad = front_wheel_angle_rad;
  msg->rear_wheel_angle_rad = rear_wheel_angle_rad;
  msg->stamp = stamp;
  return msg;
}

#endif  // TEST_UTILS_HPP_
