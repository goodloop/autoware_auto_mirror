// Copyright 2021 Robotec.ai
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

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "autoware_auto_msgs/msg/complex32.hpp"
#include "autoware_auto_msgs/msg/engage.hpp"
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

Engage::SharedPtr prepareEngageMsg(bool engage, rclcpp::Time stamp = {0, 0})
{
  auto msg = std::make_shared<Engage>();
  msg->engage = true;
  msg->stamp = stamp;
  return msg;
}

VehicleStateReport::SharedPtr prepareVehicleStateReportMsg(bool autonomous_mode)
{
  auto msg = std::make_shared<VehicleStateReport>();
  if (autonomous_mode) {
    msg->mode = VehicleStateReport::MODE_AUTONOMOUS;
  }
  else {
    msg->mode = VehicleStateReport::MODE_MANUAL;
  }
  return msg;
}

Point getPoint(float x = 0, float y = 0, float z = 0)
{
  Point pt;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  return pt;
}

PoseStamped::SharedPtr preparePoseStampedMsg(
  const Point & position, const Quaternion & orientation = Quaternion())
{
  auto msg = std::make_shared<PoseStamped>();
  msg->pose.position = position;
  msg->pose.orientation = orientation;
  return msg;
}

RoutePoint::SharedPtr prepareRoutePointMsg(
  const Point & position, const Complex32 & heading = Complex32())
{
  auto msg = std::make_shared<RoutePoint>();
  msg->position = position;
  msg->heading = heading;
  return msg;
}

VehicleOdometry::SharedPtr prepareVehicleOdometryMsg(
  const float velocity_mps = 0, const float front_wheel_angle_rad = 0,
  const float rear_wheel_angle_rad = 0)
{
  auto msg = std::make_shared<VehicleOdometry>();
  msg->velocity_mps = velocity_mps;
  msg->front_wheel_angle_rad = front_wheel_angle_rad;
  msg->rear_wheel_angle_rad = rear_wheel_angle_rad;
  return msg;
}