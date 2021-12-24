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

#include <vector>

#include "astar_search/base_planning_algorithm.hpp"

namespace autoware
{
namespace planning
{
namespace astar_search
{
double normalizeRadian(
  const double rad, const double min_rad, const double max_rad)
{
  const auto value = std::fmod(rad, 2 * M_PI);
  if (min_rad < value && value <= max_rad) {
    return value;
  } else {
    return value - std::copysign(2 * M_PI, value);
  }
}
int discretizeAngle(const double theta, const size_t theta_size)
{
  const double one_angle_range = 2.0 * M_PI / static_cast<double>(theta_size);
  return static_cast<int>(normalizeRadian(
           theta, 0.0,
           2.0 * M_PI) / one_angle_range) % static_cast<int>(theta_size);
}

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  geometry_msgs::msg::PoseStamped pose_orig;
  pose_orig.pose = pose;
  tf2::doTransform(pose_orig, transformed_pose, transform);

  return transformed_pose.pose;
}

geometry_msgs::msg::Pose global2local(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_global)
{
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin.inverse());

  return transformPose(pose_global, transform);
}

geometry_msgs::msg::Pose local2global(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local)
{
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin);

  return transformPose(pose_local, transform);
}

IndexXYT pose2index(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local,
  const size_t theta_size)
{
  const auto resolution = static_cast<double>(costmap.info.resolution);
  const int index_x = static_cast<int>(std::floor(pose_local.position.x / resolution));
  const int index_y = static_cast<int>(std::floor(pose_local.position.y / resolution));
  const int index_theta = discretizeAngle(tf2::getYaw(pose_local.orientation), theta_size);
  return {index_x, index_y, index_theta};
}

geometry_msgs::msg::Pose index2pose(
  const nav_msgs::msg::OccupancyGrid & costmap, const IndexXYT & index, const size_t theta_size)
{
  geometry_msgs::msg::Pose pose_local;

  pose_local.position.x = static_cast<float>(index.x) * costmap.info.resolution;
  pose_local.position.y = static_cast<float>(index.y) * costmap.info.resolution;

  const double one_angle_range = 2.0 * M_PI / static_cast<double>(theta_size);
  const double yaw = index.theta * one_angle_range;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw);
  tf2::convert(quat, pose_local.orientation);

  return pose_local;
}

void BasePlanningAlgorithm::setOccupancyGrid(const nav_msgs::msg::OccupancyGrid & costmap)
{
  costmap_ = costmap;
  const auto height = costmap_.info.height;
  const auto width = costmap_.info.width;

  // Initialize status
  std::vector<std::vector<bool>> is_obstacle_table;
  is_obstacle_table.resize(height);
  for (uint32_t i = 0; i < height; i++) {
    is_obstacle_table.at(i).resize(width);
    for (uint32_t j = 0; j < width; j++) {
      const int cost = costmap_.data[i * width + j];

      if (cost < 0 || astar_param_.obstacle_threshold <= cost) {
        is_obstacle_table[i][j] = true;
      }
    }
  }
  is_obstacle_table_ = is_obstacle_table;
}


bool BasePlanningAlgorithm::hasObstacleOnTrajectory(const geometry_msgs::msg::PoseArray & trajectory) const
{
  for (const auto & pose : trajectory.poses) {
    const auto pose_local = global2local(costmap_, pose);
    const auto index = pose2index(costmap_, pose_local, astar_param_.theta_size);

    if (detectCollision(index)) {
      return true;
    }
  }

  return false;
}


bool BasePlanningAlgorithm::detectCollision(const IndexXYT & base_index) const
{
  const RobotShape & robot_shape = astar_param_.robot_shape;

  // Define the robot as rectangle
  const double back = -1.0 * robot_shape.cg2back;
  const double front = robot_shape.length - robot_shape.cg2back;
  const double right = -1.0 * robot_shape.width / 2.0;
  const double left = robot_shape.width / 2.0;

  const auto base_pose = index2pose(costmap_, base_index, astar_param_.theta_size);
  const auto base_theta = tf2::getYaw(base_pose.orientation);

  // Convert each point to index and check if the node is Obstacle
  const auto costmap_resolution = static_cast<double>(costmap_.info.resolution);
  for (double x = back; x <= front; x += costmap_resolution) {
    for (double y = right; y <= left; y += costmap_resolution) {
      // Calculate offset in rotated frame
      const double offset_x = std::cos(base_theta) * x - std::sin(base_theta) * y;
      const double offset_y = std::sin(base_theta) * x + std::cos(base_theta) * y;

      geometry_msgs::msg::Pose pose_local;
      pose_local.position.x = base_pose.position.x + offset_x;
      pose_local.position.y = base_pose.position.y + offset_y;

      const auto index = pose2index(costmap_, pose_local, astar_param_.theta_size);

      if (isOutOfRange(index)) {
        return true;
      }

      if (isObs(index)) {
        return true;
      }
    }
  }

  return false;
}


}  // namespace astar_search
}  // namespace planning
}  // namespace autoware
