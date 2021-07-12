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


#include <memory>
#include <vector>

#include "trajectory_follower/mpc_utils.hpp"
#include "trajectory_follower/mpc_trajectory.hpp"

#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "autoware_auto_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gtest/gtest.h"

namespace
{
namespace MPCUtils = ::autoware::motion::control::trajectory_follower::MPCUtils;
typedef autoware_auto_msgs::msg::Trajectory Trajectory;
typedef autoware_auto_msgs::msg::TrajectoryPoint TrajectoryPoint;
typedef geometry_msgs::msg::Pose Pose;
typedef geometry_msgs::msg::PoseStamped PoseStamped;

TEST(test_mpc_utils, calculate_distance) {
  PoseStamped p0;
  p0.pose.position.x = 0.0;
  p0.pose.position.y = 0.0;
  p0.pose.position.z = 0.0;
  PoseStamped p1;
  p1.pose.position.x = 10.0;
  p1.pose.position.y = 0.0;
  p1.pose.position.z = 10.0;
  PoseStamped p2;
  p2.pose.position.x = 0.0;
  p2.pose.position.y = 10.0;
  p2.pose.position.z = 10.0;
  // calcDist2d (PoseStamped)
  EXPECT_EQ(MPCUtils::calcDist2d(p0, p0), 0.0);
  EXPECT_EQ(MPCUtils::calcDist2d(p0, p1), 10.0);
  EXPECT_EQ(MPCUtils::calcDist2d(p0, p2), 10.0);
  EXPECT_EQ(MPCUtils::calcDist2d(p1, p2), std::sqrt(200.0));
  // calcDist2d (Pose)
  EXPECT_EQ(MPCUtils::calcDist2d(p0.pose, p0.pose), 0.0);
  EXPECT_EQ(MPCUtils::calcDist2d(p0.pose, p1.pose), 10.0);
  EXPECT_EQ(MPCUtils::calcDist2d(p0.pose, p2.pose), 10.0);
  EXPECT_EQ(MPCUtils::calcDist2d(p1.pose, p2.pose), std::sqrt(200.0));
  // calcSquaredDist2d (Point)
  EXPECT_EQ(MPCUtils::calcSquaredDist2d(p0.pose.position, p0.pose.position), 0.0);
  EXPECT_EQ(MPCUtils::calcSquaredDist2d(p0.pose.position, p1.pose.position), 100.0);
  EXPECT_EQ(MPCUtils::calcSquaredDist2d(p0.pose.position, p2.pose.position), 100.0);
  EXPECT_EQ(MPCUtils::calcSquaredDist2d(p1.pose.position, p2.pose.position), 200.0);
  // calcDist3d (Point)
  EXPECT_EQ(MPCUtils::calcDist3d(p0.pose.position, p0.pose.position), 0.0);
  EXPECT_EQ(MPCUtils::calcDist3d(p0.pose.position, p1.pose.position), std::sqrt(200.0));
  EXPECT_EQ(MPCUtils::calcDist3d(p0.pose.position, p2.pose.position), std::sqrt(200.0));
  EXPECT_EQ(MPCUtils::calcDist3d(p1.pose.position, p2.pose.position), std::sqrt(200.0));
}

/* cppcheck-suppress syntaxError */
TEST(test_mpc_utils, calcNearestIndex) {
  Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  Trajectory trajectory;
  TrajectoryPoint p;
  p.x = -2.0;
  p.y = 1.0;
  trajectory.points.push_back(p);
  p.x = -1.0;
  p.y = 1.0;
  trajectory.points.push_back(p);
  p.x = 0.0;
  p.y = 1.0;
  trajectory.points.push_back(p);
  p.x = 1.0;
  p.y = 1.0;
  trajectory.points.push_back(p);
  EXPECT_EQ(MPCUtils::calcNearestIndex(trajectory, pose), 2);
}

TEST(test_mpc, calcStopDistance) {
  using autoware_auto_msgs::msg::Trajectory;
  using autoware_auto_msgs::msg::TrajectoryPoint;

  Trajectory trajectory_msg;
  TrajectoryPoint p;
  // Point 0
  p.x = 0.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 1
  p.x = 1.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 2 - STOP
  p.x = 2.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  trajectory_msg.points.push_back(p);
  // Point 3
  p.x = 3.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 4
  p.x = 4.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 5
  p.x = 5.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 6 - STOP
  p.x = 6.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  trajectory_msg.points.push_back(p);
  // Point 7 - STOP
  p.x = 7.0;
  p.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  trajectory_msg.points.push_back(p);

  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 0), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 1), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 2), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 3), 3.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 4), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 5), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 6), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 7), -1.0);
}
}  // namespace
