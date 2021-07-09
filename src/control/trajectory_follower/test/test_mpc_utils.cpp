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

#include "gtest/gtest.h"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "autoware_auto_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace
{
namespace MPCUtils = ::autoware::motion::control::trajectory_follower::MPCUtils;
typedef autoware::motion::control::trajectory_follower::MPCTrajectory MPCTrajectory;
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
TEST(test_mpc_utils, splineInterpMPCTrajectory) {
  std::vector<double> in_index = {0.0, 1.0, 2.0};
  std::vector<double> out_index = {0.5, 1.5};
  MPCTrajectory traj;
  traj.push_back(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  traj.push_back(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
  traj.push_back(2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0);
  MPCTrajectory out_traj;
  MPCUtils::splineInterpMPCTrajectory(in_index, traj, out_index, &out_traj);
  ASSERT_EQ(out_traj.size(), out_index.size());
  // NOTE: only the relative_time value is not interpolated
  EXPECT_EQ(out_traj.x[0], 0.5);
  EXPECT_EQ(out_traj.y[0], 0.5);
  EXPECT_EQ(out_traj.z[0], 0.5);
  EXPECT_EQ(out_traj.yaw[0], 0.5);
  EXPECT_EQ(out_traj.vx[0], 0.5);
  EXPECT_EQ(out_traj.k[0], 0.5);
  EXPECT_EQ(out_traj.smooth_k[0], 0.5);
  EXPECT_EQ(out_traj.x[1], 1.5);
  EXPECT_EQ(out_traj.y[1], 1.5);
  EXPECT_EQ(out_traj.z[1], 1.5);
  EXPECT_EQ(out_traj.yaw[1], 1.5);
  EXPECT_EQ(out_traj.vx[1], 1.5);
  EXPECT_EQ(out_traj.k[1], 1.5);
  EXPECT_EQ(out_traj.smooth_k[1], 1.5);
}
}  // namespace
