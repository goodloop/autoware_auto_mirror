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
//
/// \file
/// \brief This file includes tests for functions in trajectory_common

#include <experimental/optional>
#include <cmath>

#include "autoware_auto_msgs/msg/complex32.hpp"
#include "common/types.hpp"
#include "gtest/gtest.h"
#include "helper_functions/angle_utils.hpp"
#include "motion_common/trajectory_common.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using autoware::common::types::float64_t;

TEST(trajectory_common_tests, validateNonEmpty) {
  using autoware::motion::motion_common::validateNonEmpty;
  using autoware::motion::motion_common::Point;
  using autoware::motion::motion_common::Points;

  Points points;
  EXPECT_THROW(validateNonEmpty(points), std::invalid_argument);

  points.emplace_back();
  EXPECT_NO_THROW(validateNonEmpty(points));
}

TEST(trajectory_common_tests, calcYawDeviation) {
  using autoware::motion::motion_common::calcYawDeviation;
  EXPECT_EQ(calcYawDeviation(0.0, 0.0), 0.0);
  EXPECT_EQ(calcYawDeviation(M_PI, 0.0), -M_PI);
  EXPECT_EQ(calcYawDeviation(0.0, M_PI), -M_PI);
  EXPECT_EQ(calcYawDeviation(2 * M_PI, 0.0), 0.0);
  EXPECT_EQ(calcYawDeviation(0.0, 2 * M_PI), 0);
  EXPECT_EQ(calcYawDeviation(3 * M_PI, 0.0), -M_PI);
  EXPECT_EQ(calcYawDeviation(0.0, 3 * M_PI), -M_PI);
  EXPECT_EQ(calcYawDeviation(M_PI, -M_PI), 0.0);
  EXPECT_EQ(calcYawDeviation(M_PI, M_PI_2), -M_PI_2);
  EXPECT_EQ(calcYawDeviation(M_PI_2, M_PI), M_PI_2);
}

TEST(trajectory_common_tests, searchZeroVelocityIndex) {
  using autoware::motion::motion_common::searchZeroVelocityIndex;
  using autoware::motion::motion_common::Point;
  using autoware::motion::motion_common::Points;

  Points points;
  EXPECT_THROW(searchZeroVelocityIndex(points), std::invalid_argument);

  Point p;
  p.longitudinal_velocity_mps = 1.0;
  points.push_back(p);
  EXPECT_EQ(searchZeroVelocityIndex(points), std::experimental::nullopt);

  // Making a trajectory with velocities [1 0 0 2 0]
  p.longitudinal_velocity_mps = 0.0;
  points.push_back(p);
  p.longitudinal_velocity_mps = 0.0;
  points.push_back(p);
  p.longitudinal_velocity_mps = 2.0;
  points.push_back(p);
  p.longitudinal_velocity_mps = 0.0;
  points.push_back(p);
  ASSERT_NE(searchZeroVelocityIndex(points), std::experimental::nullopt);
  EXPECT_EQ(searchZeroVelocityIndex(points).value(), size_t(1));
  ASSERT_NE(searchZeroVelocityIndex(points, 3, 5), std::experimental::nullopt);
  EXPECT_EQ(searchZeroVelocityIndex(points, 3, 5).value(), size_t(4));
  EXPECT_EQ(searchZeroVelocityIndex(points, 0, 1), std::experimental::nullopt);
  EXPECT_EQ(searchZeroVelocityIndex(points, 3, 4), std::experimental::nullopt);

  // Changing the epsilon for zero velocity
  EXPECT_EQ(searchZeroVelocityIndex(points, 0, points.size(), -1.0), std::experimental::nullopt);
  EXPECT_EQ(searchZeroVelocityIndex(points, 0, points.size(), 2.0).value(), size_t(0));
  EXPECT_EQ(searchZeroVelocityIndex(points, 3, 4, 2.0), std::experimental::nullopt);
  EXPECT_EQ(searchZeroVelocityIndex(points, 3, 4, 2.5).value(), size_t(3));
}

TEST(trajectory_common_tests, findNearestIndex) {
  using autoware::motion::motion_common::findNearestIndex;
  using autoware::motion::motion_common::Point;
  using autoware::motion::motion_common::Points;
  using geometry_msgs::msg::Pose;
  using motion::motion_common::from_angle;

  Points points;
  Pose pose;
  tf2::Quaternion quat;

  EXPECT_THROW(findNearestIndex(points, pose), std::invalid_argument);

  // Making a trajectory with positions [(0,0) (1,1) (2,2) (2,4) (4,4)]
  Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.heading = from_angle(M_PI_4);
  points.push_back(p);
  p.x = 1.0;
  p.y = 1.0;
  p.heading = from_angle(M_PI_4);
  points.push_back(p);
  p.x = 2.0;
  p.y = 2.0;
  p.heading = from_angle(M_PI_2);
  points.push_back(p);
  p.x = 2.0;
  p.y = 4.0;
  p.heading = from_angle(0.0);
  points.push_back(p);
  p.x = 4.0;
  p.y = 4.0;
  p.heading = from_angle(0.0);
  points.push_back(p);

  // No limits on the distance/orientation
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  EXPECT_EQ(findNearestIndex(points, pose).value(), size_t(0));

  pose.position.x = -100.0;
  pose.position.y = -100.0;
  EXPECT_EQ(findNearestIndex(points, pose).value(), size_t(0));

  pose.position.x = 4.0;
  pose.position.y = 4.0;
  EXPECT_EQ(findNearestIndex(points, pose).value(), size_t(4));

  pose.position.x = 100.0;
  pose.position.y = 100.0;
  EXPECT_EQ(findNearestIndex(points, pose).value(), size_t(4));

  // limit on the distance
  EXPECT_EQ(findNearestIndex(points, pose, 10.0), std::experimental::nullopt);
  pose.position.x = 10.0;
  pose.position.y = 10.0;
  EXPECT_EQ(findNearestIndex(points, pose, 10.0).value(), size_t(4));

  // limit on the orientation
  quat.setRPY(0.0, 0.0, M_PI_2);
  pose.orientation = tf2::toMsg(quat);
  EXPECT_EQ(findNearestIndex(points, pose, 100.0, 0.1).value(), size_t(2));
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  EXPECT_EQ(findNearestIndex(points, pose, 100.0, 0.1).value(), size_t(2));
  EXPECT_EQ(findNearestIndex(points, pose, 100.0, M_PI_2).value(), size_t(0));
}

TEST(trajectory_common_tests, findNearestSegmentIndex) {
  using autoware::motion::motion_common::findNearestSegmentIndex;
  using autoware::motion::motion_common::Point;
  using autoware::motion::motion_common::Points;
  using geometry_msgs::msg::Pose;

  // TODO
  // size_t findNearestSegmentIndex(const Points & points, const geometry_msgs::msg::Point & point)
}

TEST(trajectory_common_tests, calcLongitudinalOffsetToSegment) {
  using autoware::motion::motion_common::calcLongitudinalOffsetToSegment;
  using autoware::motion::motion_common::Point;
  using autoware::motion::motion_common::Points;

  // TODO
  // size_t findNearestSegmentIndex(const Points & points, const geometry_msgs::msg::Point & point)
}

TEST(trajectory_common_tests, calcSignedArcLength) {
  using autoware::motion::motion_common::calcSignedArcLength;
  using autoware::motion::motion_common::Point;
  using autoware::motion::motion_common::Points;

  // TODO
}

TEST(trajectory_common_tests, calcLongitudinalOffsetToSegment) {
  using autoware::motion::motion_common::calcLongitudinalOffsetToSegment;
  using geometry_msgs::msg::Point;
  using geometry_msgs::msg::Pose;

  // TODO
}
