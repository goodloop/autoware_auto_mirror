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

#include "helper_functions/message_distance.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"

namespace
{
using autoware::common::helper_functions::calcDist2d;
using autoware::common::helper_functions::calcSquaredDist2d;
using autoware::common::helper_functions::calcDist3d;
using geometry_msgs::msg::PoseStamped;
}  // namespace

/// @test       Calculate distances
TEST(TestMessageDistance, Dist2d3d) {
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
  EXPECT_EQ(calcDist2d(p0, p0), 0.0);
  EXPECT_EQ(calcDist2d(p0, p1), 10.0);
  EXPECT_EQ(calcDist2d(p0, p2), 10.0);
  EXPECT_EQ(calcDist2d(p1, p2), std::sqrt(200.0));
  // calcDist2d (Pose)
  EXPECT_EQ(calcDist2d(p0.pose, p0.pose), 0.0);
  EXPECT_EQ(calcDist2d(p0.pose, p1.pose), 10.0);
  EXPECT_EQ(calcDist2d(p0.pose, p2.pose), 10.0);
  EXPECT_EQ(calcDist2d(p1.pose, p2.pose), std::sqrt(200.0));
  // calcSquaredDist2d (Point)
  EXPECT_EQ(calcSquaredDist2d(p0.pose.position, p0.pose.position), 0.0);
  EXPECT_EQ(calcSquaredDist2d(p0.pose.position, p1.pose.position), 100.0);
  EXPECT_EQ(calcSquaredDist2d(p0.pose.position, p2.pose.position), 100.0);
  EXPECT_EQ(calcSquaredDist2d(p1.pose.position, p2.pose.position), 200.0);
  // calcDist3d (Point)
  EXPECT_EQ(calcDist3d(p0.pose.position, p0.pose.position), 0.0);
  EXPECT_EQ(calcDist3d(p0.pose.position, p1.pose.position), std::sqrt(200.0));
  EXPECT_EQ(calcDist3d(p0.pose.position, p2.pose.position), std::sqrt(200.0));
  EXPECT_EQ(calcDist3d(p1.pose.position, p2.pose.position), std::sqrt(200.0));
}
