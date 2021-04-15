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

#include <chrono>

#include "gtest/gtest.h"
#include "autoware_auto_msgs/msg/detected_dynamic_object.hpp"
#include "tracking/tracked_object.hpp"

using TrackedObject = autoware::perception::tracking::TrackedObject;
using DetectedObjectMsg = autoware_auto_msgs::msg::DetectedDynamicObject;

TEST(test_tracked_object, test_pose_required) {
  DetectedObjectMsg msg;
  EXPECT_THROW((TrackedObject{msg, 1.0F, 1.0F}), std::runtime_error);
  msg.kinematics.has_pose = true;
  EXPECT_NO_THROW((TrackedObject{msg, 1.0F, 1.0F}));
}

TEST(test_tracked_object, test_optional_twist) {
  DetectedObjectMsg msg;
  msg.kinematics.has_pose = true;
  msg.kinematics.twist.twist.linear.x = 3.0;
  TrackedObject object{msg, 1.0F, 30.0F};
  // The twist is ignored and set to 0 when has_twist == false
  EXPECT_EQ(object.msg().kinematics.twist.twist.linear.x, 0.0);
  msg.kinematics.has_twist = true;
  object = TrackedObject {msg, 1.0F, 30.0F};
  EXPECT_EQ(object.msg().kinematics.twist.twist.linear.x, 3.0);
}

TEST(test_tracked_object, test_optional_covariance) {
  DetectedObjectMsg msg;
  msg.kinematics.has_pose = true;
  msg.kinematics.pose.covariance[0] = 3.0;
  TrackedObject object{msg, 1.0F, 30.0F};
  EXPECT_EQ(object.msg().kinematics.pose.covariance[0], 1.0);
  msg.kinematics.has_pose_covariance = true;
  object = TrackedObject {msg, 1.0F, 30.0F};
  EXPECT_EQ(object.msg().kinematics.pose.covariance[0], 3.0);
  msg.kinematics.twist.covariance[0] = 4.0;
  object = TrackedObject {msg, 1.0F, 30.0F};
  EXPECT_EQ(object.msg().kinematics.twist.covariance[0], 1.0);
  msg.kinematics.has_twist_covariance = true;
  object = TrackedObject {msg, 1.0F, 30.0F};
  EXPECT_EQ(object.msg().kinematics.twist.covariance[0], 4.0);
}

TEST(test_tracked_object, test_predict) {
  DetectedObjectMsg msg;
  msg.kinematics.has_pose = true;
  msg.kinematics.has_twist = true;
  msg.kinematics.twist.twist.linear.x = 3.0;
  TrackedObject object{msg, 1.0F, 30.0F};
  EXPECT_EQ(object.msg().kinematics.pose.pose.position.x, 0.0);
  object.predict(std::chrono::milliseconds(500));
  EXPECT_NE(object.msg().kinematics.pose.pose.position.x, 0.0);
}
