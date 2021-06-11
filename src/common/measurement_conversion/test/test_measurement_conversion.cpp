// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>

#include <common/types.hpp>
#include <measurement_conversion/measurement_conversion.hpp>

using autoware::common::state_estimation::Stamped;
using autoware::common::state_estimation::MeasurementXYPos64;
using autoware::common::state_estimation::MeasurementXYSpeed64;
using autoware::common::state_estimation::MeasurementXYPosAndSpeed64;
using autoware::common::state_estimation::MeasurementXYZRPYPos64;
using autoware::common::state_estimation::MeasurementXYZRPYSpeed64;
using autoware::common::state_estimation::MeasurementXYZRPYPosAndSpeed64;
using autoware::common::state_estimation::convert_to;
using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::Z;
using autoware::common::state_vector::variable::ROLL;
using autoware::common::state_vector::variable::PITCH;
using autoware::common::state_vector::variable::YAW;
using autoware::common::state_vector::variable::X_VELOCITY;
using autoware::common::state_vector::variable::Y_VELOCITY;
using autoware::common::state_vector::variable::Z_VELOCITY;
using autoware::common::state_vector::variable::ROLL_CHANGE_RATE;
using autoware::common::state_vector::variable::PITCH_CHANGE_RATE;
using autoware::common::state_vector::variable::YAW_CHANGE_RATE;
using autoware::common::types::float32_t;

namespace
{

std_msgs::msg::Header create_header() noexcept
{
  std_msgs::msg::Header msg{};
  msg.frame_id = "map";
  msg.stamp.sec = 42;
  msg.stamp.nanosec = 0;
  return msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped create_pose_msg() noexcept
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg{};
  msg.header = create_header();
  msg.pose.pose.position.x = 42.0;
  msg.pose.pose.position.y = 23.0;
  msg.pose.pose.position.z = 1.0;
  // Rotation around z axis by 90 degrees.
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.7071068;
  msg.pose.pose.orientation.w = 0.7071068;
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 2.0;
  msg.pose.covariance[14] = 3.0;
  msg.pose.covariance[21] = 4.0;
  msg.pose.covariance[28] = 5.0;
  msg.pose.covariance[35] = 6.0;
  return msg;
}

geometry_msgs::msg::TwistWithCovarianceStamped create_twist_msg() noexcept
{
  geometry_msgs::msg::TwistWithCovarianceStamped msg{};
  msg.header = create_header();
  msg.twist.twist.linear.x = 23.0;
  msg.twist.twist.linear.y = 42.0;
  msg.twist.twist.linear.z = 1.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = 42.42;
  msg.twist.covariance[0] = 1.0;
  msg.twist.covariance[7] = 2.0;
  msg.twist.covariance[14] = 3.0;
  msg.twist.covariance[21] = 4.0;
  msg.twist.covariance[28] = 5.0;
  msg.twist.covariance[35] = 6.0;
  return msg;
}

nav_msgs::msg::Odometry create_odometry_msg() noexcept
{
  const auto pose_msg = create_pose_msg();
  const auto twist_msg = create_twist_msg();
  nav_msgs::msg::Odometry msg{};
  msg.child_frame_id = "base_link";
  msg.header = create_header();
  msg.pose = pose_msg.pose;
  msg.twist = twist_msg.twist;
  return msg;
}

autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped create_relative_pos_msg() noexcept
{
  const auto pose_msg = create_pose_msg();
  autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped msg{};
  msg.child_frame_id = "base_link";
  msg.header = create_header();
  msg.position.x = pose_msg.pose.pose.position.x;
  msg.position.y = pose_msg.pose.pose.position.y;
  msg.position.z = pose_msg.pose.pose.position.z;
  msg.covariance[0] = 1.0;
  msg.covariance[4] = 2.0;
  msg.covariance[8] = 3.0;
  return msg;
}

}  // namespace

/// \test Create a measurement from odometry.
TEST(Measurement2dConversionTest, Odometry) {
  const auto msg = create_odometry_msg();
  const auto measurement = convert_to<Stamped<MeasurementXYPosAndSpeed64>>::from(msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<X>(), 42.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<Y>(), 23.0);
  const auto x_idx = measurement.measurement.state().index_of<X>();
  const auto y_idx = measurement.measurement.state().index_of<Y>();
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(x_idx, x_idx), 1.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(y_idx, y_idx), 2.0);

  const auto kPrecision = 0.00001;
  // Note that the expected values for x and y are switched because of the 90 deg rotation.
  EXPECT_NEAR(measurement.measurement.state().at<X_VELOCITY>(), -42.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.state().at<Y_VELOCITY>(), 23.0, kPrecision);
  // Note that the expected values for x and y are switched because of the 90 deg rotation.
  const auto x_speed_idx = measurement.measurement.state().index_of<X_VELOCITY>();
  const auto y_speed_idx = measurement.measurement.state().index_of<Y_VELOCITY>();
  EXPECT_NEAR(measurement.measurement.covariance()(x_speed_idx, x_speed_idx), 2.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.covariance()(y_speed_idx, y_speed_idx), 1.0, kPrecision);

  EXPECT_EQ(measurement.timestamp.time_since_epoch(), std::chrono::seconds{42LL});
}

/// \test Create a measurement from pose.
TEST(Measurement2dConversionTest, pose) {
  const auto msg = create_pose_msg();
  const auto measurement = convert_to<Stamped<MeasurementXYPos64>>::from(msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().x(), 42.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().y(), 23.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(1, 1), 2.0);
  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}

/// \test Create a measurement from a relative pose.
TEST(Measurement2dConversionTest, RelativePose) {
  const auto msg = create_relative_pos_msg();
  const auto measurement = convert_to<Stamped<MeasurementXYPos64>>::from(msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().x(), 42.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().y(), 23.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(1, 1), 2.0);
  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}

/// \test Create a measurement from twist.
TEST(Measurement2dConversionTest, twist) {
  const auto msg = create_twist_msg();
  const auto measurement = convert_to<Stamped<MeasurementXYSpeed64>>::from(msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector()[0], 23.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector()[1], 42.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(1, 1), 2.0);

  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}


/// \test Create a measurement from odometry.
TEST(Measurement3dRollPitchYawConversionTest, Odometry) {
  const auto msg = create_odometry_msg();
  const auto kPrecision = 0.00001;
  const auto measurement = convert_to<Stamped<MeasurementXYZRPYPosAndSpeed64>>::from(msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<X>(), 42.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<Y>(), 23.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<Z>(), 1.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<ROLL>(), 0.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<PITCH>(), 0.0);
  EXPECT_NEAR(measurement.measurement.state().at<YAW>(), M_PI_2, kPrecision);
  const auto x_idx = measurement.measurement.state().index_of<X>();
  const auto y_idx = measurement.measurement.state().index_of<Y>();
  const auto z_idx = measurement.measurement.state().index_of<Z>();
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(x_idx, x_idx), 1.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(y_idx, y_idx), 2.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(z_idx, z_idx), 3.0);

  // Note that the expected values for x and y are switched because of the 90 deg rotation.
  EXPECT_NEAR(measurement.measurement.state().at<X_VELOCITY>(), -42.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.state().at<Y_VELOCITY>(), 23.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.state().at<Z_VELOCITY>(), 1.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.state().at<ROLL_CHANGE_RATE>(), 0.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.state().at<PITCH_CHANGE_RATE>(), 0.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.state().at<YAW_CHANGE_RATE>(), 42.42, kPrecision);
  // Note that the expected values for x and y are switched because of the 90 deg rotation.
  const auto x_speed_idx = measurement.measurement.state().index_of<X_VELOCITY>();
  const auto y_speed_idx = measurement.measurement.state().index_of<Y_VELOCITY>();
  const auto z_speed_idx = measurement.measurement.state().index_of<Z_VELOCITY>();
  EXPECT_NEAR(measurement.measurement.covariance()(x_speed_idx, x_speed_idx), 2.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.covariance()(y_speed_idx, y_speed_idx), 1.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.covariance()(z_speed_idx, z_speed_idx), 3.0, kPrecision);

  EXPECT_EQ(measurement.timestamp.time_since_epoch(), std::chrono::seconds{42LL});
}

/// \test Create a measurement from pose.
TEST(Measurement3dRollPitchYawConversionTest, Pose) {
  const auto msg = create_pose_msg();
  const auto measurement = convert_to<Stamped<MeasurementXYZRPYPos64>>::from(msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().x(), 42.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().y(), 23.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().z(), 1.0);
  const auto x_idx = measurement.measurement.state().index_of<X>();
  const auto y_idx = measurement.measurement.state().index_of<Y>();
  const auto z_idx = measurement.measurement.state().index_of<Z>();
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(x_idx, x_idx), 1.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(y_idx, y_idx), 2.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(z_idx, z_idx), 3.0);
  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}
