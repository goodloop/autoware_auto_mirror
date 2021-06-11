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

#include <measurement_conversion/measurement_conversion.hpp>

#include <common/types.hpp>
#include <measurement_conversion/eigen_utils.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

constexpr auto kCovarianceMatrixRows = 6U;
constexpr auto kCovarianceMatrixRowsRelativePos = 3U;
constexpr auto kAngleOffset = 3U * kCovarianceMatrixRows + kCovarianceMatrixRows / 2U;
constexpr auto kIndexX = 0U;
constexpr auto kIndexXY = kIndexX + 1U;
constexpr auto kIndexXZ = kIndexX + 2U;
constexpr auto kIndexY = kCovarianceMatrixRows + 1U;
constexpr auto kIndexYX = kIndexY - 1U;
constexpr auto kIndexYZ = kIndexY + 1U;
constexpr auto kIndexZ = 2U * kCovarianceMatrixRows + 2U;
constexpr auto kIndexZX = kIndexZ - 2U;
constexpr auto kIndexZY = kIndexZ - 1U;
constexpr auto kIndexXRelativePos = 0U;
constexpr auto kIndexXYRelativePos = 1U;
constexpr auto kIndexXZRelativePos = 2U;
constexpr auto kIndexYRelativePos = kCovarianceMatrixRowsRelativePos + 1U;
constexpr auto kIndexYXRelativePos = kIndexYRelativePos - 1U;
constexpr auto kIndexYZRelativePos = kIndexYRelativePos + 1U;
constexpr auto kIndexZRelativePos = 2U * kCovarianceMatrixRowsRelativePos + 2U;
constexpr auto kIndexZXRelativePos = kIndexZRelativePos - 2U;
constexpr auto kIndexZYRelativePos = kIndexZRelativePos - 1U;
constexpr auto kCovarianceMatrixRowsSquared = kCovarianceMatrixRows * kCovarianceMatrixRows;
constexpr auto kCovarianceMatrixRowsRelativePosSquared =
  kCovarianceMatrixRowsRelativePos * kCovarianceMatrixRowsRelativePos;
static_assert(
  std::tuple_size<
  geometry_msgs::msg::PoseWithCovariance::_covariance_type>::value ==
  kCovarianceMatrixRowsSquared, "We expect the covariance matrix to have 36 entries.");
static_assert(
  std::tuple_size<
  autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped::_covariance_type>::value ==
  kCovarianceMatrixRowsRelativePosSquared, "We expect the covariance matrix to have 9 entries.");
}  // namespace

namespace autoware
{
namespace common
{
namespace state_estimation
{

MeasurementXYPos64 convert_to<MeasurementXYPos64>::from(
  const geometry_msgs::msg::PoseWithCovariance & msg)
{
  Eigen::Vector2d mean{msg.pose.position.x, msg.pose.position.y};
  Eigen::Matrix2d covariance;
  covariance <<
    msg.covariance[kIndexX], msg.covariance[kIndexXY],
    msg.covariance[kIndexYX], msg.covariance[kIndexY];
  return MeasurementXYPos64{
    mean,
    covariance};
}

MeasurementXYPosAndSpeed64 convert_to<MeasurementXYPosAndSpeed64>::from(
  const nav_msgs::msg::Odometry & msg)
{
  Eigen::Isometry3d tf__msg_frame_id__msg_child_frame_id;
  tf2::fromMsg(msg.pose.pose, tf__msg_frame_id__msg_child_frame_id);
  const Eigen::Matrix2d rx__msg_frame_id__msg_child_frame_id = downscale_isometry<2>(
    tf__msg_frame_id__msg_child_frame_id).rotation();

  const Eigen::Vector2d pos_state {
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
  };
  const Eigen::Vector2d speed_in_child_frame{
    msg.twist.twist.linear.x,
    msg.twist.twist.linear.y,
  };
  const Eigen::Vector2d speed{rx__msg_frame_id__msg_child_frame_id * speed_in_child_frame};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Zero()};
  covariance.topLeftCorner(2, 2) <<
    msg.pose.covariance[kIndexX], msg.pose.covariance[kIndexXY],
    msg.pose.covariance[kIndexYX], msg.pose.covariance[kIndexY];
  covariance.bottomRightCorner(2, 2) <<
    msg.twist.covariance[kIndexX], msg.twist.covariance[kIndexXY],
    msg.twist.covariance[kIndexYX], msg.twist.covariance[kIndexY];
  // Rotate the speed covariance as the speed is now in frame_id frame and not in child_frame_id.
  covariance.bottomRightCorner(2, 2) =
    rx__msg_frame_id__msg_child_frame_id *
    covariance.bottomRightCorner(2, 2) *
    rx__msg_frame_id__msg_child_frame_id.transpose();

  const Eigen::Vector4d mean = (Eigen::Vector4d{} << pos_state, speed).finished();
  return MeasurementXYPosAndSpeed64{mean, covariance};
}

MeasurementXYZRPYPosAndSpeed64 convert_to<MeasurementXYZRPYPosAndSpeed64>::from(
  const nav_msgs::msg::Odometry & msg)
{
  using Vector12d = Eigen::Matrix<float64_t, 12, 1>;
  using Matrix12d = Eigen::Matrix<float64_t, 12, 12>;
  Eigen::Isometry3d tf__msg_frame_id__msg_child_frame_id;
  tf2::fromMsg(msg.pose.pose, tf__msg_frame_id__msg_child_frame_id);
  const Eigen::Matrix3d rx__msg_frame_id__msg_child_frame_id =
    tf__msg_frame_id__msg_child_frame_id.rotation();
  const Eigen::Vector3d roll_pitch_yaw = rx__msg_frame_id__msg_child_frame_id.eulerAngles(0, 1, 2);
  const Eigen::Vector3d pos_state {
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    msg.pose.pose.position.z,
  };
  const Eigen::Vector3d speed_in_child_frame{
    msg.twist.twist.linear.x,
    msg.twist.twist.linear.y,
    msg.twist.twist.linear.z,
  };
  const Eigen::Vector3d angular_speed_in_child_frame{
    msg.twist.twist.angular.x,
    msg.twist.twist.angular.y,
    msg.twist.twist.angular.z,
  };
  const Eigen::Vector3d speed{rx__msg_frame_id__msg_child_frame_id * speed_in_child_frame};
  // TODO(@niosus) This is wrong! I don't know how exactly to rotate the angular velocity right now.
  // Not rotating it will work for preserving 2D rotation around z but will not work for a more
  // general case.
  const Eigen::Vector3d angular_speed{angular_speed_in_child_frame};
  Matrix12d covariance{Matrix12d::Zero()};
  {
    const auto & cov = msg.pose.covariance;
    covariance.block<3, 3>(0, 0) <<
      cov[kIndexX], cov[kIndexXY], cov[kIndexXZ],
      cov[kIndexYX], cov[kIndexY], cov[kIndexYZ],
      cov[kIndexZX], cov[kIndexZY], cov[kIndexZ];
    covariance.block<3, 3>(3, 3) <<
      cov[kAngleOffset + kIndexX], cov[kAngleOffset + kIndexXY], cov[kAngleOffset + kIndexXZ],
      cov[kAngleOffset + kIndexYX], cov[kAngleOffset + kIndexY], cov[kAngleOffset + kIndexYZ],
      cov[kAngleOffset + kIndexZX], cov[kAngleOffset + kIndexZY], cov[kAngleOffset + kIndexZ];
  }
  {
    const auto & cov = msg.twist.covariance;
    covariance.block<3, 3>(6, 6) <<
      cov[kIndexX], cov[kIndexXY], cov[kIndexXZ],
      cov[kIndexYX], cov[kIndexY], cov[kIndexYZ],
      cov[kIndexZX], cov[kIndexZY], cov[kIndexZ];
    covariance.block<3, 3>(9, 9) <<
      cov[kAngleOffset + kIndexX], cov[kAngleOffset + kIndexXY], cov[kAngleOffset + kIndexXZ],
      cov[kAngleOffset + kIndexYX], cov[kAngleOffset + kIndexY], cov[kAngleOffset + kIndexYZ],
      cov[kAngleOffset + kIndexZX], cov[kAngleOffset + kIndexZY], cov[kAngleOffset + kIndexZ];
  }
  // Rotate the speed covariance as the speed is now in frame_id frame and not in child_frame_id.
  covariance.block<3, 3>(6, 6) =
    rx__msg_frame_id__msg_child_frame_id *
    covariance.block<3, 3>(6, 6) *
    rx__msg_frame_id__msg_child_frame_id.transpose();

  const Vector12d mean =
    (Vector12d{} << pos_state, roll_pitch_yaw, speed, angular_speed).finished();
  return MeasurementXYZRPYPosAndSpeed64{mean, covariance};
}

MeasurementXYPos64 convert_to<MeasurementXYPos64>::from(
  const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg)
{
  Eigen::Vector2d mean{msg.position.x, msg.position.y};
  Eigen::Matrix2d covariance;
  covariance <<
    msg.covariance[kIndexXRelativePos], msg.covariance[kIndexXYRelativePos],
    msg.covariance[kIndexYXRelativePos], msg.covariance[kIndexYRelativePos];
  return MeasurementXYPos64{mean, covariance};
}

MeasurementXYZRPYPos64 convert_to<MeasurementXYZRPYPos64>::from(
  const geometry_msgs::msg::PoseWithCovariance & msg)
{
  using Vector6d = Eigen::Matrix<float64_t, 6, 1>;
  using Matrix6d = Eigen::Matrix<float64_t, 6, 6>;
  double roll, pitch, yaw;
  tf2::Quaternion quaternion;
  tf2::fromMsg(msg.pose.orientation, quaternion);
  tf2::Matrix3x3{quaternion}.getRPY(roll, pitch, yaw);
  const Vector6d mean = (Vector6d{} <<
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw).finished();
  Matrix6d covariance = Matrix6d::Zero();
  const auto & cov = msg.covariance;
  covariance.topLeftCorner(3, 3) <<
    cov[kIndexX], cov[kIndexXY], cov[kIndexXZ],
    cov[kIndexYX], cov[kIndexY], cov[kIndexYZ],
    cov[kIndexZX], cov[kIndexZY], cov[kIndexZ];
  covariance.bottomRightCorner(3, 3) <<
    cov[kAngleOffset + kIndexX], cov[kAngleOffset + kIndexXY], cov[kAngleOffset + kIndexXZ],
    cov[kAngleOffset + kIndexYX], cov[kAngleOffset + kIndexY], cov[kAngleOffset + kIndexYZ],
    cov[kAngleOffset + kIndexZX], cov[kAngleOffset + kIndexZY], cov[kAngleOffset + kIndexZ];
  return MeasurementXYZRPYPos64{
    mean,
    covariance};
}

MeasurementXYZRPYSpeed64 convert_to<MeasurementXYZRPYSpeed64>::from(
  const geometry_msgs::msg::TwistWithCovariance & msg)
{
  // TODO(niosus): this probably should be changed as we probably expect twist to be in the moving
  // reference frame
  using Vector6d = Eigen::Matrix<float64_t, 6, 1>;
  using Matrix6d = Eigen::Matrix<float64_t, 6, 6>;
  const Vector6d mean = (Vector6d{} <<
    msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
    msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z).finished();
  Matrix6d covariance = Matrix6d::Zero();
  const auto cov = msg.covariance;
  covariance.topLeftCorner(3, 3) <<
    cov[kIndexX], cov[kIndexXY], cov[kIndexXZ],
    cov[kIndexYX], cov[kIndexY], cov[kIndexYZ],
    cov[kIndexZX], cov[kIndexZY], cov[kIndexZ];
  covariance.bottomRightCorner(3, 3) <<
    cov[kAngleOffset + kIndexX], cov[kAngleOffset + kIndexXY], cov[kAngleOffset + kIndexXZ],
    cov[kAngleOffset + kIndexYX], cov[kAngleOffset + kIndexY], cov[kAngleOffset + kIndexYZ],
    cov[kAngleOffset + kIndexZX], cov[kAngleOffset + kIndexZY], cov[kAngleOffset + kIndexZ];
  return MeasurementXYZRPYSpeed64{
    mean,
    covariance};
}

MeasurementXYZPos64 convert_to<MeasurementXYZPos64>::from(
  const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg)
{
  const Eigen::Vector3d mean{msg.position.x, msg.position.y, msg.position.z};
  const auto & cov = msg.covariance;
  const Eigen::Matrix3d covariance = (Eigen::Matrix3d{} <<
    cov[kIndexXRelativePos], cov[kIndexXYRelativePos], cov[kIndexXZRelativePos],
    cov[kIndexYXRelativePos], cov[kIndexYRelativePos], cov[kIndexYZRelativePos],
    cov[kIndexZXRelativePos], cov[kIndexZYRelativePos], cov[kIndexZRelativePos]).finished();
  return MeasurementXYZPos64{
    mean,
    covariance};
}

MeasurementXYSpeed64 convert_to<MeasurementXYSpeed64>::from(
  const geometry_msgs::msg::TwistWithCovariance & msg)
{
  // TODO(niosus): this probably should be changed as we probably expect twist to be in the moving
  // reference frame aka "child_frame_id", similar to twist in Odometry message.
  Eigen::Vector2d mean{msg.twist.linear.x, msg.twist.linear.y};
  Eigen::Matrix2d covariance;
  covariance <<
    msg.covariance[kIndexX], msg.covariance[kIndexXY],
    msg.covariance[kIndexYX], msg.covariance[kIndexY];
  return MeasurementXYSpeed64{
    mean,
    covariance};
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
