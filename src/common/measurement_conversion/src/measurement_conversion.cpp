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

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
