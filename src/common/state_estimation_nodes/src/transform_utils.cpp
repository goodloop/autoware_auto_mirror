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
// Developed by Apex.AI, Inc.

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief Description.


#include <state_estimation_nodes/transform_utils.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace
{
template<typename ArrayT>
Eigen::Matrix3d to_eigen(
  const ArrayT & covariance,
  const std::size_t start,
  const std::size_t stride)
{
  if (covariance.size() < start + 2 * stride + 2) {
    throw std::domain_error(
            "The array representing the covariance matrix does not have enough elements. "
            "Expected " + std::to_string(start + 2 * stride + 2) +
            " but only has " + std::to_string(covariance.size()));
  }
  return (Eigen::Matrix3d{} <<
         covariance[start], covariance[start + stride], covariance[start + 2 * stride],
         covariance[start + 1], covariance[start + stride + 1], covariance[start + 2 * stride + 1],
         covariance[start + 2], covariance[start + stride + 2], covariance[start + 2 * stride + 2]).
         finished();
}

template<typename ArrayT>
void fill_covariance_from_eigen(
  ArrayT & covariance,
  const Eigen::Matrix3d & matrix,
  const std::size_t start,
  const std::size_t stride)
{
  if (covariance.size() < start + 2 * stride + 2) {
    throw std::domain_error(
            "The array representing the covariance matrix does not have enough elements. "
            "Expected " + std::to_string(start + 2 * stride + 2) +
            " but only has " + std::to_string(covariance.size()));
  }
  covariance[start] = matrix(0, 0);
  covariance[start + 1] = matrix(1, 0);
  covariance[start + 2] = matrix(2, 0);
  covariance[start + stride] = matrix(0, 1);
  covariance[start + stride + 1] = matrix(1, 1);
  covariance[start + stride + 2] = matrix(2, 1);
  covariance[start + 2 * stride] = matrix(0, 2);
  covariance[start + 2 * stride + 1] = matrix(1, 2);
  covariance[start + 2 * stride + 2] = matrix(2, 2);
}

///
/// @brief      Rotate the covariance matrix
///
/// @param[in]  tf__new_child__old_child  The transform from the old child frame to the new one
/// @param[in]  tf__new__old              The transform from the old parent frame to the new one
/// @param[in]  covariance                The covariance to be rotated
///
/// @return     The rotated covariance matrix.
///
Eigen::Matrix3d rotate_covariance(
  const Eigen::Isometry3d & tf__new_child__old_child,
  const Eigen::Isometry3d & tf__new__old,
  const Eigen::Matrix3d & covariance)
{
  return
    tf__new_child__old_child.rotation().transpose() *
    (tf__new__old.rotation() * covariance * tf__new__old.rotation().transpose()) *
    tf__new_child__old_child.rotation();
}

///
/// @brief      Rotate the covariance matrix of the odometry twist.
///
/// @param[in]  tf__new__old  The transform from the old frame to the new one.
/// @param[in]  covariance                The covariance to be rotated.
///
/// @return     The rotated covariance matrix.
///
Eigen::Matrix3d rotate_covariance(
  const Eigen::Isometry3d & tf__new__old,
  const Eigen::Matrix3d & covariance)
{
  return tf__new__old.rotation() * covariance * tf__new__old.rotation().transpose();
}

}  // namespace

namespace autoware
{
namespace common
{
namespace state_estimation
{

geometry_msgs::msg::TransformStamped get_tf__target__source(
  const tf2::BufferCore & buffer,
  const std_msgs::msg::Header::_frame_id_type & target_frame_id,
  const std_msgs::msg::Header::_frame_id_type & source_frame_id,
  const std_msgs::msg::Header::_stamp_type & timestamp)
{
  // Get the transform between the msg and the output frame. We treat the
  // possible exceptions as unrecoverable and let them bubble up.
  return buffer.lookupTransform(
    target_frame_id, source_frame_id, tf2_ros::fromMsg(timestamp));
}

template<>
nav_msgs::msg::Odometry switch_frames(
  const nav_msgs::msg::Odometry & original_msg,
  const std_msgs::msg::Header::_frame_id_type & new_frame_id,
  const std_msgs::msg::Header::_frame_id_type & new_child_frame_id,
  const tf2::BufferCore & buffer)
{
  if ((new_frame_id == original_msg.header.frame_id) &&
    (new_child_frame_id == original_msg.child_frame_id))
  {
    return original_msg;
  }
  nav_msgs::msg::Odometry result_msg = original_msg;
  const Eigen::Isometry3d tf__new_frame_id__msg_frame_id = tf2::transformToEigen(
    get_tf__target__source(
      buffer, new_frame_id, original_msg.header.frame_id, original_msg.header.stamp));
  const Eigen::Isometry3d tf__new_child_frame_id__msg_child_frame_id = tf2::transformToEigen(
    get_tf__target__source(
      buffer, new_child_frame_id, original_msg.child_frame_id, original_msg.header.stamp));
  // 1. Represent odom position and orientation as transformation tf__parent__child
  Eigen::Isometry3d tf__msg_frame_id__msg_child_frame_id;
  tf2::fromMsg(original_msg.pose.pose, tf__msg_frame_id__msg_child_frame_id);
  // 2. Convert tf__parent__child to tf__new_parent__new_child as follows:
  // tf__new_parent__new_child = tf__new_parent__parent * tf__parent__child * tf__child__new_child
  const Eigen::Isometry3d tf__new_frame_id__new_child_frame_id =
    tf__new_frame_id__msg_frame_id *
    tf__msg_frame_id__msg_child_frame_id *
    tf__new_child_frame_id__msg_child_frame_id.inverse();
  // 3. Represent tf__new_parent__new_child as odom message again
  result_msg.pose.pose = tf2::toMsg(tf__new_frame_id__new_child_frame_id);
  // 4. Rotate the twist with the rotation part of tf__new_child__child
  Eigen::Vector3d speed_in_child_frame;
  tf2::fromMsg(original_msg.twist.twist.linear, speed_in_child_frame);
  result_msg.twist.twist.linear = tf2::toMsg2(
    tf__new_child_frame_id__msg_child_frame_id.rotation() * speed_in_child_frame);
  Eigen::Vector3d angular_speed_in_child_frame;
  tf2::fromMsg(original_msg.twist.twist.angular, angular_speed_in_child_frame);
  result_msg.twist.twist.angular = tf2::toMsg2(
    tf__new_child_frame_id__msg_child_frame_id.rotation() * angular_speed_in_child_frame);

  const auto cov_stride = 6UL;
  // Update position covariance
  fill_covariance_from_eigen(
    result_msg.pose.covariance,
    rotate_covariance(
      tf__new_child_frame_id__msg_child_frame_id,
      tf__new_frame_id__msg_frame_id,
      to_eigen(original_msg.pose.covariance, 0UL, cov_stride)),
    0UL, cov_stride);
  // Update orientation covariance
  fill_covariance_from_eigen(
    result_msg.pose.covariance,
    rotate_covariance(
      tf__new_child_frame_id__msg_child_frame_id,
      tf__new_frame_id__msg_frame_id,
      to_eigen(original_msg.pose.covariance, 21UL, cov_stride)),
    21UL, cov_stride);
  // Update linear speed covariance
  fill_covariance_from_eigen(
    result_msg.twist.covariance,
    rotate_covariance(
      tf__new_child_frame_id__msg_child_frame_id,
      to_eigen(original_msg.pose.covariance, 0UL, cov_stride)),
    0UL, cov_stride);
  // Update angular speed covariance
  fill_covariance_from_eigen(
    result_msg.twist.covariance,
    rotate_covariance(
      tf__new_child_frame_id__msg_child_frame_id,
      to_eigen(original_msg.pose.covariance, 21UL, cov_stride)),
    21UL, cov_stride);
  result_msg.header.frame_id = new_frame_id;
  result_msg.child_frame_id = new_child_frame_id;
  return result_msg;
}

template<>
autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped switch_frames(
  const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & original_msg,
  const std_msgs::msg::Header::_frame_id_type & new_frame_id,
  const std_msgs::msg::Header::_frame_id_type & new_child_frame_id,
  const tf2::BufferCore & buffer)
{
  if ((new_frame_id == original_msg.header.frame_id) &&
    (new_child_frame_id == original_msg.child_frame_id))
  {
    return original_msg;
  }
  autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped result_msg = original_msg;
  const Eigen::Isometry3d tf__new_frame_id__msg_frame_id = tf2::transformToEigen(
    get_tf__target__source(
      buffer, new_frame_id, original_msg.header.frame_id, original_msg.header.stamp));
  const Eigen::Isometry3d tf__new_child_frame_id__msg_child_frame_id = tf2::transformToEigen(
    get_tf__target__source(
      buffer, new_child_frame_id, original_msg.child_frame_id, original_msg.header.stamp));
  // 1. Represent position as transformation tf__parent__child
  Eigen::Vector3d translation;
  tf2::fromMsg(original_msg.position, translation);
  Eigen::Isometry3d tf__msg_frame_id__msg_child_frame_id{Eigen::Isometry3d::Identity()};
  tf__msg_frame_id__msg_child_frame_id.translation() = translation;
  // 2. Convert tf__parent__child to tf__new_parent__new_child as follows:
  // tf__new_parent__new_child = tf__new_parent__parent * tf__parent__child * tf__child__new_child
  const Eigen::Isometry3d tf__new_frame_id__new_child_frame_id =
    tf__new_frame_id__msg_frame_id *
    tf__msg_frame_id__msg_child_frame_id *
    tf__new_child_frame_id__msg_child_frame_id.inverse();
  // 3. Represent tf__new_parent__new_child as odom message again
  result_msg.position = tf2::toMsg2(tf__new_frame_id__new_child_frame_id.translation());

  const auto cov_stride = 3UL;
  // Update position covariance
  fill_covariance_from_eigen(
    result_msg.covariance,
    rotate_covariance(
      tf__new_child_frame_id__msg_child_frame_id,
      tf__new_frame_id__msg_frame_id,
      to_eigen(original_msg.covariance, 0UL, cov_stride)),
    0UL, cov_stride);
  result_msg.header.frame_id = new_frame_id;
  result_msg.child_frame_id = new_child_frame_id;
  return result_msg;
}

template<>
geometry_msgs::msg::PoseWithCovarianceStamped switch_frames(
  const geometry_msgs::msg::PoseWithCovarianceStamped & original_msg,
  const std_msgs::msg::Header::_frame_id_type & new_frame_id,
  const tf2::BufferCore & buffer)
{
  if (new_frame_id == original_msg.header.frame_id) {
    return original_msg;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped result_msg = original_msg;
  const Eigen::Isometry3d tf__new_frame_id__msg_frame_id = tf2::transformToEigen(
    get_tf__target__source(
      buffer, new_frame_id, original_msg.header.frame_id, original_msg.header.stamp));
  Eigen::Isometry3d tf__msg_frame_id__msg_child_frame_id{};
  tf2::fromMsg(original_msg.pose.pose, tf__msg_frame_id__msg_child_frame_id);
  const Eigen::Isometry3d tf__new_frame_id__msg_child_frame_id =
    tf__new_frame_id__msg_frame_id * tf__msg_frame_id__msg_child_frame_id;
  result_msg.pose.pose = tf2::toMsg(tf__new_frame_id__msg_child_frame_id);

  const auto cov_stride = 6UL;
  // Update position covariance
  fill_covariance_from_eigen(
    result_msg.pose.covariance,
    rotate_covariance(
      tf__new_frame_id__msg_frame_id, to_eigen(original_msg.pose.covariance, 0UL, cov_stride)),
    0UL, cov_stride);
  // Update orientation covariance
  fill_covariance_from_eigen(
    result_msg.pose.covariance,
    rotate_covariance(
      tf__new_frame_id__msg_frame_id, to_eigen(original_msg.pose.covariance, 21UL, cov_stride)),
    21UL, cov_stride);
  result_msg.header.frame_id = new_frame_id;
  return result_msg;
}


template<>
geometry_msgs::msg::TwistWithCovarianceStamped switch_frames(
  const geometry_msgs::msg::TwistWithCovarianceStamped & original_msg,
  const std_msgs::msg::Header::_frame_id_type & new_frame_id,
  const tf2::BufferCore & buffer)
{
  if (new_frame_id == original_msg.header.frame_id) {
    return original_msg;
  }
  geometry_msgs::msg::TwistWithCovarianceStamped result_msg = original_msg;
  const Eigen::Isometry3d tf__new_frame_id__msg_frame_id = tf2::transformToEigen(
    get_tf__target__source(
      buffer, new_frame_id, original_msg.header.frame_id, original_msg.header.stamp));
  Eigen::Vector3d angular_speed_in_child_frame;
  tf2::fromMsg(original_msg.twist.twist.angular, angular_speed_in_child_frame);
  result_msg.twist.twist.angular = tf2::toMsg2(
    tf__new_frame_id__msg_frame_id.rotation() * angular_speed_in_child_frame);

  const auto cov_stride = 6UL;
  // Update linear speed covariance
  fill_covariance_from_eigen(
    result_msg.twist.covariance,
    rotate_covariance(
      tf__new_frame_id__msg_frame_id,
      to_eigen(original_msg.twist.covariance, 0UL, cov_stride)),
    0UL, cov_stride);
  // Update angular speed covariance
  fill_covariance_from_eigen(
    result_msg.twist.covariance,
    rotate_covariance(
      tf__new_frame_id__msg_frame_id,
      to_eigen(original_msg.twist.covariance, 21UL, cov_stride)),
    21UL, cov_stride);
  result_msg.header.frame_id = new_frame_id;
  return result_msg;
}


}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
