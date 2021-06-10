// Copyright 2021 Apex.AI, Inc.
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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <state_estimation_nodes/kalman_filter_wrapper.hpp>

#include <common/types.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>
#include <limits>


namespace
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
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
using autoware::common::state_vector::ConstAccelerationXY32;
using autoware::common::state_vector::ConstAccelerationXYZRPY32;

constexpr auto kCovarianceMatrixRows = 6U;
constexpr auto kIndexX = 0U;
constexpr auto kIndexY = kCovarianceMatrixRows + 1U;
constexpr auto kIndexZ = 2U * kCovarianceMatrixRows + 2U;
constexpr auto kDefaultChildFrameId = "base_link";
constexpr auto kCovarianceMatrixRowsSquared = kCovarianceMatrixRows * kCovarianceMatrixRows;
static_assert(
  std::tuple_size<
    geometry_msgs::msg::PoseWithCovariance::_covariance_type>::value ==
  kCovarianceMatrixRowsSquared, "We expect the covariance matrix to have 36 entries.");

/// Convert a chrono timepoint to ROS time.
rclcpp::Time to_ros_time(const std::chrono::system_clock::time_point & time_point)
{
  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;
  return rclcpp::Time{duration_cast<nanoseconds>(time_point.time_since_epoch()).count()};
}

/// Get the speed magnitude.
autoware::common::types::float64_t get_speed(
  const autoware::common::types::float32_t x_speed,
  const autoware::common::types::float32_t y_speed) noexcept
{
  return static_cast<autoware::common::types::float64_t>(
    std::sqrt(x_speed * x_speed + y_speed * y_speed));
}

/// Get the speed magnitude.
autoware::common::types::float64_t get_speed(
  const float32_t x_speed,
  const float32_t y_speed,
  const float32_t z_speed) noexcept
{
  return static_cast<autoware::common::types::float64_t>(
    std::sqrt(x_speed * x_speed + y_speed * y_speed + z_speed * z_speed));
}

template<typename StateT>
struct OdometryFiller
{
  static_assert(sizeof(StateT) == 0UL, "This struct must be specialized.");
};


template<>
struct OdometryFiller<ConstAccelerationXY32>
{
  template<typename InputStateT>
  static nav_msgs::msg::Odometry fill_odom_msg(
    const InputStateT & state,
    const typename InputStateT::Matrix & covariance)
  {
    using std::chrono::duration_cast;
    using std::chrono::nanoseconds;
    nav_msgs::msg::Odometry msg{};
    // Fill state.
    msg.pose.pose.position.x = static_cast<float64_t>(state.template at<X>());
    msg.pose.pose.position.y = static_cast<float64_t>(state.template at<Y>());
    msg.pose.pose.position.z = 0.0;
    // Odometry message has velocity stored in the local frame of the message. In this case, the car
    // always moves along its orientation vector with the speed along it's "forward" direction being
    // the magnitude of the velocity vector estimated by the filter.
    msg.twist.twist.linear.x =
      get_speed(state.template at<X_VELOCITY>(), state.template at<Y_VELOCITY>());
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.linear.z = 0.0;
    msg.twist.twist.angular.set__x(0.0).set__y(0.0).set__z(0.0);

    const auto rotation_around_z = std::atan2(
      static_cast<float64_t>(state.template at<Y_VELOCITY>()),
      static_cast<float64_t>(state.template at<X_VELOCITY>()));
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, rotation_around_z);
    msg.pose.pose.orientation = tf2::toMsg(rotation);

    // Fill covariances.
    const auto x_index = state.template index_of<X>();
    const auto y_index = state.template index_of<Y>();
    const auto x_speed_index = state.template index_of<X_VELOCITY>();
    const auto y_speed_index = state.template index_of<Y_VELOCITY>();
    msg.pose.covariance[kIndexX] = static_cast<float64_t>(covariance(x_index, x_index));
    msg.pose.covariance[kIndexY] = static_cast<float64_t>(covariance(y_index, y_index));
    msg.twist.covariance[kIndexX] =
      static_cast<float64_t>(covariance(x_speed_index, x_speed_index));
    msg.twist.covariance[kIndexY] =
      static_cast<float64_t>(covariance(y_speed_index, y_speed_index));
    // TODO(igor): do we need elements off the diagonal?
    return msg;
  }
};

template<>
struct OdometryFiller<ConstAccelerationXYZRPY32>
{
  template<typename InputStateT>
  static nav_msgs::msg::Odometry fill_odom_msg(
    const InputStateT & state,
    const typename InputStateT::Matrix & covariance)
  {
    auto msg = OdometryFiller<ConstAccelerationXY32>::fill_odom_msg(state, covariance);
    msg.pose.pose.position.z = static_cast<float64_t>(state.template at<Z>());
    // Odometry message has velocity stored in the local frame of the message. In this case, the car
    // always moves along its orientation vector with the speed along it's "forward" direction being
    // the magnitude of the velocity vector estimated by the filter.
    msg.twist.twist.linear.x = get_speed(
      state.template at<X_VELOCITY>(),
      state.template at<Y_VELOCITY>(),
      state.template at<Z_VELOCITY>());
    msg.twist.twist.angular.x = static_cast<float64_t>(state.template at<ROLL_CHANGE_RATE>());
    msg.twist.twist.angular.y = static_cast<float64_t>(state.template at<PITCH_CHANGE_RATE>());
    msg.twist.twist.angular.z = static_cast<float64_t>(state.template at<YAW_CHANGE_RATE>());

    tf2::Quaternion rotation;
    rotation.setRPY(
      static_cast<float64_t>(state.template at<ROLL>()),
      static_cast<float64_t>(state.template at<PITCH>()),
      static_cast<float64_t>(state.template at<YAW>()));
    msg.pose.pose.orientation = tf2::toMsg(rotation);

    // Fill covariances.
    const auto z_index = state.template index_of<Z>();
    const auto z_speed_index = state.template index_of<Z_VELOCITY>();
    msg.pose.covariance[kIndexZ] = static_cast<float64_t>(
      covariance(z_index, z_index));
    msg.twist.covariance[kIndexZ] = static_cast<float64_t>(
      covariance(z_speed_index, z_speed_index));
    // TODO(igor): do we need elements off the diagonal?
    return msg;
  }
};

}  // namespace

namespace autoware
{
namespace common
{
namespace state_estimation
{

template<typename FilterT>
nav_msgs::msg::Odometry KalmanFilterWrapper<FilterT>::get_state() const
{
  if (!is_initialized()) {
    throw std::runtime_error("Filter not is_initialized, cannot get state.");
  }
  const auto last_event = m_history.get_last_event();
  const auto state = last_event.stored_state();
  const auto & covariance = last_event.stored_covariance();
  auto msg = OdometryFiller<typename FilterT::State>::fill_odom_msg(state, covariance);
  msg.header.stamp = rclcpp::Time{to_ros_time(m_history.get_last_timestamp())};
  msg.header.frame_id = m_frame_id;
  msg.child_frame_id = kDefaultChildFrameId;
  return msg;
}

template class STATE_ESTIMATION_NODES_PUBLIC
    KalmanFilterWrapper<ConstAccelerationKalmanFilterXY>;
template class STATE_ESTIMATION_NODES_PUBLIC
    KalmanFilterWrapper<ConstAccelerationKalmanFilterXYZRPY>;

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
