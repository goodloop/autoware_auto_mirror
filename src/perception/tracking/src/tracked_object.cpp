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

#include "tracking/tracked_object.hpp"

#include <measurement_conversion/measurement_conversion.hpp>
#include <measurement_conversion/measurement_typedefs.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <stdexcept>

namespace autoware
{
namespace perception
{
namespace tracking
{

namespace
{

using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::X_VELOCITY;
using autoware::common::state_vector::variable::Y_VELOCITY;
using autoware::common::state_vector::variable::X_ACCELERATION;
using autoware::common::state_vector::variable::Y_ACCELERATION;

using common::types::float64_t;

using CA = autoware::common::state_vector::ConstAccelerationXY64;
using MotionModel = autoware::common::motion_model::LinearMotionModel<CA>;
using NoiseModel = autoware::common::state_estimation::WienerNoise<CA>;
using EKF = autoware::common::state_estimation::KalmanFilter<MotionModel, NoiseModel>;
using TrackedObjectMsg = autoware_auto_msgs::msg::TrackedObject;
using DetectedObjectMsg = autoware_auto_msgs::msg::DetectedObject;

using Measurement2dPose = autoware::common::state_estimation::Measurement2dPose64;
using Measurement2dSpeed = autoware::common::state_estimation::Measurement2dSpeed64;
using Measurement2dPoseAndSpeed = autoware::common::state_estimation::Measurement2dPoseAndSpeed64;

EKF init_ekf(
  const DetectedObjectMsg & detection, float64_t default_variance,
  float64_t noise_variance)
{
  if (!detection.kinematics.has_pose) {
    throw std::invalid_argument("A TrackedObject can only be created from a detection with pose.");
  }
  auto state = MotionModel::State {};
  state.at<X>() = detection.kinematics.pose.pose.position.x;
  state.at<Y>() = detection.kinematics.pose.pose.position.y;
  // When there is no twist available, velocity will be initialized to 0
  if (detection.kinematics.has_twist) {
    state.at<X_VELOCITY>() = detection.kinematics.twist.twist.linear.x;
    state.at<Y_VELOCITY>() = detection.kinematics.twist.twist.linear.y;
  }
  using CovarianceMatrix = Eigen::Matrix<float64_t, state.size(), state.size()>;
  CovarianceMatrix cov = default_variance * CovarianceMatrix::Identity();
  if (detection.kinematics.has_pose_covariance) {
    cov(
      state.index_of<X>(),
      state.index_of<X>()) = detection.kinematics.pose.covariance[0];
    cov(
      state.index_of<X>(),
      state.index_of<Y>()) = detection.kinematics.pose.covariance[1];
    cov(
      state.index_of<Y>(),
      state.index_of<X>()) = detection.kinematics.pose.covariance[6];
    cov(
      state.index_of<Y>(),
      state.index_of<Y>()) = detection.kinematics.pose.covariance[7];
  }
  if (detection.kinematics.has_twist_covariance) {
    cov(
      state.index_of<X_VELOCITY>(),
      state.index_of<X_VELOCITY>()) =
      detection.kinematics.twist.covariance[0];
    cov(
      state.index_of<X_VELOCITY>(),
      state.index_of<Y_VELOCITY>()) =
      detection.kinematics.twist.covariance[1];
    cov(
      state.index_of<Y_VELOCITY>(),
      state.index_of<X_VELOCITY>()) =
      detection.kinematics.twist.covariance[6];
    cov(
      state.index_of<Y_VELOCITY>(),
      state.index_of<Y_VELOCITY>()) =
      detection.kinematics.twist.covariance[7];
  }
  return make_kalman_filter(
    MotionModel {}, NoiseModel {{noise_variance, noise_variance}},
    state, cov);
}

}  // anonymous namespace

/// \relates autoware::perception::tracking::TrackedObject
TrackedObject::TrackedObject(
  const DetectedObjectMsg & detection, float64_t default_variance,
  float64_t noise_variance)
: m_msg{},
  m_ekf{init_ekf(detection, default_variance, noise_variance)},
  m_default_variance{default_variance}
{
  static uint64_t object_id = 0;
  m_msg.object_id = ++object_id;
  m_msg.existence_probability = detection.existence_probability;
  m_msg.classification = detection.classification;
  m_msg.shape.push_back(detection.shape);
  // Kinematics are owned by the EKF and only filled in in the msg() getter
}

void TrackedObject::predict(std::chrono::nanoseconds dt)
{
  m_ekf.predict(dt);
  m_time_since_last_seen += dt;
}

void TrackedObject::update(const DetectedObjectMsg & detection)
{
  m_time_since_last_seen = std::chrono::nanoseconds::zero();
  m_ticks_alive++;
  m_ticks_since_last_seen = 0;
  // Update the shape
  m_msg.shape = {detection.shape};

  // It needs to be determined which parts of the DetectedObject message are set, and can be used
  // to update the state. Also, even if a variable is set, its covariance might not be set.

  // Speculatively convert the message to measurement and fix up the covariance. "Speculatively"
  // because the "has_pose" and "has_twist" fields are not checked yet. This is done to avoid
  // repeating this conversion code.
  auto pose_measurement =
    autoware::common::state_estimation::message_to_measurement<Measurement2dPose>(
    detection.kinematics.pose);
  if (!detection.kinematics.has_pose_covariance) {
    pose_measurement.covariance() = m_default_variance *
      Measurement2dPose::State::Matrix::Identity();
  }
  auto twist_measurement =
    autoware::common::state_estimation::message_to_measurement<Measurement2dSpeed>(
    detection.kinematics.twist);
  if (!detection.kinematics.has_twist_covariance) {
    twist_measurement.covariance() = m_default_variance *
      Measurement2dSpeed::State::Matrix::Identity();
  }

  if (detection.kinematics.has_pose && detection.kinematics.has_twist) {
    // Combine both into one measurement
    Eigen::Vector4d state{};
    state << pose_measurement.state().vector(), twist_measurement.state().vector();
    Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
    covariance.topLeftCorner<2, 2>() = pose_measurement.covariance();
    covariance.bottomRightCorner<2, 2>() = twist_measurement.covariance();
    Measurement2dPoseAndSpeed full_measurement = Measurement2dPoseAndSpeed{
      state,
      covariance};
    m_ekf.correct(full_measurement);
  } else if (detection.kinematics.has_pose) {
    m_ekf.correct(pose_measurement);
  } else if (detection.kinematics.has_twist) {
    m_ekf.correct(twist_measurement);
  } else {
    // Impossible, because validation checks this condition.
    throw std::logic_error("DetectedObject with no pose and no twist encountered.");
  }
}

void TrackedObject::no_update()
{
  m_ticks_alive++;
  m_ticks_since_last_seen++;
}

const TrackedObject::TrackedObjectMsg & TrackedObject::msg()
{
  // Fill the message fields from the filter state
  m_msg.kinematics.pose.pose.position.x = m_ekf.state().at<X>();
  m_msg.kinematics.pose.pose.position.y = m_ekf.state().at<Y>();
  m_msg.kinematics.twist.twist.linear.x = m_ekf.state().at<X_VELOCITY>();
  m_msg.kinematics.twist.twist.linear.y = m_ekf.state().at<Y_VELOCITY>();
  m_msg.kinematics.acceleration.accel.linear.x =
    m_ekf.state().at<X_ACCELERATION>();
  m_msg.kinematics.acceleration.accel.linear.y =
    m_ekf.state().at<Y_ACCELERATION>();

  // Set covariances
  m_msg.kinematics.pose.covariance[0] =
    m_ekf.covariance()(
    m_ekf.state().index_of<X>(),
    m_ekf.state().index_of<X>());
  m_msg.kinematics.pose.covariance[1] =
    m_ekf.covariance()(
    m_ekf.state().index_of<X>(),
    m_ekf.state().index_of<Y>());
  m_msg.kinematics.pose.covariance[6] =
    m_ekf.covariance()(
    m_ekf.state().index_of<Y>(),
    m_ekf.state().index_of<X>());
  m_msg.kinematics.pose.covariance[7] =
    m_ekf.covariance()(
    m_ekf.state().index_of<Y>(),
    m_ekf.state().index_of<Y>());
  m_msg.kinematics.twist.covariance[0] =
    m_ekf.covariance()(
    m_ekf.state().index_of<X_VELOCITY>(),
    m_ekf.state().index_of<X_VELOCITY>());
  m_msg.kinematics.twist.covariance[1] =
    m_ekf.covariance()(
    m_ekf.state().index_of<X_VELOCITY>(),
    m_ekf.state().index_of<Y_VELOCITY>());
  m_msg.kinematics.twist.covariance[6] =
    m_ekf.covariance()(
    m_ekf.state().index_of<Y_VELOCITY>(),
    m_ekf.state().index_of<X_VELOCITY>());
  m_msg.kinematics.twist.covariance[7] =
    m_ekf.covariance()(
    m_ekf.state().index_of<Y_VELOCITY>(),
    m_ekf.state().index_of<Y_VELOCITY>());
  // TODO(nikolai.morin): Set is_stationary, classification etc.
  return m_msg;
}

bool TrackedObject::should_be_removed(
  const std::chrono::nanoseconds time_threshold,
  const std::size_t ticks_threshold) const
{
  return m_time_since_last_seen >= time_threshold || m_ticks_since_last_seen >= ticks_threshold;
}

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
