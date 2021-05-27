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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief Implementation of the localization evaluator
#ifndef LOCALIZATION_SYSTEM_TESTS__LOCALIZATION_EVALUATION_NODE_HPP_
#define LOCALIZATION_SYSTEM_TESTS__LOCALIZATION_EVALUATION_NODE_HPP_

#include <localization_system_tests/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <time_utils/time_utils.hpp>
#include <common/types.hpp>
#include <Eigen/Core>
#include <unordered_set>
#include <chrono>
#include <string>
#include <Eigen/Geometry>

namespace localization_system_tests
{
// Hash *only* on the timestamp
template<typename T>
struct TimeStampHash
{
  std::size_t operator()(const T & key) const noexcept
  {
    const auto msg_stamp = key.header.stamp;
    // Convert to single value to avoid hash collisions
    const auto stamp = time_utils::from_message(msg_stamp);
    const auto count = stamp.time_since_epoch().count();
    return std::hash<std::decay_t<decltype(count)>>{} (count);
  }
};

// Compare *only* on the timestamp
template<typename T>
struct TimeStampEqual
{
  bool operator()(const T & lhs, const T & rhs) const noexcept
  {
    const auto diff = time_utils::from_message(lhs.header.stamp) - time_utils::from_message(
      rhs
      .header.stamp);
    const auto abs_diff = (diff > std::chrono::milliseconds{0U}) ? diff : -diff;
    const auto eps = std::chrono::milliseconds{50U};
    return abs_diff <= eps;
  }
};

/// \brief A node that is ran beside a localization application and a source of ground truth and
// estimates the error with respect to the ground truth.
class LOCALIZATION_SYSTEM_TESTS_PUBLIC LocalizationEvaluationNode
  : public rclcpp::Node
{
public:
  using float64_t = autoware::common::types::float64_t;
  using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
  template<typename T>
  using Set = std::unordered_set<T, TimeStampHash<T>, TimeStampEqual<T>>;
  using EigTransform = Eigen::Transform<float64_t, 3, Eigen::Affine, Eigen::ColMajor>;
  using EigTranslationPart = decltype(std::declval<const EigTransform>().translation());
  using EigRotationPart = decltype(std::declval<const EigTransform>().rotation());

  /// \brief Basic node constructor.
  explicit LocalizationEvaluationNode(const rclcpp::NodeOptions & options);

  /// \brief Finalize and report the metrics in the destructor.
  ~LocalizationEvaluationNode() noexcept;

private:
  /// \brief Either find a match between the estimate and a ground truth and compute the metric or
  // buffer the estimate to be matched later.
  /// \param estimate Localization pose estimate.
  void localizer_callback(Pose::ConstSharedPtr & estimate);

  /// \brief Either find a match and compute the metric or buffer the estimate to be matched later.
  /// \param estimate Localization pose estimate.
  void ground_truth_callback(Pose::ConstSharedPtr & estimate);

  /// \brief Compute the translation and rotation error metrics for the given estimate - ground
  /// truth pair.
  /// \param estimate Localization Estimate
  /// \param ground_truth Ground truth estimate
  void compute(const Pose & estimate, const Pose & ground_truth);

  /// \brief Given the current average and the increment of a single sample, compute the moving
  /// average using the number of points stored in the class
  /// \param current_avg Current average
  /// \param addition New sample's contribution
  /// \return The new average
  float64_t get_new_average(float64_t current_avg, float64_t addition);

  /// \brief Compute metrics and update the values
  void compute_and_update_metrics(
    const EigTransform & ground_truth,
    const EigTransform & estimate);

  /// Compute the euclidean distance between two translation values.
  float64_t translation_error(const EigTranslationPart & tf1, const EigTranslationPart & tf2);

  /// Compute the angular distance in radian between two rotation matrices
  float64_t rotation_error(const EigRotationPart & tf1, const EigRotationPart & tf2);

  tf2::BufferCore m_tf_core;
  tf2_ros::TransformListener m_listener;
  rclcpp::Subscription<Pose>::SharedPtr m_localizer_pose_sub;
  rclcpp::Subscription<Pose>::SharedPtr m_ground_truth_sub;

  std::chrono::steady_clock::time_point m_start_tp{std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point m_end_tp{std::chrono::steady_clock::time_point::min()};
  float64_t m_average_translation_err;
  float64_t m_average_rotation_err;
  std::uint32_t m_num_computed{0U};
  // TODO(yunus.caliskan): Currently we have a lot of unmatched samples on both containers.
  //  Idealy one of them should be empty. Figure out why this happens exactly.
  Set<Pose> m_estimates{};
  Set<Pose> m_ground_truths{};
  EigTransform m_ground_truth_to_estimate_tf;
};  // class LocalizationEvaluationNode

}  // namespace localization_system_tests

#endif  // LOCALIZATION_SYSTEM_TESTS__LOCALIZATION_EVALUATION_NODE_HPP_
