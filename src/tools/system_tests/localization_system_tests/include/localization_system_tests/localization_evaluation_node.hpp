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
#ifndef LGSVL_INTERFACE__LGSVL_INTERFACE_NODE_HPP_
#define LGSVL_INTERFACE__LGSVL_INTERFACE_NODE_HPP_

#include <localization_system_tests/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <time_utils/time_utils.hpp>
#include <Eigen/Core>
#include <unordered_set>
#include <chrono>
#include <string>
#include <Eigen/Geometry>

namespace localization_system_tests
{
// Hash *only* on the timestamp
template<typename T>
struct TimeStampHash {
  std::size_t operator()(const T &key) const noexcept {
    const auto msg_stamp = key.header.stamp;
    // Convert to single value to avoid hash collisions
    const auto stamp = time_utils::from_message(msg_stamp);
    const auto count = stamp.time_since_epoch().count();
    return std::hash<std::decay_t<decltype(count)>>{}(count);
  }
};

// Compare *only* on the timestamp
template<typename T>
struct TimeStampEqual {
  bool operator()(const T &lhs, const T &rhs) const noexcept {
    const auto diff = time_utils::from_message(lhs.header.stamp) - time_utils::from_message(rhs
    .header.stamp);
    const auto abs_diff = (diff > std::chrono::milliseconds{0U}) ? diff : -diff;
    const auto eps = std::chrono::milliseconds{50U};
    return abs_diff <= eps;
  }
};

class LOCALIZATION_SYSTEM_TESTS_PUBLIC LocalizationEvaluationNode
  : public rclcpp::Node
{
public:
  using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
  template<typename T>
  using Set = std::unordered_set<T, TimeStampHash<T>, TimeStampEqual<T>>;
  using EigTransform = Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>;
  /// ROS 2 parameter constructor
  /// \param[in] options An rclcpp::NodeOptions object
  explicit LocalizationEvaluationNode(const rclcpp::NodeOptions & options);

  void localizer_callback(Pose::ConstSharedPtr & estimate);
  void ground_truth_callback(Pose::ConstSharedPtr & estimate);

  void compute(const Pose & estimate, const Pose & ground_truth);

  ~LocalizationEvaluationNode() noexcept;

private:

  double get_new_average(double current_avg, double addition);

  tf2::BufferCore m_tf_core;
  tf2_ros::TransformListener m_listener;
  rclcpp::Subscription<Pose>::SharedPtr m_localizer_pose_sub;
  rclcpp::Subscription<Pose>::SharedPtr m_ground_truth_sub;

  // TODO(cvasfi): Change to common::types
  std::chrono::steady_clock::time_point m_start_tp{std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point m_end_tp{std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point
  m_last_estimate_tp{std::chrono::steady_clock::time_point::min()};
  std::chrono::nanoseconds m_avg_period;
  double m_average_translation_err;
  double m_average_rotation_err;
  double m_estimate_period;
  std::uint32_t m_num_computed{0U};
  Set<Pose> m_estimates{};
  Set<Pose> m_ground_truths{};
  EigTransform m_map_odom_tf;
};  // class LocalizationEvaluationNode

}  // namespace localization_system_tests

#endif  // LGSVL_INTERFACE__LGSVL_INTERFACE_NODE_HPP_
