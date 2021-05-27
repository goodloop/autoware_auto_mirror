// Copyright 2020 the Autoware Foundation
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

#include <localization_system_tests/localization_evaluation_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <ratio>
#include <string>

namespace localization_system_tests
{

LocalizationEvaluationNode::LocalizationEvaluationNode(
  const rclcpp::NodeOptions & options)
: Node("localization_evaluator", options),
  m_listener{m_tf_core},
  m_localizer_pose_sub{
    create_subscription<Pose>(
      "localization/ndt_pose",
      rclcpp::QoS{rclcpp::KeepLast{20U}},
      [this](typename Pose::ConstSharedPtr msg) {localizer_callback(msg);}),
  },
  m_ground_truth_sub{
  create_subscription<Pose>(
    "/vehicle/odom_pose",
    rclcpp::QoS{rclcpp::KeepLast{20U}},
    [this](typename Pose::ConstSharedPtr msg) {ground_truth_callback(msg);}),
} {
  EigTransform map_bl_tf;
  map_bl_tf.setIdentity();
  map_bl_tf.translate(Eigen::Vector3d{-58.22, -40.98, -1.9});

  EigTransform odom_bl_tf;
  odom_bl_tf.setIdentity();
  map_bl_tf.translate(Eigen::Vector3d{-0.00358963, -0.00601959, 0.0491562});

  m_map_odom_tf = map_bl_tf * odom_bl_tf.inverse();
}

LocalizationEvaluationNode::~LocalizationEvaluationNode() noexcept
{
  std::stringstream report;
  const std::chrono::nanoseconds duration = m_end_tp - m_start_tp;
  const auto duration_in_ms =
    static_cast<double>((m_end_tp - m_start_tp).count()) * std::milli::den /
    std::nano::den;
  const auto avg_localization_dur =
    duration_in_ms / static_cast<double>(m_num_computed + m_estimates.size());

  report <<
    "Period of localization estimates: " << avg_localization_dur << " ms." << std::endl <<
    "Processed " << m_num_computed << " samples. Left out " << m_estimates.size() <<
    " estimates." << std::endl <<
    "Average translation error: " << m_average_translation_err << " meters." << std::endl <<
    "Average rotation error: " << m_average_rotation_err << " radians." << std::endl;

  std::cout << report.str();
}

void LocalizationEvaluationNode::ground_truth_callback(Pose::ConstSharedPtr & ground_truth)
{
  // Check if there's a correspondence in ground truth
  const auto it = m_estimates.find(*ground_truth);
  // Assume unique
  if (it != m_estimates.end()) {
    compute(*it, *ground_truth);
    // remove
    (void)m_estimates.erase(it);
  } else {  // insert
    (void)m_ground_truths.insert(*ground_truth);
  }
}

void LocalizationEvaluationNode::localizer_callback(Pose::ConstSharedPtr & estimate)
{
  if (m_start_tp == std::chrono::steady_clock::time_point::min()) {
    m_start_tp = std::chrono::steady_clock::now();
  }
  m_end_tp = std::chrono::steady_clock::now();
  // Check if there's a correspondence in ground truth
  const auto it = m_ground_truths.find(*estimate);
  // Assume unique
  if (it != m_ground_truths.end()) {
    compute(*estimate, *it);
    // remove
    (void)m_ground_truths.erase(it);
  } else {  // insert
    (void)m_estimates.insert(*estimate);
  }
}

double LocalizationEvaluationNode::get_new_average(double current_avg, double addition)
{
  const auto current_num_samples = static_cast<double>(m_num_computed);
  const auto total_err = (current_avg * current_num_samples) + addition;
  return total_err / (current_num_samples + 1.0);
}

void LocalizationEvaluationNode::compute(const Pose & estimate, const Pose & ground_truth)
{
  EigTransform estimate_eig;
  EigTransform ground_truth_eig;
  EigTransform ground_truth_in_map_frame_eig;

  tf2::fromMsg(estimate.pose.pose, estimate_eig);
  tf2::fromMsg(ground_truth.pose.pose, ground_truth_eig);

  ground_truth_in_map_frame_eig = m_map_odom_tf * ground_truth_eig;

  Eigen::Quaterniond estimate_rot{estimate_eig.rotation()};
  Eigen::Quaterniond ground_truth_rot{ground_truth_in_map_frame_eig.rotation()};

  const auto translation_err =
    (ground_truth_in_map_frame_eig.translation() - estimate_eig.translation()).norm();
  const auto rotation_err = std::fabs(ground_truth_rot.angularDistance(estimate_rot));

  m_average_translation_err = get_new_average(m_average_translation_err, translation_err);
  m_average_rotation_err = get_new_average(m_average_rotation_err, rotation_err);
  ++m_num_computed;
}
}  // namespace localization_system_tests

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(localization_system_tests::LocalizationEvaluationNode)
