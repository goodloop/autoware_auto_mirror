// Copyright 2019 Apex.AI, Inc.
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

#ifndef NDT__NDT_LOCALIZER_HPP_
#define NDT__NDT_LOCALIZER_HPP_

#include <localization_common/localizer_interface.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <ndt/ndt_common.hpp>
#include <ndt/ndt_optimization_problem.hpp>
#include <optimization/optimizer_options.hpp>
#include <utility>
#include <string>

namespace autoware
{
namespace localization
{
namespace ndt
{
/// Base class for NDT based localizers. Implementations must implement the validation logic.
template<typename MapT, typename OptimizerT>
class NDT_PUBLIC NdtLocalizer
  : public localization_common::LocalizerInterface<sensor_msgs::msg::PointCloud2>
{
public:
  explicit NdtLocalizer(
    MapT && map,  // TODO(igor): does it make sense to *take* ownership? Can we create it?
    const OptimizerT & optimizer,
    const Real outlier_ratio,
    const std::chrono::nanoseconds guess_time_tolerance)
  : m_map{std::move(map)},
    m_optimizer{optimizer},
    m_outlier_ratio{outlier_ratio},
    m_guess_time_tolerance{guess_time_tolerance} {}

  /// Register a measurement to the current map and return the transformation from map to the
  /// measurement.
  /// \param[in] msg Point cloud to be registered.
  /// \param[in] transform_initial Initial transformation guess.
  /// \param[out] pose_out Transformation from the map frame to the measurement's frame.
  /// \return Registration summary. T
  /// \throws std::logic_error on measurements older than the map.
  /// \throws std::domain_error on pose estimates that are not within the configured duration
  /// range from the measurement.
  /// \throws std::runtime_error on numerical errors in the optimizer.
  common::optimization::OptimizationSummary register_measurement(
    const sensor_msgs::msg::PointCloud2 & msg,
    const geometry_msgs::msg::TransformStamped & transform_initial,
    geometry_msgs::msg::PoseWithCovarianceStamped & pose_out) override
  {
    validate_msg(msg);
    validate_guess(msg, transform_initial);
    // Initial checks passed, proceed with initialization
    // Eigen representations to be used for internal computations.
    EigenPose<Real> eig_pose_initial, eig_pose_result;
    eig_pose_initial.setZero();
    eig_pose_result.setZero();
    // Convert the ros transform/pose to eigen pose vector
    transform_adapters::transform_to_pose(transform_initial.transform, eig_pose_initial);

    // Set the scan
    P2DNDTScan scan{msg};

    // Define and solve the problem.
    P2DNDTOptimizationProblem<MapT> problem(scan, m_map, m_outlier_ratio);
    const auto opt_summary = m_optimizer.solve(problem, eig_pose_initial, eig_pose_result);

    if (opt_summary.termination_type() == common::optimization::TerminationType::FAILURE) {
      throw std::runtime_error("NDT localizer has likely encountered a numerical "
              "error during optimization.");
    }

    // Convert eigen pose back to ros pose/transform
    transform_adapters::pose_to_transform(eig_pose_result,
      pose_out.pose.pose);

    pose_out.header.stamp = msg.header.stamp;
    pose_out.header.frame_id = m_map.frame_id();

    // Populate covariance information. It is implementation defined.
    set_covariance(problem, eig_pose_initial, eig_pose_result, pose_out);
    return opt_summary;
  }

private:
  /// Populate the covariance information of an ndt estimate using the information using existing
  /// information regarding scan, map and the optimization problem.
  /// \param[in] problem Optimization problem.
  /// \param[in] initial_guess Initial transformation guess as a pose.
  /// \param[in] pose_result Estimated transformation as a pose.
  /// \param[out] solution Estimated transform message.
  virtual void set_covariance(
    const P2DNDTOptimizationProblem<MapT> & problem,
    const EigenPose<Real> & initial_guess,
    const EigenPose<Real> & pose_result,
    geometry_msgs::msg::PoseWithCovarianceStamped & solution) const
  {
    (void) problem;
    (void) initial_guess;
    (void) pose_result;
    (void) solution;
    // For now, do nothing.
  }

  /// Check if the received message is valid to be registered. Following checks are made:
  /// * Message timestamp is not older than the map timestamp.
  /// \param msg Message to register.
  /// \throws std::logic_error on old data.
  void validate_msg(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    const auto message_time = ::time_utils::from_message(msg.header.stamp);
    // Map shouldn't be newer than a measurement.
    if (message_time < m_map.stamp()) {
      throw std::logic_error("Lidar scan should not have a timestamp older than the timestamp of"
              "the current map.");
    }
  }

  /// Check if the initial guess is valid. Following checks are made:
  /// * pose guess timestamp is within a tolerated range from the scan timestamp.
  /// \param msg Message to register
  /// \param transform_initial Initial pose estimate
  /// \throws std::domain_error on untimely initial pose.
  void validate_guess(
    const sensor_msgs::msg::PointCloud2 & msg,
    const geometry_msgs::msg::TransformStamped & transform_initial) const
  {
    const auto message_time = ::time_utils::from_message(msg.header.stamp);

    const auto guess_scan_diff =
      ::time_utils::from_message(transform_initial.header.stamp) - message_time;
    // An initial estimate should be comparable in time to the measurement's time stamp
    using TimeToleranceDurationT = std::decay_t<decltype(m_guess_time_tolerance)>;
    if (std::abs(std::chrono::duration_cast<TimeToleranceDurationT>(guess_scan_diff).count()) >
      std::abs(m_guess_time_tolerance.count()))
    {
      throw std::domain_error("Initial guess is not within: " +
              std::to_string(m_guess_time_tolerance.count()) +
              "ns range of the scan's time stamp. Either increase the tolerance range or"
              "make sure the localizer takes in timely initial pose guesses.");
    }
  }

  MapT m_map;
  OptimizerT m_optimizer;
  Real m_outlier_ratio;
  std::chrono::nanoseconds m_guess_time_tolerance;
};

}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_LOCALIZER_HPP_
