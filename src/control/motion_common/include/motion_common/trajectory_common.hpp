// Copyright 2021 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOTION_COMMON__TRAJECTORY_COMMON_HPP_
#define MOTION_COMMON__TRAJECTORY_COMMON_HPP_

#include <experimental/optional>
#include <limits>
#include <stdexcept>
#include <vector>

#include "autoware_auto_msgs/msg/trajectory_point.hpp"
#include "common/types.hpp"
#include "eigen3/Eigen/Core"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "helper_functions/message_distance.hpp"
#include "helper_functions/angle_utils.hpp"
#include "motion_common/motion_common.hpp"
#include "tf2/utils.h"

namespace autoware
{
namespace motion
{
namespace motion_common
{
typedef autoware_auto_msgs::msg::TrajectoryPoint Point;
typedef std::vector<Point> Points;
using autoware::common::types::float64_t;
typedef Eigen::Matrix<float64_t, 3, 1> Vector3f;

/**
 * @brief throws an exception if the given list of points is empty
 * @param [in] points list of points to check
 */
void validateNonEmpty(const Points & points)
{
  if (points.empty()) {
    throw std::invalid_argument("Empty points");
  }
}

/**
 * @brief calculate the yaw deviation between two angles
 * @param [in] base_yaw base yaw angle [radians]
 * @param [in] target_yaw target yaw angle [radians]
 * @return normalized angle from the base to the target [radians]
 */
inline float64_t calcYawDeviation(const float64_t & base_yaw, const float64_t & target_yaw)
{
  return autoware::common::helper_functions::wrap_angle(target_yaw - base_yaw);
}


/**
 * @brief search first index with a velocity of zero in the given range of points
 * @param [in] points list of points to check
 * @param [in] src_idx starting search index
 * @param [in] dst_idx ending (excluded) search index
 * @param [in] epsilon optional value to use to determine zero velocities
 * @return index of the first zero velocity point found
 */
std::experimental::optional<size_t> searchZeroVelocityIndex(
  const Points & points, const size_t src_idx, const size_t dst_idx, const float64_t epsilon = 1e-3)
{
  validateNonEmpty(points);

  for (size_t i = src_idx; i < dst_idx; ++i) {
    if (static_cast<float64_t>(std::fabs(points.at(i).longitudinal_velocity_mps)) < epsilon) {
      return i;
    }
  }

  return {};
}

/**
 * @brief search first index with a velocity of zero in the given points
 * @param [in] points list of points to check
 */
std::experimental::optional<size_t> searchZeroVelocityIndex(const Points & points)
{
  return searchZeroVelocityIndex(points, 0, points.size());
}

/**
 * @brief search the index of the point nearest to the given target
 * @param [in] points list of points to search
 * @param [in] point target point
 * @return index of the point nearest to the target
 */
size_t findNearestIndex(const Points & points, const geometry_msgs::msg::Point & point)
{
  validateNonEmpty(points);

  float64_t min_dist = std::numeric_limits<float64_t>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = autoware::common::helper_functions::calcDist2d(points.at(i), point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

/**
 * @brief search the index of the point nearest to the given target with limits on the distance and yaw deviation
 * @param [in] points list of points to search
 * @param [in] pose target point
 * @return index of the point nearest to the target
 */
std::experimental::optional<size_t> findNearestIndex(
  const Points & points, const geometry_msgs::msg::Pose & pose,
  const float64_t max_dist = std::numeric_limits<float64_t>::max(),
  const float64_t max_yaw = std::numeric_limits<float64_t>::max())
{
  validateNonEmpty(points);

  float64_t min_dist = std::numeric_limits<float64_t>::max();
  bool is_nearest_found = false;
  size_t min_idx = 0;

  const auto target_yaw = tf2::getYaw(pose.orientation);
  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = autoware::common::helper_functions::calcDist2d(points.at(i), pose.position);
    if (dist > max_dist) {
      continue;
    }

    const auto base_yaw = ::motion::motion_common::to_angle(points.at(i).heading);
    const auto yaw = calcYawDeviation(base_yaw, target_yaw);
    if (std::fabs(yaw) > max_yaw) {
      continue;
    }

    if (dist >= min_dist) {
      continue;
    }

    min_dist = dist;
    min_idx = i;
    is_nearest_found = true;
  }
  return is_nearest_found ? std::experimental::optional<size_t>(min_idx) : std::experimental::
         nullopt;
}

/**
  * @brief calculate length along trajectory from seg_idx point to nearest point to p_target on trajectory
  *        If seg_idx point is after that nearest point, length is negative
  * @param points trajectory points
  * @param seg_idx segment index of point at beginning of length
  * @param p_target target point at end of length
  * @return signed length
  */
float64_t calcLongitudinalOffsetToSegment(
  const Points & points, const size_t seg_idx, const geometry_msgs::msg::Point & p_target)
{
  validateNonEmpty(points);

  const auto p_front = points.at(seg_idx);
  const auto p_back = points.at(seg_idx + 1);
  const auto x_front = static_cast<float64_t>(p_front.x);
  const auto y_front = static_cast<float64_t>(p_front.y);
  const auto x_back = static_cast<float64_t>(p_back.x);
  const auto y_back = static_cast<float64_t>(p_back.y);

  const Vector3f segment_vec{x_back - x_front, y_back - y_front, 0.0};
  const Vector3f target_vec{static_cast<float64_t>(p_target.x) - x_front,
    static_cast<float64_t>(p_target.y) - y_front, 0.0};

  if (segment_vec.norm() == 0.0) {
    throw std::runtime_error("Same points are given.");
  }

  return segment_vec.dot(target_vec) / segment_vec.norm();
}

/**
  * @brief find nearest segment index to point
  *        segment is straight path between two continuous points of trajectory
  *        When point is on a trajectory point whose index is nearest_idx, return nearest_idx - 1
  * @param points points of trajectory
  * @param point point to which to find nearest segment index
  * @return nearest index
  */
size_t findNearestSegmentIndex(const Points & points, const geometry_msgs::msg::Point & point)
{
  const size_t nearest_idx = findNearestIndex(points, point);

  if (nearest_idx == 0) {
    return 0;
  } else if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const float64_t signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, point);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}

/**
  * @brief calculate arc length along points
  * @param [in] points input points
  * @param [in] src_point source point
  * @param [in] dst_idx destination index
  * @return arc length distance from source to destination along the input points
  */
float64_t calcSignedArcLength(const Points & points, const size_t src_idx, const size_t dst_idx)
{
  validateNonEmpty(points);

  if (src_idx > dst_idx) {
    return -calcSignedArcLength(points, dst_idx, src_idx);
  }

  float64_t dist_sum = 0.0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    dist_sum += autoware::common::helper_functions::calcDist2d(points.at(i), points.at(i + 1));
  }
  return dist_sum;
}

/**
  * @brief calculate arc length along points
  * @param [in] points input points
  * @param [in] src_point source point
  * @param [in] dst_idx destination index
  * @return arc length distance from source to destination along the input points
  */
float64_t calcSignedArcLength(
  const Points & points, const geometry_msgs::msg::Point & src_point, const size_t & dst_idx)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);

  const float64_t signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_idx);
  const float64_t signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return signed_length_on_traj - signed_length_src_offset;
}

/**
  * @brief calculate arc length along points
  * @param [in] points input points
  * @param [in] src_point source point
  * @param [in] dst_point destination point
  * @return arc length distance from source to destination along the input points
  */
float64_t calcSignedArcLength(
  const Points & points, const geometry_msgs::msg::Point & src_point,
  const geometry_msgs::msg::Point & dst_point)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);
  const size_t dst_seg_idx = findNearestSegmentIndex(points, dst_point);

  const float64_t signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_seg_idx);
  const float64_t signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);
  const float64_t signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj - signed_length_src_offset + signed_length_dst_offset;
}

/**
  * @brief calculate longitudinal deviation of a point relative to a pose
  * @param [in] base_pose base from which to calculate the deviation
  * @param [in] target_point point for which to calculate the deviation
  * @return longitudinal distance between the base and the target
  */
inline float64_t calcLongitudinalDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point)
{
  const auto & base_point = base_pose.position;

  const auto yaw = tf2::getYaw(base_pose.orientation);
  const Vector3f base_unit_vec{std::cos(yaw), std::sin(yaw), 0};

  const auto dx = target_point.x - base_point.x;
  const auto dy = target_point.y - base_point.y;
  const Vector3f diff_vec{dx, dy, 0};

  return base_unit_vec.dot(diff_vec);
}

}  // namespace motion_common
}  // namespace motion
}  // namespace autoware

#endif  // MOTION_COMMON__TRAJECTORY_COMMON_HPP_
