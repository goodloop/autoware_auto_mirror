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

#ifndef TRAJECTORY_FOLLOWER__MPC_UTILS_HPP_
#define TRAJECTORY_FOLLOWER__MPC_UTILS_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "trajectory_follower/interpolate.hpp"
#include "trajectory_follower/mpc_trajectory.hpp"
#include "trajectory_follower/visibility_control.hpp"

#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "autoware_auto_msgs/msg/trajectory_point.hpp"
#include "common/types.hpp"
#include "eigen3/Eigen/Core"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "motion_common/motion_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
namespace MPCUtils
{
using autoware::common::types::float64_t;
/**
 * @brief convert from yaw to ros-Quaternion
 * @param [in] yaw input yaw angle
 * @return quaternion
 */
TRAJECTORY_FOLLOWER_PUBLIC geometry_msgs::msg::Quaternion getQuaternionFromYaw(
  const float64_t & yaw);

/**
 * @brief normalize angle into [-pi to pi]
 * @param [in] angle input angle
 * @return normalized angle
 */
TRAJECTORY_FOLLOWER_PUBLIC float64_t normalizeRadian(const float64_t angle);

/**
 * @brief convert Euler angle vector including +-2pi to 0 jump to continuous series data
 * @param [out] a input angle vector
 */
TRAJECTORY_FOLLOWER_PUBLIC void convertEulerAngleToMonotonic(std::vector<float64_t> * a);
TRAJECTORY_FOLLOWER_PUBLIC float64_t calcDist2d(
  const geometry_msgs::msg::PoseStamped & p0, const geometry_msgs::msg::PoseStamped & p1);
TRAJECTORY_FOLLOWER_PUBLIC float64_t calcDist2d(
  const geometry_msgs::msg::Pose & p0,
  const geometry_msgs::msg::Pose & p1);
template<typename T0, typename T1>
TRAJECTORY_FOLLOWER_PUBLIC float64_t calcDist2d(const T0 & p0, const T1 & p1)
{
  return std::hypot(p0.x - p1.x, p0.y - p1.y);
}
TRAJECTORY_FOLLOWER_PUBLIC float64_t calcDist3d(
  const geometry_msgs::msg::Point & p0,
  const geometry_msgs::msg::Point & p1);
TRAJECTORY_FOLLOWER_PUBLIC float64_t calcSquaredDist2d(
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1);
TRAJECTORY_FOLLOWER_PUBLIC float64_t calcLateralError(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & ref_pose);

TRAJECTORY_FOLLOWER_PUBLIC bool convertToMPCTrajectory(
  const autoware_auto_msgs::msg::Trajectory & input, MPCTrajectory * output);
TRAJECTORY_FOLLOWER_PUBLIC void calcMPCTrajectoryArclength(
  const MPCTrajectory & trajectory,
  std::vector<float64_t> * arclength);
TRAJECTORY_FOLLOWER_PUBLIC bool resampleMPCTrajectoryByDistance(
  const MPCTrajectory & input, const float64_t resample_interval_dist, MPCTrajectory * output);
TRAJECTORY_FOLLOWER_PUBLIC bool linearInterpMPCTrajectory(
  const std::vector<float64_t> & in_index, const MPCTrajectory & in_traj,
  const std::vector<float64_t> & out_index, MPCTrajectory * out_traj);
TRAJECTORY_FOLLOWER_PUBLIC bool calcMPCTrajectoryTime(MPCTrajectory * traj);
TRAJECTORY_FOLLOWER_PUBLIC void dynamicSmoothingVelocity(
  const size_t start_idx, const float64_t start_vel, const float64_t acc_lim, const float64_t tau,
  MPCTrajectory * traj);

/**
 * @brief calculate yaw angle in MPCTrajectory from xy vector
 * @param [inout] traj object trajectory
 */
TRAJECTORY_FOLLOWER_PUBLIC void calcTrajectoryYawFromXY(MPCTrajectory * traj);

/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3 points when num = 1)
 * @param [in] curvature_smoothing_num index distance for 3 points for curvature calculation
 * @param [inout] traj object trajectory
 */
TRAJECTORY_FOLLOWER_PUBLIC bool calcTrajectoryCurvature(
  const size_t curvature_smoothing_num,
  MPCTrajectory * traj);

/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3 points when num = 1)
 * @param [in] curvature_smoothing_num index distance for 3 points for curvature calculation
 * @param [in] traj input trajectory
 * @return vector of curvatures at each point of the given trajectory
 */
TRAJECTORY_FOLLOWER_PUBLIC std::vector<float64_t> calcTrajectoryCurvature(
  const size_t curvature_smoothing_num, const MPCTrajectory & traj);

/**
 * @brief calculate nearest pose on MPCTrajectory with linear interpolation
 * @param [in] traj reference trajectory
 * @param [in] self_pose object pose
 * @param [out] nearest_pose nearest pose on path
 * @param [out] nearest_index path index of nearest pose
 * @param [out] nearest_time time of nearest pose on trajectory
 * @param [out] logger to output the reason for failure
 * @param [in] clock to throttle log output
 * @return false when nearest pose couldn't find for some reasons
 */
TRAJECTORY_FOLLOWER_PUBLIC bool calcNearestPoseInterp(
  const MPCTrajectory & traj, const geometry_msgs::msg::Pose & self_pose,
  geometry_msgs::msg::Pose * nearest_pose, size_t * nearest_index, float64_t * nearest_time,
  rclcpp::Logger logger, rclcpp::Clock & clock);

TRAJECTORY_FOLLOWER_PUBLIC int calcNearestIndex(
  const MPCTrajectory & traj,
  const geometry_msgs::msg::Pose & self_pose);
TRAJECTORY_FOLLOWER_PUBLIC int calcNearestIndex(
  const autoware_auto_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Pose & self_pose);
}  // namespace MPCUtils
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
#endif  // TRAJECTORY_FOLLOWER__MPC_UTILS_HPP_
