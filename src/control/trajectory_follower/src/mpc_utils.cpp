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

#include "trajectory_follower/mpc_utils.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

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
geometry_msgs::msg::Quaternion getQuaternionFromYaw(const double & yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void convertEulerAngleToMonotonic(std::vector<double> * a)
{
  if (!a) {
    return;
  }
  for (unsigned int i = 1; i < a->size(); ++i) {
    const double da = a->at(i) - a->at(i - 1);
    a->at(i) = a->at(i - 1) + normalizeRadian(da);
  }
}

double normalizeRadian(const double angle)
{
  double n_angle = std::fmod(angle, 2 * M_PI);
  n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
  return n_angle;
}

double calcDist2d(
  const geometry_msgs::msg::PoseStamped & p0, const geometry_msgs::msg::PoseStamped & p1)
{
  return calcDist2d(p0.pose.position, p1.pose.position);
}

double calcDist2d(
  const geometry_msgs::msg::Pose & p0, const geometry_msgs::msg::Pose & p1)
{
  return calcDist2d(p0.position, p1.position);
}

double calcSquaredDist2d(
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1)
{
  double dx = p1.x - p0.x;
  double dy = p1.y - p0.y;
  return dx * dx + dy * dy;
}

double calcDist3d(
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1)
{
  double dx = p1.x - p0.x;
  double dy = p1.y - p0.y;
  double dz = p1.z - p0.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double calcLateralError(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & ref_pose)
{
  const double err_x = ego_pose.position.x - ref_pose.position.x;
  const double err_y = ego_pose.position.y - ref_pose.position.y;
  const double ref_yaw = tf2::getYaw(ref_pose.orientation);
  const double lat_err = -std::sin(ref_yaw) * err_x + std::cos(ref_yaw) * err_y;
  return lat_err;
}

void calcMPCTrajectoryArclength(
  const MPCTrajectory & trajectory, std::vector<double> * arclength)
{
  double dist = 0.0;
  arclength->clear();
  arclength->push_back(dist);
  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    double dx = trajectory.x.at(i) - trajectory.x.at(i - 1);
    double dy = trajectory.y.at(i) - trajectory.y.at(i - 1);
    dist += std::sqrt(dx * dx + dy * dy);
    arclength->push_back(dist);
  }
}

bool resampleMPCTrajectoryByDistance(
  const MPCTrajectory & input, const double resample_interval_dist, MPCTrajectory * output)
{
  if (!output) {
    return false;
  }
  if (input.size() == 0) {
    *output = input;
    return true;
  }
  std::vector<double> input_arclength;
  calcMPCTrajectoryArclength(input, &input_arclength);

  if (input_arclength.size() == 0) {
    return false;
  }

  std::vector<double> output_arclength;
  for (double s = 0; s < input_arclength.back(); s += resample_interval_dist) {
    output_arclength.push_back(s);
  }

  std::vector<double> input_yaw = input.yaw;
  convertEulerAngleToMonotonic(&input_yaw);

  SplineInterpolate spline_interp;
  if (
    !spline_interp.interpolate(input_arclength, input.x, output_arclength, output->x) ||
    !spline_interp.interpolate(input_arclength, input.y, output_arclength, output->y) ||
    !spline_interp.interpolate(input_arclength, input.z, output_arclength, output->z) ||
    !spline_interp.interpolate(input_arclength, input_yaw, output_arclength, output->yaw) ||
    !linearInterpolate(input_arclength, input.vx, output_arclength, output->vx) ||
    !spline_interp.interpolate(input_arclength, input.k, output_arclength, output->k) ||
    !spline_interp.interpolate(
      input_arclength, input.smooth_k, output_arclength, output->smooth_k) ||
    !linearInterpolate(
      input_arclength, input.relative_time, output_arclength, output->relative_time))
  {
    std::cerr << "linearInterpMPCTrajectory error!" << std::endl;
    return false;
  }

  return true;
}

bool linearInterpMPCTrajectory(
  const std::vector<double> & in_index, const MPCTrajectory & in_traj,
  const std::vector<double> & out_index, MPCTrajectory * out_traj)
{
  if (!out_traj) {
    return false;
  }

  if (in_traj.size() == 0) {
    *out_traj = in_traj;
    return true;
  }

  std::vector<double> in_traj_yaw = in_traj.yaw;
  convertEulerAngleToMonotonic(&in_traj_yaw);

  if (
    !linearInterpolate(in_index, in_traj.x, out_index, out_traj->x) ||
    !linearInterpolate(in_index, in_traj.y, out_index, out_traj->y) ||
    !linearInterpolate(in_index, in_traj.z, out_index, out_traj->z) ||
    !linearInterpolate(in_index, in_traj_yaw, out_index, out_traj->yaw) ||
    !linearInterpolate(in_index, in_traj.vx, out_index, out_traj->vx) ||
    !linearInterpolate(in_index, in_traj.k, out_index, out_traj->k) ||
    !linearInterpolate(in_index, in_traj.smooth_k, out_index, out_traj->smooth_k) ||
    !linearInterpolate(
      in_index, in_traj.relative_time, out_index, out_traj->relative_time))
  {
    std::cerr << "linearInterpMPCTrajectory error!" << std::endl;
    return false;
  }

  if (out_traj->size() == 0) {
    std::cerr << "[mpc util] linear interpolation error" << std::endl;
    return false;
  }

  return true;
}

void calcTrajectoryYawFromXY(MPCTrajectory * traj)
{
  if (traj->yaw.size() == 0) {return;}

  for (unsigned int i = 1; i < traj->yaw.size() - 1; ++i) {
    const double dx = traj->x[i + 1] - traj->x[i - 1];
    const double dy = traj->y[i + 1] - traj->y[i - 1];
    traj->yaw[i] = std::atan2(dy, dx);
  }
  if (traj->yaw.size() > 1) {
    traj->yaw[0] = traj->yaw[1];
    traj->yaw.back() = traj->yaw[traj->yaw.size() - 2];
  }
}

bool calcTrajectoryCurvature(const size_t curvature_smoothing_num, MPCTrajectory * traj)
{
  if (!traj) {
    return false;
  }

  traj->k = calcTrajectoryCurvature(1, *traj);
  traj->smooth_k = calcTrajectoryCurvature(curvature_smoothing_num, *traj);
  return true;
}

std::vector<double> calcTrajectoryCurvature(
  const size_t curvature_smoothing_num, const MPCTrajectory & traj)
{
  const int traj_size = static_cast<int>(traj.x.size());
  std::vector<double> curvature_vec(traj.x.size());

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::msg::Point p1, p2, p3;
  const size_t max_smoothing_num = static_cast<size_t>(std::floor(0.5 * (traj_size - 1)));
  const size_t L = std::min(curvature_smoothing_num, max_smoothing_num);
  for (size_t i = L; i < traj.x.size() - L; ++i) {
    const size_t curr_idx = i;
    const size_t prev_idx = curr_idx - L;
    const size_t next_idx = curr_idx + L;
    p1.x = traj.x[prev_idx];
    p2.x = traj.x[curr_idx];
    p3.x = traj.x[next_idx];
    p1.y = traj.y[prev_idx];
    p2.y = traj.y[curr_idx];
    p3.y = traj.y[next_idx];
    double den = std::max(
      calcDist2d(p1, p2) * calcDist2d(p2, p3) * calcDist2d(
        p3,
        p1),
      std::numeric_limits<double>::epsilon());
    const double curvature =
      2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    curvature_vec.at(curr_idx) = curvature;
  }

  /* first and last curvature is copied from next value */
  for (size_t i = 0; i < std::min(L, traj.x.size()); ++i) {
    curvature_vec.at(i) = curvature_vec.at(std::min(L, traj.x.size() - 1));
    curvature_vec.at(
      traj.x.size() - i -
      1) = curvature_vec.at(std::max(traj.x.size() - L - 1, size_t(0)));
  }
  return curvature_vec;
}

bool convertToMPCTrajectory(
  const autoware_auto_msgs::msg::Trajectory & input, MPCTrajectory * output)
{
  if (!output) {
    return false;
  }

  output->clear();
  for (const autoware_auto_msgs::msg::TrajectoryPoint & p : input.points) {
    const double x = p.x;
    const double y = p.y;
    const double z = 0.0;
    const double yaw = ::motion::motion_common::to_angle(p.heading);
    const double vx = p.longitudinal_velocity_mps;
    const double k = 0.0;
    const double t = 0.0;
    output->push_back(x, y, z, yaw, vx, k, k, t);
  }
  calcMPCTrajectoryTime(output);
  return true;
}

bool calcMPCTrajectoryTime(MPCTrajectory * traj)
{
  if (!traj) {
    return false;
  }
  double t = 0.0;
  traj->relative_time.clear();
  traj->relative_time.push_back(t);
  for (size_t i = 0; i < traj->x.size() - 1; ++i) {
    double dx = traj->x.at(i + 1) - traj->x.at(i);
    double dy = traj->y.at(i + 1) - traj->y.at(i);
    double dz = traj->z.at(i + 1) - traj->z.at(i);
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double v = std::max(std::fabs(traj->vx.at(i)), 0.1);
    t += (dist / v);
    traj->relative_time.push_back(t);
  }
  return true;
}

void dynamicSmoothingVelocity(
  const size_t start_idx, const double start_vel, const double acc_lim, const double tau,
  MPCTrajectory * traj)
{
  double curr_v = start_vel;
  std::vector<double> smoothed_vel;
  MPCTrajectory tmp = *traj;
  traj->vx.at(start_idx) = start_vel;

  for (size_t i = start_idx + 1; i < traj->size(); ++i) {
    const double ds =
      std::hypot(traj->x.at(i) - traj->x.at(i - 1), traj->y.at(i) - traj->y.at(i - 1));
    const double dt = ds / std::max(std::fabs(curr_v), std::numeric_limits<double>::epsilon());
    const double a = tau / std::max(tau + dt, std::numeric_limits<double>::epsilon());
    const double updated_v = a * curr_v + (1.0 - a) * traj->vx.at(i);
    const double dv = std::max(-acc_lim * dt, std::min(acc_lim * dt, updated_v - curr_v));
    curr_v = curr_v + dv;
    traj->vx.at(i) = curr_v;
  }
  calcMPCTrajectoryTime(traj);
}

int calcNearestIndex(
  const MPCTrajectory & traj, const geometry_msgs::msg::Pose & self_pose)
{
  if (traj.size() == 0) {
    return -1;
  }
  const double my_yaw = tf2::getYaw(self_pose.orientation);
  int nearest_idx = -1;
  double min_dist_squared = std::numeric_limits<double>::max();
  for (size_t i = 0; i < traj.size(); ++i) {
    const double dx = self_pose.position.x - traj.x[i];
    const double dy = self_pose.position.y - traj.y[i];
    const double dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const double err_yaw = normalizeRadian(my_yaw - traj.yaw[i]);
    if (std::fabs(err_yaw) > (M_PI / 3.0)) {
      continue;
    }
    if (dist_squared < min_dist_squared) {
      min_dist_squared = dist_squared;
      nearest_idx = static_cast<int>(i);
    }
  }
  return nearest_idx;
}

int calcNearestIndex(
  const autoware_auto_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Pose & self_pose)
{
  if (traj.points.size() == 0) {
    return -1;
  }
  const double my_yaw = tf2::getYaw(self_pose.orientation);
  int nearest_idx = -1;
  double min_dist_squared = std::numeric_limits<double>::max();
  for (size_t i = 0; i < traj.points.size(); ++i) {
    const double dx = self_pose.position.x - static_cast<double>(traj.points.at(i).x);
    const double dy = self_pose.position.y - static_cast<double>(traj.points.at(i).y);
    const double dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const double traj_yaw = ::motion::motion_common::to_angle(traj.points.at(i).heading);
    const double err_yaw = normalizeRadian(my_yaw - traj_yaw);
    if (std::fabs(err_yaw) > (M_PI / 3.0)) {
      continue;
    }
    if (dist_squared < min_dist_squared) {
      min_dist_squared = dist_squared;
      nearest_idx = static_cast<int>(i);
    }
  }
  return nearest_idx;
}

bool calcNearestPoseInterp(
  const MPCTrajectory & traj, const geometry_msgs::msg::Pose & self_pose,
  geometry_msgs::msg::Pose * nearest_pose, size_t * nearest_index, double * nearest_time,
  rclcpp::Logger logger, rclcpp::Clock & clock)
{
  if (traj.size() == 0 || !nearest_pose || !nearest_index || !nearest_time) {
    return false;
  }
  int nearest_idx = calcNearestIndex(traj, self_pose);
  if (nearest_idx == -1) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger, clock, 3.0, "[calcNearestPoseInterp] fail to get nearest. traj.size = %ul",
      traj.size());
    return false;
  }

  const int traj_size = static_cast<int>(traj.size());

  *nearest_index = static_cast<size_t>(nearest_idx);

  if (traj.size() == 1) {
    nearest_pose->position.x = traj.x[*nearest_index];
    nearest_pose->position.y = traj.y[*nearest_index];
    nearest_pose->orientation = getQuaternionFromYaw(traj.yaw[*nearest_index]);
    *nearest_time = traj.relative_time[*nearest_index];
    return true;
  }

  auto calcSquaredDist =
    [](const geometry_msgs::msg::Pose & p, const MPCTrajectory & t, const size_t idx) {
      const double dx = p.position.x - t.x[idx];
      const double dy = p.position.y - t.y[idx];
      return dx * dx + dy * dy;
    };

  /* get second nearest index = next to nearest_index */
  const size_t next = static_cast<size_t>(std::min(nearest_idx + 1, traj_size - 1));
  const size_t prev = static_cast<size_t>(std::max(nearest_idx - 1, 0));
  const double dist_to_next = calcSquaredDist(self_pose, traj, next);
  const double dist_to_prev = calcSquaredDist(self_pose, traj, prev);
  const size_t second_nearest_index = (dist_to_next < dist_to_prev) ? next : prev;

  const double a_sq = calcSquaredDist(self_pose, traj, *nearest_index);
  const double b_sq = calcSquaredDist(self_pose, traj, second_nearest_index);
  const double dx3 = traj.x[*nearest_index] - traj.x[second_nearest_index];
  const double dy3 = traj.y[*nearest_index] - traj.y[second_nearest_index];
  const double c_sq = dx3 * dx3 + dy3 * dy3;

  /* if distance between two points are too close */
  if (c_sq < 1.0E-5) {
    nearest_pose->position.x = traj.x[*nearest_index];
    nearest_pose->position.y = traj.y[*nearest_index];
    nearest_pose->orientation = getQuaternionFromYaw(traj.yaw[*nearest_index]);
    *nearest_time = traj.relative_time[*nearest_index];
    return true;
  }

  /* linear interpolation */
  const double alpha = std::max(std::min(0.5 * (c_sq - a_sq + b_sq) / c_sq, 1.0), 0.0);
  nearest_pose->position.x =
    alpha * traj.x[*nearest_index] + (1 - alpha) * traj.x[second_nearest_index];
  nearest_pose->position.y =
    alpha * traj.y[*nearest_index] + (1 - alpha) * traj.y[second_nearest_index];
  double tmp_yaw_err = normalizeRadian(traj.yaw[*nearest_index] - traj.yaw[second_nearest_index]);
  const double nearest_yaw = normalizeRadian(traj.yaw[second_nearest_index] + alpha * tmp_yaw_err);
  nearest_pose->orientation = getQuaternionFromYaw(nearest_yaw);
  *nearest_time = alpha * traj.relative_time[*nearest_index] +
    (1 - alpha) * traj.relative_time[second_nearest_index];
  return true;
}
}  // namespace MPCUtils
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
