// Copyright 2020 The Autoware Foundation
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

#include "trajectory_spoofer/trajectory_spoofer.hpp"

#include <motion_common/motion_common.hpp>
#include <time_utils/time_utils.hpp>

#include <chrono>
#include <cmath>

namespace autoware
{
namespace trajectory_spoofer
{
TrajectorySpoofer::TrajectorySpoofer()
: target_speed_(0.0)
{
}

TrajectorySpoofer::TrajectorySpoofer(float32_t target_speed)
: target_speed_(target_speed)
{
}

std::chrono::nanoseconds TrajectorySpoofer::get_travel_time_ns(float32_t dist, float32_t speed)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<float64_t>(dist / speed));
}

float32_t TrajectorySpoofer::get_target_speed()
{
  return target_speed_;
}

void TrajectorySpoofer::set_target_speed(float32_t target_speed)
{
  target_speed_ = target_speed;
}

float64_t TrajectorySpoofer::to_yaw_angle(const Complex32 & quat_2d)
{
  // theta = atan2(2qxqw, 1-2(qw)^2)
  const float32_t sin_y = 2.0F * quat_2d.real * quat_2d.imag;
  const float32_t cos_y = 1.0F - 2.0F * quat_2d.imag * quat_2d.imag;
  const float32_t rad_quad = std::atan2(sin_y, cos_y);

  if (rad_quad < 0) {
    return rad_quad + TAU;
  } else {
    return rad_quad;
  }
}

Trajectory TrajectorySpoofer::init_trajectory(
  const VehicleKinematicState & starting_state,
  TrajectoryPoint & first_point)
{
  Trajectory trajectory;
  trajectory.header = starting_state.header;

  // Start from the current location and time
  first_point = starting_state.state;
  trajectory.points.push_back(first_point);

  return trajectory;
}

Trajectory TrajectorySpoofer::spoof_straight_trajectory(
  const VehicleKinematicState & starting_state,
  int32_t num_of_points,
  float32_t length,
  bool8_t speed_ramp_on)
{
  TrajectoryPoint pt;
  Trajectory straight_trajectory = init_trajectory(starting_state, pt);
  straight_trajectory.points[0].longitudinal_velocity_mps = target_speed_;


  const auto yaw_angle = ::motion::motion_common::to_angle(starting_state.state.pose.orientation);
  const float64_t seg_len =
    static_cast<float64_t>(length) / static_cast<float64_t>(num_of_points - 1);
  const auto start_time = time_utils::from_message(straight_trajectory.points[0].time_from_start);
  const auto time_delta = get_travel_time_ns(static_cast<float32_t>(seg_len), target_speed_);

  for (int i = 1; i < num_of_points; ++i) {
    pt.time_from_start = time_utils::to_message(start_time + i * time_delta);
    pt.pose.position.x = std::cos(yaw_angle) * i * seg_len;
    pt.pose.position.y = std::sin(yaw_angle) * i * seg_len;
    pt.longitudinal_velocity_mps = target_speed_;
    straight_trajectory.points.push_back(pt);
  }

  return straight_trajectory;
}

Trajectory TrajectorySpoofer::spoof_circular_trajectory(
  const VehicleKinematicState & starting_state,
  int32_t num_of_points,
  float32_t radius,
  bool8_t speed_ramp_on)
{
  TrajectoryPoint pt;
  Trajectory circular_trajectory = init_trajectory(starting_state, pt);
  circular_trajectory.points[0].longitudinal_velocity_mps = target_speed_;


  // Number of segments = number of points - 1
  const float64_t seg_angle_rad =
    static_cast<float64_t>(TAU) / static_cast<float64_t>(num_of_points - 1);
  const float64_t seg_len =
    2.0 * static_cast<float64_t>(radius) * std::sin(seg_angle_rad / 2.0);
  const auto start_time = time_utils::from_message(circular_trajectory.points[0].time_from_start);
  const auto time_delta = get_travel_time_ns(static_cast<float32_t>(seg_len), target_speed_);

  for (int i = 1; i < num_of_points; ++i) {
    const float64_t old_head = ::motion::motion_common::to_angle(pt.pose.orientation);
    const float64_t angle_dist_remain =
      static_cast<float64_t>(TAU) - static_cast<float64_t>(i) * seg_angle_rad;

    if (i < num_of_points - 1) {
      // TODO(josh.whitley): Y values still not quite right
      pt.pose.position.x = pt.pose.position.x + std::cos(old_head) * seg_len;
      pt.pose.position.y = pt.pose.position.y + std::sin(old_head) * seg_len;
      pt.longitudinal_velocity_mps = target_speed_;

      float64_t new_head = old_head + angle_dist_remain / (num_of_points - i - 1);

      // Clip to 0 to 2pi
      if (new_head < 0) {
        new_head += static_cast<float64_t>(TAU);
      } else if (new_head >= static_cast<float64_t>(TAU)) {
        new_head -= static_cast<float64_t>(TAU);
      }

      pt.pose.orientation = ::motion::motion_common::from_angle(new_head);
    } else {  // last point  (num_of_points -1)
      pt.pose.position = starting_state.state.pose.position;
      pt.pose.orientation = starting_state.state.pose.orientation;
      pt.longitudinal_velocity_mps = target_speed_;
    }

    pt.time_from_start = time_utils::to_message(start_time + i * time_delta);

    circular_trajectory.points.push_back(pt);
  }

  return circular_trajectory;
}

Trajectory TrajectorySpoofer::spoof_curved_trajectory(
  const VehicleKinematicState & starting_state,
  int32_t num_of_points,
  float32_t radius,
  float32_t length,
  CurveType mode,
  bool8_t speed_ramp_on)
{
  TrajectoryPoint pt;
  Trajectory curved_trajectory = init_trajectory(starting_state, pt);


  // TODO(josh.whitley): Populate

  return curved_trajectory;
}
}  // namespace trajectory_spoofer
}  // namespace autoware
