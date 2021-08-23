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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the behavior_planner class.

#ifndef BEHAVIOR_PLANNER__TRAJECTORY_MANAGER_HPP_
#define BEHAVIOR_PLANNER__TRAJECTORY_MANAGER_HPP_

#include <behavior_planner/visibility_control.hpp>

#include <common/types.hpp>
#include <autoware_auto_msgs/msg/had_map_route.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

#include <iostream>
#include <vector>

namespace autoware
{
/// \brief TODO(ryohsuke.mitsudome): Document namespaces!
namespace behavior_planner
{

using autoware_auto_msgs::msg::Trajectory;
using autoware_auto_msgs::msg::TrajectoryPoint;
using State = autoware_auto_msgs::msg::VehicleKinematicState;

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

struct BEHAVIOR_PLANNER_PUBLIC PlannerConfig
{
  float32_t goal_distance_thresh;
  float32_t stop_velocity_thresh;
  float32_t heading_weight;
  float32_t subroute_goal_offset_lane2parking;
  float32_t subroute_goal_offset_parking2lane;
  float32_t cg_to_vehicle_center;
  bool8_t include_current_state;
  float32_t prepend_distance;
};


// TODO(mitsudome-r): remove this class after making controller
// able to handle whole trajectory with turning points
/// \class TrajectoryManager
/// \brief Class that splits trajectory at gear changing points and publish them separately
class BEHAVIOR_PLANNER_PUBLIC TrajectoryManager
{
public:
  explicit TrajectoryManager(const PlannerConfig & config);

  void clear_trajectory();
  void set_trajectory(const Trajectory & trajectory);
  Trajectory get_trajectory(const State & state);
  size_t get_remaining_length(const State & state);
  bool8_t is_trajectory_ready();
  bool8_t has_arrived_subgoal(const State & state);

private:
  void set_sub_trajectories();
  std::size_t get_closest_state(const State & state, const Trajectory & trajectory);
  void set_time_from_start(Trajectory * trajectory, const size_t start_index);

  /// \brief Generate trajectory points before the given index with up to prepend_distance
  /// \param [in] from_index generate points before this index
  /// \param [out] previous_points calculated previous points
  /// \return length along the previous points (starting from from_index)
  float32_t generate_previous_points(
    const size_t from_index,
    std::vector<TrajectoryPoint> & previous_points);

  /// \brief Create an extrapolated point behind the trajectory
  /// \param [in] first_index first index of the trajectory
  /// \param [in] previous_point points before the first point of the trajectory
  /// \param [in] extra_dist distance to extrapolate
  /// \param [out] extra_point resulting extrapolated point
  /// \return true if the extrapolation was successful, false otherwise
  bool extrapolate(
    const size_t first_index, const std::vector<TrajectoryPoint> & previous_points,
    const float32_t extra_dist, TrajectoryPoint & extra_point);

  // parameters
  const PlannerConfig m_config;
  Trajectory m_trajectory;
  std::vector<Trajectory> m_sub_trajectories;
  size_t m_selected_trajectory;
};

}  // namespace behavior_planner
}  // namespace autoware

#endif  // BEHAVIOR_PLANNER__TRAJECTORY_MANAGER_HPP_
