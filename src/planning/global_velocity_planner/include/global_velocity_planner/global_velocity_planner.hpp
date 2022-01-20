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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the global_velocity_planner class.

#ifndef GLOBAL_VELOCITY_PLANNER__GLOBAL_VELOCITY_PLANNER_HPP_
#define GLOBAL_VELOCITY_PLANNER__GLOBAL_VELOCITY_PLANNER_HPP_

#include <common/types.hpp>
#include <global_velocity_planner/visibility_control.hpp>
#include <motion_common/config.hpp>
#include <motion_common/motion_common.hpp>

#include <autoware_auto_mapping_msgs/srv/had_map_service.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_report.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace autoware
{
/// \brief TODO(berkay): Document namespaces!
namespace global_velocity_planner
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using lanelet::LaneletMapConstPtr;
using motion::motion_common::VehicleConfig;
using State = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;

struct GLOBAL_VELOCITY_PLANNER_PUBLIC point
{
  TrajectoryPoint point;
  float32_t speed_limit;
  float32_t curvature;
};

struct GLOBAL_VELOCITY_PLANNER_PUBLIC GlobalVelocityPlannerConfig
{
  float32_t trajectory_resolution;
  float32_t lateral_acceleration;
  float32_t longitudinal_acceleration;
};

class GLOBAL_VELOCITY_PLANNER_PUBLIC GlobalVelocityPlanner
{
public:
  explicit GlobalVelocityPlanner(
    const VehicleConfig & vehicle_param, const GlobalVelocityPlannerConfig & planner_config);
  std::shared_ptr<std::vector<point>> way_points;
  bool8_t is_route_ready = false;
  Trajectory trajectory;
  VehicleConfig vehicle_param;
  GlobalVelocityPlannerConfig velocity_planner_config;
  // functions
  void set_route(const HADMapRoute & route, const lanelet::LaneletMapPtr & lanelet_map_ptr);
  bool8_t is_route_empty();
  bool8_t is_route_over();
  void clear_route();
  void calculate_waypoints();
  void calculate_trajectory(const State & pose);

private:
  void calculate_curvatures();
  void vel_wrt_lateral_acceleration();
  void vel_wrt_longitudinal_acceleration();
  void set_steering_angle(point & pt);
  void set_orientation(size_t i);
  void set_acceleration();
  void set_time_from_start();
  bool8_t need_trajectory();
  size_t get_closest_index(const State & pose);
  // variables

  std::shared_ptr<HADMapRoute> route;
  lanelet::LaneletMapPtr map;
  size_t last_point;
};
/// \brief TODO(berkay): Document your functions
}  // namespace global_velocity_planner
}  // namespace autoware

#endif  // GLOBAL_VELOCITY_PLANNER__GLOBAL_VELOCITY_PLANNER_HPP_
