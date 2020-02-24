// Copyright 2020 Embotech AG, Zurich, Switzerland
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

#ifndef RECORDREPLAY_PLANNER__GEOMETRY_HPP_
#define RECORDREPLAY_PLANNER__GEOMETRY_HPP_


#include <motion_common/config.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/bounding_box.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace motion
{
namespace planning
{
namespace recordreplay_planner
{
using motion::motion_common::VehicleConfig;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::BoundingBox;

BoundingBox compute_boundingbox_from_trajectorypoint(
  const TrajectoryPoint & state,
  const VehicleConfig & vehicle_param);


// Check if two bounding boxes collide
bool boxes_collide(const BoundingBox & box1, const BoundingBox & box2);
}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion

#endif  // RECORDREPLAY_PLANNER__GEOMETRY_HPP_
