// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__DATA_MANAGER_HPP_
#define BEHAVIOR_PATH_PLANNER__DATA_MANAGER_HPP_

#include <memory>
#include <string>

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"

#include "behavior_path_planner/predicted_objects_msg.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "behavior_path_planner/parameters.hpp"
#include "behavior_path_planner/route_handler.hpp"

namespace behavior_path_planner
{
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
struct BoolStamped
{
  explicit BoolStamped(bool in_data)
  : data(in_data) {}
  bool data{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct ModuleNameStamped
{
  std::string module_name = "NONE";
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct Approval
{
  BoolStamped is_approved{false};
  ModuleNameStamped is_force_approved{};
};

struct PlannerData
{
  PlannerData()
  : self_pose{std::make_shared<PoseStamped>()},
    self_velocity{std::make_shared<TwistStamped>()},
    dynamic_object{std::make_shared<PredictedObjects>()},
    reference_path{std::make_shared<PathWithLaneId>()},
    prev_output_path{std::make_shared<PathWithLaneId>()},
    route_handler{std::make_shared<RouteHandler>()}
  {
  }
  PoseStamped::ConstSharedPtr self_pose;
  TwistStamped::ConstSharedPtr self_velocity;
  std::shared_ptr<const PredictedObjects> dynamic_object;
  PathWithLaneId::SharedPtr reference_path;
  PathWithLaneId::SharedPtr prev_output_path;
  BehaviorPathPlannerParameters parameters;
  lanelet::ConstLanelets current_lanes;
  std::shared_ptr<RouteHandler> route_handler;
  Approval approval;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__DATA_MANAGER_HPP_
