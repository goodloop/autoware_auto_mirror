// Copyright 2021 Robotec.ai
// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_
#define AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_

// Core
#include <deque>
#include <string>
#include <vector>

// ROS
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"

// Autoware
#include "autoware_auto_msgs/msg/autoware_state.hpp"
#include "autoware_auto_msgs/msg/emergency_mode.hpp"
#include "autoware_auto_msgs/msg/engage.hpp"
#include "autoware_auto_msgs/msg/had_map_route.hpp"
#include "autoware_auto_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_msgs/msg/vehicle_state_report.hpp"

// Local
#include "autoware_state_monitor/autoware_state.hpp"
#include "autoware_state_monitor/config.hpp"
#include "autoware_state_monitor/module_name.hpp"

struct StateInput
{
  ParamStats param_stats;
  TfStats tf_stats;

  rclcpp::Time current_time;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  autoware_auto_msgs::msg::RoutePoint::ConstSharedPtr goal_pose;

  autoware_auto_msgs::msg::Engage::ConstSharedPtr engage;
  autoware_auto_msgs::msg::VehicleStateReport::ConstSharedPtr vehicle_state_report;
  autoware_auto_msgs::msg::EmergencyMode::ConstSharedPtr emergency_mode;
  bool is_finalizing = false;
  bool is_route_reset_required = false;
  autoware_auto_msgs::msg::HADMapRoute::ConstSharedPtr route;
  autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr odometry;
  std::deque<autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr> odometry_buffer;
};

struct StateParam
{
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
};

struct Times
{
  rclcpp::Time arrived_goal;
  rclcpp::Time initializing_completed;
  rclcpp::Time planning_completed;
};

struct Flags
{
  bool waiting_after_initializing = false;
  bool waiting_after_planning = false;
};

class StateMachine
{
public:
  explicit StateMachine(const StateParam & state_param)
  : state_param_(state_param) {}

  AutowareState getCurrentState() const {return autoware_state_;}
  AutowareState updateState(const StateInput & state_input);
  std::vector<std::string> getMessages() const {return msgs_;}

private:
  AutowareState autoware_state_ = AutowareState::InitializingVehicle;
  StateInput state_input_;
  const StateParam state_param_;

  mutable AutowareState state_before_emergency_ = AutowareState::InitializingVehicle;
  mutable std::vector<std::string> msgs_;
  mutable Times times_;
  mutable Flags flags_;
  mutable autoware_auto_msgs::msg::HADMapRoute::ConstSharedPtr executing_route_ = nullptr;

  AutowareState judgeAutowareState() const;

  bool isModuleInitialized(const char * module_name) const;
  bool isVehicleInitialized() const;
  bool hasRoute() const;
  bool isRouteReceived() const;
  bool isPlanningCompleted() const;
  bool isEngaged() const;
  bool isOverridden() const;
  bool isEmergency() const;
  bool hasArrivedGoal() const;
  bool isFinalizing() const;
  bool isRouteResetRequired() const;
};

#endif  // AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_
