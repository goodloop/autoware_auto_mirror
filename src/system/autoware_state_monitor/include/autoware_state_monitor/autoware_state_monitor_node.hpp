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

#ifndef AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_
#define AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_

// Core
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

// ROS
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

// Autoware
#include "autoware_auto_msgs/msg/autoware_state.hpp"
#include "autoware_auto_msgs/msg/emergency_mode.hpp"
#include "autoware_auto_msgs/msg/engage.hpp"
#include "autoware_auto_msgs/msg/had_map_route.hpp"
#include "autoware_auto_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_msgs/msg/vehicle_state_report.hpp"

// Local
#include "autoware_state_monitor/state_machine.hpp"
#include "autoware_state_monitor/autoware_state.hpp"
#include "autoware_state_monitor/config.hpp"

class AutowareStateMonitorNode : public rclcpp::Node
{
public:
  AutowareStateMonitorNode();

private:
  // Parameter
  double update_rate_;

  std::vector<ParamConfig> param_configs_;
  std::vector<TfConfig> tf_configs_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;

  // CallbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // Subscriber
  rclcpp::Subscription<autoware_auto_msgs::msg::Engage>::SharedPtr sub_engage_;
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleStateReport>::SharedPtr
    sub_vehicle_state_report_;
  rclcpp::Subscription<autoware_auto_msgs::msg::EmergencyMode>::SharedPtr sub_is_emergency_;
  rclcpp::Subscription<autoware_auto_msgs::msg::HADMapRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;

  void onAutowareEngage(const autoware_auto_msgs::msg::Engage::ConstSharedPtr msg);
  void onVehicleStateReport(const autoware_auto_msgs::msg::VehicleStateReport::ConstSharedPtr msg);
  void onIsEmergency(const autoware_auto_msgs::msg::EmergencyMode::ConstSharedPtr msg);
  void onRoute(const autoware_auto_msgs::msg::HADMapRoute::ConstSharedPtr msg);
  void onOdometry(const autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr msg);

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_shutdown_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_route_;

  bool onShutdownService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool onResetRouteService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Publisher
  rclcpp::Publisher<autoware_auto_msgs::msg::AutowareState>::SharedPtr pub_autoware_state_;

  bool isEngaged();

  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr timer_;

  // Stats
  ParamStats getParamStats() const;
  TfStats getTfStats() const;

  // State Machine
  std::shared_ptr<StateMachine> state_machine_;
  StateInput state_input_;
  StateParam state_param_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void setupDiagnosticUpdater();
  void checkTopicStatus(
    diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & module_name);
  void checkTFStatus(
    diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & module_name);
};

#endif  // AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_
