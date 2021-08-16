// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef EMERGENCY_HANDLER__EMERGENCY_HANDLER_NODE_HPP_
#define EMERGENCY_HANDLER__EMERGENCY_HANDLER_NODE_HPP_

// Core
#include <memory>
#include <string>

// ROS
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

// Autoware
#include "autoware_auto_msgs/msg/autoware_state.hpp"
#include "autoware_auto_msgs/msg/driving_capability.hpp"
#include "autoware_auto_msgs/msg/emergency_mode.hpp"
#include "autoware_auto_msgs/msg/hazard_status_stamped.hpp"
#include "autoware_auto_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_msgs/msg/vehicle_control_command.hpp"
#include "autoware_auto_msgs/msg/vehicle_state_report.hpp"
#include "autoware_auto_msgs/msg/vehicle_state_command.hpp"

// Local
#include "emergency_handler/heartbeat_checker.hpp"

namespace autoware
{
namespace emergency_handler
{

/// \brief A node for handling the emergency state of the system
///
/// The node deals with emergency state detection based on the provided data.
/// When the emergency state is detected, then the main purpose of this component is
/// to generate vehicle velocities commands and state commands to stop the vehicle safelty.
class EMERGENCY_HANDLER_PUBLIC EmergencyHandlerNode : public rclcpp::Node
{
public:
  /// \brief Constructor
  /// \param[in] node_options Node options
  explicit EmergencyHandlerNode(const rclcpp::NodeOptions & node_options);

private:
  // Callbacks
  void onAutowareState(const autoware_auto_msgs::msg::AutowareState::ConstSharedPtr msg);
  void onDrivingCapability(const autoware_auto_msgs::msg::DrivingCapability::ConstSharedPtr msg);
  void onPrevControlCommand(
    const autoware_auto_msgs::msg::VehicleControlCommand::ConstSharedPtr msg);
  void onStateReport(const autoware_auto_msgs::msg::VehicleStateReport::ConstSharedPtr msg);
  void onOdometry(const autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr msg);

  void publishHazardStatus(const autoware_auto_msgs::msg::HazardStatus & hazard_status);
  void publishControlAndStateCommands();

  bool onClearEmergencyService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool isDataReady();
  void onTimer();

  bool isStopped();
  bool isEmergency(const autoware_auto_msgs::msg::HazardStatus & hazard_status);
  autoware_auto_msgs::msg::HazardStatus judgeHazardStatus();

  diagnostic_msgs::msg::DiagnosticStatus createDiagnosticStatus(
    const int level, const std::string & name, const std::string & message = "");

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_emergency_;

  // Publishers
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleControlCommand>::SharedPtr
    pub_control_command_;
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleStateCommand>::SharedPtr pub_state_command_;
  rclcpp::Publisher<autoware_auto_msgs::msg::EmergencyMode>::SharedPtr pub_is_emergency_;
  rclcpp::Publisher<autoware_auto_msgs::msg::HazardStatusStamped>::SharedPtr pub_hazard_status_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_err_;

  // Subscribers
  rclcpp::Subscription<autoware_auto_msgs::msg::AutowareState>::SharedPtr sub_autoware_state_;
  rclcpp::Subscription<autoware_auto_msgs::msg::DrivingCapability>::SharedPtr
    sub_driving_capability_;
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleControlCommand>::SharedPtr
    sub_prev_control_command_;
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleStateReport>::SharedPtr sub_state_report_;
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;

  autoware_auto_msgs::msg::AutowareState::ConstSharedPtr autoware_state_;
  autoware_auto_msgs::msg::DrivingCapability::ConstSharedPtr driving_capability_;
  autoware_auto_msgs::msg::VehicleControlCommand::ConstSharedPtr prev_control_command_;
  autoware_auto_msgs::msg::VehicleStateReport::ConstSharedPtr state_report_;
  autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr odometry_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double update_rate_;
  double data_ready_timeout_;
  double timeout_driving_capability_;
  int64_t emergency_hazard_level_;
  bool use_emergency_hold_;
  double emergency_stop_acceleration_mps2_;
  bool use_parking_after_stopped_;
  double stopped_velocity_threshold_;

  // Heartbeat/watchdog
  rclcpp::Time initialized_time_;
  std::shared_ptr<HeartbeatChecker<autoware_auto_msgs::msg::DrivingCapability>>
  heartbeat_driving_capability_;

  // Algorithm
  bool is_emergency_ = false;
  autoware_auto_msgs::msg::HazardStatus hazard_status_;
};

}  // namespace emergency_handler
}  // namespace autoware

#endif  // EMERGENCY_HANDLER__EMERGENCY_HANDLER_NODE_HPP_
