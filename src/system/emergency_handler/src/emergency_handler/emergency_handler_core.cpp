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

#include <memory>
#include <string>
#include <utility>

#include "emergency_handler/emergency_handler_core.hpp"

namespace
{
diagnostic_msgs::msg::DiagnosticStatus createDiagnosticStatus(
  const int level, const std::string & name, const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticStatus diag;

  diag.level = level;
  diag.name = name;
  diag.message = message;
  diag.hardware_id = "emergency_handler";

  return diag;
}

diagnostic_msgs::msg::DiagnosticArray convertHazardStatusToDiagnosticArray(
  rclcpp::Clock::SharedPtr clock, const autoware_auto_msgs::msg::HazardStatus & hazard_status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = clock->now();

  const auto decorateDiag = [](const auto & hazard_diag, const std::string & label) {
      auto diag = hazard_diag;

      diag.message = label + diag.message;

      return diag;
    };

  for (const auto & hazard_diag : hazard_status.diag_no_fault) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[No Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diag_safe_fault) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Safe Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diag_latent_fault) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Latent Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diag_single_point_fault) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Single Point Fault]"));
  }

  return diag_array;
}
}  // namespace

EmergencyHandler::EmergencyHandler()
: Node("emergency_handler"),
  update_rate_(declare_parameter<int>("update_rate", 10)),
  data_ready_timeout_(declare_parameter<double>("data_ready_timeout", 30.0)),
  timeout_driving_capability_(declare_parameter<double>("timeout_driving_capability", 0.5)),
  emergency_hazard_level_(declare_parameter<int>("emergency_hazard_level", 2)),
  use_emergency_hold_(declare_parameter<bool>("use_emergency_hold", false)),
  use_parking_after_stopped_(declare_parameter<bool>("use_parking_after_stopped", false))
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  // Subscriber
  sub_autoware_state_ = create_subscription<autoware_auto_msgs::msg::AutowareState>(
    "input/autoware_state", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onAutowareState, this, _1));
  sub_driving_capability_ = create_subscription<autoware_auto_msgs::msg::DrivingCapability>(
    "input/driving_capability", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onDrivingCapability, this, _1));
  sub_prev_control_command_ = create_subscription<autoware_auto_msgs::msg::VehicleControlCommand>(
    "input/prev_control_command", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onPrevControlCommand, this, _1));
  sub_state_report_ = create_subscription<autoware_auto_msgs::msg::VehicleStateReport>(
    "input/state_report", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onStateReport, this, _1));
  sub_odometry_ = create_subscription<autoware_auto_msgs::msg::VehicleOdometry>(
    "input/odometry", rclcpp::QoS{1}, std::bind(&EmergencyHandler::onOdometry, this, _1));

  // Heartbeat
  heartbeat_driving_capability_ =
    std::make_shared<HeaderlessHeartbeatChecker<autoware_auto_msgs::msg::DrivingCapability>>(
    *this, "input/driving_capability", timeout_driving_capability_);

  // Service
  srv_clear_emergency_ = this->create_service<std_srvs::srv::Trigger>(
    "service/clear_emergency",
    std::bind(&EmergencyHandler::onClearEmergencyService, this, _1, _2, _3));

  // Publisher
  pub_control_command_ = create_publisher<autoware_auto_msgs::msg::VehicleControlCommand>(
    "output/control_command", rclcpp::QoS{1});
  pub_state_command_ = create_publisher<autoware_auto_msgs::msg::VehicleStateCommand>(
    "output/state_command", rclcpp::QoS{1});
  pub_is_emergency_ = create_publisher<autoware_auto_msgs::msg::EmergencyMode>(
    "output/is_emergency", rclcpp::QoS{1});
  pub_hazard_status_ = create_publisher<autoware_auto_msgs::msg::HazardStatusStamped>(
    "output/hazard_status", rclcpp::QoS{1});
  pub_diagnostics_err_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "output/diagnostics_err", rclcpp::QoS{1});

  // Initialize
  odometry_ = autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr(
    new autoware_auto_msgs::msg::VehicleOdometry);
  prev_control_command_ = autoware_auto_msgs::msg::VehicleControlCommand::ConstSharedPtr(
    new autoware_auto_msgs::msg::VehicleControlCommand);

  // Timer
  initialized_time_ = this->now();
  auto timer_callback = std::bind(&EmergencyHandler::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void EmergencyHandler::onAutowareState(
  const autoware_auto_msgs::msg::AutowareState::ConstSharedPtr msg)
{
  autoware_state_ = msg;
}

void EmergencyHandler::onDrivingCapability(
  const autoware_auto_msgs::msg::DrivingCapability::ConstSharedPtr msg)
{
  driving_capability_ = msg;
}

void EmergencyHandler::onPrevControlCommand(
  const autoware_auto_msgs::msg::VehicleControlCommand::ConstSharedPtr msg)
{
  prev_control_command_ = msg;
}

void EmergencyHandler::onStateReport(
  const autoware_auto_msgs::msg::VehicleStateReport::ConstSharedPtr msg)
{
  state_report_ = msg;
}

void EmergencyHandler::onOdometry(
  const autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr msg)
{
  odometry_ = msg;
}

bool EmergencyHandler::onClearEmergencyService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request_header;
  const auto hazard_status = judgeHazardStatus();
  if (!isEmergency(hazard_status)) {
    is_emergency_ = false;
    hazard_status_ = hazard_status;

    response->success = true;
    response->message = "Emergency state was cleared.";
  } else {
    response->success = false;
    response->message = "There are still errors, can't clear emergency state.";
  }

  return true;
}

void EmergencyHandler::publishHazardStatus(
  const autoware_auto_msgs::msg::HazardStatus & hazard_status)
{
  // Create msg of is_emergency
  autoware_auto_msgs::msg::EmergencyMode emergency_mode;
  emergency_mode.is_emergency = isEmergency(hazard_status);

  // Create msg of hazard_status
  autoware_auto_msgs::msg::HazardStatusStamped hazard_status_stamped;
  hazard_status_stamped.stamp = this->now();
  hazard_status_stamped.status = hazard_status;

  // Publish data
  pub_is_emergency_->publish(emergency_mode);
  pub_hazard_status_->publish(hazard_status_stamped);
  pub_diagnostics_err_->publish(
    convertHazardStatusToDiagnosticArray(this->get_clock(), hazard_status_stamped.status));
}

void EmergencyHandler::publishControlCommands()
{
  // Create timestamp
  const auto stamp = this->now();

  // Publish ControlCommand
  {
    auto msg = selectAlternativeControlCommand();
    msg.stamp = stamp;
    pub_control_command_->publish(msg);
  }

  // Publish StateCommand
  {
    autoware_auto_msgs::msg::VehicleStateCommand msg;
    msg.stamp = stamp;
    msg.blinker = autoware_auto_msgs::msg::VehicleStateCommand::BLINKER_HAZARD;
    msg.gear = autoware_auto_msgs::msg::VehicleStateCommand::GEAR_PARK;
    pub_state_command_->publish(msg);
  }
}

bool EmergencyHandler::isDataReady()
{
  if (!autoware_state_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for autoware_state msg...");
    return false;
  }

  if (!driving_capability_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for driving_capability msg...");
    return false;
  }

  if (!state_report_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for state_report msg...");
    return false;
  }

  return true;
}

void EmergencyHandler::onTimer()
{
  // Wait for data ready
  if (!isDataReady()) {
    if ((this->now() - initialized_time_).seconds() > data_ready_timeout_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "input data is timeout");

      autoware_auto_msgs::msg::HazardStatus hazard_status;
      hazard_status.level = autoware_auto_msgs::msg::HazardStatus::SINGLE_POINT_FAULT;

      diagnostic_msgs::msg::DiagnosticStatus diag;
      diag.name = "emergency_handler/input_data_timeout";
      diag.hardware_id = "emergency_handler";
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      hazard_status.diag_single_point_fault.push_back(diag);

      publishHazardStatus(hazard_status);
    }

    return;
  }

  // Check if emergency
  if (use_emergency_hold_) {
    if (!is_emergency_) {
      // Update only when it is not emergency
      hazard_status_ = judgeHazardStatus();
      is_emergency_ = isEmergency(hazard_status_);
    }
  } else {
    // Update always
    hazard_status_ = judgeHazardStatus();
    is_emergency_ = isEmergency(hazard_status_);
  }

  // Publish data
  publishHazardStatus(hazard_status_);
  publishControlCommands();
}

bool EmergencyHandler::isStopped()
{
  constexpr auto th_stopped_velocity = 0.001;
  if (odometry_->velocity_mps < th_stopped_velocity) {
    return true;
  }

  return false;
}

bool EmergencyHandler::isEmergency(const autoware_auto_msgs::msg::HazardStatus & hazard_status)
{
  return hazard_status.level >= emergency_hazard_level_;
}

autoware_auto_msgs::msg::HazardStatus EmergencyHandler::judgeHazardStatus()
{
  using autoware_auto_msgs::msg::VehicleStateReport;
  const auto vehicle_mode = state_report_->mode;

  // Get hazard status
  auto hazard_status = vehicle_mode == VehicleStateReport::MODE_AUTONOMOUS ?
    driving_capability_->autonomous_driving :
    driving_capability_->remote_control;

  // Ignore initializing and finalizing state
  {
    using autoware_auto_msgs::msg::AutowareState;
    using autoware_auto_msgs::msg::HazardStatus;

    const auto is_in_auto_ignore_state =
      (autoware_state_->state == AutowareState::INITIALIZING_VEHICLE) ||
      (autoware_state_->state == AutowareState::WAITING_FOR_ROUTE) ||
      (autoware_state_->state == AutowareState::PLANNING) ||
      (autoware_state_->state == AutowareState::FINALIZING);

    if (vehicle_mode == VehicleStateReport::MODE_AUTONOMOUS && is_in_auto_ignore_state) {
      hazard_status.level = HazardStatus::NO_FAULT;
    }

    const auto is_in_remote_ignore_state =
      (autoware_state_->state == AutowareState::INITIALIZING_VEHICLE) ||
      (autoware_state_->state == AutowareState::FINALIZING);

    if (vehicle_mode == VehicleStateReport::MODE_MANUAL && is_in_remote_ignore_state) {
      hazard_status.level = HazardStatus::NO_FAULT;
    }
  }

  // Check timeout
  {
    using autoware_auto_msgs::msg::AutowareState;
    using autoware_auto_msgs::msg::HazardStatus;
    using diagnostic_msgs::msg::DiagnosticStatus;

    const auto is_in_heartbeat_timeout_ignore_state =
      (autoware_state_->state == AutowareState::INITIALIZING_VEHICLE);

    if (!is_in_heartbeat_timeout_ignore_state && heartbeat_driving_capability_->isTimeout()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "heartbeat_driving_capability is timeout");
      hazard_status.level = HazardStatus::SINGLE_POINT_FAULT;
      hazard_status.diag_single_point_fault.push_back(
        createDiagnosticStatus(
          DiagnosticStatus::ERROR, "emergency_handler/heartbeat_timeout",
          "heartbeat_driving_capability is timeout"));
    }
  }

  return hazard_status;
}

autoware_auto_msgs::msg::VehicleControlCommand EmergencyHandler::selectAlternativeControlCommand()
{
  // TODO(jilaada): Add safe_stop planner

  // Emergency Stop
  {
    autoware_auto_msgs::msg::VehicleControlCommand emergency_stop_cmd;
    emergency_stop_cmd.front_wheel_angle_rad = prev_control_command_->front_wheel_angle_rad;
    emergency_stop_cmd.velocity_mps = 0.0;
    emergency_stop_cmd.long_accel_mps2 = -2.5;

    return emergency_stop_cmd;
  }
}
