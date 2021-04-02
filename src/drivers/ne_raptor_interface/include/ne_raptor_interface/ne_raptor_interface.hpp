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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file ne_raptor_interface.hpp
/// \brief This file defines the ne_raptor_interface class.

#ifndef NE_RAPTOR_INTERFACE__NE_RAPTOR_INTERFACE_HPP_
#define NE_RAPTOR_INTERFACE__NE_RAPTOR_INTERFACE_HPP_

#include <ne_raptor_interface/visibility_control.hpp>

#include <common/types.hpp>
#include <vehicle_interface/dbw_state_machine.hpp>
#include <vehicle_interface/platform_interface.hpp>

#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/gear_cmd.hpp>
#include <raptor_dbw_msgs/msg/global_enable_cmd.hpp>
#include <raptor_dbw_msgs/msg/misc_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>

#include <raptor_dbw_msgs/msg/brake_report.hpp>
#include <raptor_dbw_msgs/msg/gear_report.hpp>
#include <raptor_dbw_msgs/msg/misc_report.hpp>
#include <raptor_dbw_msgs/msg/other_actuators_report.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>

#include <raptor_dbw_msgs/msg/actuator_control_mode.hpp>
#include <raptor_dbw_msgs/msg/door_request.hpp>
#include <raptor_dbw_msgs/msg/door_state.hpp>
#include <raptor_dbw_msgs/msg/gear.hpp>
#include <raptor_dbw_msgs/msg/high_beam.hpp>
#include <raptor_dbw_msgs/msg/high_beam_state.hpp>
#include <raptor_dbw_msgs/msg/horn_state.hpp>
#include <raptor_dbw_msgs/msg/ignition.hpp>
#include <raptor_dbw_msgs/msg/low_beam.hpp>
#include <raptor_dbw_msgs/msg/parking_brake.hpp>
#include <raptor_dbw_msgs/msg/turn_signal.hpp>
#include <raptor_dbw_msgs/msg/wiper_front.hpp>
#include <raptor_dbw_msgs/msg/wiper_rear.hpp>

#include <autoware_auto_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>

#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

#include <autoware_auto_msgs/srv/autonomy_mode_change.hpp>

#include <std_msgs/msg/bool.hpp>
#include <motion_common/motion_common.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::TAU;
using autoware::common::types::PI;

using raptor_dbw_msgs::msg::AcceleratorPedalCmd;
using raptor_dbw_msgs::msg::BrakeCmd;
using raptor_dbw_msgs::msg::GearCmd;
using raptor_dbw_msgs::msg::GlobalEnableCmd;
using raptor_dbw_msgs::msg::MiscCmd;
using raptor_dbw_msgs::msg::SteeringCmd;

using raptor_dbw_msgs::msg::BrakeReport;
using raptor_dbw_msgs::msg::GearReport;
using raptor_dbw_msgs::msg::MiscReport;
using raptor_dbw_msgs::msg::OtherActuatorsReport;
using raptor_dbw_msgs::msg::SteeringReport;
using raptor_dbw_msgs::msg::WheelSpeedReport;

using raptor_dbw_msgs::msg::ActuatorControlMode;
using raptor_dbw_msgs::msg::DoorRequest;
using raptor_dbw_msgs::msg::DoorState;
using raptor_dbw_msgs::msg::Gear;
using raptor_dbw_msgs::msg::HighBeam;
using raptor_dbw_msgs::msg::HighBeamState;
using raptor_dbw_msgs::msg::HornState;
using raptor_dbw_msgs::msg::Ignition;
using raptor_dbw_msgs::msg::LowBeam;
using raptor_dbw_msgs::msg::ParkingBrake;
using raptor_dbw_msgs::msg::TurnSignal;
using raptor_dbw_msgs::msg::WiperFront;
using raptor_dbw_msgs::msg::WiperRear;

using autoware_auto_msgs::msg::HighLevelControlCommand;
using autoware_auto_msgs::msg::RawControlCommand;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;

using autoware_auto_msgs::msg::VehicleStateReport;
using autoware_auto_msgs::msg::VehicleOdometry;
using autoware_auto_msgs::msg::VehicleKinematicState;

using autoware_auto_msgs::srv::AutonomyModeChange;
using ModeChangeRequest = autoware_auto_msgs::srv::AutonomyModeChange_Request;

using autoware::drivers::vehicle_interface::DbwStateMachine;
using autoware::drivers::vehicle_interface::DbwState;

namespace autoware
{
namespace ne_raptor_interface
{
static constexpr float32_t KPH_TO_MPS_RATIO = 1000.0F / (60.0F * 60.0F);
static constexpr float32_t DEGREES_TO_RADIANS = PI / 360.0F;
static constexpr float32_t STEERING_PCT_TO_DEGREE_RATIO = 500.0F;  // steering range +/- 500 degrees

/// \brief Class for interfacing with NE Raptor DBW
class NE_RAPTOR_INTERFACE_PUBLIC NERaptorInterface
  : public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
  /// \brief Default constructor.
  /// \param[in] node Reference to node
  /// \param[in] ecu_build_num ECU build #
  /// \param[in] front_axle_to_cog Distance from front axle to center-of-gravity in meters
  /// \param[in] rear_axle_to_cog Distance from rear axle to center-of-gravity in meters
  /// \param[in] steer_to_tire_ratio Ratio between steering wheel angle & tire angle
  /// \param[in] acceleration_limit m/s^2, zero = no limit
  /// \param[in] deceleration_limit m/s^2, zero = no limit
  /// \param[in] acceleration_positive_jerk_limit m/s^3
  /// \param[in] deceleration_negative_jerk_limit m/s^3
  explicit NERaptorInterface(
    rclcpp::Node & node,
    uint16_t ecu_build_num,
    float32_t front_axle_to_cog,
    float32_t rear_axle_to_cog,
    float32_t steer_to_tire_ratio,
    float32_t acceleration_limit,
    float32_t deceleration_limit,
    float32_t acceleration_positive_jerk_limit,
    float32_t deceleration_negative_jerk_limit
  );
/// \brief Default destructor
  ~NERaptorInterface() noexcept override = default;

  /// \brief Try to receive data from the vehicle platform, and update StateReport and Odometry.
  ///   Exceptions may be thrown on errors
  /// \param[in] timeout The maximum amount of time to check/receive data
  /// \return True if data was received before the timeout, false otherwise
  bool8_t update(std::chrono::nanoseconds timeout) override;

  /// \brief Send the state command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The state command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_state_command(const VehicleStateCommand & msg) override;

  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const HighLevelControlCommand & msg);

  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const RawControlCommand & msg) override;

  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const VehicleControlCommand & msg) override;

  /// \brief Handle a request from the user to enable or disable the DBW system.
  ///   Exceptions may be thrown on errors
  /// \param[in] request The requested autonomy mode
  /// \return false only if enabling the DBW system actually failed, true otherwise
  bool8_t handle_mode_change_request(ModeChangeRequest::SharedPtr request) override;

private:
  // Publishers (to Raptor DBW)
  rclcpp::Publisher<AcceleratorPedalCmd>::SharedPtr m_accel_cmd_pub;
  rclcpp::Publisher<BrakeCmd>::SharedPtr m_brake_cmd_pub;
  rclcpp::Publisher<GearCmd>::SharedPtr m_gear_cmd_pub;
  rclcpp::Publisher<GlobalEnableCmd>::SharedPtr m_enable_cmd_pub;
  rclcpp::Publisher<MiscCmd>::SharedPtr m_misc_cmd_pub;
  rclcpp::Publisher<SteeringCmd>::SharedPtr m_steer_cmd_pub;

  // Publishers (to Autoware)
  rclcpp::Publisher<VehicleStateReport>::SharedPtr m_vehicle_state_pub;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr m_vehicle_odo_pub;
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr m_vehicle_kin_state_pub;

  // Subscribers (from Raptor DBW)
  rclcpp::SubscriptionBase::SharedPtr m_dbw_state_sub,
    m_brake_rpt_sub, m_gear_rpt_sub, m_misc_rpt_sub,
    m_other_acts_rpt_sub, m_steering_rpt_sub, m_wheel_spd_rpt_sub;

  rclcpp::Logger m_logger;
  uint16_t m_ecu_build_num;
  float32_t m_front_axle_to_cog;
  float32_t m_rear_axle_to_cog;
  float32_t m_steer_to_tire_ratio;
  float32_t m_acceleration_limit;
  float32_t m_deceleration_limit;
  float32_t m_acceleration_positive_jerk_limit;
  float32_t m_deceleration_negative_jerk_limit;
  std::unique_ptr<DbwStateMachine> m_dbw_state_machine;
  uint8_t m_rolling_counter_vsc;
  uint8_t m_rolling_counter_hlcc;
  uint8_t m_rolling_counter_vcc;

  /* Vehicle Odometry, Vehicle State, &
   * Vehicle Kinematic State are stored
   * because they need data from multiple reports.
   */
  VehicleOdometry m_vehicle_odometry;
  VehicleStateReport m_vehicle_state_report;
  VehicleKinematicState m_vehicle_kin_state;
  bool8_t m_seen_dbw_rpt{false};
  bool8_t m_seen_brake_rpt{false};
  bool8_t m_seen_gear_rpt{false};
  bool8_t m_seen_misc_rpt{false};
  bool8_t m_seen_steering_rpt{false};
  bool8_t m_seen_wheel_spd_rpt{false};
  float32_t m_travel_direction{0.0F};

  // In case multiple signals arrive at the same time
  std::mutex m_vehicle_odometry_mutex;
  std::mutex m_vehicle_state_report_mutex;
  std::mutex m_vehicle_kin_state_mutex;

  void on_dbw_state_report(const std_msgs::msg::Bool::SharedPtr & msg);
  void on_brake_report(const BrakeReport::SharedPtr & msg);
  void on_gear_report(const GearReport::SharedPtr & msg);
  void on_misc_report(const MiscReport::SharedPtr & msg);
  void on_other_act_report(const OtherActuatorsReport::SharedPtr & msg);
  void on_steering_report(const SteeringReport::SharedPtr & msg);
  void on_wheel_spd_report(const WheelSpeedReport::SharedPtr & msg);
};  // class NERaptorInterface
}  // namespace ne_raptor_interface
}  // namespace autoware
#endif  // NE_RAPTOR_INTERFACE__NE_RAPTOR_INTERFACE_HPP_
