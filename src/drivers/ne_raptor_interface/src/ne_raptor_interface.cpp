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

#include "ne_raptor_interface/ne_raptor_interface.hpp"

#include <rclcpp/logging.hpp>
#include <time_utils/time_utils.hpp>

#include <cmath>
#include <stdexcept>
#include <iostream>

namespace autoware
{
namespace ne_raptor_interface
{

NERaptorInterface::NERaptorInterface(
  rclcpp::Node & node,
  uint16_t ecu_build_num,
  float32_t front_axle_to_cog,
  float32_t rear_axle_to_cog,
  float32_t steer_to_tire_ratio,
  float32_t max_steer_angle,
  float32_t acceleration_limit,
  float32_t deceleration_limit,
  float32_t acceleration_positive_jerk_limit,
  float32_t deceleration_negative_jerk_limit
)
: m_logger{node.get_logger()},
  m_ecu_build_num{ecu_build_num},
  m_front_axle_to_cog{front_axle_to_cog},
  m_rear_axle_to_cog{rear_axle_to_cog},
  m_steer_to_tire_ratio{steer_to_tire_ratio},
  m_max_steer_angle{max_steer_angle},
  m_acceleration_limit{acceleration_limit},
  m_deceleration_limit{deceleration_limit},
  m_acceleration_positive_jerk_limit{acceleration_positive_jerk_limit},
  m_deceleration_negative_jerk_limit{deceleration_negative_jerk_limit},
  m_dbw_state_machine(new DbwStateMachine{3}),
  m_rolling_counter_vsc{0},
  m_rolling_counter_hlcc{0},
  m_rolling_counter_vcc{0},
  m_clock{RCL_SYSTEM_TIME}
{
  // Publishers (to Raptor DBW)
  m_accel_cmd_pub = node.create_publisher<AcceleratorPedalCmd>("accelerator_pedal_cmd", 1);
  m_brake_cmd_pub = node.create_publisher<BrakeCmd>("brake_cmd", 1);
  m_gear_cmd_pub = node.create_publisher<GearCmd>("gear_cmd", 1);
  m_global_enable_cmd_pub = node.create_publisher<GlobalEnableCmd>("global_enable_cmd", 1);
  m_misc_cmd_pub = node.create_publisher<MiscCmd>("misc_cmd", 1);
  m_steer_cmd_pub = node.create_publisher<SteeringCmd>("steering_cmd", 1);
  m_dbw_enable_cmd_pub = node.create_publisher<std_msgs::msg::Empty>("enable", 10);
  m_dbw_disable_cmd_pub = node.create_publisher<std_msgs::msg::Empty>("disable", 10);

  // Publishers (to Autoware)
  m_vehicle_state_pub = node.create_publisher<VehicleStateReport>("vehicle_state_report", 10);
  m_vehicle_odo_pub = node.create_publisher<VehicleOdometry>("vehicle_odometry", 10);
  m_vehicle_kin_state_pub = node.create_publisher<VehicleKinematicState>(
    "vehicle_kinematic_state",
    10);

  // Subscribers (from Raptor DBW)
  m_dbw_state_sub =
    node.create_subscription<std_msgs::msg::Bool>(
    "dbw_enabled", rclcpp::QoS{1},
    [this](std_msgs::msg::Bool::SharedPtr msg) {on_dbw_state_report(msg);});
  m_brake_rpt_sub =
    node.create_subscription<BrakeReport>(
    "brake_report", rclcpp::QoS{20},
    [this](BrakeReport::SharedPtr msg) {on_brake_report(msg);});
  m_gear_rpt_sub =
    node.create_subscription<GearReport>(
    "gear_report", rclcpp::QoS{20},
    [this](GearReport::SharedPtr msg) {on_gear_report(msg);});
  m_misc_rpt_sub =
    node.create_subscription<MiscReport>(
    "misc_report", rclcpp::QoS{2},
    [this](MiscReport::SharedPtr msg) {on_misc_report(msg);});
  m_other_acts_rpt_sub =
    node.create_subscription<OtherActuatorsReport>(
    "other_actuators_report", rclcpp::QoS{20},
    [this](OtherActuatorsReport::SharedPtr msg) {on_other_actuators_report(msg);});
  m_steering_rpt_sub =
    node.create_subscription<SteeringReport>(
    "steering_report", rclcpp::QoS{20},
    [this](SteeringReport::SharedPtr msg) {on_steering_report(msg);});
  m_wheel_spd_rpt_sub =
    node.create_subscription<WheelSpeedReport>(
    "wheel_speed_report", rclcpp::QoS{20},
    [this](WheelSpeedReport::SharedPtr msg) {on_wheel_spd_report(msg);});
}

bool8_t NERaptorInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  return true;
}

bool8_t NERaptorInterface::send_state_command(const VehicleStateCommand & msg)
{
  bool8_t is_dbw_enabled = m_dbw_state_machine->enabled() ? true : false;
  bool8_t ret{true};
  std_msgs::msg::Empty send_req{};
  ModeChangeRequest::SharedPtr t_mode{new ModeChangeRequest};
  // keep previous for Enable commands, if valid Enable command not received
  static GearCmd gc{};
  static GlobalEnableCmd gec{};
  static MiscCmd mc{};

  // Set rolling counters
  if (m_rolling_counter_vsc < 0xFF) {
    m_rolling_counter_vsc++;
  } else {
    m_rolling_counter_vsc = 0;
  }
  gc.rolling_counter = m_rolling_counter_vsc;
  gec.rolling_counter = m_rolling_counter_vsc;
  mc.rolling_counter = m_rolling_counter_vsc;

  // Set gear values
  switch (msg.gear) {
    case VehicleStateCommand::GEAR_NO_COMMAND:
      gc.cmd.gear = Gear::NONE;
      break;
    case VehicleStateCommand::GEAR_DRIVE:
      gc.cmd.gear = Gear::DRIVE;
      break;
    case VehicleStateCommand::GEAR_REVERSE:
      gc.cmd.gear = Gear::REVERSE;
      break;
    case VehicleStateCommand::GEAR_PARK:
      gc.cmd.gear = Gear::PARK;
      break;
    case VehicleStateCommand::GEAR_LOW:
      gc.cmd.gear = Gear::LOW;
      break;
    case VehicleStateCommand::GEAR_NEUTRAL:
      gc.cmd.gear = Gear::NEUTRAL;
      break;
    default:  // error
      gc.cmd.gear = Gear::NONE;
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received command for invalid gear state.");
      ret = false;
      break;
  }

  // Set global enable command values
  gec.ecu_build_number = m_ecu_build_num;

  // Set misc command values
  mc.door_request_right_rear.value = DoorRequest::NO_REQUEST;
  mc.door_request_left_rear.value = DoorRequest::NO_REQUEST;
  mc.door_request_lift_gate.value = DoorRequest::NO_REQUEST;
  mc.rear_wiper_cmd.status = WiperRear::OFF;
  mc.ignition_cmd.status = Ignition::NO_REQUEST;
  mc.low_beam_cmd.status = LowBeam::OFF;

  mc.horn_cmd = msg.horn;

  switch (msg.blinker) {
    case VehicleStateCommand::BLINKER_NO_COMMAND:
      // Keep previous
      break;
    case VehicleStateCommand::BLINKER_OFF:
      mc.cmd.value = TurnSignal::NONE;
      break;
    case VehicleStateCommand::BLINKER_LEFT:
      mc.cmd.value = TurnSignal::LEFT;
      break;
    case VehicleStateCommand::BLINKER_RIGHT:
      mc.cmd.value = TurnSignal::RIGHT;
      break;
    case VehicleStateCommand::BLINKER_HAZARD:
      mc.cmd.value = TurnSignal::HAZARDS;
      break;
    default:
      mc.cmd.value = TurnSignal::SNA;
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received command for invalid turn signal state.");
      ret = false;
      break;
  }

  switch (msg.headlight) {
    case VehicleStateCommand::HEADLIGHT_NO_COMMAND:
      // Keep previous
      break;
    case VehicleStateCommand::HEADLIGHT_OFF:
    case VehicleStateCommand::HEADLIGHT_ON:
      mc.high_beam_cmd.status = HighBeam::OFF;
      break;
    case VehicleStateCommand::HEADLIGHT_HIGH:
      mc.high_beam_cmd.status = HighBeam::ON;
      break;
    default:
      // Keep previous
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received command for invalid headlight state.");
      ret = false;
      break;
  }

  switch (msg.wiper) {
    case VehicleStateCommand::WIPER_NO_COMMAND:
      // Keep previous
      break;
    case VehicleStateCommand::WIPER_OFF:
      mc.front_wiper_cmd.status = WiperFront::OFF;
      break;
    case VehicleStateCommand::WIPER_LOW:
      mc.front_wiper_cmd.status = WiperFront::CONSTANT_LOW;
      break;
    case VehicleStateCommand::WIPER_HIGH:
      mc.front_wiper_cmd.status = WiperFront::CONSTANT_HIGH;
      break;
    case VehicleStateCommand::WIPER_CLEAN:
      mc.front_wiper_cmd.status = WiperFront::WASH_BRIEF;
      break;
    default:
      mc.front_wiper_cmd.status = WiperFront::SNA;
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received command for invalid wiper state.");
      ret = false;
      break;
  }

  // Request DBW mode change
  switch (msg.mode) {
    case VehicleStateCommand::MODE_NO_COMMAND:
      // No change requested: keep previous
      break;
    case VehicleStateCommand::MODE_AUTONOMOUS:
      if (!is_dbw_enabled) {
        t_mode->mode = ModeChangeRequest::MODE_AUTONOMOUS;
        handle_mode_change_request(t_mode);
      }
      gc.enable = true;
      gec.global_enable = true;
      gec.enable_joystick_limits = true;
      mc.block_standard_cruise_buttons = true;
      mc.block_adaptive_cruise_buttons = true;
      mc.block_turn_signal_stalk = true;
      break;
    case VehicleStateCommand::MODE_MANUAL:
      if (is_dbw_enabled) {
        t_mode->mode = ModeChangeRequest::MODE_MANUAL;
        handle_mode_change_request(t_mode);
      }
      gc.enable = false;
      gec.global_enable = false;
      gec.enable_joystick_limits = false;
      mc.block_standard_cruise_buttons = false;
      mc.block_adaptive_cruise_buttons = false;
      mc.block_turn_signal_stalk = false;
      break;
    default:
      RCLCPP_ERROR_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Got invalid autonomy mode request value.");
      ret = false;
      break;
  }

  std::lock_guard<std::mutex> guard_bc(m_brake_cmd_mutex);
  m_brake_cmd.park_brake_cmd.status =
    (msg.hand_brake) ? ParkingBrake::ON : ParkingBrake::OFF;

  m_gear_cmd_pub->publish(gc);
  m_global_enable_cmd_pub->publish(gec);
  m_misc_cmd_pub->publish(mc);

  m_seen_vehicle_state_cmd = true;

  return ret;
}

/* Apparently HighLevelControlCommand will be obsolete soon.
 */
bool8_t NERaptorInterface::send_control_command(const HighLevelControlCommand & msg)
{
  bool8_t is_dbw_enabled = m_dbw_state_machine->enabled() ? true : false;
  bool8_t ret{true};
  float32_t velocity_checked{0.0F};
  AcceleratorPedalCmd apc{};
  SteeringCmd sc{};

  std::lock_guard<std::mutex> guard_bc(m_brake_cmd_mutex);

  // Using curvature for control
  apc.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;
  sc.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;
  m_brake_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_VEHICLE;

  // Set enable variables
  apc.enable = is_dbw_enabled;
  m_brake_cmd.enable = is_dbw_enabled;
  sc.enable = is_dbw_enabled;
  apc.ignore = false;
  sc.ignore = false;

  // Set rolling counters
  if (m_rolling_counter_hlcc < 0xFF) {
    m_rolling_counter_hlcc++;
  } else {
    m_rolling_counter_hlcc = 0;
  }
  apc.rolling_counter = m_rolling_counter_hlcc;
  m_brake_cmd.rolling_counter = m_rolling_counter_hlcc;
  sc.rolling_counter = m_rolling_counter_hlcc;

  // Set limits
  apc.accel_limit = m_acceleration_limit;
  m_brake_cmd.decel_limit = m_deceleration_limit;
  apc.accel_positive_jerk_limit = m_acceleration_positive_jerk_limit;
  m_brake_cmd.decel_negative_jerk_limit = m_deceleration_negative_jerk_limit;

  // Check for invalid changes in direction
  if ( ( (m_vehicle_state_report.gear == VehicleStateReport::GEAR_DRIVE) &&
    (msg.velocity_mps < 0.0F) ) ||
    ( (m_vehicle_state_report.gear == VehicleStateReport::GEAR_REVERSE) &&
    (msg.velocity_mps > 0.0F) ) )
  {
    velocity_checked = 0.0F;
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Got invalid speed request value: speed direction does not match current gear.");
    ret = false;
  } else {
    velocity_checked = std::fabs(msg.velocity_mps);
  }
  // Send commands
  apc.speed_cmd = velocity_checked;
  sc.vehicle_curvature_cmd = msg.curvature;

  // Publish commands to NE Raptor DBW
  m_accel_cmd_pub->publish(apc);
  m_steer_cmd_pub->publish(sc);

  if (m_seen_vehicle_state_cmd) {
    m_brake_cmd_pub->publish(m_brake_cmd);
  }

  m_dbw_state_machine->control_cmd_sent();
  return ret;
}

/* Apparently RawControlCommand will be obsolete soon.
 * Function not supported - AutoWare RawControlCommand message units are undefined.
 */
bool8_t NERaptorInterface::send_control_command(const RawControlCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR_THROTTLE(
    m_logger, m_clock, CLOCK_1_SEC,
    "NE Raptor does not support sending raw pedal controls directly.");
  return false;
}

bool8_t NERaptorInterface::send_control_command(const VehicleControlCommand & msg)
{
  bool8_t is_dbw_enabled = m_dbw_state_machine->enabled() ? true : false;
  bool8_t ret{true};
  float32_t velocity_checked{0.0F};
  float32_t angle_checked{0.0F};
  AcceleratorPedalCmd apc{};
  SteeringCmd sc{};

  std::lock_guard<std::mutex> guard_bc(m_brake_cmd_mutex);

  // Using steering wheel angle for control
  apc.control_type.value = ActuatorControlMode::CLOSED_LOOP_ACTUATOR;
  sc.control_type.value = ActuatorControlMode::CLOSED_LOOP_ACTUATOR;
  m_brake_cmd.control_type.value = ActuatorControlMode::CLOSED_LOOP_ACTUATOR;

  // Set enable variables
  apc.enable = is_dbw_enabled;
  m_brake_cmd.enable = is_dbw_enabled;
  sc.enable = is_dbw_enabled;
  apc.ignore = false;
  sc.ignore = false;

  // Set rolling counters
  if (m_rolling_counter_vcc < 0xFF) {
    m_rolling_counter_vcc++;
  } else {
    m_rolling_counter_vcc = 0;
  }
  apc.rolling_counter = m_rolling_counter_vcc;
  m_brake_cmd.rolling_counter = m_rolling_counter_vcc;
  sc.rolling_counter = m_rolling_counter_vcc;

  // Set limits
  apc.accel_limit = m_acceleration_limit;
  m_brake_cmd.decel_limit = m_deceleration_limit;
  apc.accel_positive_jerk_limit = m_acceleration_positive_jerk_limit;
  m_brake_cmd.decel_negative_jerk_limit = m_deceleration_negative_jerk_limit;

  if (msg.long_accel_mps2 > 0.0F) {  // acceleration limit
    apc.accel_limit = std::fabs(msg.long_accel_mps2);
  } else if (msg.long_accel_mps2 < 0.0F) {  // deceleration limit
    m_brake_cmd.decel_limit = std::fabs(msg.long_accel_mps2);
  } else {}  // no change

  // Check for invalid changes in direction
  if ( ( (m_vehicle_state_report.gear == VehicleStateReport::GEAR_DRIVE) &&
    (msg.velocity_mps < 0.0F) ) ||
    ( (m_vehicle_state_report.gear == VehicleStateReport::GEAR_REVERSE) &&
    (msg.velocity_mps > 0.0F) ) )
  {
    velocity_checked = 0.0F;
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Got invalid speed request value: speed direction does not match current gear.");
    ret = false;
  } else {
    velocity_checked = std::fabs(msg.velocity_mps);
  }

  // Limit steering angle to valid range
  angle_checked = (msg.front_wheel_angle_rad * m_steer_to_tire_ratio) / DEGREES_TO_RADIANS;
  if (angle_checked > m_max_steer_angle) {
    angle_checked = m_max_steer_angle;
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Got invalid steering angle value: request exceeds max angle.");
    ret = false;
  }
  if (angle_checked < (-1.0F * m_max_steer_angle)) {
    angle_checked = -1.0F * m_max_steer_angle;
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Got invalid steering angle value: request exceeds max angle.");
    ret = false;
  }

  // Send commands
  apc.speed_cmd = velocity_checked;
  sc.angle_cmd = angle_checked;

  // Publish commands to NE Raptor DBW
  m_accel_cmd_pub->publish(apc);
  m_steer_cmd_pub->publish(sc);

  if (m_seen_vehicle_state_cmd) {
    m_brake_cmd_pub->publish(m_brake_cmd);
  }

  m_dbw_state_machine->control_cmd_sent();
  return ret;
}

bool8_t NERaptorInterface::handle_mode_change_request(ModeChangeRequest::SharedPtr request)
{
  bool8_t ret{true};
  std_msgs::msg::Empty send_req{};
  if (request->mode == ModeChangeRequest::MODE_MANUAL) {
    m_dbw_state_machine->user_request(false);
    m_dbw_state_machine->state_cmd_sent();
    m_dbw_disable_cmd_pub->publish(send_req);
  } else if (request->mode == ModeChangeRequest::MODE_AUTONOMOUS) {
    m_dbw_state_machine->user_request(true);
    m_dbw_state_machine->state_cmd_sent();
    m_dbw_enable_cmd_pub->publish(send_req);
  } else {
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Got invalid autonomy mode request value.");
    ret = false;
  }
  return ret;
}

void NERaptorInterface::on_dbw_state_report(const std_msgs::msg::Bool::SharedPtr & msg)
{
  std::lock_guard<std::mutex> guard_vsr(m_vehicle_state_report_mutex);
  if (msg->data) {
    m_vehicle_state_report.mode = VehicleStateReport::MODE_AUTONOMOUS;
  } else {
    m_vehicle_state_report.mode = VehicleStateReport::MODE_MANUAL;
  }
  m_seen_dbw_rpt = true;
  m_dbw_state_machine->dbw_feedback(msg->data);
}

void NERaptorInterface::on_brake_report(const BrakeReport::SharedPtr & msg)
{
  std::lock_guard<std::mutex> guard_vsr(m_vehicle_state_report_mutex);
  switch (msg->parking_brake.status) {
    case ParkingBrake::OFF:
      m_vehicle_state_report.hand_brake = false;
      break;
    case ParkingBrake::ON:
      m_vehicle_state_report.hand_brake = true;
      break;
    case ParkingBrake::NO_REQUEST:
    case ParkingBrake::FAULT:
    default:
      m_vehicle_state_report.hand_brake = false;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid parking brake value from NE Raptor DBW.");
      break;
  }
  m_seen_brake_rpt = true;
}

void NERaptorInterface::on_gear_report(const GearReport::SharedPtr & msg)
{
  std::lock_guard<std::mutex> guard_vsr(m_vehicle_state_report_mutex);
  switch (msg->state.gear) {
    case Gear::PARK:
      m_vehicle_state_report.gear = VehicleStateReport::GEAR_PARK;
      break;
    case Gear::REVERSE:
      m_vehicle_state_report.gear = VehicleStateReport::GEAR_REVERSE;
      break;
    case Gear::NEUTRAL:
      m_vehicle_state_report.gear = VehicleStateReport::GEAR_NEUTRAL;
      break;
    case Gear::DRIVE:
      m_vehicle_state_report.gear = VehicleStateReport::GEAR_DRIVE;
      break;
    case Gear::LOW:
      m_vehicle_state_report.gear = VehicleStateReport::GEAR_LOW;
      break;
    case Gear::NONE:
    default:
      m_vehicle_state_report.gear = 0;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid gear value from NE Raptor DBW.");
      break;
  }
  m_seen_gear_rpt = true;
}

void NERaptorInterface::on_misc_report(const MiscReport::SharedPtr & msg)
{
  const float32_t speed_mps = msg->vehicle_speed * KPH_TO_MPS_RATIO * m_travel_direction;
  const float32_t wheelbase = m_rear_axle_to_cog + m_front_axle_to_cog;
  float32_t delta{0.0F};
  float32_t beta{0.0F};
  float32_t prev_speed_mps{0.0F};
  float32_t dT{0.0F};

  std::lock_guard<std::mutex> guard_vo(m_vehicle_odometry_mutex);
  m_vehicle_odometry.velocity_mps = speed_mps;

  std::lock_guard<std::mutex> guard_vsr(m_vehicle_state_report_mutex);
  m_vehicle_state_report.fuel = static_cast<uint8_t>(msg->fuel_level);

  std::lock_guard<std::mutex> guard_vks(m_vehicle_kin_state_mutex);

  /**
   * Input velocity is (assumed to be) measured at the rear axle, but we're
   * producing a velocity at the center of gravity.
   * Lateral velocity increases linearly from 0 at the rear axle to the maximum
   * at the front axle, where it is tan(δ)*v_lon.
   */
  delta = m_vehicle_kin_state.state.front_wheel_angle_rad;
  if (m_seen_misc_rpt &&
    m_seen_wheel_spd_rpt)
  {
    prev_speed_mps = m_vehicle_kin_state.state.longitudinal_velocity_mps;
  }
  m_vehicle_kin_state.state.longitudinal_velocity_mps = speed_mps;
  m_vehicle_kin_state.state.lateral_velocity_mps = (m_rear_axle_to_cog / wheelbase) * speed_mps *
    std::tan(delta);

  m_vehicle_kin_state.header.frame_id = "odom";

  // need >1 message in to calculate dT
  if (!m_seen_misc_rpt) {
    m_seen_misc_rpt = true;
    m_vehicle_kin_state.header.stamp = msg->header.stamp;
    // Position = (0,0) at time = 0
    m_vehicle_kin_state.state.x = 0.0F;
    m_vehicle_kin_state.state.y = 0.0F;
    m_vehicle_kin_state.state.heading =
      motion::motion_common::from_angle(0.0F);
    return;
  }

  // Calculate dT (seconds)
  dT = static_cast<float32_t>(msg->header.stamp.sec - m_vehicle_kin_state.header.stamp.sec);
  dT +=
    static_cast<float32_t>(msg->header.stamp.nanosec - m_vehicle_kin_state.header.stamp.nanosec) /
    1000000000.0F;

  if (dT < 0.0F) {
    RCLCPP_ERROR_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Received inconsistent timestamps.");
    return;
  }

  m_vehicle_kin_state.header.stamp = msg->header.stamp;

  if (m_seen_steering_rpt &&
    m_seen_wheel_spd_rpt)
  {
    m_vehicle_kin_state.state.acceleration_mps2 = (speed_mps - prev_speed_mps) / dT;  // m/s^2

    beta = std::atan2(m_rear_axle_to_cog * std::tan(delta), wheelbase);
    m_vehicle_kin_state.state.heading_rate_rps = std::cos(beta) * std::tan(delta) / wheelbase;

    // Update position (x, y), yaw
    kinematic_bicycle_model(dT, &m_vehicle_kin_state);

    m_vehicle_kin_state_pub->publish(m_vehicle_kin_state);
  }
}

void NERaptorInterface::on_other_actuators_report(const OtherActuatorsReport::SharedPtr & msg)
{
  std::lock_guard<std::mutex> guard_vsr(m_vehicle_state_report_mutex);

  switch (msg->turn_signal_state.value) {
    case TurnSignal::NONE:
      m_vehicle_state_report.blinker = VehicleStateReport::BLINKER_OFF;
      break;
    case TurnSignal::LEFT:
      m_vehicle_state_report.blinker = VehicleStateReport::BLINKER_LEFT;
      break;
    case TurnSignal::RIGHT:
      m_vehicle_state_report.blinker = VehicleStateReport::BLINKER_RIGHT;
      break;
    case TurnSignal::HAZARDS:
      m_vehicle_state_report.blinker = VehicleStateReport::BLINKER_HAZARD;
      break;
    case TurnSignal::SNA:
    default:
      m_vehicle_state_report.blinker = 0;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid turn signal value from NE Raptor DBW.");
      break;
  }

  switch (msg->high_beam_state.value) {
    case HighBeamState::OFF:
      m_vehicle_state_report.headlight = VehicleStateReport::HEADLIGHT_OFF;
      break;
    case HighBeamState::ON:
      m_vehicle_state_report.headlight = VehicleStateReport::HEADLIGHT_HIGH;
      break;
    case HighBeamState::RESERVED:
    default:
      m_vehicle_state_report.headlight = 0;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid headlight value from NE Raptor DBW.");
      break;
  }

  switch (msg->front_wiper_state.status) {
    case WiperFront::OFF:
      m_vehicle_state_report.wiper = VehicleStateReport::WIPER_OFF;
      break;
    case WiperFront::CONSTANT_LOW:
      m_vehicle_state_report.wiper = VehicleStateReport::WIPER_LOW;
      break;
    case WiperFront::CONSTANT_HIGH:
      m_vehicle_state_report.wiper = VehicleStateReport::WIPER_HIGH;
      break;
    case WiperFront::WASH_BRIEF:
      m_vehicle_state_report.wiper = VehicleStateReport::WIPER_CLEAN;
      break;
    case WiperFront::SNA:
    default:
      m_vehicle_state_report.wiper = 0;
      RCLCPP_WARN_THROTTLE(
        m_logger, m_clock, CLOCK_1_SEC,
        "Received invalid wiper value from NE Raptor DBW.");
      break;
  }

  if (m_seen_dbw_rpt &&
    m_seen_brake_rpt &&
    m_seen_gear_rpt &&
    m_seen_misc_rpt)
  {
    m_vehicle_state_report.stamp = msg->header.stamp;
    m_vehicle_state_pub->publish(m_vehicle_state_report);
  }
}

void NERaptorInterface::on_steering_report(const SteeringReport::SharedPtr & msg)
{
  const float32_t f_wheel_angle_rad = (msg->steering_wheel_angle * DEGREES_TO_RADIANS) /
    m_steer_to_tire_ratio;

  std::lock_guard<std::mutex> guard_vo(m_vehicle_odometry_mutex);
  m_vehicle_odometry.front_wheel_angle_rad = f_wheel_angle_rad;
  m_vehicle_odometry.rear_wheel_angle_rad = 0.0F;

  std::lock_guard<std::mutex> guard_vks(m_vehicle_kin_state_mutex);
  m_vehicle_kin_state.state.front_wheel_angle_rad = f_wheel_angle_rad;
  m_vehicle_kin_state.state.rear_wheel_angle_rad = 0.0F;

  m_seen_steering_rpt = true;

  if (m_seen_misc_rpt &&
    m_seen_wheel_spd_rpt)
  {
    m_vehicle_odometry.stamp = msg->header.stamp;
    m_vehicle_odo_pub->publish(m_vehicle_odometry);
  }
}

void NERaptorInterface::on_wheel_spd_report(const WheelSpeedReport::SharedPtr & msg)
{
  // Detect direction of travel
  float32_t fr = msg->front_right;
  float32_t fl = msg->front_left;
  float32_t rr = msg->rear_right;
  float32_t rl = msg->rear_left;

  if ((fr == 0.0F) && (fl == 0.0F) && (rr == 0.0F) && (rl == 0.0F)) {
    // car is not moving
    m_travel_direction = 0.0F;
  } else if ((fr >= 0.0F) && (fl >= 0.0F) && (rr >= 0.0F) && (rl >= 0.0F)) {
    // car is moving forward
    m_travel_direction = 1.0F;
  } else if ((fr <= 0.0F) && (fl <= 0.0F) && (rr <= 0.0F) && (rl <= 0.0F)) {
    // car is moving backward
    m_travel_direction = -1.0F;
  } else {
    // Wheels are moving different directions. This is probably bad.
    m_travel_direction = 0.0F;
    RCLCPP_WARN_THROTTLE(
      m_logger, m_clock, CLOCK_1_SEC,
      "Received inconsistent wheel speeds.");
  }

  m_seen_wheel_spd_rpt = true;
}

// Update x, y, heading, and heading_rate
void NERaptorInterface::kinematic_bicycle_model(
  float32_t dt, VehicleKinematicState * vks)
{
  const float32_t wheelbase = m_rear_axle_to_cog + m_front_axle_to_cog;

  // convert to yaw – copied from trajectory_spoofer.cpp
  // The below formula could probably be simplified if it would be derived directly for heading
  const float32_t sin_y = 2.0F * vks->state.heading.real * vks->state.heading.imag;
  const float32_t cos_y = 1.0F - 2.0F * vks->state.heading.imag * vks->state.heading.imag;
  float32_t yaw = std::atan2(sin_y, cos_y);
  if (yaw < 0) {
    yaw += TAU;
  }

  /* δ: tire angle (relative to car's main axis)
   * φ: heading/yaw
   * β: direction of movement at point of reference (relative to car's main axis)
   * m_rear_axle_to_cog: distance of point of reference to rear axle
   * m_front_axle_to_cog: distance of point of reference to front axle
   * wheelbase: m_rear_axle_to_cog + m_front_axle_to_cog
   * x, y, v are at the point of reference
   * x' = v cos(φ + β)
   * y' = v sin(φ + β)
   * φ' = (cos(β)tan(δ)) / wheelbase
   * v' = a
   * β = arctan((m_rear_axle_to_cog*tan(δ))/wheelbase)
   */

  float32_t v0_lat = vks->state.lateral_velocity_mps;
  float32_t v0_lon = vks->state.longitudinal_velocity_mps;
  float32_t v0 = std::sqrt(v0_lat * v0_lat + v0_lon * v0_lon);
  float32_t delta = vks->state.front_wheel_angle_rad;
  float32_t a = vks->state.acceleration_mps2;
  float32_t beta = std::atan2(m_rear_axle_to_cog * std::tan(delta), wheelbase);

  // This is the direction in which the POI is moving at the beginning of the
  // integration step. "Course" may not be super accurate, but it's to
  // emphasize that the POI doesn't travel in the heading direction.
  const float32_t course = yaw + beta;

  // How much the yaw changes per meter traveled (at the reference point)
  const float32_t yaw_change =
    std::cos(beta) * std::tan(delta) / wheelbase;

  // How much the yaw rate
  const float32_t yaw_rate = yaw_change * v0;

  // Threshold chosen so as to not result in division by 0
  if (std::abs(yaw_rate) < 1e-18f) {
    vks->state.x += std::cos(course) * (v0 * dt + 0.5f * a * dt * dt);
    vks->state.y += std::sin(course) * (v0 * dt + 0.5f * a * dt * dt);
  } else {
    vks->state.x +=
      (v0 + a * dt) / yaw_rate * std::sin(course + yaw_rate * dt) -
      v0 / yaw_rate * std::sin(course) +
      a / (yaw_rate * yaw_rate) * std::cos(course + yaw_rate * dt) -
      a / (yaw_rate * yaw_rate) * std::cos(course);
    vks->state.y +=
      -(v0 + a * dt) / yaw_rate * std::cos(course + yaw_rate * dt) +
      v0 / yaw_rate * std::cos(course) +
      a / (yaw_rate * yaw_rate) * std::sin(course + yaw_rate * dt) -
      a / (yaw_rate * yaw_rate) * std::sin(course);
  }
  yaw += std::cos(beta) * std::tan(delta) / wheelbase * (v0 * dt + 0.5f * a * dt * dt);
  vks->state.heading.real = std::cos(yaw / 2.0f);
  vks->state.heading.imag = std::sin(yaw / 2.0f);

  // Rotations per second or rad per second?
  vks->state.heading_rate_rps = yaw_rate;
}

}  // namespace ne_raptor_interface
}  // namespace autoware
