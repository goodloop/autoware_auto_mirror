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

DbwStateMachine::DbwStateMachine(uint16_t dbw_disabled_debounce)
: m_first_control_cmd_sent{false},
  m_first_state_cmd_sent{false},
  m_disabled_feedback_count{0},
  DISABLED_FEEDBACK_THRESH{dbw_disabled_debounce},
  m_state{DbwState::DISABLED}
{
}

bool8_t DbwStateMachine::enabled() const
{
  return m_state == DbwState::ENABLED ||
         m_state == DbwState::ENABLE_SENT ||
         (m_state == DbwState::ENABLE_REQUESTED &&
         m_first_control_cmd_sent &&
         m_first_state_cmd_sent);
}

DbwState DbwStateMachine::get_state() const {return m_state;}

void DbwStateMachine::dbw_feedback(bool8_t enabled)
{
  if (enabled) {                             // DBW system says enabled
    if (m_state == DbwState::ENABLE_SENT) {  // and state is ENABLE_SENT
      m_state = DbwState::ENABLED;
      m_disabled_feedback_count = 0;
    }
  } else {                                   // DBW system says disabled
    if (m_state == DbwState::ENABLE_SENT) {  // and state is ENABLE_SENT
      m_disabled_feedback_count++;           // Increase debounce count

      if (m_disabled_feedback_count > DISABLED_FEEDBACK_THRESH) {  // check debounce
        disable_and_reset();
      }
    } else if (m_state == DbwState::ENABLED) {  // and state is ENABLED
      disable_and_reset();
    }
  }
}

void DbwStateMachine::control_cmd_sent()
{
  if (m_state == DbwState::ENABLE_REQUESTED &&
    m_first_control_cmd_sent &&
    m_first_state_cmd_sent)
  {
    // We just sent a control command with
    // enable == true so we can transition
    // to ENABLE_SENT
    m_state = DbwState::ENABLE_SENT;
  }

  if (m_state == DbwState::ENABLE_REQUESTED) {
    m_first_control_cmd_sent = true;
  }
}

void DbwStateMachine::state_cmd_sent()
{
  if (m_state == DbwState::ENABLE_REQUESTED &&
    m_first_control_cmd_sent &&
    m_first_state_cmd_sent)
  {
    // We just sent a state command with
    // enable == true so we can transition
    // to ENABLE_SENT
    m_state = DbwState::ENABLE_SENT;
  }

  if (m_state == DbwState::ENABLE_REQUESTED) {
    m_first_state_cmd_sent = true;
  }
}

void DbwStateMachine::user_request(bool8_t enable)
{
  if (enable) {                           // Enable is being requested
    if (m_state == DbwState::DISABLED) {  // Only change states if currently in DISABLED
      m_state = DbwState::ENABLE_REQUESTED;
    }
  } else {                               // Disable is being requested
    disable_and_reset();                 // Disable in any state if user requests it
  }
}

void DbwStateMachine::disable_and_reset()
{
  m_state = DbwState::DISABLED;
  m_first_control_cmd_sent = false;
  m_first_state_cmd_sent = false;
  m_disabled_feedback_count = 0;
}

NERaptorInterface::NERaptorInterface(
  rclcpp::Node & node,
  uint16_t ecu_build_num,
  float32_t front_axle_to_cog,
  float32_t rear_axle_to_cog,
  float32_t steer_to_tire_ratio,
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
  m_acceleration_limit{acceleration_limit},
  m_deceleration_limit{deceleration_limit},
  m_acceleration_positive_jerk_limit{acceleration_positive_jerk_limit},
  m_deceleration_negative_jerk_limit{deceleration_negative_jerk_limit},
  m_dbw_state_machine(new DbwStateMachine{3})
{
  // Publishers (to Raptor DBW)
  m_accel_cmd_pub = node.create_publisher<AcceleratorPedalCmd>("accelerator_pedal_cmd", 10);
  m_brake_cmd_pub = node.create_publisher<BrakeCmd>("brake_cmd", 10);
  m_gear_cmd_pub = node.create_publisher<GearCmd>("gear_cmd", 10);
  m_enable_cmd_pub = node.create_publisher<GlobalEnableCmd>("global_enable_cmd", 10);
  m_misc_cmd_pub = node.create_publisher<MiscCmd>("misc_cmd", 10);
  m_steer_cmd_pub = node.create_publisher<SteeringCmd>("steering_cmd", 10);

  // Publishers (to Autoware)
  m_vehicle_state_pub = node.create_publisher<VehicleStateReport>("vehicle_state_report", 10);
  m_vehicle_odo_pub = node.create_publisher<VehicleOdometry>("vehicle_odometry", 10);
  m_vehicle_kin_state_pub = node.create_publisher<VehicleKinematicState>(
    "vehicle_kinematic_state",
    10);

  // Subscribers (from Raptor DBW)
  m_dbw_state_sub =
    node.create_subscription<std_msgs::msg::Bool>(
    "dbw_enabled_feedback", rclcpp::QoS{10},
    [this](std_msgs::msg::Bool::SharedPtr msg) {on_dbw_state_report(msg);});
  m_brake_rpt_sub =
    node.create_subscription<BrakeReport>(
    "brake_report", rclcpp::QoS{10},
    [this](BrakeReport::SharedPtr msg) {on_brake_report(msg);});
  m_gear_rpt_sub =
    node.create_subscription<GearReport>(
    "gear_report", rclcpp::QoS{10},
    [this](GearReport::SharedPtr msg) {on_gear_report(msg);});
  m_misc_rpt_sub =
    node.create_subscription<MiscReport>(
    "misc_report", rclcpp::QoS{10},
    [this](MiscReport::SharedPtr msg) {on_misc_report(msg);});
  m_other_acts_rpt_sub =
    node.create_subscription<OtherActuatorsReport>(
    "other_actuators_report", rclcpp::QoS{10},
    [this](OtherActuatorsReport::SharedPtr msg) {on_other_act_report(msg);});
  m_steering_rpt_sub =
    node.create_subscription<SteeringReport>(
    "steering_report", rclcpp::QoS{10},
    [this](SteeringReport::SharedPtr msg) {on_steering_report(msg);});
  m_wheel_spd_rpt_sub =
    node.create_subscription<WheelSpeedReport>(
    "wheel_speed_report", rclcpp::QoS{10},
    [this](WheelSpeedReport::SharedPtr msg) {on_wheel_spd_report(msg);});
}

bool8_t NERaptorInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  return true;
}

bool8_t NERaptorInterface::send_state_command(const VehicleStateCommand & msg)
{
  static uint8_t s_rolling_counter_vsc = 0;
  bool8_t is_dbw_enabled = m_dbw_state_machine->enabled() ? true : false;
  GearCmd gc;
  GlobalEnableCmd gec;
  MiscCmd mc;
  // msg.fuel unused
  // msg.hand_brake unused

  // Set enable variables
  gc.enable = is_dbw_enabled;
  gec.global_enable = is_dbw_enabled;
  gec.enable_joystick_limits = is_dbw_enabled;
  mc.block_standard_cruise_buttons = is_dbw_enabled;
  mc.block_adaptive_cruise_buttons = is_dbw_enabled;
  mc.block_turn_signal_stalk = is_dbw_enabled;

  // Set rolling counters
  if (s_rolling_counter_vsc < 0xFF) {
    s_rolling_counter_vsc++;
  } else {
    s_rolling_counter_vsc = 0;
  }
  gc.rolling_counter = s_rolling_counter_vsc;
  gec.rolling_counter = s_rolling_counter_vsc;
  mc.rolling_counter = s_rolling_counter_vsc;

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
      RCLCPP_ERROR(m_logger, "Received command for invalid gear state.");
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
      RCLCPP_ERROR(m_logger, "Received command for invalid turn signal state.");
      break;
  }

  switch (msg.headlight) {
    case VehicleStateCommand::HEADLIGHT_NO_COMMAND:
    case VehicleStateCommand::HEADLIGHT_OFF:
    case VehicleStateCommand::HEADLIGHT_ON:
      mc.high_beam_cmd.status = HighBeam::OFF;
      break;
    case VehicleStateCommand::HEADLIGHT_HIGH:
      mc.high_beam_cmd.status = HighBeam::ON;
      break;
    default:
      mc.high_beam_cmd.status = HighBeam::RESERVED;
      RCLCPP_ERROR(m_logger, "Received command for invalid headlight state.");
      break;
  }

  switch (msg.wiper) {
    case VehicleStateCommand::WIPER_NO_COMMAND:
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
      RCLCPP_ERROR(m_logger, "Received command for invalid wiper state.");
      break;
  }

  m_gear_cmd_pub->publish(gc);
  m_enable_cmd_pub->publish(gec);
  m_misc_cmd_pub->publish(mc);

  m_dbw_state_machine->state_cmd_sent();
  return is_dbw_enabled;
}

/* Apparently HighLevelControlCommand will be obsolete soon.
 */
bool8_t NERaptorInterface::send_control_command(const HighLevelControlCommand & msg)
{
  static uint8_t s_rolling_counter_hlcc = 0;
  bool8_t is_dbw_enabled = m_dbw_state_machine->enabled() ? true : false;
  float32_t velocity_checked = 0.0F;
  AcceleratorPedalCmd apc;
  BrakeCmd bc;
  SteeringCmd sc;

  // Set enable variables
  apc.enable = is_dbw_enabled;
  bc.enable = is_dbw_enabled;
  sc.enable = is_dbw_enabled;
  apc.ignore = false;
  sc.ignore = false;

  // Set rolling counters
  if (s_rolling_counter_hlcc < 0xFF) {
    s_rolling_counter_hlcc++;
  } else {
    s_rolling_counter_hlcc = 0;
  }
  apc.rolling_counter = s_rolling_counter_hlcc;
  bc.rolling_counter = s_rolling_counter_hlcc;
  sc.rolling_counter = s_rolling_counter_hlcc;

  // Set limits
  apc.accel_limit = m_acceleration_limit;
  bc.decel_limit = m_deceleration_limit;
  apc.accel_positive_jerk_limit = m_acceleration_positive_jerk_limit;
  bc.decel_negative_jerk_limit = m_deceleration_negative_jerk_limit;

  // Check for invalid changes in direction
  if ( ( (state_report().gear == VehicleStateReport::GEAR_DRIVE) &&
    (msg.velocity_mps < 0.0F) ) ||
    ( (state_report().gear == VehicleStateReport::GEAR_REVERSE) &&
    (msg.velocity_mps > 0.0F) ) )
  {
    velocity_checked = 0.0F;
  } else {
    velocity_checked = std::fabs(msg.velocity_mps);
  }
  // Send commands
  apc.speed_cmd = velocity_checked;
  sc.vehicle_curvature_cmd = msg.curvature;

  // Publish commands to NE Raptor DBW
  m_accel_cmd_pub->publish(apc);
  m_brake_cmd_pub->publish(bc);
  m_steer_cmd_pub->publish(sc);

  m_dbw_state_machine->control_cmd_sent();
  return is_dbw_enabled;
}

/* Apparently RawControlCommand will be obsolete soon.
 * Function not supported - AutoWare RawControlCommand message units are undefined.
 */
bool8_t NERaptorInterface::send_control_command(const RawControlCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR(m_logger, "NE Raptor does not support sending raw pedal controls directly.");
  return false;

  /* In case message units become defined

  static uint8_t s_rolling_counter_rcc = 0;
  bool8_t is_dbw_enabled = m_dbw_state_machine->enabled() ? true : false;
  float32_t velocity_checked = 0.0F;
  AcceleratorPedalCmd apc;
  BrakeCmd bc;
  SteeringCmd sc;

  // Set enable variables
  apc.enable = is_dbw_enabled;
  bc.enable = is_dbw_enabled;
  sc.enable = is_dbw_enabled;
  apc.ignore = false;
  sc.ignore = false;

  // Set rolling counters
  if(s_rolling_counter_rcc < 0xFF)
  {
    s_rolling_counter_rcc++;
  }
  else
  {
    s_rolling_counter_rcc = 0;
  }
  apc.rolling_counter = s_rolling_counter_rcc;
  bc.rolling_counter = s_rolling_counter_rcc;
  sc.rolling_counter = s_rolling_counter_rcc;

  // Set limits
  apc.accel_limit = m_acceleration_limit;
  bc.decel_limit = m_deceleration_limit;
  apc.accel_positive_jerk_limit = m_acceleration_positive_jerk_limit;
  bc.decel_negative_jerk_limit = m_deceleration_negative_jerk_limit;

  // Send commands
  apc.pedal_cmd = (float32_t)msg.throttle;
  bpc.pedal_cmd = (float32_t)msg.brake;
  sc.angle_cmd = (float32_t)msg.front_steer * STEERING_PCT_TO_DEGREE_RATIO / m_steer_to_tire_ratio;

  // Publish commands to NE Raptor DBW
  m_accel_cmd_pub->publish(apc);
  m_brake_cmd_pub->publish(bc);
  m_steer_cmd_pub->publish(sc);

  m_dbw_state_machine->control_cmd_sent();
  return is_dbw_enabled;
  */
}

bool8_t NERaptorInterface::send_control_command(const VehicleControlCommand & msg)
{
  static uint8_t s_rolling_counter_vcc = 0;
  bool8_t is_dbw_enabled = m_dbw_state_machine->enabled() ? true : false;
  float32_t velocity_checked = 0.0F;
  AcceleratorPedalCmd apc;
  BrakeCmd bc;
  SteeringCmd sc;

  // Set enable variables
  apc.enable = is_dbw_enabled;
  bc.enable = is_dbw_enabled;
  sc.enable = is_dbw_enabled;
  apc.ignore = false;
  sc.ignore = false;

  // Set rolling counters
  if (s_rolling_counter_vcc < 0xFF) {
    s_rolling_counter_vcc++;
  } else {
    s_rolling_counter_vcc = 0;
  }
  apc.rolling_counter = s_rolling_counter_vcc;
  bc.rolling_counter = s_rolling_counter_vcc;
  sc.rolling_counter = s_rolling_counter_vcc;

  // Set limits
  apc.accel_limit = m_acceleration_limit;
  bc.decel_limit = m_deceleration_limit;
  apc.accel_positive_jerk_limit = m_acceleration_positive_jerk_limit;
  bc.decel_negative_jerk_limit = m_deceleration_negative_jerk_limit;

  if (msg.long_accel_mps2 > 0.0F) {  // acceleration limit
    apc.accel_limit = std::fabs(msg.long_accel_mps2);
  } else if (msg.long_accel_mps2 < 0.0F) {  // deceleration limit
    bc.decel_limit = std::fabs(msg.long_accel_mps2);
  } else {}  // no change

  // Check for invalid changes in direction
  if ( ( (state_report().gear == VehicleStateReport::GEAR_DRIVE) &&
    (msg.velocity_mps < 0.0F) ) ||
    ( (state_report().gear == VehicleStateReport::GEAR_REVERSE) &&
    (msg.velocity_mps > 0.0F) ) )
  {
    velocity_checked = 0.0F;
  } else {
    velocity_checked = std::fabs(msg.velocity_mps);
  }

  // Send commands
  apc.speed_cmd = velocity_checked;
  sc.angle_cmd = msg.front_wheel_angle_rad / DEGREES_TO_RADIANS;

  // Publish commands to NE Raptor DBW
  m_accel_cmd_pub->publish(apc);
  m_brake_cmd_pub->publish(bc);
  m_steer_cmd_pub->publish(sc);

  m_dbw_state_machine->control_cmd_sent();
  return is_dbw_enabled;
}

bool8_t NERaptorInterface::handle_mode_change_request(ModeChangeRequest::SharedPtr request)
{
  if (request->mode == ModeChangeRequest::MODE_MANUAL) {
    m_dbw_state_machine->user_request(false);
    return true;
  } else if (request->mode == ModeChangeRequest::MODE_AUTONOMOUS) {
    m_dbw_state_machine->user_request(true);
    return true;
  } else {
    RCLCPP_ERROR(m_logger, "Got invalid autonomy mode request value.");
    return false;
  }
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
    case ParkingBrake::TRANS:
      m_vehicle_state_report.hand_brake = false;
      break;
    case ParkingBrake::ON:
      m_vehicle_state_report.hand_brake = true;
      break;
    case ParkingBrake::FAULT:
    default:
      m_vehicle_state_report.hand_brake = false;
      RCLCPP_WARN(m_logger, "Received invalid parking brake value from NE Raptor DBW.");
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
      RCLCPP_WARN(m_logger, "Received invalid gear value from NE Raptor DBW.");
      break;
  }

  m_seen_gear_rpt = true;
}

void NERaptorInterface::on_misc_report(const MiscReport::SharedPtr & msg)
{
  const float32_t speed_mps = msg->vehicle_speed * KPH_TO_MPS_RATIO * m_travel_direction;
  const float32_t wheelbase = m_rear_axle_to_cog + m_front_axle_to_cog;
  float32_t delta;
  float32_t beta;
  float32_t prev_speed_mps = 0.0F;
  float32_t dT;

  std::lock_guard<std::mutex> guard_vo(m_vehicle_odometry_mutex);
  m_vehicle_odometry.velocity_mps = speed_mps;

  std::lock_guard<std::mutex> guard_vsr(m_vehicle_state_report_mutex);
  m_vehicle_state_report.fuel = static_cast<uint8_t>(msg->fuel_level);

  std::lock_guard<std::mutex> guard_vks(m_vehicle_kin_state_mutex);
  /// \brief TODO(NE_Raptor)::Math taken from ssc_interface.cpp. Verify it.
  /* Input velocity is (assumed to be) measured at the rear axle, but we're
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

  m_vehicle_kin_state.header.frame_id = "odom";  // TODO(NE_Raptor) : Check this

  // need >1 message in to calculate dT
  if (!m_seen_misc_rpt) {
    m_seen_misc_rpt = true;
    m_vehicle_kin_state.header.stamp = msg->header.stamp;
    return;
  }

  // Calculate dT (seconds)
  dT = static_cast<float32_t>(msg->header.stamp.sec - m_vehicle_kin_state.header.stamp.sec);
  dT += static_cast<float32_t>(msg->header.stamp.sec - m_vehicle_kin_state.header.stamp.sec) /
    1000000000.0F;

  if (dT < 0.0F) {
    RCLCPP_WARN(m_logger, "Received inconsistent timestamps.");
  }

  m_vehicle_kin_state.header.stamp = msg->header.stamp;

  if (m_seen_steering_rpt &&
    m_seen_wheel_spd_rpt)
  {
    m_vehicle_kin_state.state.acceleration_mps2 = (speed_mps - prev_speed_mps) / dT;  // m/s^2
    m_vehicle_kin_state.state.x = 0.0F;
    m_vehicle_kin_state.state.y = 0.0F;
    m_vehicle_kin_state.state.heading.real = std::cos(0.0F);  // TODO(NE_Raptor) : yaw?
    m_vehicle_kin_state.state.heading.imag = std::sin(0.0F);  // TODO(NE_Raptor) : yaw?

    beta = std::atan2(m_rear_axle_to_cog * std::tan(delta), wheelbase);
    m_vehicle_kin_state.state.heading_rate_rps = std::cos(beta) * std::tan(delta) / wheelbase;

    m_vehicle_kin_state_pub->publish(m_vehicle_kin_state);
  }
}

void NERaptorInterface::on_other_act_report(const OtherActuatorsReport::SharedPtr & msg)
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
      RCLCPP_WARN(m_logger, "Received invalid turn signal value from NE Raptor DBW.");
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
    // case HighBeamState::SNA:
    default:
      m_vehicle_state_report.headlight = 0;
      RCLCPP_WARN(m_logger, "Received invalid headlight value from NE Raptor DBW.");
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
    case WiperFront::INTERVAL_1:
    case WiperFront::INTERVAL_2:
    case WiperFront::INTERVAL_3:
    case WiperFront::INTERVAL_4:
    case WiperFront::INTERVAL_5:
    case WiperFront::INTERVAL_6:
    case WiperFront::WASH_CONTINUOUS:
    case WiperFront::SNA:
    default:
      m_vehicle_state_report.wiper = 0;
      RCLCPP_WARN(m_logger, "Received invalid wiper value from NE Raptor DBW.");
      break;
  }

  switch (msg->horn_state.status) {
    case HornState::OFF:
      m_vehicle_state_report.horn = false;
      break;
    case HornState::ON:
      m_vehicle_state_report.horn = true;
      break;
    case HornState::SNA:
    default:
      m_vehicle_state_report.horn = false;
      RCLCPP_WARN(m_logger, "Received invalid horn value from NE Raptor DBW.");
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
  const float32_t f_wheel_angle_rad = msg->steering_wheel_angle * m_steer_to_tire_ratio *
    DEGREES_TO_RADIANS;

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
    RCLCPP_WARN(m_logger, "Received inconsistent wheel speeds.");
  }

  m_seen_wheel_spd_rpt = true;
}

}  // namespace ne_raptor_interface
}  // namespace autoware
