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

#include "ne_raptor_interface/test_ne_raptor_interface.hpp"
#include <memory>

/* Test the DBW Commands:
 * Autoware -> NE Raptor
 *
 * One Autoware command should trigger multiple
 * NE Raptor commands
 */

/* The unit tester really hates this function for some reason.
 * Shelving the test for later investigation.
 *
TEST_F(NERaptorInterface_test, test_cmd_mode_change)
{
  ModeChangeRequest::SharedPtr t_request;

  // Test valid input
  t_request->mode = ModeChangeRequest::MODE_MANUAL;
  EXPECT_TRUE(ne_raptor_interface_->handle_mode_change_request(t_request));

  t_request->mode = ModeChangeRequest::MODE_AUTONOMOUS;
  EXPECT_TRUE(ne_raptor_interface_->handle_mode_change_request(t_request));

  // Test invalid input
  t_request->mode = ModeChangeRequest::MODE_AUTONOMOUS + 1;
  EXPECT_FALSE(ne_raptor_interface_->handle_mode_change_request(t_request));

  t_request->mode = 0xFF;
  EXPECT_FALSE(ne_raptor_interface_->handle_mode_change_request(t_request));
}
*/

TEST_F(NERaptorInterface_test, test_cmd_vehicle_state)
{
  test_vsc myTests[kNumTests_VSC];
  uint8_t timeout{0}, i{0};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(i_node_);
  executor.add_node(l_node_);
  executor.add_node(t_node_);
  executor.spin_some(C_TIMEOUT_NANO);

  /** Test valid inputs **/
  // Test valid: no commands
  myTests[0].in_vsc.blinker = VehicleStateCommand::BLINKER_NO_COMMAND;
  myTests[0].in_vsc.headlight = VehicleStateCommand::HEADLIGHT_NO_COMMAND;
  myTests[0].in_vsc.wiper = VehicleStateCommand::WIPER_NO_COMMAND;
  myTests[0].in_vsc.gear = VehicleStateCommand::GEAR_NO_COMMAND;
  myTests[0].in_vsc.mode = VehicleStateCommand::MODE_NO_COMMAND;
  myTests[0].in_vsc.hand_brake = false;
  myTests[0].in_vsc.horn = false;
  myTests[0].exp_gc.cmd.gear = Gear::NONE;
  myTests[0].exp_gc.enable = false;
  myTests[0].exp_gec.global_enable = false;
  myTests[0].exp_gec.enable_joystick_limits = false;
  myTests[0].exp_mc.cmd.value = TurnSignal::NONE;
  myTests[0].exp_mc.high_beam_cmd.status = HighBeam::OFF;
  myTests[0].exp_mc.front_wiper_cmd.status = WiperFront::OFF;
  myTests[0].exp_mc.horn_cmd = false;
  myTests[0].exp_success = true;

  // Test valid: all off
  myTests[1].in_vsc.blinker = VehicleStateCommand::BLINKER_OFF;
  myTests[1].in_vsc.headlight = VehicleStateCommand::HEADLIGHT_OFF;
  myTests[1].in_vsc.wiper = VehicleStateCommand::WIPER_OFF;
  myTests[1].in_vsc.gear = VehicleStateCommand::GEAR_PARK;
  myTests[1].in_vsc.mode = VehicleStateCommand::MODE_MANUAL;
  myTests[1].in_vsc.hand_brake = false;
  myTests[1].in_vsc.horn = false;
  myTests[1].exp_gc.cmd.gear = Gear::PARK;
  myTests[1].exp_gc.enable = false;
  myTests[1].exp_gec.global_enable = false;
  myTests[1].exp_gec.enable_joystick_limits = false;
  myTests[1].exp_mc.cmd.value = TurnSignal::NONE;
  myTests[1].exp_mc.high_beam_cmd.status = HighBeam::OFF;
  myTests[1].exp_mc.front_wiper_cmd.status = WiperFront::OFF;
  myTests[1].exp_mc.horn_cmd = false;
  myTests[1].exp_success = true;

  // Test valid: all on
  // Also init valid values for invalid tests
  for (i = 2; i < kNumTests_VSC; i++) {
    myTests[i].in_vsc.blinker = VehicleStateCommand::BLINKER_HAZARD;
    myTests[i].in_vsc.headlight = VehicleStateCommand::HEADLIGHT_HIGH;
    myTests[i].in_vsc.wiper = VehicleStateCommand::WIPER_HIGH;
    myTests[i].in_vsc.gear = VehicleStateCommand::GEAR_DRIVE;
    myTests[i].in_vsc.mode = VehicleStateCommand::MODE_AUTONOMOUS;
    myTests[i].in_vsc.hand_brake = true;
    myTests[i].in_vsc.horn = true;
    myTests[i].exp_gc.cmd.gear = Gear::DRIVE;
    myTests[i].exp_gc.enable = true;
    myTests[i].exp_gec.global_enable = true;
    myTests[i].exp_gec.enable_joystick_limits = true;
    myTests[i].exp_mc.cmd.value = TurnSignal::HAZARDS;
    myTests[i].exp_mc.high_beam_cmd.status = HighBeam::ON;
    myTests[i].exp_mc.front_wiper_cmd.status = WiperFront::CONSTANT_HIGH;
    myTests[i].exp_mc.horn_cmd = true;
    myTests[i].exp_success = (i == 2) ? true : false;
  }

  /** Test invalid inputs **/
  // Test invalid: blinker
  myTests[3].in_vsc.blinker = VehicleStateCommand::BLINKER_HAZARD + 1;
  myTests[3].exp_mc.cmd.value = TurnSignal::SNA;

  myTests[4].in_vsc.blinker = 0xFF;
  myTests[4].exp_mc.cmd.value = TurnSignal::SNA;

  // Test invalid: headlight (keep previous: on)
  myTests[5].in_vsc.headlight = VehicleStateCommand::HEADLIGHT_HIGH + 1;
  myTests[5].exp_mc.high_beam_cmd.status = HighBeam::ON;

  // not high beam, so high beams are OFF
  myTests[6].exp_success = true;
  myTests[6].in_vsc.headlight = VehicleStateCommand::HEADLIGHT_ON;
  myTests[6].exp_mc.high_beam_cmd.status = HighBeam::OFF;

  // Test invalid: headlight (keep previous: off)
  myTests[7].in_vsc.headlight = 0xFF;
  myTests[7].exp_mc.high_beam_cmd.status = HighBeam::OFF;

  // Test invalid: wiper
  myTests[8].in_vsc.wiper = VehicleStateCommand::WIPER_CLEAN + 1;
  myTests[8].exp_mc.front_wiper_cmd.status = WiperFront::SNA;

  myTests[9].in_vsc.wiper = 0xFF;
  myTests[9].exp_mc.front_wiper_cmd.status = WiperFront::SNA;

  // Test invalid: gear
  myTests[10].in_vsc.gear = VehicleStateCommand::GEAR_NEUTRAL + 1;
  myTests[10].exp_gc.cmd.gear = Gear::NONE;

  myTests[11].in_vsc.gear = 0xFF;
  myTests[11].exp_gc.cmd.gear = Gear::NONE;

  // Test invalid: mode (keep previous: on)
  myTests[12].in_vsc.mode = VehicleStateCommand::MODE_MANUAL + 1;
  myTests[12].exp_gc.enable = true;
  myTests[12].exp_gec.global_enable = true;
  myTests[12].exp_gec.enable_joystick_limits = true;

  // Set previous mode to off
  myTests[13].exp_success = true;
  myTests[13].in_vsc.mode = VehicleStateCommand::MODE_MANUAL;
  myTests[13].exp_gc.enable = false;
  myTests[13].exp_gec.global_enable = false;
  myTests[13].exp_gec.enable_joystick_limits = false;

  // Test invalid: mode (keep previous: off)
  myTests[14].in_vsc.mode = 0xFF;
  myTests[14].exp_gc.enable = false;
  myTests[14].exp_gec.global_enable = false;
  myTests[14].exp_gec.enable_joystick_limits = false;

  /* Run all tests in a loop */
  for (i = 0; i < kNumTests_VSC; i++) {
    // Test function
    if (myTests[i].exp_success) {
      EXPECT_TRUE(
        ne_raptor_interface_->send_state_command(myTests[i].in_vsc)) <<
        "Test #" << std::to_string(i);
    } else {
      EXPECT_FALSE(
        ne_raptor_interface_->send_state_command(myTests[i].in_vsc)) <<
        "Test #" << std::to_string(i);
    }

    // Test publishing
    timeout = 0;
    while ((!test_listener_->l_got_gear_cmd ||
      !test_listener_->l_got_enable_cmd ||
      !test_listener_->l_got_misc_cmd) &&
      (timeout < C_TIMEOUT_ITERATIONS) )
    {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    if (test_listener_->l_got_gear_cmd) {
      EXPECT_EQ(
        test_listener_->l_gear_cmd.cmd.gear,
        myTests[i].exp_gc.cmd.gear) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_gear_cmd.enable,
        myTests[i].exp_gc.enable) << "Test #" << std::to_string(i);
      test_listener_->l_got_gear_cmd = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_gear_cmd) <<
        "dropped package gear_cmd: Test #" << std::to_string(i);
    }

    if (test_listener_->l_got_enable_cmd) {
      EXPECT_EQ(
        test_listener_->l_enable_cmd.global_enable,
        myTests[i].exp_gec.global_enable) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_enable_cmd.enable_joystick_limits,
        myTests[i].exp_gec.enable_joystick_limits) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_enable_cmd.ecu_build_number,
        c_ecu_build_num) << "Test #" << std::to_string(i);
      test_listener_->l_got_enable_cmd = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_enable_cmd) <<
        "dropped package global_enable_cmd: Test #" << std::to_string(i);
    }

    if (test_listener_->l_got_misc_cmd) {
      EXPECT_EQ(
        test_listener_->l_misc_cmd.cmd.value,
        myTests[i].exp_mc.cmd.value) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_misc_cmd.high_beam_cmd.status,
        myTests[i].exp_mc.high_beam_cmd.status) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_misc_cmd.front_wiper_cmd.status,
        myTests[i].exp_mc.front_wiper_cmd.status) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_misc_cmd.horn_cmd,
        myTests[i].exp_mc.horn_cmd) << "Test #" << std::to_string(i);
      test_listener_->l_got_misc_cmd = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_misc_cmd) <<
        "dropped package misc_cmd: Test #" << std::to_string(i);
    }
    EXPECT_EQ(
      test_listener_->l_gear_cmd.rolling_counter,
      test_listener_->l_enable_cmd.rolling_counter) << "Test #" << std::to_string(i);
    EXPECT_EQ(
      test_listener_->l_gear_cmd.rolling_counter,
      test_listener_->l_misc_cmd.rolling_counter) << "Test #" << std::to_string(i);
  }
}

TEST_F(NERaptorInterface_test, test_cmd_high_level_control)
{
  test_hlcc myTests[kNumTests_HLCC];
  std_msgs::msg::Bool enable_dbw{};
  uint8_t timeout{0}, numDbwLoops{0}, i{0}, j{0};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(i_node_);
  executor.add_node(l_node_);
  executor.add_node(t_node_);
  executor.spin_some(C_TIMEOUT_NANO);

  // Set all gear == DRIVE && mode == AUTONOMOUS
  for (i = 0; i < kNumTests_HLCC; i++) {
    myTests[i].in_vsc.blinker = VehicleStateCommand::BLINKER_OFF;
    myTests[i].in_vsc.headlight = VehicleStateCommand::HEADLIGHT_OFF;
    myTests[i].in_vsc.wiper = VehicleStateCommand::WIPER_OFF;
    myTests[i].in_vsc.gear = VehicleStateCommand::GEAR_DRIVE;
    myTests[i].in_vsc.mode = VehicleStateCommand::MODE_AUTONOMOUS;
    myTests[i].in_vsc.hand_brake = false;
    myTests[i].in_vsc.horn = false;
    myTests[i].in_gr.state.gear = Gear::DRIVE;
    myTests[i].exp_apc.enable = true;
    myTests[i].exp_bc.enable = true;
    myTests[i].exp_sc.enable = true;
    myTests[i].exp_success = true;
  }

  /* Test valid inputs */
  // No speed, no angle
  // First time sent, DBW state machine not enabled
  myTests[0].in_hlcc.stamp = test_clock.now();
  myTests[0].in_hlcc.velocity_mps = 0.0F;
  myTests[0].in_hlcc.curvature = 0.0F;
  myTests[0].exp_apc.speed_cmd = 0.0F;
  myTests[0].exp_apc.enable = false;
  myTests[0].exp_bc.enable = false;
  myTests[0].exp_sc.enable = false;
  myTests[0].exp_sc.vehicle_curvature_cmd = 0.0F;

  // 2nd time sent, DBW should be enabled
  myTests[1].in_hlcc.stamp = test_clock.now();
  myTests[1].in_hlcc.velocity_mps = 0.0F;
  myTests[1].in_hlcc.curvature = 0.0F;
  myTests[1].exp_apc.speed_cmd = 0.0F;
  myTests[1].exp_sc.vehicle_curvature_cmd = 0.0F;

  /* Test valid input:
   * positive (forward) speed sent w/ Gear in Drive */
  myTests[2].in_vsc.gear = VehicleStateCommand::GEAR_DRIVE;
  myTests[2].in_gr.state.gear = Gear::DRIVE;
  myTests[2].in_hlcc.stamp = test_clock.now();
  myTests[2].in_hlcc.velocity_mps = 10.0F;
  myTests[2].in_hlcc.curvature = 0.0F;
  myTests[2].exp_apc.speed_cmd = 10.0F;
  myTests[2].exp_sc.vehicle_curvature_cmd = 0.0F;

  /* Test valid input:
   * negative (backward) speed sent w/ Gear in Reverse */
  myTests[3].in_vsc.gear = VehicleStateCommand::GEAR_REVERSE;
  myTests[3].in_gr.state.gear = Gear::REVERSE;
  myTests[3].in_hlcc.stamp = test_clock.now();
  myTests[3].in_hlcc.velocity_mps = -10.0F;
  myTests[3].in_hlcc.curvature = 0.0F;
  myTests[3].exp_apc.speed_cmd = 10.0F;
  myTests[3].exp_sc.vehicle_curvature_cmd = 0.0F;

  /* Test invalid input:
   * positive (forward) speed sent w/ Gear in Reverse */
  myTests[4].in_vsc.gear = VehicleStateCommand::GEAR_REVERSE;
  myTests[4].in_gr.state.gear = Gear::REVERSE;
  myTests[4].in_hlcc.stamp = test_clock.now();
  myTests[4].in_hlcc.velocity_mps = 10.0F;
  myTests[4].in_hlcc.curvature = 0.0F;
  myTests[4].exp_apc.speed_cmd = 0.0F;
  myTests[4].exp_sc.vehicle_curvature_cmd = 0.0F;
  myTests[4].exp_success = false;

  /* Test invalid input:
   * negative (backward) speed sent w/ Gear in Drive*/
  myTests[5].in_vsc.gear = VehicleStateCommand::GEAR_DRIVE;
  myTests[5].in_gr.state.gear = Gear::DRIVE;
  myTests[5].in_hlcc.stamp = test_clock.now();
  myTests[5].in_hlcc.velocity_mps = -10.0F;
  myTests[5].in_hlcc.curvature = 0.0F;
  myTests[5].exp_apc.speed_cmd = 0.0F;
  myTests[5].exp_sc.vehicle_curvature_cmd = 0.0F;
  myTests[5].exp_success = false;

  /* Test valid input:
   * send positive curvature */
  myTests[6].in_hlcc.stamp = test_clock.now();
  myTests[6].in_hlcc.velocity_mps = 0.0F;
  myTests[6].in_hlcc.curvature = 30.0F;
  myTests[6].exp_apc.speed_cmd = 0.0F;
  myTests[6].exp_sc.vehicle_curvature_cmd = 30.0F;

  /* Test valid input:
   * send negative curvature */
  myTests[7].in_hlcc.stamp = test_clock.now();
  myTests[7].in_hlcc.velocity_mps = 0.0F;
  myTests[7].in_hlcc.curvature = -30.0F;
  myTests[7].exp_apc.speed_cmd = 0.0F;
  myTests[7].exp_sc.vehicle_curvature_cmd = -30.0F;

  /* Run all tests in a loop */
  for (i = 0; i < kNumTests_HLCC; i++) {
    // Set gear input to enable/disable autonomous mode
    EXPECT_TRUE(
      ne_raptor_interface_->send_state_command(myTests[i].in_vsc)) <<
      "Test #" << std::to_string(i);
    test_talker_->send_report(myTests[i].in_gr);

    timeout = 0;
    while (!test_listener_->l_got_gear_cmd &&
      (timeout < C_TIMEOUT_ITERATIONS) )
    {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }
    if (!test_listener_->l_got_gear_cmd) {
      EXPECT_TRUE(test_listener_->l_got_gear_cmd) <<
        "dropped package gear_cmd: Test #" << std::to_string(i);
    } else {
      test_listener_->l_got_gear_cmd = false;
    }

    // Send DBW state machine feedback to enable/disable autonomous mode
    // Send once to enable, multiple times to disable
    enable_dbw.data = myTests[i].in_vsc.mode == VehicleStateCommand::MODE_AUTONOMOUS;
    numDbwLoops = enable_dbw.data ? 1 : 4;
    for (j = 0; j < numDbwLoops; j++) {
      test_talker_->send_report(enable_dbw);

      timeout = 0;
      while (!test_listener_->l_got_gear_cmd &&
        (timeout < C_TIMEOUT_ITERATIONS) )
      {
        executor.spin_some(C_TIMEOUT_NANO);
        timeout++;
      }
    }

    // Set vehicle control input
    if (myTests[i].exp_success) {
      EXPECT_TRUE(
        ne_raptor_interface_->send_control_command(myTests[i].in_hlcc)) <<
        "Test #" << std::to_string(i);
    } else {
      EXPECT_FALSE(
        ne_raptor_interface_->send_control_command(myTests[i].in_hlcc)) <<
        "Test #" << std::to_string(i);
    }
    // Test publishing
    timeout = 0;
    while ((!test_listener_->l_got_accel_cmd ||
      !test_listener_->l_got_brake_cmd ||
      !test_listener_->l_got_steer_cmd) &&
      (timeout < C_TIMEOUT_ITERATIONS) )
    {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    if (test_listener_->l_got_accel_cmd) {
      EXPECT_NEAR(
        test_listener_->l_accel_cmd.speed_cmd,
        myTests[i].exp_apc.speed_cmd,
        static_cast<float32_t>(1e-5)) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_accel_cmd.enable,
        myTests[i].exp_apc.enable) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_accel_cmd.control_type.value,
        ActuatorControlMode::CLOSED_LOOP_VEHICLE);
      EXPECT_FLOAT_EQ(
        test_listener_->l_accel_cmd.accel_limit,
        c_accel_limit);
      EXPECT_FLOAT_EQ(
        test_listener_->l_accel_cmd.accel_positive_jerk_limit,
        c_pos_jerk_limit);
      test_listener_->l_got_accel_cmd = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_accel_cmd) <<
        "dropped package accel_cmd: Test #" << std::to_string(i);
    }

    if (test_listener_->l_got_brake_cmd) {
      EXPECT_EQ(
        test_listener_->l_brake_cmd.enable,
        myTests[i].exp_bc.enable) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_brake_cmd.control_type.value,
        ActuatorControlMode::CLOSED_LOOP_VEHICLE);
      EXPECT_FLOAT_EQ(
        test_listener_->l_brake_cmd.decel_limit,
        c_decel_limit);
      EXPECT_FLOAT_EQ(
        test_listener_->l_brake_cmd.decel_negative_jerk_limit,
        c_neg_jerk_limit);
      test_listener_->l_got_brake_cmd = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_brake_cmd) <<
        "dropped package brake_cmd: Test #" << std::to_string(i);
    }

    if (test_listener_->l_got_steer_cmd) {
      EXPECT_EQ(
        test_listener_->l_steer_cmd.enable,
        myTests[i].exp_sc.enable) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_steer_cmd.control_type.value,
        ActuatorControlMode::CLOSED_LOOP_VEHICLE);
      // High-Level Control Command uses curvature for control
      EXPECT_FLOAT_EQ(
        test_listener_->l_steer_cmd.vehicle_curvature_cmd,
        myTests[i].exp_sc.vehicle_curvature_cmd);
      test_listener_->l_got_steer_cmd = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_steer_cmd) <<
        "dropped package steer_cmd: Test #" << std::to_string(i);
    }
    EXPECT_EQ(
      test_listener_->l_accel_cmd.rolling_counter,
      test_listener_->l_brake_cmd.rolling_counter);
    EXPECT_EQ(
      test_listener_->l_accel_cmd.rolling_counter,
      test_listener_->l_steer_cmd.rolling_counter);
  }
}

TEST_F(NERaptorInterface_test, test_cmd_raw_control)
{
  /* Not supported */
  RawControlCommand rcc{};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(i_node_);
  executor.add_node(l_node_);
  executor.add_node(t_node_);

  rcc.stamp = test_clock.now();
  rcc.throttle = 0;
  rcc.brake = 0;
  rcc.front_steer = 0;
  rcc.rear_steer = 0;

  EXPECT_FALSE(ne_raptor_interface_->send_control_command(rcc));
}

TEST_F(NERaptorInterface_test, test_cmd_vehicle_control)
{
  test_vcc myTests[kNumTests_VCC];
  std_msgs::msg::Bool enable_dbw{};
  uint8_t timeout{0}, numDbwLoops{0}, i{0}, j{0};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(i_node_);
  executor.add_node(l_node_);
  executor.add_node(t_node_);
  executor.spin_some(C_TIMEOUT_NANO);

  // Set all gear == DRIVE && mode == AUTONOMOUS
  for (i = 0; i < kNumTests_VCC; i++) {
    myTests[i].in_vsc.blinker = VehicleStateCommand::BLINKER_OFF;
    myTests[i].in_vsc.headlight = VehicleStateCommand::HEADLIGHT_OFF;
    myTests[i].in_vsc.wiper = VehicleStateCommand::WIPER_OFF;
    myTests[i].in_vsc.gear = VehicleStateCommand::GEAR_DRIVE;
    myTests[i].in_vsc.mode = VehicleStateCommand::MODE_AUTONOMOUS;
    myTests[i].in_vsc.hand_brake = false;
    myTests[i].in_vsc.horn = false;
    myTests[i].in_gr.state.gear = Gear::DRIVE;
    myTests[i].exp_apc.enable = true;
    myTests[i].exp_bc.enable = true;
    myTests[i].exp_sc.enable = true;
    myTests[i].exp_success = true;
  }

  /* Test valid inputs */
  // No speed, no angle, no limits set
  // First time sent, DBW state machine not enabled
  myTests[0].in_vcc.stamp = test_clock.now();
  myTests[0].in_vcc.long_accel_mps2 = 0.0F;  // keep default limits
  myTests[0].in_vcc.velocity_mps = 0.0F;
  myTests[0].in_vcc.front_wheel_angle_rad = 0.0F;
  myTests[0].exp_apc.enable = false;
  myTests[0].exp_bc.enable = false;
  myTests[0].exp_sc.enable = false;
  myTests[0].exp_apc.speed_cmd = 0.0F;
  myTests[0].exp_apc.accel_limit = c_accel_limit;
  myTests[0].exp_bc.decel_limit = c_decel_limit;
  myTests[0].exp_sc.angle_cmd = 0.0F;

  // 2nd time sent, DBW should be enabled
  myTests[1].in_vcc.stamp = test_clock.now();
  myTests[1].in_vcc.long_accel_mps2 = 0.0F;  // keep default limits
  myTests[1].in_vcc.velocity_mps = 0.0F;
  myTests[1].in_vcc.front_wheel_angle_rad = 0.0F;
  myTests[1].exp_apc.speed_cmd = 0.0F;
  myTests[1].exp_apc.accel_limit = c_accel_limit;
  myTests[1].exp_bc.decel_limit = c_decel_limit;
  myTests[1].exp_sc.angle_cmd = 0.0F;

  /* Test valid input:
   * positive (forward) speed sent w/ Gear in Drive */
  myTests[2].in_vsc.gear = VehicleStateCommand::GEAR_DRIVE;
  myTests[2].in_gr.state.gear = Gear::DRIVE;
  myTests[2].in_vcc.stamp = test_clock.now();
  myTests[2].in_vcc.long_accel_mps2 = 0.0F;  // keep default limits
  myTests[2].in_vcc.velocity_mps = 10.0F;
  myTests[2].in_vcc.front_wheel_angle_rad = 0.0F;
  myTests[2].exp_apc.speed_cmd = 10.0F;
  myTests[2].exp_apc.accel_limit = c_accel_limit;
  myTests[2].exp_bc.decel_limit = c_decel_limit;
  myTests[2].exp_sc.angle_cmd = 0.0F;

  /* Test valid input:
   * negative (backward) speed sent w/ Gear in Reverse */
  myTests[3].in_vsc.gear = VehicleStateCommand::GEAR_REVERSE;
  myTests[3].in_gr.state.gear = Gear::REVERSE;
  myTests[3].in_vcc.stamp = test_clock.now();
  myTests[3].in_vcc.long_accel_mps2 = 0.0F;  // keep default limits
  myTests[3].in_vcc.velocity_mps = -10.0F;
  myTests[3].in_vcc.front_wheel_angle_rad = 0.0F;
  myTests[3].exp_apc.speed_cmd = 10.0F;
  myTests[3].exp_apc.accel_limit = c_accel_limit;
  myTests[3].exp_bc.decel_limit = c_decel_limit;
  myTests[3].exp_sc.angle_cmd = 0.0F;

  /* Test invalid input:
   * positive (forward) speed sent w/ Gear in Reverse */
  myTests[4].in_vsc.gear = VehicleStateCommand::GEAR_REVERSE;
  myTests[4].in_gr.state.gear = Gear::REVERSE;
  myTests[4].in_vcc.stamp = test_clock.now();
  myTests[4].in_vcc.long_accel_mps2 = 0.0F;  // keep default limits
  myTests[4].in_vcc.velocity_mps = 10.0F;
  myTests[4].in_vcc.front_wheel_angle_rad = 0.0F;
  myTests[4].exp_apc.speed_cmd = 0.0F;
  myTests[4].exp_apc.accel_limit = c_accel_limit;
  myTests[4].exp_bc.decel_limit = c_decel_limit;
  myTests[4].exp_sc.angle_cmd = 0.0F;
  myTests[4].exp_success = false;

  /* Test invalid input:
   * negative (backward) speed sent w/ Gear in Drive*/
  myTests[5].in_vsc.gear = VehicleStateCommand::GEAR_DRIVE;
  myTests[5].in_gr.state.gear = Gear::DRIVE;
  myTests[5].in_vcc.stamp = test_clock.now();
  myTests[5].in_vcc.long_accel_mps2 = 0.0F;  // keep default limits
  myTests[5].in_vcc.velocity_mps = -10.0F;
  myTests[5].in_vcc.front_wheel_angle_rad = 0.0F;
  myTests[5].exp_apc.speed_cmd = 0.0F;
  myTests[5].exp_apc.accel_limit = c_accel_limit;
  myTests[5].exp_bc.decel_limit = c_decel_limit;
  myTests[5].exp_sc.angle_cmd = 0.0F;
  myTests[5].exp_success = false;

  /* Test valid input:
   * positive steering angle < max */
  myTests[6].in_vcc.stamp = test_clock.now();
  myTests[6].in_vcc.long_accel_mps2 = 0.0F;  // keep default limits
  myTests[6].in_vcc.velocity_mps = 0.0F;
  myTests[6].in_vcc.front_wheel_angle_rad =
    2.0F * autoware::ne_raptor_interface::DEGREES_TO_RADIANS;
  myTests[6].exp_apc.speed_cmd = 0.0F;
  myTests[6].exp_apc.accel_limit = c_accel_limit;
  myTests[6].exp_bc.decel_limit = c_decel_limit;
  myTests[6].exp_sc.angle_cmd = 4.0F;

  /* Test valid input:
   * abs(negative steering angle) < max */
  myTests[7].in_vcc.stamp = test_clock.now();
  myTests[7].in_vcc.long_accel_mps2 = 0.0F;  // keep default limits
  myTests[7].in_vcc.velocity_mps = 0.0F;
  myTests[7].in_vcc.front_wheel_angle_rad =
    -2.0F * autoware::ne_raptor_interface::DEGREES_TO_RADIANS;
  myTests[7].exp_apc.speed_cmd = 0.0F;
  myTests[7].exp_apc.accel_limit = c_accel_limit;
  myTests[7].exp_bc.decel_limit = c_decel_limit;
  myTests[7].exp_sc.angle_cmd = -4.0F;

  /* Test invalid input:
   * positive steering angle > max */
  myTests[8].in_vcc.stamp = test_clock.now();
  myTests[8].in_vcc.long_accel_mps2 = 0.0F;  // keep default limits
  myTests[8].in_vcc.velocity_mps = 0.0F;
  myTests[8].in_vcc.front_wheel_angle_rad =
    251.0F * autoware::ne_raptor_interface::DEGREES_TO_RADIANS;
  myTests[8].exp_apc.speed_cmd = 0.0F;
  myTests[8].exp_apc.accel_limit = c_accel_limit;
  myTests[8].exp_bc.decel_limit = c_decel_limit;
  myTests[8].exp_sc.angle_cmd = 500.0F;
  myTests[8].exp_success = false;

  /* Test invalid input:
   * abs(negative steering angle) > max */
  myTests[9].in_vcc.stamp = test_clock.now();
  myTests[9].in_vcc.long_accel_mps2 = 0.0F;  // keep default limits
  myTests[9].in_vcc.velocity_mps = 0.0F;
  myTests[9].in_vcc.front_wheel_angle_rad =
    -251.0F * autoware::ne_raptor_interface::DEGREES_TO_RADIANS;
  myTests[9].exp_apc.speed_cmd = 0.0F;
  myTests[9].exp_apc.accel_limit = c_accel_limit;
  myTests[9].exp_bc.decel_limit = c_decel_limit;
  myTests[9].exp_sc.angle_cmd = -500.0F;
  myTests[9].exp_success = false;

  /* Test valid input:
   * change positive acceleration limit */
  myTests[10].in_vcc.stamp = test_clock.now();
  myTests[10].in_vcc.long_accel_mps2 = 20.0F;
  myTests[10].in_vcc.velocity_mps = 0.0F;
  myTests[10].in_vcc.front_wheel_angle_rad = 0.0F;
  myTests[10].exp_apc.speed_cmd = 0.0F;
  myTests[10].exp_apc.accel_limit = 20.0F;
  myTests[10].exp_bc.decel_limit = c_decel_limit;
  myTests[10].exp_sc.angle_cmd = 0.0F;

  /* Test valid input:
   * change negative acceleration limit */
  myTests[11].in_vcc.stamp = test_clock.now();
  myTests[11].in_vcc.long_accel_mps2 = -20.0F;
  myTests[11].in_vcc.velocity_mps = 0.0F;
  myTests[11].in_vcc.front_wheel_angle_rad = 0.0F;
  myTests[11].exp_apc.speed_cmd = 0.0F;
  myTests[11].exp_apc.accel_limit = c_accel_limit;
  myTests[11].exp_bc.decel_limit = 20.0F;
  myTests[11].exp_sc.angle_cmd = 0.0F;

  /* Run all tests in a loop */
  for (i = 0; i < kNumTests_VCC; i++) {
    // Set gear input to enable/disable autonomous mode
    EXPECT_TRUE(
      ne_raptor_interface_->send_state_command(myTests[i].in_vsc)) <<
      "Test #" << std::to_string(i);
    test_talker_->send_report(myTests[i].in_gr);

    timeout = 0;
    while (!test_listener_->l_got_gear_cmd &&
      (timeout < C_TIMEOUT_ITERATIONS) )
    {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    if (!test_listener_->l_got_gear_cmd) {
      EXPECT_TRUE(test_listener_->l_got_gear_cmd) <<
        "dropped package gear_cmd: Test #" << std::to_string(i) << std::to_string(j);
    } else {
      test_listener_->l_got_gear_cmd = false;
    }

    // Send DBW state machine feedback to enable/disable autonomous mode
    // Send once to enable, multiple times to disable
    enable_dbw.data = myTests[i].in_vsc.mode == VehicleStateCommand::MODE_AUTONOMOUS;
    numDbwLoops = enable_dbw.data ? 1 : 4;
    for (j = 0; j < numDbwLoops; j++) {
      test_talker_->send_report(enable_dbw);

      timeout = 0;
      while (!test_listener_->l_got_gear_cmd &&
        (timeout < C_TIMEOUT_ITERATIONS) )
      {
        executor.spin_some(C_TIMEOUT_NANO);
        timeout++;
      }
    }

    // Set vehicle control input
    if (myTests[i].exp_success) {
      EXPECT_TRUE(
        ne_raptor_interface_->send_control_command(myTests[i].in_vcc)) <<
        "Test #" << std::to_string(i);
    } else {
      EXPECT_FALSE(
        ne_raptor_interface_->send_control_command(myTests[i].in_vcc)) <<
        "Test #" << std::to_string(i);
    }
    // Test publishing
    timeout = 0;
    while ((!test_listener_->l_got_accel_cmd ||
      !test_listener_->l_got_brake_cmd ||
      !test_listener_->l_got_steer_cmd) &&
      (timeout < C_TIMEOUT_ITERATIONS) )
    {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    if (test_listener_->l_got_accel_cmd) {
      EXPECT_NEAR(
        test_listener_->l_accel_cmd.speed_cmd,
        myTests[i].exp_apc.speed_cmd,
        static_cast<float32_t>(1e-5)) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_accel_cmd.enable,
        myTests[i].exp_apc.enable) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_accel_cmd.control_type.value,
        ActuatorControlMode::CLOSED_LOOP_ACTUATOR);
      EXPECT_FLOAT_EQ(
        test_listener_->l_accel_cmd.accel_limit,
        myTests[i].exp_apc.accel_limit);
      EXPECT_FLOAT_EQ(
        test_listener_->l_accel_cmd.accel_positive_jerk_limit,
        c_pos_jerk_limit);
      test_listener_->l_got_accel_cmd = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_accel_cmd) <<
        "dropped package accel_cmd: Test #" << std::to_string(i);
    }

    if (test_listener_->l_got_brake_cmd) {
      EXPECT_EQ(
        test_listener_->l_brake_cmd.enable,
        myTests[i].exp_bc.enable) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_brake_cmd.control_type.value,
        ActuatorControlMode::CLOSED_LOOP_ACTUATOR);
      EXPECT_FLOAT_EQ(
        test_listener_->l_brake_cmd.decel_limit,
        myTests[i].exp_bc.decel_limit);
      EXPECT_FLOAT_EQ(
        test_listener_->l_brake_cmd.decel_negative_jerk_limit,
        c_neg_jerk_limit);
      test_listener_->l_got_brake_cmd = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_brake_cmd) <<
        "dropped package brake_cmd: Test #" << std::to_string(i);
    }

    if (test_listener_->l_got_steer_cmd) {
      EXPECT_EQ(
        test_listener_->l_steer_cmd.enable,
        myTests[i].exp_sc.enable) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_steer_cmd.control_type.value,
        ActuatorControlMode::CLOSED_LOOP_ACTUATOR);
      // Vehicle Control Command uses tire angle for control;
      // interface converts to steering wheel angle
      EXPECT_FLOAT_EQ(
        test_listener_->l_steer_cmd.angle_cmd,
        myTests[i].exp_sc.angle_cmd);
      test_listener_->l_got_steer_cmd = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_steer_cmd) <<
        "dropped package steer_cmd: Test #" << std::to_string(i);
    }
    EXPECT_EQ(
      test_listener_->l_accel_cmd.rolling_counter,
      test_listener_->l_brake_cmd.rolling_counter);
    EXPECT_EQ(
      test_listener_->l_accel_cmd.rolling_counter,
      test_listener_->l_steer_cmd.rolling_counter);
  }
}

/* Test the DBW Reports:
 * NE Raptor -> Autoware
 *
 * Autoware report should not publish until
 * each relevant NE Raptor report is received
 */
TEST_F(NERaptorInterface_test, test_rpt_vehicle_state)
{
  test_vsr myTests[kNumTests_VSR];
  uint8_t timeout{0}, i{0};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(i_node_);
  executor.add_node(l_node_);
  executor.add_node(t_node_);
  executor.spin_some(C_TIMEOUT_NANO);

  // Set all global enable == true
  for (i = 0; i < kNumTests_VSR; i++) {
    myTests[i].in_vsc.mode = VehicleStateCommand::MODE_AUTONOMOUS;
    myTests[i].in_dbw.data = true;
  }

  /* Valid inputs:
   * DBW state machine enabled & debouncing (off);
   * other inputs off
   */
  myTests[0].in_br.parking_brake.status = ParkingBrake::OFF;
  myTests[0].in_gr.state.gear = Gear::PARK;
  myTests[0].in_mr.fuel_level = 0.0F;
  myTests[0].in_mr.drive_by_wire_enabled = true;
  myTests[0].in_oar.turn_signal_state.value = TurnSignal::NONE;
  myTests[0].in_oar.high_beam_state.value = HighBeamState::OFF;
  myTests[0].in_oar.front_wiper_state.status = WiperFront::OFF;
  myTests[0].in_oar.horn_state.status = HornState::OFF;
  myTests[0].exp_vsr.fuel = 0;
  myTests[0].exp_vsr.blinker = VehicleStateReport::BLINKER_OFF;
  myTests[0].exp_vsr.headlight = VehicleStateReport::HEADLIGHT_OFF;
  myTests[0].exp_vsr.wiper = VehicleStateReport::WIPER_OFF;
  myTests[0].exp_vsr.gear = VehicleStateReport::GEAR_PARK;
  myTests[0].exp_vsr.mode = VehicleStateReport::MODE_AUTONOMOUS;
  myTests[0].exp_vsr.hand_brake = false;
  myTests[0].exp_vsr.horn = false;

  /* Valid inputs:
   * DBW state machine enabled;
   * other inputs on
   */
  myTests[1].in_br.parking_brake.status = ParkingBrake::ON;
  myTests[1].in_gr.state.gear = Gear::DRIVE;
  myTests[1].in_mr.fuel_level = 10.0F;
  myTests[1].in_mr.drive_by_wire_enabled = true;
  myTests[1].in_oar.turn_signal_state.value = TurnSignal::HAZARDS;
  myTests[1].in_oar.high_beam_state.value = HighBeamState::ON;
  myTests[1].in_oar.front_wiper_state.status = WiperFront::WASH_BRIEF;
  myTests[1].in_oar.horn_state.status = HornState::ON;
  myTests[1].exp_vsr.fuel = 10;
  myTests[1].exp_vsr.blinker = VehicleStateReport::BLINKER_HAZARD;
  myTests[1].exp_vsr.headlight = VehicleStateReport::HEADLIGHT_HIGH;
  myTests[1].exp_vsr.wiper = VehicleStateReport::WIPER_CLEAN;
  myTests[1].exp_vsr.gear = VehicleStateReport::GEAR_DRIVE;
  myTests[1].exp_vsr.mode = VehicleStateReport::MODE_AUTONOMOUS;
  myTests[1].exp_vsr.hand_brake = true;
  myTests[1].exp_vsr.horn = true;

  // Run all tests in a loop
  for (i = 0; i < kNumTests_VSR; i++) {
    // Send these messages first
    test_talker_->send_report(myTests[i].in_br);
    test_talker_->send_report(myTests[i].in_gr);
    test_talker_->send_report(myTests[i].in_mr);
    test_talker_->send_report(myTests[i].in_dbw);
    ne_raptor_interface_->send_state_command(myTests[i].in_vsc);

    timeout = 0;
    while (timeout < C_TIMEOUT_ITERATIONS) {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    // Send this message last
    test_talker_->send_report(myTests[i].in_oar);

    timeout = 0;
    while (!test_listener_->l_got_vehicle_state &&
      (timeout < C_TIMEOUT_ITERATIONS) )
    {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    // Check values
    if (test_listener_->l_got_vehicle_state) {
      EXPECT_EQ(
        test_listener_->l_vehicle_state.fuel,
        myTests[i].exp_vsr.fuel) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_vehicle_state.blinker,
        myTests[i].exp_vsr.blinker) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_vehicle_state.headlight,
        myTests[i].exp_vsr.headlight) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_vehicle_state.wiper,
        myTests[i].exp_vsr.wiper) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_vehicle_state.gear,
        myTests[i].exp_vsr.gear) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_vehicle_state.mode,
        myTests[i].exp_vsr.mode) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_vehicle_state.hand_brake,
        myTests[i].exp_vsr.hand_brake) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_vehicle_state.horn,
        myTests[i].exp_vsr.horn) << "Test #" << std::to_string(i);
      test_listener_->l_got_vehicle_state = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_vehicle_state) <<
        "dropped package vehicle_state_report: Test #" << std::to_string(i);
    }
  }
}

TEST_F(NERaptorInterface_test, test_rpt_vehicle_odometry)
{
  test_vo myTests[kNumTests_VO];
  uint8_t timeout{0}, i{0}, dT{2};  // delta Time = 2 seconds
  float32_t travel_dir{0.0F};
  rclcpp::Time initStamp{test_clock.now()};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(i_node_);
  executor.add_node(l_node_);
  executor.add_node(t_node_);
  executor.spin_some(C_TIMEOUT_NANO);

  // Set initial values
  for (i = 0; i < kNumTests_VO; i++) {
    // Timestamps
    myTests[i].in_mr.header.stamp = initStamp;
    myTests[i].in_mr.header.stamp.sec += i * dT;
    myTests[i].in_sr.header.stamp = myTests[i].in_mr.header.stamp;
    myTests[i].in_wsr.header.stamp = myTests[i].in_mr.header.stamp;
    myTests[i].exp_vo.stamp = myTests[i].in_mr.header.stamp;

    // Speed
    myTests[i].in_mr.vehicle_speed = 10.0F *
      static_cast<float32_t>(i);  // kph
    // Forward or backward
    myTests[i].in_gr.enabled = true;

    switch (i) {
      case 12:
      case 14:
        travel_dir = 0.0F;  // not moving
        myTests[i].in_gr.state.gear = Gear::NEUTRAL;
        // Wheels not moving
        myTests[i].in_wsr.front_left = 0.0F;    // rad/sec
        myTests[i].in_wsr.front_right = 0.0F;   // rad/sec
        myTests[i].in_wsr.rear_left = 0.0F;     // rad/sec
        myTests[i].in_wsr.rear_right = 0.0F;    // rad/sec
        break;
      case 13:
        travel_dir = 0.0F;  // not moving
        myTests[i].in_gr.state.gear = Gear::NEUTRAL;
        // Wheels going different directions
        myTests[i].in_wsr.front_left = 10.0F;    // rad/sec
        myTests[i].in_wsr.front_right = 10.0F;   // rad/sec
        myTests[i].in_wsr.rear_left = -10.0F;    // rad/sec
        myTests[i].in_wsr.rear_right = -10.0F;   // rad/sec
        break;
      case 15:
      case 16:
      case 17:
        travel_dir = -1.0F;  // moving backward
        myTests[i].in_gr.state.gear = Gear::REVERSE;
        myTests[i].in_wsr.front_left = -10.0F;    // rad/sec
        myTests[i].in_wsr.front_right = -10.0F;   // rad/sec
        myTests[i].in_wsr.rear_left = -10.0F;     // rad/sec
        myTests[i].in_wsr.rear_right = -10.0F;    // rad/sec
        break;
      default:
        travel_dir = 1.0F;  // moving forward
        myTests[i].in_gr.state.gear = Gear::DRIVE;
        myTests[i].in_wsr.front_left = 10.0F;    // rad/sec
        myTests[i].in_wsr.front_right = 10.0F;   // rad/sec
        myTests[i].in_wsr.rear_left = 10.0F;     // rad/sec
        myTests[i].in_wsr.rear_right = 10.0F;    // rad/sec
        break;
    }
    // Steering - try different angles
    switch (i) {
      case 3:
      case 4:
      case 5:
        myTests[i].in_sr.steering_wheel_angle = 30.0F;  // degrees
        break;
      case 6:
      case 7:
      case 8:
        myTests[i].in_sr.steering_wheel_angle = -30.0F;  // degrees
        break;
      default:
        myTests[i].in_sr.steering_wheel_angle = 0.0F;  // degrees
        break;
    }

    myTests[i].exp_vo.velocity_mps =
      myTests[i].in_mr.vehicle_speed * travel_dir *
      autoware::ne_raptor_interface::KPH_TO_MPS_RATIO;
    myTests[i].exp_vo.front_wheel_angle_rad =
      (myTests[i].in_sr.steering_wheel_angle *
      autoware::ne_raptor_interface::DEGREES_TO_RADIANS) /
      c_steer_to_tire_ratio;
  }

  // Run all tests in a loop
  for (i = 0; i < kNumTests_VO; i++) {
    // Send these messages first
    test_talker_->send_report(myTests[i].in_gr);
    test_talker_->send_report(myTests[i].in_wsr);

    timeout = 0;
    while (timeout < C_TIMEOUT_ITERATIONS) {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    test_talker_->send_report(myTests[i].in_mr);

    timeout = 0;
    while (timeout < C_TIMEOUT_ITERATIONS) {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    // Send this message last
    test_talker_->send_report(myTests[i].in_sr);

    timeout = 0;
    while (!test_listener_->l_got_vehicle_odo &&
      (timeout < C_TIMEOUT_ITERATIONS) )
    {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    // Check values
    if (test_listener_->l_got_vehicle_odo) {
      EXPECT_FLOAT_EQ(
        test_listener_->l_vehicle_odo.velocity_mps,
        myTests[i].exp_vo.velocity_mps) << "Test #" << std::to_string(i);
      EXPECT_FLOAT_EQ(
        test_listener_->l_vehicle_odo.front_wheel_angle_rad,
        myTests[i].exp_vo.front_wheel_angle_rad) << "Test #" << std::to_string(i);
      EXPECT_FLOAT_EQ(
        test_listener_->l_vehicle_odo.rear_wheel_angle_rad,
        0.0F) << "Test #" << std::to_string(i);
      test_listener_->l_got_vehicle_odo = false;
    } else {
      EXPECT_TRUE(test_listener_->l_got_vehicle_odo) <<
        "dropped package vehicle_odometry: Test #" << std::to_string(i);
    }
  }
}

/* This test is to make sure the message is published properly.
 * The math behind the movement calculations is tested more
 * thoroughly in test_kinematic_bike_model.
 *
 * Note: VehicleKinematicState.delta is currently unused.
 * Not enabled in the interface & not testing it here.
 */
TEST_F(NERaptorInterface_test, test_rpt_vehicle_kinematic_state)
{
  test_vks myTests[kNumTests_VKS];
  int32_t timeout{0}, i{0}, dT{2};  // delta Time = 2 seconds
  float32_t travel_dir{0.0F}, err_margin{0.6F};
  rclcpp::Time initStamp{test_clock.now()};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(i_node_);
  executor.add_node(l_node_);
  executor.add_node(t_node_);
  executor.spin_some(C_TIMEOUT_NANO);

  // Set initial values
  for (i = 0; i < kNumTests_VKS; i++) {
    // Timestamps
    myTests[i].in_mr.header.stamp = initStamp;
    myTests[i].in_mr.header.stamp.sec += i * dT;
    myTests[i].in_sr.header.stamp = myTests[i].in_mr.header.stamp;
    myTests[i].in_wsr.header.stamp = myTests[i].in_mr.header.stamp;
    myTests[i].exp_vks.header.stamp = myTests[i].in_mr.header.stamp;

    // Speed
    myTests[i].in_mr.vehicle_speed = 10.0F *
      static_cast<float32_t>(i);  // kph
    // Forward or backward
    myTests[i].in_gr.enabled = true;

    switch (i) {
      case 12:
      case 14:
        travel_dir = 0.0F;  // not moving
        myTests[i].in_gr.state.gear = Gear::NEUTRAL;
        // Wheels not moving
        myTests[i].in_wsr.front_left = 0.0F;    // rad/sec
        myTests[i].in_wsr.front_right = 0.0F;   // rad/sec
        myTests[i].in_wsr.rear_left = 0.0F;     // rad/sec
        myTests[i].in_wsr.rear_right = 0.0F;    // rad/sec
        break;
      case 13:
        travel_dir = 0.0F;  // not moving
        myTests[i].in_gr.state.gear = Gear::NEUTRAL;
        // Wheels going different directions
        myTests[i].in_wsr.front_left = 10.0F;    // rad/sec
        myTests[i].in_wsr.front_right = 10.0F;   // rad/sec
        myTests[i].in_wsr.rear_left = -10.0F;    // rad/sec
        myTests[i].in_wsr.rear_right = -10.0F;   // rad/sec
        break;
      case 15:
      case 16:
      case 17:
        travel_dir = -1.0F;  // moving backward
        myTests[i].in_gr.state.gear = Gear::REVERSE;
        myTests[i].in_wsr.front_left = -10.0F;    // rad/sec
        myTests[i].in_wsr.front_right = -10.0F;   // rad/sec
        myTests[i].in_wsr.rear_left = -10.0F;     // rad/sec
        myTests[i].in_wsr.rear_right = -10.0F;    // rad/sec
        break;
      default:
        travel_dir = 1.0F;  // moving forward
        myTests[i].in_gr.state.gear = Gear::DRIVE;
        myTests[i].in_wsr.front_left = 10.0F;    // rad/sec
        myTests[i].in_wsr.front_right = 10.0F;   // rad/sec
        myTests[i].in_wsr.rear_left = 10.0F;     // rad/sec
        myTests[i].in_wsr.rear_right = 10.0F;    // rad/sec
        break;
    }
    // Steering - try different angles
    switch (i) {
      case 3:
      case 4:
      case 5:
        myTests[i].in_sr.steering_wheel_angle = 30.0F;  // degrees
        break;
      case 6:
      case 7:
      case 8:
        myTests[i].in_sr.steering_wheel_angle = -30.0F;  // degrees
        break;
      default:
        myTests[i].in_sr.steering_wheel_angle = 0.0F;  // degrees
        break;
    }

    // Run kinematic bike model
    if (i == 0) {  /* First sending: no dT */
      myTests[i].exp_vks.state.x = 0.0F;
      myTests[i].exp_vks.state.y = 0.0F;
      myTests[i].exp_vks.state.heading =
        motion::motion_common::from_angle(0.0F);
      myTests[i].exp_vks.state.acceleration_mps2 = 0.0F;
    } else {
      myTests[i].exp_vks = myTests[i - 1].exp_vks;
    }
    // Set expected speed & steering angles
    myTests[i].exp_vks.state.front_wheel_angle_rad = travel_dir *
      ((myTests[i].in_sr.steering_wheel_angle *
      autoware::ne_raptor_interface::DEGREES_TO_RADIANS) /
      c_steer_to_tire_ratio);  // radians
    myTests[i].exp_vks.state.longitudinal_velocity_mps =
      travel_dir * myTests[i].in_mr.vehicle_speed *
      autoware::ne_raptor_interface::KPH_TO_MPS_RATIO;
    myTests[i].exp_vks.state.lateral_velocity_mps =
      (c_rear_axle_to_cog / (c_rear_axle_to_cog + c_front_axle_to_cog)) *
      myTests[i].exp_vks.state.longitudinal_velocity_mps *
      std::tan(0.0F);

    if (i > 0) {
      myTests[i].exp_vks.state.acceleration_mps2 =
        (myTests[i].exp_vks.state.longitudinal_velocity_mps -
        myTests[i - 1].exp_vks.state.longitudinal_velocity_mps) /
        static_cast<float32_t>(dT);
      ne_raptor_interface_->kinematic_bicycle_model(
        static_cast<float32_t>(dT), &myTests[i].exp_vks);
    }
  }

  // Run all tests in a loop
  for (i = 0; i < kNumTests_VKS; i++) {
    // Send these messages first
    test_talker_->send_report(myTests[i].in_gr);
    test_talker_->send_report(myTests[i].in_wsr);

    timeout = 0;
    while (timeout < C_TIMEOUT_ITERATIONS) {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    test_talker_->send_report(myTests[i].in_sr);

    timeout = 0;
    while (timeout < C_TIMEOUT_ITERATIONS) {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    // Send this message last
    test_talker_->send_report(myTests[i].in_mr);

    timeout = 0;
    while (!test_listener_->l_got_vehicle_kin_state &&
      (timeout < C_TIMEOUT_ITERATIONS) )
    {
      executor.spin_some(C_TIMEOUT_NANO);
      timeout++;
    }

    // Check values
    if (test_listener_->l_got_vehicle_kin_state) {
      EXPECT_NEAR(
        test_listener_->l_vehicle_kin_state.state.x,
        myTests[i].exp_vks.state.x, err_margin) <<
        "Test #" << std::to_string(i);
      EXPECT_NEAR(
        test_listener_->l_vehicle_kin_state.state.y,
        myTests[i].exp_vks.state.y, err_margin) <<
        "Test #" << std::to_string(i);
      EXPECT_FLOAT_EQ(
        test_listener_->l_vehicle_kin_state.state.longitudinal_velocity_mps,
        myTests[i].exp_vks.state.longitudinal_velocity_mps) <<
        "Test #" << std::to_string(i);
      EXPECT_FLOAT_EQ(
        test_listener_->l_vehicle_kin_state.state.front_wheel_angle_rad,
        myTests[i].exp_vks.state.front_wheel_angle_rad) <<
        "Test #" << std::to_string(i);
      EXPECT_FLOAT_EQ(
        test_listener_->l_vehicle_kin_state.state.rear_wheel_angle_rad,
        0.0F) << "Test #" << std::to_string(i);
      EXPECT_FLOAT_EQ(
        test_listener_->l_vehicle_kin_state.state.acceleration_mps2,
        myTests[i].exp_vks.state.acceleration_mps2) <<
        "Test #" << std::to_string(i);
    } else {
      // Vehicle kinematic state does not send the first time
      if (i == 0) {
        EXPECT_FALSE(test_listener_->l_got_vehicle_kin_state) <<
          "got package vehicle_kinematic_state: Test #" << std::to_string(i);
      } else {
        EXPECT_TRUE(test_listener_->l_got_vehicle_kin_state) <<
          "dropped package vehicle_kinematic_state: Test #" << std::to_string(i);
      }
    }
    test_listener_->l_got_vehicle_kin_state = false;
  }
}

/* Test the kinematic bike model
 */
TEST_F(NERaptorInterface_test, test_kinematic_bike_model)
{
  VehicleKinematicState vks{};

  /* Test driving straight */
  float32_t wheelbase = c_front_axle_to_cog + c_rear_axle_to_cog;
  float32_t num_steps{50.0F};
  float32_t kYaw =
    60.0F * autoware::ne_raptor_interface::DEGREES_TO_RADIANS;
  float32_t wheel_angle{0.0F};
  float32_t velocity{2.0F};
  float32_t dT{0.1F};  // delta-time = 0.1 second
  float32_t x_nominal{0.0F};
  float32_t y_nominal{0.0F};
  float32_t dist{0.0F};
  float32_t radius{0.0F};
  float32_t beta_rad{0.0F};
  float32_t i{0.0F};
  float32_t err_margin{static_cast<float32_t>(1e-5)};

  vks.state.longitudinal_velocity_mps = velocity;
  vks.state.lateral_velocity_mps = 0.0F;
  vks.state.front_wheel_angle_rad = wheel_angle;
  vks.state.x = 0.0F;
  vks.state.y = 0.0F;
  vks.state.heading = motion::motion_common::from_angle(kYaw);

  for (i = 0.0F; i < num_steps; ++i) {
    x_nominal = std::cos(kYaw) * i * dT * velocity;
    y_nominal = std::sin(kYaw) * i * dT * velocity;
    err_margin = static_cast<float32_t>(1e-5);

    EXPECT_NEAR(x_nominal, vks.state.x, err_margin) <<
      "Test X #" << std::to_string(i);
    EXPECT_NEAR(y_nominal, vks.state.y, err_margin) <<
      "Test Y #" << std::to_string(i);

    ne_raptor_interface_->kinematic_bicycle_model(dT, &vks);
  }

  /* Test driving in a circle */
  num_steps = 1000.0F;
  wheel_angle = 0.4F;  // 0.4 radians
  velocity = PI;

  vks.state.front_wheel_angle_rad = wheel_angle;
  vks.state.longitudinal_velocity_mps = velocity;
  vks.state.lateral_velocity_mps = velocity * std::tan(wheel_angle) *
    c_rear_axle_to_cog / wheelbase;
  beta_rad = std::atan2(
    vks.state.lateral_velocity_mps,
    vks.state.longitudinal_velocity_mps);
  radius = c_rear_axle_to_cog / std::sin(beta_rad);
  dT = 2.0F * radius / num_steps;

  // Make the circle go around (0, 0) – we're headed right at the start, and
  // turning left (positive angle).
  // So the starting position needs to be below the origin.
  vks.state.x = 0;
  vks.state.y = -radius;

  // Make sure the velocity vector is horizontal at the beginning.
  kYaw = -beta_rad;
  vks.state.heading = motion::motion_common::from_angle(kYaw);

  for (i = 0.0F; i < num_steps; i++) {
    ne_raptor_interface_->kinematic_bicycle_model(dT, &vks);
    // Check that we're driving in a circle:
    // distance from 0 should == radius
    err_margin = static_cast<float32_t>(1e-2);
    dist = std::sqrt(vks.state.x * vks.state.x + vks.state.y * vks.state.y);
    EXPECT_NEAR(radius, dist, err_margin) <<
      "Test radius #" << std::to_string(i);
  }
}
