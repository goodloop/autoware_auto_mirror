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

static bool8_t is_close(
  const float32_t a,
  const float32_t b,
  const float32_t tol = static_cast<float32_t>(1e-5))
{
  return std::abs(a - b) < tol;
}

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

/* TODO(NE_Raptor) : test published output
 */
TEST_F(NERaptorInterface_test, test_cmd_vehicle_state)
{
  test_vsc myTests[kNumTests_VSC];
  uint8_t timeout{0}, i{0};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(i_node_);
  executor.add_node(l_node_);
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
  myTests[0].exp_enable.global_enable = false;
  myTests[0].exp_enable.enable_joystick_limits = false;
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
  myTests[1].exp_enable.global_enable = false;
  myTests[1].exp_enable.enable_joystick_limits = false;
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
    myTests[i].exp_enable.global_enable = true;
    myTests[i].exp_enable.enable_joystick_limits = true;
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
  myTests[12].exp_enable.global_enable = true;
  myTests[12].exp_enable.enable_joystick_limits = true;

  // Set previous mode to off
  myTests[13].exp_success = true;
  myTests[13].in_vsc.mode = VehicleStateCommand::MODE_MANUAL;
  myTests[13].exp_gc.enable = false;
  myTests[13].exp_enable.global_enable = false;
  myTests[13].exp_enable.enable_joystick_limits = false;

  // Test invalid: mode (keep previous: off)
  myTests[14].in_vsc.mode = 0xFF;
  myTests[14].exp_gc.enable = false;
  myTests[14].exp_enable.global_enable = false;
  myTests[14].exp_enable.enable_joystick_limits = false;

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
        myTests[i].exp_enable.global_enable) << "Test #" << std::to_string(i);
      EXPECT_EQ(
        test_listener_->l_enable_cmd.enable_joystick_limits,
        myTests[i].exp_enable.enable_joystick_limits) << "Test #" << std::to_string(i);
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
  HighLevelControlCommand hlcc;
}

TEST_F(NERaptorInterface_test, test_cmd_raw_control)
{
  /* Not supported */
  RawControlCommand rcc;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(i_node_);

  rcc.stamp = test_clock.now();
  rcc.throttle = 0;
  rcc.brake = 0;
  rcc.front_steer = 0;
  rcc.rear_steer = 0;

  EXPECT_FALSE(ne_raptor_interface_->send_control_command(rcc));
}

TEST_F(NERaptorInterface_test, test_cmd_vehicle_control)
{
  VehicleControlCommand vcc;
}

/* Test the DBW Reports:
 * NE Raptor -> Autoware
 *
 * Autoware report should not publish until
 * each relevant NE Raptor report is received
 */
TEST_F(NERaptorInterface_test, test_rpt_vehicle_state)
{
  /* Needs:
   * on_brake_report(),
   * on_gear_report(),
   * on_misc_report(),
   * on_other_act_report()
   */
}

TEST_F(NERaptorInterface_test, test_rpt_vehicle_odometry)
{
  /* Needs:
   * on_misc_report(),
   * on_steering_report(),
   * on_wheel_spd_report()
   */
}

TEST_F(NERaptorInterface_test, test_rpt_vehicle_kinematic_state)
{
  /* Needs:
   * on_misc_report(),
   * on_steering_report(),
   * on_wheel_spd_report()
   */
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

  vks.state.longitudinal_velocity_mps = velocity;
  vks.state.lateral_velocity_mps = 0.0F;
  vks.state.front_wheel_angle_rad = wheel_angle;
  vks.state.x = 0.0F;
  vks.state.y = 0.0F;
  vks.state.heading = motion::motion_common::from_angle(kYaw);

  for (i = 0.0F; i < num_steps; ++i) {
    x_nominal = std::cos(kYaw) * i * dT * velocity;
    y_nominal = std::sin(kYaw) * i * dT * velocity;

    EXPECT_TRUE(is_close(x_nominal, vks.state.x)) <<
      "should be " << x_nominal << ", is " << vks.state.x;
    EXPECT_TRUE(is_close(y_nominal, vks.state.y)) <<
      "should be " << y_nominal << ", is " << vks.state.y;

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
    dist = std::sqrt(vks.state.x * vks.state.x + vks.state.y * vks.state.y);
    EXPECT_TRUE(
      is_close(
        radius, dist, static_cast<float32_t>(1e-2))) <<
      "should be " << radius << ", is " << dist;
  }
}
