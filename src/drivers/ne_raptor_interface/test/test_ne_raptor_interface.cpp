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
  VehicleStateCommand vsc;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  /** Test valid inputs **/
  // Test valid: no commands
  vsc.blinker = VehicleStateCommand::BLINKER_NO_COMMAND;
  vsc.headlight = VehicleStateCommand::HEADLIGHT_NO_COMMAND;
  vsc.wiper = VehicleStateCommand::WIPER_NO_COMMAND;
  vsc.gear = VehicleStateCommand::GEAR_NO_COMMAND;
  vsc.mode = VehicleStateCommand::MODE_NO_COMMAND;
  vsc.hand_brake = false;
  vsc.horn = false;
  EXPECT_TRUE(ne_raptor_interface_->send_state_command(vsc));

  // Test valid: all off
  vsc.blinker = VehicleStateCommand::BLINKER_OFF;
  vsc.headlight = VehicleStateCommand::HEADLIGHT_OFF;
  vsc.wiper = VehicleStateCommand::WIPER_OFF;
  vsc.gear = VehicleStateCommand::GEAR_PARK;
  vsc.mode = VehicleStateCommand::MODE_MANUAL;
  vsc.hand_brake = false;
  vsc.horn = false;
  EXPECT_TRUE(ne_raptor_interface_->send_state_command(vsc));

  // Test valid: all on
  vsc.blinker = VehicleStateCommand::BLINKER_HAZARD;
  vsc.headlight = VehicleStateCommand::HEADLIGHT_HIGH;
  vsc.wiper = VehicleStateCommand::WIPER_HIGH;
  vsc.gear = VehicleStateCommand::GEAR_DRIVE;
  vsc.mode = VehicleStateCommand::MODE_AUTONOMOUS;
  vsc.hand_brake = true;
  vsc.horn = true;
  EXPECT_TRUE(ne_raptor_interface_->send_state_command(vsc));

  /** Test invalid inputs **/
  // Test invalid: blinker
  vsc.blinker = VehicleStateCommand::BLINKER_HAZARD + 1;
  EXPECT_FALSE(ne_raptor_interface_->send_state_command(vsc));

  vsc.blinker = 0xFF;
  EXPECT_FALSE(ne_raptor_interface_->send_state_command(vsc));

  // Test invalid: headlight
  vsc.blinker = VehicleStateCommand::BLINKER_NO_COMMAND;
  vsc.headlight = VehicleStateCommand::HEADLIGHT_HIGH + 1;
  EXPECT_FALSE(ne_raptor_interface_->send_state_command(vsc));

  vsc.headlight = 0xFF;
  EXPECT_FALSE(ne_raptor_interface_->send_state_command(vsc));

  // Test invalid: wiper
  vsc.headlight = VehicleStateCommand::HEADLIGHT_NO_COMMAND;
  vsc.wiper = VehicleStateCommand::WIPER_CLEAN + 1;
  EXPECT_FALSE(ne_raptor_interface_->send_state_command(vsc));

  vsc.wiper = 0xFF;
  EXPECT_FALSE(ne_raptor_interface_->send_state_command(vsc));

  // Test invalid: gear
  vsc.wiper = VehicleStateCommand::WIPER_NO_COMMAND;
  vsc.gear = VehicleStateCommand::GEAR_NEUTRAL + 1;
  EXPECT_FALSE(ne_raptor_interface_->send_state_command(vsc));

  vsc.gear = 0xFF;
  EXPECT_FALSE(ne_raptor_interface_->send_state_command(vsc));

  // Test invalid: mode
  vsc.gear = VehicleStateCommand::GEAR_NO_COMMAND;
  vsc.mode = VehicleStateCommand::MODE_MANUAL + 1;
  EXPECT_FALSE(ne_raptor_interface_->send_state_command(vsc));

  vsc.mode = 0xFF;
  EXPECT_FALSE(ne_raptor_interface_->send_state_command(vsc));
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
  executor.add_node(node_);

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
