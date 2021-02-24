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
TEST_F(NERaptorInterface_test, test_cmd_mode_change)
{
  ModeChangeRequest t_request;
}

TEST_F(NERaptorInterface_test, test_cmd_vehicle_state)
{
  VehicleStateCommand vsc;
  // uint8_t t_blinker, t_headlight, t_wiper, t_gear, t_mode;
  // bool8_t t_hand_brake, t_horn;
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

  // RCLCPP_ERROR does not throw exeptions, just prints warnings for logging
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
