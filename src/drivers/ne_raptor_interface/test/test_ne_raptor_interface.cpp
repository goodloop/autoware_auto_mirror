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

/* Test the DBW Commands:
 * Autoware -> NE Raptor
 *
 * One Autoware command should trigger multiple
 * NE Raptor commands
 */
TEST(test_ne_raptor_interface, test_cmd_vehicle_state)
{
  VehicleStateCommand vsc;
}

TEST(test_ne_raptor_interface, test_cmd_high_level_control)
{
  HighLevelControlCommand hlcc;
}

TEST(test_ne_raptor_interface, test_cmd_raw_control)
{
  /* Not supported */
  RawControlCommand rcc;
  // rcc.stamp = rclcpp::Time::now();
  rcc.throttle = 0;
  rcc.brake = 0;
  rcc.front_steer = 0;
  rcc.rear_steer = 0;

  // EXPECT_ANY_THROW(NERaptorInterface::send_control_command(rcc));
}

TEST(test_ne_raptor_interface, test_cmd_vehicle_control)
{
  VehicleControlCommand vcc;
}

/* Test the DBW Reports:
 * NE Raptor -> Autoware
 *
 * Autoware report should not publish until
 * each relevant NE Raptor report is received
 */
TEST(test_ne_raptor_interface, test_rpt_vehicle_state)
{
  /* Needs:
   * on_brake_report(),
   * on_gear_report(),
   * on_misc_report(),
   * on_other_act_report()
   */
}

TEST(test_ne_raptor_interface, test_rpt_vehicle_odometry)
{
  /* Needs:
   * on_misc_report(),
   * on_steering_report(),
   * on_wheel_spd_report()
   */
}

TEST(test_ne_raptor_interface, test_rpt_vehicle_kinematic_state)
{
  /* Needs:
   * on_misc_report(),
   * on_steering_report(),
   * on_wheel_spd_report()
   */
}
