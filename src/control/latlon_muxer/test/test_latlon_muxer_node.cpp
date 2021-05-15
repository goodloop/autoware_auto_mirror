// Copyright 2021 Tier IV, Inc.
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


#include <memory>
#include <vector>

#include "latlon_muxer/node.hpp"

#include "autoware_auto_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_msgs/msg/longitudinal_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "gtest/gtest.h"


using LatLonMuxer = autoware::motion::control::latlon_muxer::LatLonMuxer;
using LateralCommand = autoware_auto_msgs::msg::AckermannLateralCommand;
using LongitudinalCommand = autoware_auto_msgs::msg::LongitudinalCommand;
using ControlCommand = autoware_auto_msgs::msg::AckermannControlCommand;

class TestROS : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestROS, test_output)
{
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<LatLonMuxer>(node_options);

  //// Set up
  // Prepare empty messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  ControlCommand cmd_msg;
  // Create callback for output command
  bool received_combined_command = false;
  auto handle_output_cmd =
    [&lat_msg, &lon_msg, &received_combined_command,
      &cmd_msg](const ControlCommand::SharedPtr combined_msg)
    -> void {
      cmd_msg = *combined_msg;
      received_combined_command = true;
    };
  // Create publishers of node inputs and subscribe to the node output
  auto lat_pub = node->create_publisher<LateralCommand>(
    "input/lateral/control_cmd",
    rclcpp::QoS(10));
  auto lon_pub = node->create_publisher<LongitudinalCommand>(
    "input/longitudinal/control_cmd",
    rclcpp::QoS(10));
  auto cmd_sub_ptr = node->create_subscription<ControlCommand>(
    "output/control_cmd",
    rclcpp::QoS(10), handle_output_cmd);

  // Publish messages
  lat_msg.stamp = node->now();
  lon_msg.stamp = node->now();
  lat_pub->publish(lat_msg);
  lon_pub->publish(lon_msg);
  rclcpp::spin_some(node);
  rclcpp::spin_some(node);
  // Ensure the combined control command was published and contains correct data
  ASSERT_TRUE(received_combined_command);
  ASSERT_EQ(cmd_msg.lateral.steering_tire_angle, lat_msg.steering_tire_angle);
  ASSERT_EQ(cmd_msg.lateral.steering_tire_rotation_rate, lat_msg.steering_tire_rotation_rate);
  ASSERT_EQ(cmd_msg.longitudinal.speed, lon_msg.speed);
  ASSERT_EQ(cmd_msg.longitudinal.acceleration, lon_msg.acceleration);
  ASSERT_EQ(cmd_msg.longitudinal.jerk, lon_msg.jerk);
  ASSERT_GT(rclcpp::Time(cmd_msg.stamp), rclcpp::Time(lat_msg.stamp));
  ASSERT_GT(rclcpp::Time(cmd_msg.stamp), rclcpp::Time(lon_msg.stamp));

  // Generate a timeout of the node's inputs
  received_combined_command = false;
  node->set_parameter(rclcpp::Parameter("timeout_thr_sec", 0.5));
  const rclcpp::Duration one_second(1, 0);
  lat_msg.stamp = node->now() - one_second;
  lon_msg.stamp = node->now() - one_second;
  lat_pub->publish(lat_msg);
  lon_pub->publish(lon_msg);
  rclcpp::spin_some(node);
  rclcpp::spin_some(node);
  // Ensure the inputs were not combined
  ASSERT_FALSE(received_combined_command);
}
