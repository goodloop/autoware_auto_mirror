// Copyright 2021 The Autoware Foundation
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
  std::shared_ptr<LatLonMuxer> node_;

  rclcpp::Publisher<LateralCommand>::SharedPtr lat_pub_;
  rclcpp::Publisher<LongitudinalCommand>::SharedPtr lon_pub_;
  rclcpp::Subscription<ControlCommand>::SharedPtr cmd_sub_;

  ControlCommand cmd_msg_;
  bool received_combined_command_ = false;

  void SetUp()
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions node_options;
    node_ = std::make_shared<LatLonMuxer>(node_options);

    lat_pub_ = node_->create_publisher<LateralCommand>(
      "input/lateral/control_cmd",
      rclcpp::QoS(10));
    lon_pub_ = node_->create_publisher<LongitudinalCommand>(
      "input/longitudinal/control_cmd",
      rclcpp::QoS(10));
    cmd_sub_ = node_->create_subscription<ControlCommand>(
      "output/control_cmd",
      rclcpp::QoS(10), std::bind(&TestROS::HandleOutputCommand, this, std::placeholders::_1));
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

  void HandleOutputCommand(const ControlCommand::SharedPtr combined_msg)
  {
    cmd_msg_ = *combined_msg;
    received_combined_command_ = true;
  }
};

const rclcpp::Duration one_second(1, 0);

TEST_F(TestROS, test_correct_output)
{
  // Prepare messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  lat_msg.steering_tire_angle = 1.5;
  lat_msg.steering_tire_rotation_rate = 0.2f;
  lon_msg.speed = 5.0;
  lon_msg.acceleration = -1.0;
  lon_msg.jerk = 0.25;
  // Publish messages
  lat_msg.stamp = node_->now();
  lon_msg.stamp = node_->now();
  lat_pub_->publish(lat_msg);
  lon_pub_->publish(lon_msg);
  rclcpp::spin_some(node_);
  rclcpp::spin_some(node_);
  // Ensure the combined control command was published and contains correct data
  ASSERT_TRUE(received_combined_command_);
  ASSERT_EQ(cmd_msg_.lateral.steering_tire_angle, lat_msg.steering_tire_angle);
  ASSERT_EQ(cmd_msg_.lateral.steering_tire_rotation_rate, lat_msg.steering_tire_rotation_rate);
  ASSERT_EQ(cmd_msg_.longitudinal.speed, lon_msg.speed);
  ASSERT_EQ(cmd_msg_.longitudinal.acceleration, lon_msg.acceleration);
  ASSERT_EQ(cmd_msg_.longitudinal.jerk, lon_msg.jerk);
  ASSERT_GT(rclcpp::Time(cmd_msg_.stamp), rclcpp::Time(lat_msg.stamp));
  ASSERT_GT(rclcpp::Time(cmd_msg_.stamp), rclcpp::Time(lon_msg.stamp));
}

TEST_F(TestROS, test_lateral_timeout)
{
  // Prepare empty messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  // Generate a timeout of the lateral message
  node_->set_parameter(rclcpp::Parameter("timeout_thr_sec", 0.5));
  lat_msg.stamp = node_->now() - one_second;
  lon_msg.stamp = node_->now();
  lat_pub_->publish(lat_msg);
  lon_pub_->publish(lon_msg);
  rclcpp::spin_some(node_);
  rclcpp::spin_some(node_);
  // Ensure the inputs were not combined
  ASSERT_FALSE(received_combined_command_);
}

TEST_F(TestROS, test_longitudinal_timeout)
{
  // Prepare empty messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  // Generate a timeout of the longitudinal message
  node_->set_parameter(rclcpp::Parameter("timeout_thr_sec", 0.5));
  lat_msg.stamp = node_->now();
  lon_msg.stamp = node_->now() - one_second;
  lat_pub_->publish(lat_msg);
  lon_pub_->publish(lon_msg);
  rclcpp::spin_some(node_);
  rclcpp::spin_some(node_);
  // Ensure the inputs were not combined
  ASSERT_FALSE(received_combined_command_);
}
TEST_F(TestROS, test_latlon_timeout)
{
  // Prepare empty messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  // Generate a timeout of both messages
  node_->set_parameter(rclcpp::Parameter("timeout_thr_sec", 0.5));
  lat_msg.stamp = node_->now() - one_second;
  lon_msg.stamp = node_->now() - one_second;
  lat_pub_->publish(lat_msg);
  lon_pub_->publish(lon_msg);
  rclcpp::spin_some(node_);
  rclcpp::spin_some(node_);
  // Ensure the inputs were not combined
  ASSERT_FALSE(received_combined_command_);
}
