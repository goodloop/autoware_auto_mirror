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
#include <vector>
#include <memory>
#include <iostream>

#include "gtest/gtest.h"
#include "odom_to_state_conversion_nodes/odom_to_state_conversion_nodes.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp"
#include "common/types.hpp"

#include "nav_msgs/msg/odometry.hpp"

using autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
using autoware_auto_vehicle_msgs::msg::VehicleOdometry;
using nav_msgs::msg::Odometry;
using autoware::common::types::float32_t;

class OdomTestNode : public rclcpp::Node
{
public:
  explicit OdomTestNode(const rclcpp::NodeOptions & options)
  :  Node("odom_pub_node", options)
  {
    m_odom = std::make_shared<Odometry>();
    m_vehicle_odom = std::make_shared<VehicleOdometry>();
    set_test_vehicle_odom(*m_vehicle_odom);
    set_test_odom(*m_odom);
    m_vehicle_odom_pub =
      create_publisher<VehicleOdometry>(
      "/vehicle/odometry", rclcpp::QoS{10});
    m_odom_pub =
      create_publisher<Odometry>(
      "/hello_odom", rclcpp::QoS{10});
    m_state_sub =
      create_subscription<VehicleKinematicState>(
      "/vehicle/vehicle_state", rclcpp::QoS{10},
      std::bind(&OdomTestNode::on_state, this, std::placeholders::_1));
  }

  void publish_odom(const Odometry & msg)
  {
    m_odom_pub->publish(msg);
  }

  void publish_vehicle_odom(const VehicleOdometry & msg)
  {
    m_vehicle_odom_pub->publish(msg);
  }

  VehicleKinematicState::SharedPtr get_state()
  {
    return m_state;
  }

  void set_test_vehicle_odom(VehicleOdometry & msg)
  {
    msg.front_wheel_angle_rad = static_cast<float32_t>(0.233);
    msg.rear_wheel_angle_rad = static_cast<float32_t>(0.788);
    msg.velocity_mps = static_cast<float32_t>(1.234);
  }

  void set_test_odom(Odometry & msg)
  {
    msg.pose.pose.position.x = 1.0;
    msg.pose.pose.position.y = 2.0;
    msg.pose.pose.position.z = 3.0;
    msg.pose.pose.orientation.x = 0.1;
    msg.pose.pose.orientation.y = 0.2;
    msg.pose.pose.orientation.z = 0.3;
    msg.pose.pose.orientation.w = 0.4;
    msg.twist.twist.angular.z = 0.5;
    msg.twist.twist.linear.y = 0.6;
    msg.twist.twist.linear.x = 0.7;
  }

  Odometry::SharedPtr get_test_odom()
  {
    return m_odom;
  }

  VehicleOdometry::SharedPtr get_test_vehicle_odom()
  {
    return m_vehicle_odom;
  }

protected:
  rclcpp::Publisher<Odometry>::SharedPtr m_odom_pub{};
  rclcpp::Publisher<VehicleOdometry>::SharedPtr m_vehicle_odom_pub{};
  rclcpp::Subscription<VehicleKinematicState>::SharedPtr m_state_sub{};
  VehicleKinematicState::SharedPtr m_state{};
  Odometry::SharedPtr m_odom{};
  VehicleOdometry::SharedPtr m_vehicle_odom{};

  void on_state(VehicleKinematicState::SharedPtr msg)
  {
    m_state = msg;
  }
};

TEST(TestOdomToStateConversionNodes, VelFromVehicleOdom) {
  rclcpp::init(0, nullptr);
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("state_topic", "/vehicle/vehicle_state");
  params.emplace_back("vehicle_odom_topic", "/vehicle/odometry");
  params.emplace_back("odom_topic", "/hello_odom");
  params.emplace_back("use_vehicle_odom_for_velocity", true);
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);
  auto nd =
    std::make_shared<odom_to_state_conversion_nodes::OdomToStateConversionNode>(node_options);
  auto test_nd = std::make_shared<OdomTestNode>(rclcpp::NodeOptions());

  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(test_nd);
  exec.add_node(nd);

  auto vehicle_odom = test_nd->get_test_vehicle_odom();
  auto odom = test_nd->get_test_odom();
  test_nd->publish_vehicle_odom(*vehicle_odom);
  test_nd->publish_odom(*odom);

  while (!test_nd->get_state()) {
    exec.spin_some(std::chrono::milliseconds{1LL});
    std::this_thread::sleep_for(std::chrono::milliseconds{1LL});
  }

  VehicleKinematicState::SharedPtr state = test_nd->get_state();
  EXPECT_FLOAT_EQ(state->state.front_wheel_angle_rad, vehicle_odom->front_wheel_angle_rad);
  EXPECT_FLOAT_EQ(state->state.rear_wheel_angle_rad, vehicle_odom->rear_wheel_angle_rad);
  EXPECT_FLOAT_EQ(
    state->state.heading_rate_rps,
    static_cast<float32_t>(odom->twist.twist.angular.z));
  EXPECT_FLOAT_EQ(
    state->state.lateral_velocity_mps,
    static_cast<float32_t>(odom->twist.twist.linear.y));
  EXPECT_FLOAT_EQ(state->state.longitudinal_velocity_mps, vehicle_odom->velocity_mps);
  EXPECT_FLOAT_EQ(
    static_cast<float32_t>(state->state.pose.position.x),
    static_cast<float32_t>(odom->pose.pose.position.x));
  (void)rclcpp::shutdown();
}

TEST(TestOdomToStateConversionNodes, VelFromOdom) {
  rclcpp::init(0, nullptr);
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("state_topic", "/vehicle/vehicle_state");
  params.emplace_back("vehicle_odom_topic", "/vehicle/odometry");
  params.emplace_back("odom_topic", "/hello_odom");
  params.emplace_back("use_vehicle_odom_for_velocity", false);
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);
  auto nd =
    std::make_shared<odom_to_state_conversion_nodes::OdomToStateConversionNode>(node_options);
  auto test_nd = std::make_shared<OdomTestNode>(rclcpp::NodeOptions());


  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(test_nd);
  exec.add_node(nd);

  auto vehicle_odom = test_nd->get_test_vehicle_odom();
  auto odom = test_nd->get_test_odom();
  test_nd->publish_vehicle_odom(*vehicle_odom);
  test_nd->publish_odom(*odom);

  while (!test_nd->get_state()) {
    exec.spin_some(std::chrono::milliseconds{1LL});
    std::this_thread::sleep_for(std::chrono::milliseconds{1LL});
  }

  VehicleKinematicState::SharedPtr state = test_nd->get_state();
  EXPECT_FLOAT_EQ(state->state.front_wheel_angle_rad, vehicle_odom->front_wheel_angle_rad);
  EXPECT_FLOAT_EQ(state->state.rear_wheel_angle_rad, vehicle_odom->rear_wheel_angle_rad);
  EXPECT_FLOAT_EQ(
    state->state.heading_rate_rps,
    static_cast<float32_t>(odom->twist.twist.angular.z));
  EXPECT_FLOAT_EQ(
    state->state.lateral_velocity_mps,
    static_cast<float32_t>(odom->twist.twist.linear.y));
  EXPECT_FLOAT_EQ(
    state->state.longitudinal_velocity_mps,
    static_cast<float32_t>(odom->twist.twist.linear.x));
  EXPECT_FLOAT_EQ(
    static_cast<float32_t>(state->state.pose.position.x),
    static_cast<float32_t>(odom->pose.pose.position.x));
  (void)rclcpp::shutdown();
}
