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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include "emergency_handler/emergency_handler_node.hpp"

#include <memory>

#include "gtest/gtest.h"

#include "test_utils.hpp"

using namespace std::chrono_literals;

using autoware_auto_msgs::msg::VehicleStateReport;
using autoware_auto_msgs::msg::VehicleOdometry;
using autoware_auto_msgs::msg::DrivingCapability;
using autoware_auto_msgs::msg::AutowareState;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;
using autoware_auto_msgs::msg::EmergencyMode;
using autoware_auto_msgs::msg::HazardStatusStamped;
using autoware::emergency_handler::EmergencyHandlerNode;

using diagnostic_msgs::msg::DiagnosticArray;

template<typename T>
class Spy
{
public:
  Spy(rclcpp::Node::SharedPtr n, rclcpp::Executor::SharedPtr exec,
    const std::string & topic_name)
  : node(n), executor(exec)
  {
    using std::placeholders::_1;
    auto sub_options = rclcpp::SubscriptionOptions();

    subscription = node->create_subscription<T>(
      topic_name, 1, std::bind(&Spy::onMsg, this, _1), sub_options);
  }

  T expectMsg()
  {
    const auto t_start = node->get_clock()->now();
    constexpr double timeout = 5.0;

    while (!is_new_msg) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("rclcpp is in NOK state");
      }
      if ((node->get_clock()->now() - t_start).seconds() > timeout) {
        throw std::runtime_error("timeout occurred during waiting for msg");
      }
      executor->spin_some(10ms);
    }

    is_new_msg = false;
    return received_msg;
  }

protected:
  void onMsg(const std::shared_ptr<T> msg)
  {
    RCLCPP_INFO(node->get_logger(), "Received %s", rosidl_generator_traits::name<T>());
    is_new_msg = true;
    received_msg = *msg;
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Executor::SharedPtr executor;
  std::shared_ptr<rclcpp::Subscription<T>> subscription;

  T received_msg;
  bool is_new_msg = false;
};

class EmergencyHandlerNodeTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    // Created tested and helper nodes
    auto node_options = createNodeOptions();
    tested_node = std::make_shared<EmergencyHandlerNode>(node_options);
    test_node = rclcpp::Node::make_shared("test_node");

    // Add nodes to executor
    executor.reset(new rclcpp::executors::SingleThreadedExecutor);
    executor->add_node(tested_node);
    executor->add_node(test_node);

    // Publishers
    pub_autoware_state = test_node->create_publisher<AutowareState>("input/autoware_state", 1);
    pub_driving_capability = test_node->create_publisher<DrivingCapability>(
      "input/driving_capability", 1);
    pub_vehicle_control_command = test_node->create_publisher<VehicleControlCommand>(
      "input/prev_control_command", 1);
    pub_vehicle_state_report = test_node->create_publisher<VehicleStateReport>(
      "input/vehicle_state_report", 1);
    pub_odometry = test_node->create_publisher<VehicleOdometry>("input/odometry", 1);

    // Register spies
    control_command_spy = std::make_shared<Spy<VehicleControlCommand>>(
      test_node, executor, "output/control_command");
    state_command_spy = std::make_shared<Spy<VehicleStateCommand>>(
      test_node, executor, "output/state_command");
    emergency_mode_spy = std::make_shared<Spy<EmergencyMode>>(
      test_node, executor, "output/is_emergency");
    hazard_status_spy = std::make_shared<Spy<HazardStatusStamped>>(
      test_node, executor, "output/hazard_status");
    diagnostic_spy = std::make_shared<Spy<DiagnosticArray>>(
      test_node, executor, "output/diagnostics_err");
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:

  rclcpp::NodeOptions createNodeOptions()
  {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("update_rate", 10.0f);
    node_options.append_parameter_override("timeout_driving_capability", 0.5f);
    node_options.append_parameter_override("data_ready_timeout", 0.1f);
    node_options.append_parameter_override("use_emergency_hold", false);
    node_options.append_parameter_override("emergency_hazard_level", 2);
    return node_options;
  }

  std::shared_ptr<EmergencyHandlerNode> tested_node;
  rclcpp::Node::SharedPtr test_node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

  rclcpp::Publisher<AutowareState>::SharedPtr pub_autoware_state;
  rclcpp::Publisher<DrivingCapability>::SharedPtr pub_driving_capability;
  rclcpp::Publisher<VehicleControlCommand>::SharedPtr pub_vehicle_control_command;
  rclcpp::Publisher<VehicleStateReport>::SharedPtr pub_vehicle_state_report;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr pub_odometry;

  std::shared_ptr<Spy<VehicleControlCommand>> control_command_spy;
  std::shared_ptr<Spy<VehicleStateCommand>> state_command_spy;
  std::shared_ptr<Spy<EmergencyMode>> emergency_mode_spy;
  std::shared_ptr<Spy<HazardStatusStamped>> hazard_status_spy;
  std::shared_ptr<Spy<DiagnosticArray>> diagnostic_spy;
};

TEST_F(EmergencyHandlerNodeTest, create_destroy)
{
}

TEST_F(EmergencyHandlerNodeTest, no_any_input_data_after_initialization)
{
  auto msg = diagnostic_spy->expectMsg();
  ASSERT_EQ(msg.status.size(), 1);
  EXPECT_EQ(msg.status[0].hardware_id, "emergency_handler");
  EXPECT_EQ(msg.status[0].name, "input_data_timeout");
  EXPECT_EQ(msg.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}