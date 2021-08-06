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

using namespace std::chrono_literals;

using autoware_auto_msgs::msg::VehicleStateReport;
using autoware_auto_msgs::msg::VehicleOdometry;

using autoware::emergency_handler::EmergencyHandlerNode;

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
    test_helper_node = rclcpp::Node::make_shared("test_helper_node");

    // Add nodes to executor
    executor.reset(new rclcpp::executors::SingleThreadedExecutor);
    executor->add_node(tested_node);
    executor->add_node(test_helper_node);

    // Publishers
    pub_vehicle_state = test_helper_node->create_publisher<VehicleStateReport>(
      "input/vehicle_state_report", 1);
    pub_odometry = test_helper_node->create_publisher<VehicleOdometry>("input/odometry", 1);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

  rclcpp::NodeOptions createNodeOptions()
  {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("update_rate", 10.0f);
    node_options.append_parameter_override("timeout_driving_capability", 0.5f);
    node_options.append_parameter_override("data_ready_timeout", 30.0f);
    node_options.append_parameter_override("use_emergency_hold", false);
    node_options.append_parameter_override("emergency_hazard_level", 2);
    return node_options;
  }

protected:
  std::shared_ptr<EmergencyHandlerNode> tested_node;
  rclcpp::Node::SharedPtr test_helper_node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

  rclcpp::Publisher<VehicleStateReport>::SharedPtr pub_vehicle_state;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr pub_odometry;
};

TEST_F(EmergencyHandlerNodeTest, create_destroy)
{
}
