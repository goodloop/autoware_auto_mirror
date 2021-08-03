// Copyright 2021 Robotec.ai
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

#include "autoware_state_monitor/autoware_state_monitor_node.hpp"

#include <memory>

#include "gtest/gtest.h"

#include "test_utils.hpp"

using autoware::state_monitor::AutowareStateMonitorNode;

class AutowareStateMonitorNodeTest : public ::testing::Test
{
public:
  AutowareStateMonitorNodeTest()
  {
  }

  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("update_rate", 10.0f);
    node_options.append_parameter_override("th_arrived_distance_m", 1.0f);
    node_options.append_parameter_override("th_stopped_time_sec", 1.0f);
    node_options.append_parameter_override("th_stopped_velocity_mps", 1.0f);
    node_options.append_parameter_override("wait_time_after_initializing", 1.0f);
    node_options.append_parameter_override("wait_time_after_planning", 1.0f);
    node_options.append_parameter_override("wait_time_after_arrived_goal", 2.0f);

    node = std::make_shared<AutowareStateMonitorNode>(node_options);
  }

  ~AutowareStateMonitorNodeTest() = default;

protected:
  std::shared_ptr<AutowareStateMonitorNode> node;
};

TEST_F(AutowareStateMonitorNodeTest, create_destroy)
{
}
