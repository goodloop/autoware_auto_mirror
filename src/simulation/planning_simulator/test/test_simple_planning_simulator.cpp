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

#include <memory>
#include "gtest/gtest.h"

#include "simple_planning_simulator/simple_planning_simulator_core.hpp"

using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleKinematicState;
using geometry_msgs::msg::PoseWithCovarianceStamped;

using simulation::simple_planning_simulator::SimplePlanningSimulator;


class PubSubNode : public rclcpp::Node
{
public:
  PubSubNode()
  : Node{"test_simple_planning_simulator_pubsub"}
  {
    kinematic_state_sub_ = create_subscription<VehicleKinematicState>(
      "output/kinematic_state", rclcpp::QoS{1},
      [this](const VehicleKinematicState::SharedPtr msg) {
        std::cerr << "received kinematic_state!!!" << std::endl;
        current_state_ = msg;
      });
    pub_control_command_ = create_publisher<VehicleControlCommand>(
      "input/vehicle_control_command",
      rclcpp::QoS{1});
    pub_initialpose_ = create_publisher<PoseWithCovarianceStamped>(
      "/initialpose",
      rclcpp::QoS{1});
  }

  rclcpp::Publisher<VehicleControlCommand>::SharedPtr pub_control_command_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;
  rclcpp::Subscription<VehicleKinematicState>::SharedPtr kinematic_state_sub_;

  VehicleKinematicState::SharedPtr current_state_;
};

VehicleControlCommand cmdGen(
  const builtin_interfaces::msg::Time & t, float steer, float vel, float acc)
{
  VehicleControlCommand cmd;
  cmd.stamp = t;
  cmd.front_wheel_angle_rad = steer;
  cmd.velocity_mps = vel;
  cmd.long_accel_mps2 = acc;
  return cmd;
}

void resetInitialpose(rclcpp::Node::SharedPtr sim_node, std::shared_ptr<PubSubNode> pub_sub_node)
{
  PoseWithCovarianceStamped p;
  p.header.frame_id = "odom";
  p.header.stamp = sim_node->now();
  p.pose.pose.orientation.w = 1.0;  // yaw = 0
  for (int i = 0; i < 10; ++i) {
    pub_sub_node->pub_initialpose_->publish(p);
    rclcpp::spin_some(sim_node);
    rclcpp::spin_some(pub_sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }
}

void sendCommand(
  const VehicleControlCommand & cmd, rclcpp::Node::SharedPtr sim_node,
  std::shared_ptr<PubSubNode> pub_sub_node)
{
  for (int i = 0; i < 100; ++i) {
    pub_sub_node->pub_control_command_->publish(cmd);
    rclcpp::spin_some(sim_node);
    rclcpp::spin_some(pub_sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }
}


void isOnForward(const VehicleKinematicState & state)
{
  float forward_thr = 3.0f;
  EXPECT_GT(state.state.x, forward_thr);
}

void isOnBackward(const VehicleKinematicState & state)
{
  float backward_thr = -3.0f;
  EXPECT_LT(state.state.x, backward_thr);
}

void isOnForwardRight(const VehicleKinematicState & state)
{
  float forward_thr = 3.0f;
  float right_thr = 0.1f;
  EXPECT_GT(state.state.x, forward_thr);
  EXPECT_GT(state.state.y, right_thr);
}

void isOnBackwardLeft(const VehicleKinematicState & state)
{
  float backward_thr = -3.0f;
  float left_thr = -0.1f;
  EXPECT_LT(state.state.x, backward_thr);
  EXPECT_LT(state.state.y, left_thr);
}

// Send a control command and run the simulation.
// Then check if the vehicle is moving in the desired direction.
TEST(test_simple_planning_simulator, test_moving)
{
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("initialize_source", "INITIAL_POSE_TOPIC");
  node_options.append_parameter_override("vehicle_model_type", "IDEAL_STEER_VEL");
  const auto sim_node = std::make_shared<SimplePlanningSimulator>(
    "simple_planning_simulator", node_options);

  const auto pub_sub_node = std::make_shared<PubSubNode>();

  const float target_vel = 5.0f;
  const float target_acc = 3.0f;
  const float target_steer = 0.2f;

  auto _resetInitialpose = [&]() {resetInitialpose(sim_node, pub_sub_node);};
  auto _sendCommand = [&](const VehicleControlCommand & _cmd) {
      sendCommand(_cmd, sim_node, pub_sub_node);
    };

  // go forward
  _resetInitialpose();
  _sendCommand(cmdGen(pub_sub_node->now(), 0.0f, target_vel, target_acc));
  isOnForward(*(pub_sub_node->current_state_));

  // go backward
  _resetInitialpose();
  _sendCommand(cmdGen(pub_sub_node->now(), 0.0f, -target_vel, -target_acc));
  isOnBackward(*(pub_sub_node->current_state_));


  // go forward right
  _resetInitialpose();
  _sendCommand(cmdGen(pub_sub_node->now(), target_steer, target_vel, target_acc));
  isOnForwardRight(*(pub_sub_node->current_state_));

  // go backward left
  _resetInitialpose();
  _sendCommand(cmdGen(pub_sub_node->now(), -target_steer, -target_vel, -target_acc));
  isOnBackwardLeft(*(pub_sub_node->current_state_));

  rclcpp::shutdown();
}
