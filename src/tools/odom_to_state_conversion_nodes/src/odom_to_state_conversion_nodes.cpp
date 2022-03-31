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

#include <string>
#include <memory>

#include "odom_to_state_conversion_nodes/odom_to_state_conversion_nodes.hpp"

using std::placeholders::_1;

namespace odom_to_state_conversion_nodes
{

OdomToStateConversionNode::OdomToStateConversionNode(const rclcpp::NodeOptions & options)
:  Node("odom_to_state_conversion_node", options)
{
  m_state_pub =
    create_publisher<VehicleKinematicState>(
    declare_parameter("state_topic", "vehicle_state"), rclcpp::QoS{10});
  m_vehicle_odom_sub =
    create_subscription<VehicleOdometry>(
    declare_parameter("vehicle_odom_topic", "odometry"), rclcpp::QoS{10},
    std::bind(&OdomToStateConversionNode::on_vehicle_odom, this, _1));
  m_odom_sub =
    create_subscription<Odometry>(
    declare_parameter("odom_topic", "odom"), rclcpp::QoS{10},
    std::bind(&OdomToStateConversionNode::on_odom, this, _1));

  m_use_vehicle_odom_for_velocity = declare_parameter(
    "use_vehicle_odom_for_velocity", true);
}


void OdomToStateConversionNode::on_odom(const Odometry::SharedPtr msg)
{
  // Only publish when VehicleOdometry has been received
  if (m_state) {
    m_state->header = msg->header;
    m_state->state.pose = msg->pose.pose;
    m_state->state.heading_rate_rps = static_cast<float32_t>(msg->twist.twist.angular.z);
    m_state->state.lateral_velocity_mps = static_cast<float32_t>(msg->twist.twist.linear.y);
    if (!m_use_vehicle_odom_for_velocity) {
      m_state->state.longitudinal_velocity_mps = static_cast<float32_t>(msg->twist.twist.linear.x);
    }
    if (rclcpp::ok()) {
      m_state_pub->publish(*m_state);
    }
  }
}

void OdomToStateConversionNode::on_vehicle_odom(const VehicleOdometry::SharedPtr msg)
{
  if (!m_state) {
    m_state = std::make_unique<VehicleKinematicState>();
  }
  m_state->state.front_wheel_angle_rad = msg->front_wheel_angle_rad;
  m_state->state.rear_wheel_angle_rad = msg->rear_wheel_angle_rad;
  if (m_use_vehicle_odom_for_velocity) {
    m_state->state.longitudinal_velocity_mps = msg->velocity_mps;
  }
}
}  // namespace odom_to_state_conversion_nodes

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(odom_to_state_conversion_nodes::OdomToStateConversionNode)
