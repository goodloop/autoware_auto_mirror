// Copyright 2020 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
#include <common/types.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "lgsvl_interface/lgsvl_interface.hpp"

using autoware::common::types::bool8_t;

namespace lgsvl_interface
{

LgsvlInterface::LgsvlInterface(
  rclcpp::Node & node,
  const std::string & sim_cmd_topic,
  const std::string & sim_state_cmd_topic,
  const std::string & sim_state_report_topic,
  const std::string & sim_odom_topic,
  const std::string & kinematic_state_topic,
  Table1D && throttle_table,
  Table1D && brake_table,
  Table1D && steer_table)
: m_throttle_table{throttle_table},
  m_brake_table{brake_table},
  m_steer_table{steer_table}
{
  const auto check = [](const auto value, const auto ref) -> bool8_t {
      return std::fabs(value - ref) > std::numeric_limits<decltype(value)>::epsilon();
    };
  // check throttle table
  if (check(m_throttle_table.domain().front(), 0.0)) {
    throw std::domain_error{"Throttle table domain must be [0, ...)"};
  }
  if (check(m_throttle_table.range().front(), 0.0) ||
    check(m_throttle_table.range().back(), 100.0))
  {
    throw std::domain_error{"Throttle table range must go from 0 to 100"};
  }
  for (const auto val : m_throttle_table.range()) {
    if (val < 0.0) {
      throw std::domain_error{"Throttle table must map to nonnegative accelerations"};
    }
  }
  // Check brake table
  if (check(m_brake_table.domain().back(), 0.0)) {
    throw std::domain_error{"Brake table domain must be [..., 0)"};
  }
  if (check(m_brake_table.range().front(), 100.0) || check(m_brake_table.range().back(), 0.0)) {
    throw std::domain_error{"Brake table must go from 100 to 0"};
  }
  for (const auto val : m_brake_table.domain()) {
    if (val > 0.0) {
      throw std::domain_error{"Brake table must map negative accelerations to 0-100 values"};
    }
  }
  // Check steer table
  if (check(m_steer_table.range().front(), -100.0) || check(m_steer_table.range().back(), 100.0)) {
    throw std::domain_error{"Steer table must go from -100 to 100"};
  }
  if ((m_steer_table.domain().front() >= 0.0) ||  // Should be negative...
    (m_steer_table.domain().back() <= 0.0) ||  // to positive...
    check(m_steer_table.domain().back(), -m_steer_table.domain().front()))  // with symmetry
  {
    // Warn if steer domain is not equally straddling zero: could be right, but maybe not
    RCLCPP_WARN(node.get_logger(), "Steer table domain is not symmetric across zero. Is this ok?");
  }

  // Make publishers
  m_cmd_pub = node.create_publisher<autoware_auto_msgs::msg::RawControlCommand>(
    sim_cmd_topic, rclcpp::QoS{10});
  m_state_pub = node.create_publisher<autoware_auto_msgs::msg::VehicleStateCommand>(
    sim_state_cmd_topic, rclcpp::QoS{10});
  // Make subscribers
  if (!sim_odom_topic.empty() && ("null" != sim_odom_topic)) {
    m_odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(sim_odom_topic, rclcpp::QoS{10},
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {on_odometry(*msg);});
    // Ground truth state/transform publishers only work if there's a ground truth input
    m_kinematic_state_pub = node.create_publisher<autoware_auto_msgs::msg::VehicleKinematicState>(
      kinematic_state_topic, rclcpp::QoS{10});
    m_tf_pub = node.create_publisher<tf2_msgs::msg::TFMessage>("tf", rclcpp::QoS{10});  // standard
  }
  m_state_sub = node.create_subscription<autoware_auto_msgs::msg::VehicleStateReport>(
    sim_state_report_topic,
    rclcpp::QoS{10},
    [this](autoware_auto_msgs::msg::VehicleStateReport::SharedPtr msg) {state_report() = *msg;});
  // TODO(c.ho) real odometry
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  // Not implemented: API is not needed since everything is handled by subscription callbacks
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::send_state_command(const autoware_auto_msgs::msg::VehicleStateCommand & msg)
{

  // in autoware_auto_msgs::msg::VehicleStateCommand 1 is drive, 2 is reverse,
  // in lgsvl 0 is drive and 1 is reverse
  auto msg_corrected = msg;
  msg_corrected.gear = msg.gear == 2 ? 1 : 0;
  m_state_pub->publish(msg_corrected);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::send_control_command(
  const autoware_auto_msgs::msg::VehicleControlCommand & msg)
{
  autoware_auto_msgs::msg::RawControlCommand raw_msg;
  raw_msg.stamp = msg.stamp;
  raw_msg.throttle = 0;
  raw_msg.brake = 0;
  if (msg.long_accel_mps2 >= decltype(msg.long_accel_mps2) {}) {
    // TODO(c.ho)  cast to double...
    raw_msg.throttle =
      static_cast<decltype(raw_msg.throttle)>(m_throttle_table.lookup(msg.long_accel_mps2));
  } else {
    raw_msg.brake =
      static_cast<decltype(raw_msg.brake)>(m_brake_table.lookup(msg.long_accel_mps2));
  }
  raw_msg.front_steer =
    static_cast<decltype(raw_msg.front_steer)>(m_steer_table.lookup(msg.front_wheel_angle_rad));
  raw_msg.rear_steer = 0;

  return send_control_command(raw_msg);
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::send_control_command(const autoware_auto_msgs::msg::RawControlCommand & msg)
{
  // Front steer semantically is z up, ccw positive, but LGSVL thinks its the opposite
  auto msg_corrected = msg;
  msg_corrected.front_steer = -msg.front_steer;
  m_cmd_pub->publish(msg_corrected);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void LgsvlInterface::on_odometry(const nav_msgs::msg::Odometry & msg)
{
  if (!m_odom_set) {
    m_odom_zero.x = msg.pose.pose.position.x;
    m_odom_zero.y = msg.pose.pose.position.y;
    m_odom_zero.z = msg.pose.pose.position.z;
    m_odom_set = true;
  }
  decltype(msg.pose.pose.orientation) q{};
  {
    // Convert from LHS system to RHS system: Y forward, Z up
    tf2::Quaternion q_rhs{
      -msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.x,
      -msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.w};
    // rotate +90 degrees around +z axis to get X forward
    tf2::Quaternion q90{};
    q90.setRPY(0.0, 0.0, 90.0 * (M_PI / 180.0));
    const auto q_x_forward = q90 * q_rhs;
    q.x = q_x_forward.getX();
    q.y = q_x_forward.getY();
    q.z = q_x_forward.getZ();
    q.w = q_x_forward.getW();
  }
  const auto px = msg.pose.pose.position.x - m_odom_zero.x;
  const auto py = msg.pose.pose.position.y - m_odom_zero.y;
  const auto pz = msg.pose.pose.position.z - m_odom_zero.z;
  {
    autoware_auto_msgs::msg::VehicleKinematicState vse{};
    vse.header = msg.header;
    vse.state.x = static_cast<decltype(vse.state.x)>(px);
    vse.state.y = static_cast<decltype(vse.state.y)>(py);
    {
      const auto inv_mag = 1.0 / std::sqrt((q.z * q.z) + (q.w * q.w));
      vse.state.heading.real = static_cast<decltype(vse.state.heading.real)>(q.w * inv_mag);
      vse.state.heading.imag = static_cast<decltype(vse.state.heading.imag)>(q.z * inv_mag);
      // LGSVL is y-up and left handed
    }
    vse.state.longitudinal_velocity_mps =
      static_cast<decltype(vse.state.longitudinal_velocity_mps)>(msg.twist.twist.linear.x);
    vse.state.lateral_velocity_mps =
      static_cast<decltype(vse.state.lateral_velocity_mps)>(msg.twist.twist.linear.y);
    vse.state.acceleration_mps2 = 0.0F;
    vse.state.heading_rate_rps =
      static_cast<decltype(vse.state.heading_rate_rps)>(msg.twist.twist.angular.z);
    vse.state.front_wheel_angle_rad = 0.0F;
    vse.state.rear_wheel_angle_rad = 0.0F;

    m_kinematic_state_pub->publish(vse);
  }
  {
    geometry_msgs::msg::TransformStamped tf{};
    tf.header = msg.header;
    tf.child_frame_id = msg.child_frame_id;
    tf.transform.translation.x = px;
    tf.transform.translation.y = py;
    tf.transform.translation.z = pz;
    tf.transform.rotation = q;

    tf2_msgs::msg::TFMessage tf_msg{};
    tf_msg.transforms.emplace_back(std::move(tf));
    m_tf_pub->publish(tf_msg);
  }
}

}  // namespace lgsvl_interface
