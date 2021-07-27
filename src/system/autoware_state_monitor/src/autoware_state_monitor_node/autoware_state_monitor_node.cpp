// Copyright 2021 Robotec.ai
// Copyright 2020 Tier IV, Inc.
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

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace
{
template<class Config>
std::vector<Config> getConfigs(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface,
  const std::string & config_namespace)
{
  std::string names_key = config_namespace + ".names";
  interface->declare_parameter(names_key);
  std::vector<std::string> config_names = interface->get_parameter(names_key).as_string_array();

  std::vector<Config> configs;
  configs.reserve(config_names.size());

  for (auto config_name : config_names) {
    configs.emplace_back(interface, config_namespace + ".configs." + config_name, config_name);
  }

  return configs;
}

geometry_msgs::msg::PoseStamped::SharedPtr getCurrentPose(const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    return nullptr;
  }

  auto p = std::make_shared<geometry_msgs::msg::PoseStamped>();
  p->header = tf_current_pose.header;
  p->pose.orientation = tf_current_pose.transform.rotation;
  p->pose.position.x = tf_current_pose.transform.translation.x;
  p->pose.position.y = tf_current_pose.transform.translation.y;
  p->pose.position.z = tf_current_pose.transform.translation.z;

  return p;
}

std::string getStateMessage(const AutowareState & state)
{
  if (state == AutowareState::InitializingVehicle) {
    return "Please wait for a while. If the current pose is not estimated automatically, please "
           "set it manually.";
  }

  if (state == AutowareState::WaitingForRoute) {
    return "Please send a route.";
  }

  if (state == AutowareState::Planning) {
    return "Please wait for a while.";
  }

  if (state == AutowareState::WaitingForEngage) {
    return "Please set engage.";
  }

  if (state == AutowareState::Driving) {
    return "Under autonomous driving. Have fun!";
  }

  if (state == AutowareState::ArrivedGoal) {
    return "Autonomous driving has completed. Thank you!";
  }

  if (state == AutowareState::Emergency) {
    return "Emergency! Please recover the system.";
  }

  if (state == AutowareState::Finalizing) {
    return "Finalizing Autoware...";
  }

  throw std::runtime_error("invalid state");
}

}  // namespace

void AutowareStateMonitorNode::onAutowareEngage(
  const autoware_auto_msgs::msg::Engage::ConstSharedPtr msg)
{
  state_input_.engage = msg;
}
void AutowareStateMonitorNode::onVehicleStateReport(
  const autoware_auto_msgs::msg::VehicleStateReport::ConstSharedPtr msg)
{
  state_input_.vehicle_state_report = msg;
}

void AutowareStateMonitorNode::onIsEmergency(
  const autoware_auto_msgs::msg::EmergencyMode::ConstSharedPtr msg)
{
  state_input_.emergency_mode = msg;
}

void AutowareStateMonitorNode::onRoute(
  const autoware_auto_msgs::msg::HADMapRoute::ConstSharedPtr msg)
{
  state_input_.route = msg;

  // Get goal pose
  {
    auto p = std::make_shared<autoware_auto_msgs::msg::RoutePoint>();
    *p = msg->goal_point;
    state_input_.goal_pose = autoware_auto_msgs::msg::RoutePoint::ConstSharedPtr(p);
  }
}

void AutowareStateMonitorNode::onOdometry(
  const autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr msg)
{
  state_input_.odometry = msg;

  state_input_.odometry_buffer.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff = rclcpp::Time(msg->stamp) -
      rclcpp::Time(state_input_.odometry_buffer.front()->stamp);

    if (time_diff.seconds() < state_param_.th_stopped_time_sec) {
      break;
    }

    state_input_.odometry_buffer.pop_front();
  }
}

bool AutowareStateMonitorNode::onShutdownService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request_header;
  state_input_.is_finalizing = true;

  const auto t_start = this->get_clock()->now();
  constexpr double timeout = 3.0;
  while (rclcpp::ok()) {
    if (state_machine_->getCurrentState() == AutowareState::Finalizing) {
      response->success = true;
      response->message = "Shutdown Autoware.";
      return true;
    }

    if ((this->get_clock()->now() - t_start).seconds() > timeout) {
      response->success = false;
      response->message = "Shutdown timeout.";
      return true;
    }

    rclcpp::Rate(10.0).sleep();
  }

  response->success = false;
  response->message = "Shutdown failure.";
  return true;
}

void AutowareStateMonitorNode::onTimer()
{
  // Prepare state input
  state_input_.current_pose = getCurrentPose(tf_buffer_);
  state_input_.current_time = this->now();

  // Update state
  const auto prev_autoware_state = state_machine_->getCurrentState();
  const auto autoware_state = state_machine_->updateState(state_input_);

  if (autoware_state != prev_autoware_state) {
    RCLCPP_INFO(
      this->get_logger(), "state changed: %s -> %s", toString(prev_autoware_state).c_str(),
      toString(autoware_state).c_str());
  }

  // Publish state message
  {
    autoware_auto_msgs::msg::AutowareState autoware_state_msg;
    autoware_state_msg.state = toString(autoware_state);

    // Add messages line by line
    std::ostringstream oss;

    oss << getStateMessage(autoware_state) << std::endl;

    for (const auto & msg : state_machine_->getMessages()) {
      oss << msg << std::endl;
    }

    autoware_state_msg.msg = oss.str();

    pub_autoware_state_->publish(autoware_state_msg);
  }

  // Publish diag message
  updater_.force_update();
}

bool AutowareStateMonitorNode::isEngaged()
{
  if (!state_input_.engage) {
    return false;
  }

  return state_input_.engage->engage;
}

AutowareStateMonitorNode::AutowareStateMonitorNode()
: Node("autoware_state_monitor"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  updater_(this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  // Parameter
  update_rate_ = this->declare_parameter("update_rate", 10.0);

  // Parameter for StateMachine
  state_param_.th_arrived_distance_m = this->declare_parameter("th_arrived_distance_m", 1.0);
  state_param_.th_stopped_time_sec = this->declare_parameter("th_stopped_time_sec", 1.0);
  state_param_.th_stopped_velocity_mps = this->declare_parameter("th_stopped_velocity_mps", 0.01);

  // State Machine
  state_machine_ = std::make_shared<StateMachine>(state_param_);

  // Callback Groups
  callback_group_subscribers_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_services_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = callback_group_subscribers_;

  // Subscriber
  sub_engage_ = this->create_subscription<autoware_auto_msgs::msg::Engage>(
    "input/engage", 1,
    std::bind(&AutowareStateMonitorNode::onAutowareEngage, this, _1), subscriber_option);
  sub_vehicle_state_report_ =
    this->create_subscription<autoware_auto_msgs::msg::VehicleStateReport>(
    "input/vehicle_state_report", 1,
    std::bind(&AutowareStateMonitorNode::onVehicleStateReport, this, _1), subscriber_option);
  sub_is_emergency_ = this->create_subscription<autoware_auto_msgs::msg::EmergencyMode>(
    "input/is_emergency", 1,
    std::bind(&AutowareStateMonitorNode::onIsEmergency, this, _1), subscriber_option);
  sub_route_ = this->create_subscription<autoware_auto_msgs::msg::HADMapRoute>(
    "input/route", 1,
    std::bind(&AutowareStateMonitorNode::onRoute, this, _1), subscriber_option);
  sub_odometry_ = this->create_subscription<autoware_auto_msgs::msg::VehicleOdometry>(
    "input/odometry", 100,
    std::bind(&AutowareStateMonitorNode::onOdometry, this, _1), subscriber_option);

  // Service
  srv_shutdown_ = this->create_service<std_srvs::srv::Trigger>(
    "service/shutdown",
    std::bind(&AutowareStateMonitorNode::onShutdownService, this, _1, _2, _3),
    rmw_qos_profile_services_default, callback_group_services_);

  // Publisher
  pub_autoware_state_ =
    this->create_publisher<autoware_auto_msgs::msg::AutowareState>("output/autoware_state", 1);

  // Timer
  auto timer_callback = std::bind(&AutowareStateMonitorNode::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, callback_group_subscribers_);
}
