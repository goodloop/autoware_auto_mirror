// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
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
#include "recordreplay_planner_node/recordreplay_planner_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace motion
{
namespace planning
{
namespace recordreplay_planner_node
{
RecordReplayPlannerNode::RecordReplayPlannerNode(const std::string & name, const std::string & ns)
: Node{name, ns}
{
  // TODO(s.me) get topics from parameters
  const auto ego_topic = declare_parameter("ego_topic").get<std::string>();
  const auto trajectory_topic = declare_parameter("trajectory_topic").get<std::string>();
  const auto bounding_boxes_topic = declare_parameter("bounding_boxes_topic").get<std::string>();
  const auto heading_weight = static_cast<double>(declare_parameter("heading_weight").get<float>());

  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter("vehicle.cg_to_front_m").get<float>()),
    static_cast<Real>(declare_parameter("vehicle.cg_to_rear_m").get<float>()),
    static_cast<Real>(declare_parameter("vehicle.front_corner_stiffness").get<float>()),
    static_cast<Real>(declare_parameter("vehicle.rear_corner_stiffness").get<float>()),
    static_cast<Real>(declare_parameter("vehicle.mass_kg").get<float>()),
    static_cast<Real>(declare_parameter("vehicle.yaw_inertia_kgm2").get<float>()),
    static_cast<Real>(declare_parameter("vehicle.width_m").get<float>()),
    static_cast<Real>(declare_parameter("vehicle.front_overhang_m").get<float>()),
    static_cast<Real>(declare_parameter("vehicle.rear_overhang_m").get<float>())
  };
  init(ego_topic, trajectory_topic, bounding_boxes_topic, vehicle_param, heading_weight);
}
////////////////////////////////////////////////////////////////////////////////
RecordReplayPlannerNode::RecordReplayPlannerNode(
  const std::string & name,
  const std::string & ns,
  const std::string & ego_topic,
  const std::string & trajectory_topic,
  const std::string & bounding_boxes_topic,
  const VehicleConfig & vehicle_param,
  const double heading_weight)
: Node{name, ns}
{
  init(ego_topic, trajectory_topic, bounding_boxes_topic, vehicle_param, heading_weight);
}

void RecordReplayPlannerNode::init(
  const std::string & ego_topic,
  const std::string & trajectory_topic,
  const std::string & bounding_boxes_topic,
  const VehicleConfig & vehicle_param,
  const double heading_weight
)
{
  using rclcpp::QoS;

  // Set up action for control of recording and replaying
  m_recordserver = rclcpp_action::create_server<RecordTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "recordtrajectory",
    [this](auto uuid, auto goal) {return this->record_handle_goal(uuid, goal);},
    [this](auto goal_handle) {return this->record_handle_cancel(goal_handle);},
    [this](auto goal_handle) {return this->record_handle_accepted(goal_handle);});

  m_replayserver = rclcpp_action::create_server<ReplayTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "replaytrajectory",
    [this](auto uuid, auto goal) {return this->replay_handle_goal(uuid, goal);},
    [this](auto goal_handle) {return this->replay_handle_cancel(goal_handle);},
    [this](auto goal_handle) {return this->replay_handle_accepted(goal_handle);});

  // Set up subscribers for the actual recording
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  m_ego_sub = create_subscription<State>(ego_topic, QoS{10},
      [this](const State::SharedPtr msg) {on_ego(msg);}, SubAllocT{});

  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  m_boundingbox_sub = create_subscription<BoundingBoxArray>(bounding_boxes_topic,
      QoS{10}.transient_local(),
      [this](const BoundingBoxArray::SharedPtr msg) {on_bounding_box(msg);}, SubAllocT{});

  // Set up publishers
  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
  m_trajectory_pub =
    create_publisher<Trajectory>(trajectory_topic, QoS{10}.transient_local(), PubAllocT{});

  // Create and set a planner object that we'll talk to
  m_planner = std::make_unique<recordreplay_planner::RecordReplayPlanner>(vehicle_param);
  m_planner->set_heading_weight(heading_weight);
}


void RecordReplayPlannerNode::on_ego(const State::SharedPtr & msg)
{
  if (m_planner->is_recording()) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Recording ego position");
    m_planner->record_state(*msg);
  }

  if (m_planner->is_replaying()) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Replaying recorded ego postion as trajectory");
    const auto & traj = m_planner->plan(*msg);
    m_trajectory_pub->publish(traj);
  }
}

void RecordReplayPlannerNode::on_bounding_box(const BoundingBoxArray::SharedPtr & msg)
{
  // Update most recent bounding box internally
  m_planner->update_bounding_boxes(*msg);
}

rclcpp_action::GoalResponse RecordReplayPlannerNode::record_handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const RecordTrajectory::Goal> goal)
{
  (void)goal;
  (void)uuid;
  if (m_planner->is_recording()) {
    // Can't start recording if we already are
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RecordReplayPlannerNode::record_handle_cancel(
  const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle)
{
  (void)goal_handle;
  if (m_planner->is_recording()) {
    m_planner->stop_recording();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RecordReplayPlannerNode::record_handle_accepted(
  const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle)
{
  (void)goal_handle;

  // Store the goal handle otherwise the action gets canceled immediately
  m_recordgoalhandle = goal_handle;
  m_planner->start_recording();
}

rclcpp_action::GoalResponse RecordReplayPlannerNode::replay_handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const ReplayTrajectory::Goal> goal)
{
  (void)goal;
  (void)uuid;
  if (m_planner->is_replaying()) {
    // Can't start replaying if we already are
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RecordReplayPlannerNode::replay_handle_cancel(
  const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle)
{
  (void)goal_handle;
  if (m_planner->is_replaying()) {
    m_planner->stop_replaying();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RecordReplayPlannerNode::replay_handle_accepted(
  const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle)
{
  (void)goal_handle;

  // Store the goal handle otherwise the action gets canceled immediately
  m_replaygoalhandle = goal_handle;

  // Start the replaying process
  m_planner->start_replaying();
}
}  // namespace recordreplay_planner_node
}  // namespace planning
}  // namespace motion
