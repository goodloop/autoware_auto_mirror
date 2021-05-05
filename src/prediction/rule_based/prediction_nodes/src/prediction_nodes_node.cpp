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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "prediction_nodes/prediction_nodes_node.hpp"

#include <had_map_utils/had_map_conversion.hpp>
#include <time_utils/time_utils.hpp>

#include <functional>
#include <memory>

namespace autoware
{
namespace prediction_nodes
{
using std::placeholders::_1;

PredictionNodesNode::PredictionNodesNode(const rclcpp::NodeOptions & options)
: Node("prediction_nodes", options), verbose(true),
  // TODO(frederik.beaujean) Add topics to design doc
#if MSGS_UPDATED
  m_predicted_objects_pub{create_publisher<PredictedMsgT>("predicted_objects", rclcpp::QoS{10})},
#endif
#if TRAFFIC_LIGHTS
  m_traffic_signal_sub{create_subscription<TrafficSignalT>(
      "tracked_signals",
      rclcpp::QoS{10},
      std::bind(&PredictionNodesNode::on_traffic_signals, this, _1))}
#endif
  m_tracked_dynamic_objects_sub{
    create_subscription<TrackedMsgT>(
      "tracked_objects", rclcpp::QoS{10},
      std::bind(&PredictionNodesNode::on_tracked_objects, this,
      _1))},
  m_map_client{create_client<HADMapService>("HAD_Map_Service")},
  m_tf_listener(m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false),
  m_route_sub{this->create_subscription<RouteMsgT>(
      "route", rclcpp::QoS{10},
      std::bind(&PredictionNodesNode::on_route, this, _1))}
{
  // send all asynchronous requests
//  request_map();

  // wait for results
//  wait_for_map();
}

void PredictionNodesNode::on_map_response(rclcpp::Client<HADMapService>::SharedFuture future)
{
  autoware::common::had_map_utils::fromBinaryMsg(future.get()->map, m_map);

  RCLCPP_INFO(get_logger(), "Received map");
}

void PREDICTION_NODES_LOCAL PredictionNodesNode::on_route(RouteMsgT::ConstSharedPtr msg)
{
  m_route = msg;
}

void PredictionNodesNode::on_tracked_objects(TrackedMsgT::ConstSharedPtr msg)
{
#if MSGS_UPDATED
  PredictedMsgT predicted_objects = from_tracked(*msg);
  predict_all_stationary(predicted_objects);
  m_predicted_objects_pub->publish(predicted_objects);
#endif
}

#if TRAFFIC_LIGHTS
void PredictionNodesNode::on_traffic_signals(TrafficSignalT::ConstSharedPtr /*msg*/)
{}
#endif

void PredictionNodesNode::request_map()
{
  using namespace std::literals::chrono_literals;

  // TODO(frederik.beaujean) Investigate if this is the best way to wait for dependencies. Can we
  // use lifecycles to determine if everything is up and running? Is there are "wait for matched"
  // function in ROS2?
  while (!m_map_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for map server.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for map service...");
  }

  // For now request full map, later select subset
  auto request = std::make_shared<HADMapService::Request>();
  request->requested_primitives.push_back(HADMapService::Request::FULL_MAP);

  m_map.reset();

  // TODO(frederik.beaujean): If synchronized service request becomes available,
  // replace it with synchronized implementation
  auto result =
    m_map_client->async_send_request(
    request,
    std::bind(&PredictionNodesNode::on_map_response, this, _1));
}

void PredictionNodesNode::wait_for_map(std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  while (!m_map) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - start > timeout) {
      throw std::runtime_error("Prediction timed out waiting for map");
    }
  }
}

autoware_auto_msgs::msg::PredictedObjects from_tracked(
  const autoware_auto_msgs::msg::TrackedObjects & tracked)
{
  // safest initialization in cases new members are added to TrackedMsgT
  autoware_auto_msgs::msg::PredictedObjects predicted {rosidl_runtime_cpp::MessageInitialization::
    ZERO};
  predicted.header = tracked.header;

  predicted.objects.resize(tracked.objects.size());
  std::transform(
    tracked.objects.begin(), tracked.objects.end(),
    predicted.objects.begin(),
    [](const autoware_auto_msgs::msg::TrackedObject & t) {
      return from_tracked(t);
    });

  return predicted;
}

autoware_auto_msgs::msg::PredictedObject from_tracked(
  const autoware_auto_msgs::msg::TrackedObject & tracked)
{
  autoware_auto_msgs::msg::PredictedObject predicted{rosidl_runtime_cpp::MessageInitialization::ZERO};
  predicted.object_id = tracked.object_id;
  predicted.existence_probability = tracked.existence_probability;
  predicted.classification = tracked.classification;
  predicted.kinematics = from_tracked(tracked.kinematics);
  predicted.shape = tracked.shape;

  return predicted;
}
autoware_auto_msgs::msg::PredictedObjectKinematics from_tracked(
  const autoware_auto_msgs::msg::TrackedObjectKinematics & tracked)
{
  autoware_auto_msgs::msg::PredictedObjectKinematics predicted{rosidl_runtime_cpp::
    MessageInitialization::ZERO};
  predicted.initial_pose = tracked.pose;
  predicted.initial_twist = tracked.twist;
  predicted.initial_acceleration = tracked.acceleration;

  // empty predicted paths

  return predicted;
}

void predict_stationary(autoware_auto_msgs::msg::PredictedObject & predicted_object)
{
  using namespace std::chrono_literals;

  // TODO parameters to be read from launch file
//  builtin_interfaces::msg::Duration time_horizon{rosidl_runtime_cpp::MessageInitialization::ZERO};
//  time_horizon.sec = 7U;
//  builtin_interfaces::msg::Duration time_step{rosidl_runtime_cpp::MessageInitialization::ZERO};
//  time_step.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(100ms).count();
  const std::chrono::milliseconds time_horizon = 7s;
  const std::chrono::milliseconds time_step = 100ms;

  // TODO requires checking that `time_*` values > 0 before
  // need an extra state in the end if the division has remainder
  auto n_steps = static_cast<std::size_t>(time_horizon.count() / time_step.count());
  if (time_horizon.count() % time_step.count()) {
    ++n_steps;
  }

  // TODO issue: predicted path only has one pose, not multiple for each shape

  // TODO is path is predefined to a size of 100, does it contain 100 default-initialized
  //  poses or none if PredictedPath is initialized with strategy ZERO?
  autoware_auto_msgs::msg::PredictedPath predicted_path;
  predicted_path.path =
    decltype(predicted_path.path) {
    n_steps, predicted_object.kinematics.initial_pose.pose};
  predicted_path.confidence = 1.0;
  predicted_path.time_step = time_utils::to_message(time_step);

  predicted_object.kinematics.predicted_paths.emplace_back(predicted_path);
}

void predict_all_stationary(autoware_auto_msgs::msg::PredictedObjects & predicted_objects)
{
  std::for_each(
      predicted_objects.objects.begin(),
      predicted_objects.objects.end(), predict_stationary);
}

}  // namespace prediction_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::prediction_nodes::PredictionNodesNode)
