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

#include <functional>
#include <memory>

namespace autoware
{
namespace prediction_nodes
{
using std::placeholders::_1;

PredictionNodesNode::PredictionNodesNode(const rclcpp::NodeOptions & options)
: Node("prediction_nodes", options),
  verbose(true),
  // TODO(frederik.beaujean) Add topics to design doc
#if MSGS_UPDATED
  m_predicted_dynamic_objects_pub{create_publisher<PredictedMsgT>(
      "predicted_objects",
      rclcpp::QoS{10})}
#endif
  m_tracked_dynamic_objects_sub{create_subscription<TrackedMsgT>(
      "tracked_objects",
      rclcpp::QoS{10},
      std::bind(&PredictionNodesNode::on_tracked_objects, this, _1))},
  m_map_client{create_client<HADMapService>("HAD_Map_Service")},
  m_tf_listener(m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false),
  m_route_sub{this->create_subscription<RouteMsgT>(
      "route", rclcpp::QoS{10},
      std::bind(&PredictionNodesNode::on_route, this, _1))}
{
  // send all asynchronous requests
  request_map();

  // wait for results
  wait_for_map();
}

void PredictionNodesNode::on_tracked_objects(TrackedMsgT::ConstSharedPtr /*msg*/)
{}

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

void PredictionNodesNode::on_map_response(rclcpp::Client<HADMapService>::SharedFuture future)
{
  autoware::common::had_map_utils::fromBinaryMsg(future.get()->map, m_map);

  RCLCPP_INFO(get_logger(), "Received map");
}

void PREDICTION_NODES_LOCAL PredictionNodesNode::on_route(RouteMsgT::ConstSharedPtr msg)
{
  m_route = msg;
}

}  // namespace prediction_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::prediction_nodes::PredictionNodesNode)
