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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the prediction node

#ifndef PREDICTION_NODES__PREDICTION_NODE_HPP_
#define PREDICTION_NODES__PREDICTION_NODE_HPP_


#include <autoware_auto_msgs/srv/had_map_service.hpp>
#include <autoware_auto_msgs/msg/route.hpp>

#include "visibility_control.hpp"
// TODO(frederik.beaujean) Remove when autoware messages fully updated and available
#define MSGS_UPDATED 1
#define TRAFFIC_LIGHTS 0

#if MSGS_UPDATED
#include <autoware_auto_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#else
#include <autoware_auto_msgs/msg/tracked_dynamic_object_array.hpp>
#endif

#if TRAFFIC_LIGHTS
#include <autoware_auto_msgs/msg/traffic_signal_array.hpp>
#endif

#include <lanelet2_core/LaneletMap.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include <chrono>

namespace autoware
{
namespace prediction_nodes
{

// TODO(#1073) These conversion functions should be stored elsewhere

// copy over common members, leaving prediction-specific stuff untouched
autoware_auto_msgs::msg::PredictedObjects from_tracked(
  const autoware_auto_msgs::msg::TrackedObjects &);
autoware_auto_msgs::msg::PredictedObject from_tracked(
  const autoware_auto_msgs::msg::TrackedObject &);
autoware_auto_msgs::msg::PredictedObjectKinematics from_tracked(
  const autoware_auto_msgs::msg::TrackedObjectKinematics &);

/// \class PredictionNode
class PREDICTION_NODES_PUBLIC PredictionNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit PredictionNode(const rclcpp::NodeOptions & options);

private:
#if MSGS_UPDATED
  using PredictedMsgT = autoware_auto_msgs::msg::PredictedObjects;
  using TrackedMsgT = autoware_auto_msgs::msg::TrackedObjects;
  using RouteMsgT = autoware_auto_msgs::msg::Route;
#else
  using TrackedMsgT = autoware_auto_msgs::msg::TrackedDynamicObjectArray;
#endif

#if TRAFFIC_LIGHTS
  using TrafficSignalT = autoware_auto_msgs::msg::TrafficSignalArray;
#endif
  using HADMapService = autoware_auto_msgs::srv::HADMapService;

  void PREDICTION_NODES_LOCAL on_map_response(rclcpp::Client<HADMapService>::SharedFuture future);
  void PREDICTION_NODES_LOCAL on_route(RouteMsgT::ConstSharedPtr msg);
  void PREDICTION_NODES_LOCAL on_tracked_objects(TrackedMsgT::ConstSharedPtr msg);
#if TRAFFIC_LIGHTS
  void PREDICTION_NODES_LOCAL on_traffic_signals(TrafficSignalT::ConstSharedPtr msg);
#endif

  // sends asynchronous request for map
  void PREDICTION_NODES_LOCAL request_map();
  void PREDICTION_NODES_LOCAL wait_for_map(
    std::chrono::milliseconds timeout = std::chrono::seconds{10U});

  bool verbose;  ///< whether to use verbose output or not.
#if MSGS_UPDATED
  rclcpp::Publisher<PredictedMsgT>::SharedPtr m_predicted_objects_pub{};
#endif
  rclcpp::Subscription<TrackedMsgT>::SharedPtr m_tracked_dynamic_objects_sub{};
#if TRAFFIC_LIGHTS
  rclcpp::Subscription<TrafficSignalT>::SharedPtr m_traffic_signal_sub{};
#endif

  rclcpp::Client<HADMapService>::SharedPtr m_map_client{};
  lanelet::LaneletMapPtr m_map{};

  tf2::BufferCore m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;

  rclcpp::Subscription<RouteMsgT>::SharedPtr m_route_sub{};
  RouteMsgT::ConstSharedPtr m_route;
};
}  // namespace prediction_nodes
}  // namespace autoware

#endif  // PREDICTION_NODES__PREDICTION_NODE_HPP_
