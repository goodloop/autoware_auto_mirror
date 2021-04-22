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
/// \brief This file defines the prediction_nodes_node class.

#ifndef PREDICTION_NODES__PREDICTION_NODES_NODE_HPP_
#define PREDICTION_NODES__PREDICTION_NODES_NODE_HPP_

#include <prediction_nodes/prediction_nodes.hpp>

#define MSGS_UPDATED 0

#if MSGS_UPDATED

#include <autoware_auto_msgs/msg/predicted_dynamic_objects.hpp>
#include <autoware_auto_msgs/msg/tracked_dynamic_objects.hpp>

#else

#include <autoware_auto_msgs/msg/tracked_dynamic_object_array.hpp>

#endif

#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace prediction_nodes
{

/// \class PredictionNodesNode
class PREDICTION_NODES_PUBLIC PredictionNodesNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit PredictionNodesNode(const rclcpp::NodeOptions & options);

private:
#if MSGS_UPDATED
  using PredictedMsgT = autoware_auto_msgs::msg::PredictedDynamicObjects;
  using TrackedMsgT = autoware_auto_msgs::msg::TrackedDynamicObjects;
#else
  using TrackedMsgT = autoware_auto_msgs::msg::TrackedDynamicObjectArray;
#endif

  void PREDICTION_NODES_LOCAL on_tracked_objects(TrackedMsgT::ConstSharedPtr msg);

  bool verbose;  ///< whether to use verbose output or not.
#if MSGS_UPDATED
  rclcpp::Publisher<PredictedMsgT>::SharedPtr m_predicted_dynamic_objects_pub{};
#endif
  rclcpp::Subscription<TrackedMsgT>::SharedPtr m_tracked_dynamic_objects_sub{};
};
}  // namespace prediction_nodes
}  // namespace autoware

#endif  // PREDICTION_NODES__PREDICTION_NODES_NODE_HPP_
