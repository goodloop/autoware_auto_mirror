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

namespace autoware
{
namespace prediction_nodes
{

PredictionNodesNode::PredictionNodesNode(const rclcpp::NodeOptions & options)
:  Node("prediction_nodes", options),
  verbose(true),
#if MSGS_UPDATED
  m_predicted_dynamic_objects_pub{create_publisher<PredictedMsgT>(
      "/predicted_objects",
      rclcpp::QoS{10})}
#endif
  m_tracked_dynamic_objects_sub{create_subscription<TrackedMsgT>(
      "/tracked_objects",
      rclcpp::QoS{10},
      [this](TrackedMsgT::ConstSharedPtr msg) {on_tracked_objects(msg);})}
{
}

void PredictionNodesNode::on_tracked_objects(TrackedMsgT::ConstSharedPtr /*msg*/)
{}

}  // namespace prediction_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::prediction_nodes::PredictionNodesNode)
