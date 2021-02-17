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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the off_map_obstacles_filter_nodes_node class.

#ifndef OFF_MAP_OBSTACLES_FILTER_NODES__OFF_MAP_OBSTACLES_FILTER_NODE_HPP_
#define OFF_MAP_OBSTACLES_FILTER_NODES__OFF_MAP_OBSTACLES_FILTER_NODE_HPP_

#include <memory>
#include <string>

#include "off_map_obstacles_filter_nodes/visibility_control.hpp"
#include "autoware_auto_msgs/msg/bounding_box_array.hpp"
#include "off_map_obstacles_filter/off_map_obstacles_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


namespace autoware
{
namespace off_map_obstacles_filter_nodes
{

using OffMapObstaclesFilter = autoware::off_map_obstacles_filter::OffMapObstaclesFilter;
/// \class OffMapObstaclesFilterNode
/// \brief ROS 2 Node that removes obstacles that are off the map. See the design doc for more.
class OFF_MAP_OBSTACLES_FILTER_NODES_PUBLIC OffMapObstaclesFilterNode : public rclcpp::Node
{
public:
  /// \brief Constructor.
  /// \param options Node options.
  explicit OffMapObstaclesFilterNode(const rclcpp::NodeOptions & options);

  /// \brief The main callback of this node
  /// \param msg The BoundingBoxArray message containing obstacles
  void process_bounding_boxes(const autoware_auto_msgs::msg::BoundingBoxArray::SharedPtr msg) const;

private:
  std::unique_ptr<OffMapObstaclesFilter> m_filter;
  const rclcpp::Subscription<autoware_auto_msgs::msg::BoundingBoxArray>::SharedPtr
    m_sub_ptr;
  const rclcpp::Publisher<autoware_auto_msgs::msg::BoundingBoxArray>::SharedPtr m_pub_ptr;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub_ptr;
  tf2_ros::Buffer m_tf2_buffer;
  tf2_ros::TransformListener m_tf2_listener;
};
}  // namespace off_map_obstacles_filter_nodes
}  // namespace autoware

#endif  // OFF_MAP_OBSTACLES_FILTER_NODES__OFF_MAP_OBSTACLES_FILTER_NODE_HPP_
