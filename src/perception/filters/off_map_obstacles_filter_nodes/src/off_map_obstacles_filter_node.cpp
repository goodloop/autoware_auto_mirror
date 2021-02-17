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

#include "off_map_obstacles_filter_nodes/off_map_obstacles_filter_node.hpp"

#include <memory>
#include <string>

#include "lanelet2_map_provider/lanelet2_map_provider.hpp"
#include "common/types.hpp"
#include "tf2_ros/buffer_interface.h"

using namespace std::literals::chrono_literals;

namespace autoware
{
namespace off_map_obstacles_filter_nodes
{

using float64_t = autoware::common::types::float64_t;
using ObstacleMsg = autoware_auto_msgs::msg::BoundingBoxArray;
using MarkerArray = visualization_msgs::msg::MarkerArray;

OffMapObstaclesFilterNode::OffMapObstaclesFilterNode(const rclcpp::NodeOptions & options)
:  Node("off_map_obstacles_filter", options),
  m_sub_ptr(create_subscription<ObstacleMsg>(
      "bounding_boxes_in", rclcpp::QoS{10},
      [this](const ObstacleMsg::SharedPtr msg) {process_bounding_boxes(msg);})),
  m_pub_ptr(create_publisher<ObstacleMsg>("bounding_boxes_out", rclcpp::QoS{10})),
  m_marker_pub_ptr(create_publisher<MarkerArray>("bounding_boxes_viz", rclcpp::QoS{10})),
  m_tf2_buffer(this->get_clock()),
  m_tf2_listener(m_tf2_buffer)
{
  const std::string map_filename = declare_parameter("lanelet_map_filename").get<std::string>();
  const float64_t origin_lat = declare_parameter("latitude").get<float64_t>();
  const float64_t origin_lon = declare_parameter("longitude").get<float64_t>();
  const float64_t origin_alt = declare_parameter("elevation").get<float64_t>();
  const autoware::lanelet2_map_provider::Lanelet2MapProvider map_provider(map_filename, {origin_lat,
      origin_lon, origin_alt});
  const float64_t overlap_threshold = declare_parameter("overlap_threshold").get<float64_t>();
  m_filter = std::make_unique<OffMapObstaclesFilter>(map_provider.m_map, overlap_threshold);
}

void OffMapObstaclesFilterNode::process_bounding_boxes(const ObstacleMsg::SharedPtr msg) const
{
  if (msg->header.frame_id != "base_link") {
    // Using a different frame would not work since the code relies on the z axis to be aligned
    // with the map frame's z axis to do its projection onto the map.
    // That could be generalized in principle – but if the message is arriving in a different
    // frame, probably someone else than euclidean_cluster_node is sending it, and we can't be
    // sure anymore the 2.5D assumption is valid either. And that assumption is more fundamental
    // because the lanelet::Polygon2d needs its vertices to be in counterclockwise order, and
    // it's kind of difficult to ensure that for arbitrary rotated boxes.
    throw std::runtime_error(
            "Bounding boxes are in unexpected frame '" + msg->header.frame_id +
            "'");
  }
  try {
    geometry_msgs::msg::TransformStamped map_from_base_link = m_tf2_buffer.lookupTransform(
      "map", "base_link", tf2_ros::fromMsg(msg->header.stamp),
      tf2::durationFromSec(0.1));
    m_filter->remove_off_map_bboxes(map_from_base_link, *msg);
    const auto marker_array = m_filter->bboxes_in_map_frame_viz(map_from_base_link, *msg);
    m_marker_pub_ptr->publish(marker_array);
  } catch (...) {
    RCLCPP_INFO(get_logger(), "Did not filter boxes because no transform was available.");
  }
  m_pub_ptr->publish(*msg);
}

}  // namespace off_map_obstacles_filter_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::off_map_obstacles_filter_nodes::OffMapObstaclesFilterNode)
