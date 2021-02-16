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

#include <chrono>
#include <memory>
#include <string>

#include "lanelet2_map_provider/lanelet2_map_provider.hpp"
#include "common/types.hpp"

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
    throw std::runtime_error(
            "bounding boxes are in unexpected frame '" + msg->header.frame_id +
            "'");
  }
  const auto num_boxes_before = msg->boxes.size();
  const auto start_time = std::chrono::steady_clock::now();
  geometry_msgs::msg::TransformStamped map_from_base_link;
  try {
    map_from_base_link = m_tf2_buffer.lookupTransform(
      "map", "base_link", msg->header.stamp, rclcpp::Duration(
        100ms));
    m_filter->remove_off_map_bboxes(map_from_base_link, *msg);
    auto marker_array = m_filter->bboxes_in_map_frame_viz(map_from_base_link, *msg);
    m_marker_pub_ptr->publish(marker_array);
  } catch (...) {
    RCLCPP_INFO(get_logger(), "Did not filter boxes because no transform was available.");
  }
  m_pub_ptr->publish(*msg);
  auto end_time = std::chrono::steady_clock::now();

  // Copied verbatim from euclidean_cluster_node
  // int id_counter = 0;
  // MarkerArray marker_array;
  // for (const auto & box : msg->boxes) {
  //   visualization_msgs::msg::Marker m{};
  //   m.header.stamp = rclcpp::Time(0);
  //   m.header.frame_id = msg->header.frame_id;
  //   m.ns = "bbox";
  //   m.id = id_counter;
  //   m.type = visualization_msgs::msg::Marker::CUBE;
  //   m.action = visualization_msgs::msg::Marker::ADD;
  //   m.pose.position.x = static_cast<float64_t>(box.centroid.x);
  //   m.pose.position.y = static_cast<float64_t>(box.centroid.y);
  //   m.pose.position.z = static_cast<float64_t>(box.centroid.z);
  //   m.pose.orientation.x = static_cast<float64_t>(box.orientation.x);
  //   m.pose.orientation.y = static_cast<float64_t>(box.orientation.y);
  //   m.pose.orientation.z = static_cast<float64_t>(box.orientation.z);
  //   m.pose.orientation.w = static_cast<float64_t>(box.orientation.w);
  //   // X and Y scale are swapped between these two message types
  //   m.scale.x = static_cast<float64_t>(box.size.y);
  //   m.scale.y = static_cast<float64_t>(box.size.x);
  //   m.scale.z = static_cast<float64_t>(box.size.z);
  //   m.color.r = 1.0;
  //   m.color.g = 0.5;
  //   m.color.b = 0.0;
  //   m.color.a = 0.75;
  //   m.lifetime.sec = 0;
  //   m.lifetime.nanosec = 500000000;
  //   marker_array.markers.push_back(m);
  //   id_counter++;
  // }

  const auto num_boxes_after = msg->boxes.size();
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_time - start_time).count();
  if (true) {
    RCLCPP_INFO(
      get_logger(), "Removed %d boxes, %d ms", (num_boxes_before - num_boxes_after),
      milliseconds);
  }
}

}  // namespace off_map_obstacles_filter_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::off_map_obstacles_filter_nodes::OffMapObstaclesFilterNode)
