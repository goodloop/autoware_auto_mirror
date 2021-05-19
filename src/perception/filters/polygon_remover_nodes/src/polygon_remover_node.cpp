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

#include "polygon_remover_nodes/polygon_remover_node.hpp"
#include <polygon_remover/polygon_remover.hpp>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <memory>
#include <string>
#include <map>
#include <vector>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace polygon_remover_nodes
{

using autoware_auto_msgs::msg::Shape;

PolygonRemoverNode::PolygonRemoverNode(const rclcpp::NodeOptions & options)
:  Node("polygon_remover_nodes", options),
  pub_cloud_ptr_{
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter("topic_name_cloud_pub").get<std::string>(),
      rclcpp::QoS(rclcpp::KeepLast(1)))
  },
  pub_marker_ptr_{
    this->create_publisher<visualization_msgs::msg::Marker>(
      declare_parameter("topic_name_marker_polygon_pub").get<std::string>(),
      rclcpp::QoS(rclcpp::KeepLast(1)))
  },
  sub_cloud_ptr_{
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter("topic_name_cloud_sub").get<std::string>(),
      rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(
        &PolygonRemoverNode::callback_cloud,
        this,
        std::placeholders::_1))},
  will_visualize_{declare_parameter("will_visualize").get<bool>()}
{
  auto make_point = [](float x, float y, float z) {
      geometry_msgs::msg::Point32 point_32;
      point_32.set__x(x);
      point_32.set__y(y);
      point_32.set__z(z);
      return point_32;
    };

  std::string working_mode_str{declare_parameter("working_mode").get<std::string>()};

  std::map<std::string, WorkingMode> map_string_to_working_mode_{
    {"Static", WorkingMode::kStatic},
    {"PolygonSub", WorkingMode::kPolygonSub},
  };

  // Make sure working_mode string is within keys
  if (map_string_to_working_mode_.find(working_mode_str) == map_string_to_working_mode_.end() ) {
    std::string str_list_of_keys;
    for (const auto & key_value_pair : map_string_to_working_mode_) {
      str_list_of_keys += key_value_pair.first + ", ";
    }
    str_list_of_keys = str_list_of_keys.substr(
      0, str_list_of_keys.size() - 2);   // remove excess ", "
    throw std::runtime_error("Please set working_mode to be one of: " + str_list_of_keys);
  }

  polygon_remover_ = std::make_shared<polygon_remover::PolygonRemover>(will_visualize_);

  // Initialize based on working_mode
  switch (map_string_to_working_mode_.at(working_mode_str)) {
    case WorkingMode::kStatic: {
        // Set polygon from static input
        std::vector<double> polygon_vertices{
          declare_parameter("polygon_vertices").get<std::vector<double>>()};

        if (polygon_vertices.size() % 2 != 0) {
          throw std::length_error(
                  "polygon_vertices has odd number of elements. "
                  "It must have a list of x,y double pairs.");
        }
        Shape::SharedPtr shape = std::make_shared<Shape>();
        for (size_t i = 0UL; i < polygon_vertices.size(); i += 2) {
          auto p_x = static_cast<float>(polygon_vertices.at(i));
          auto p_y = static_cast<float>(polygon_vertices.at(i + 1));
          shape->polygon.points.emplace_back(make_point(p_x, p_y, 0.0F));
        }
        polygon_remover_->update_polygon(shape);
        break;
      }
    case WorkingMode::kPolygonSub: {
        // Set polygon from shape callback
        sub_shape_polygon_ptr_ =
          this->create_subscription<Shape>(
          declare_parameter("topic_name_shape_polygon_sub").get<std::string>(),
          rclcpp::QoS(rclcpp::KeepLast(1)),
          std::bind(
            &PolygonRemoverNode::callback_shape_polygon,
            this,
            std::placeholders::_1));
        break;
      }
  }
}


void PolygonRemoverNode::callback_cloud(const PointCloud2::ConstSharedPtr cloud_in_ptr)
{
  if (!polygon_remover_->polygon_is_initialized()) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Polygon is not initialized, publishing incoming cloud.");
    pub_cloud_ptr_->publish(*cloud_in_ptr);
    return;
  }

  PointCloud2::SharedPtr cloud_filtered_ptr =
    polygon_remover_->remove_updated_polygon_from_cloud(cloud_in_ptr);

  pub_cloud_ptr_->publish(*cloud_filtered_ptr);

  if (will_visualize_) {
    pub_marker_ptr_->publish(polygon_remover_->get_marker());
  }
}

void PolygonRemoverNode::callback_shape_polygon(const Shape::ConstSharedPtr shape_in_ptr)
{
  RCLCPP_INFO_STREAM(get_logger(), "Polygon is updated.");
  polygon_remover_->update_polygon(shape_in_ptr);
}

}  // namespace polygon_remover_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::polygon_remover_nodes::PolygonRemoverNode)
