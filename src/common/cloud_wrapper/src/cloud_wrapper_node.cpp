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

#include "common/types.hpp"
#include "cloud_wrapper/cloud_wrapper_node.hpp"
#include "rclcpp/logger.hpp"
#include <algorithm>

namespace autoware
{
namespace common
{
namespace cloud_wrapper
{

using autoware::common::types_point_cloud2::PointLgsvl;
using autoware::common::types_point_cloud2::PointXYZI;

CloudWrapperNode::CloudWrapperNode(const rclcpp::NodeOptions & options)
: Node("cloud_wrapper", options),
  pub_ptr_cloud_output_{this->create_publisher<sensor_msgs::msg::PointCloud2>(
      static_cast<std::string>(declare_parameter(
        "name_topic_cloud_output").get<std::string>()),
      10)}
  ,
  pub_ptr_cloud_output_synth_{this->create_publisher<sensor_msgs::msg::PointCloud2>(
      static_cast<std::string>(declare_parameter(
        "name_topic_cloud_output_synth").get<std::string>()),
      10)}
  ,
  sub_ptr_cloud_input_(this->create_subscription<PointCloud2>(
      static_cast<std::string>(declare_parameter(
        "name_topic_cloud_input").get<std::string>()),
      rclcpp::QoS(10),
      std::bind(&CloudWrapperNode::callback_cloud_input,
      this,
      std::placeholders::_1)))
  ,
  timer_{this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&CloudWrapperNode::callback_timer, this))}
{

}

void CloudWrapperNode::callback_timer()
{
  RCLCPP_INFO(this->get_logger(), "callback_timer is called.");

  CloudPtrWrapper<PointXYZI> cloud_wrapper(autoware::common::types_point_cloud2::fields_PointXYZI);

  cloud_wrapper.get_msg_ptr()->header.frame_id = "lidar_front";
  cloud_wrapper.get_msg_ptr()->header.stamp = this->now();

  float angle_center_from_time =
    static_cast<float>(std::fmod(this->get_clock()->now().seconds(), 360.0));
  float radius_circle_orbit = 15.0f;
  float offset_x = radius_circle_orbit * std::cos(angle_center_from_time);
  float offset_y = radius_circle_orbit * std::sin(angle_center_from_time);

  cloud_wrapper.reserve(cloud_wrapper.size() + 360 * 3);
  float radius_circle = 3.0f;
  float circle_center_height = 15.0f;
  float circle_height_max = circle_center_height + radius_circle;
  auto height_to_intensity = [circle_height_max, radius_circle](float z) {
      return 255.0f * (circle_height_max - z) / (2 * radius_circle);
    };

  for (int i = 0; i < 360; ++i) {
    double angle = static_cast<double>(i) / 180.0 * M_PI;
    float cos_part = radius_circle * static_cast<float>( std::cos(angle));
    float sin_part = radius_circle * static_cast<float>( std::sin(angle));

    float z1 = circle_center_height;
    float z2 = circle_center_height + cos_part;
    float z3 = circle_center_height + cos_part;
    PointXYZI p1(offset_x + cos_part, offset_y + sin_part, z1, height_to_intensity(z1));
    PointXYZI p2(offset_x + sin_part, offset_y + 0, z2, height_to_intensity(z2));
    PointXYZI p3(offset_x + 0, offset_y + sin_part, z3, height_to_intensity(z3));
    cloud_wrapper.push_back(p1);
    cloud_wrapper.push_back(p2);
    cloud_wrapper.push_back(p3);
  }

  pub_ptr_cloud_output_synth_->publish(*cloud_wrapper.get_msg_ptr());
}

void CloudWrapperNode::callback_cloud_input(const PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "callback_cloud_input called.");

  std::cout << "sizeof PointLgsvl = " << sizeof(PointLgsvl) << std::endl;

  CloudPtrWrapper<PointLgsvl> cloud_wrapper1(msg);

//  std::vector<PointLgsvl> points_new = cloud_wrapper1.get_vector_copy();
//  for (size_t i = 0; i < cloud_wrapper1.size(); ++i) {
//    const auto & p = cloud_wrapper1.at(i);
//    std::cout << "index #" << i <<
//      " = " << p.x <<
//      ", " << p.y <<
//      ", " << p.z <<
//      ", " << static_cast<int>(p.intensity) <<
//      std::endl;
//  }

  std::cout << "size_before_removal: " << cloud_wrapper1.size() << std::endl;

  auto end_new = std::remove_if(
    cloud_wrapper1.begin(),
    cloud_wrapper1.end(),
    [](const PointLgsvl & p) {
      bool intensity_is_too_low = p.intensity < 1;
      bool z_is_too_big = p.z > 0;
      return intensity_is_too_low || z_is_too_big;
    });


  cloud_wrapper1.erase_till_end(end_new);
  std::cout << "size_after_removal: " << cloud_wrapper1.size() << std::endl;


  std::for_each(
    cloud_wrapper1.begin(),
    cloud_wrapper1.end(),
    [](PointLgsvl & p) {
      p.z += 10.0f;
    });

  cloud_wrapper1.reserve(cloud_wrapper1.size() + 360 * 3);
  float radius_circle = 5.0f;
  float circle_center_height = 15.0f;
  float circle_height_max = circle_center_height + radius_circle;
  auto height_to_intensity = [circle_height_max, radius_circle](float z) {
      return static_cast<unsigned char>(255.0f * (circle_height_max - z) / (2 * radius_circle));
    };

  for (int i = 0; i < 360; ++i) {
    double angle = static_cast<double>(i) / 180.0 * M_PI;
    float cos_part = radius_circle * static_cast<float>( std::cos(angle));
    float sin_part = radius_circle * static_cast<float>( std::sin(angle));

    float z1 = circle_center_height;
    float z2 = circle_center_height + cos_part;
    float z3 = circle_center_height + cos_part;
    PointLgsvl p(cos_part, sin_part, z1, height_to_intensity(z1), 0);
    PointLgsvl p2(sin_part, 0, z2, height_to_intensity(z2), 0);
    PointLgsvl p3(0, sin_part, z3, height_to_intensity(z3), 0);
    cloud_wrapper1.push_back(p);
    cloud_wrapper1.push_back(p2);
    cloud_wrapper1.push_back(p3);
  }


  pub_ptr_cloud_output_->publish(*cloud_wrapper1.get_msg_ptr());


  std::vector<std::string> type_names{
    "INT8",
    "UINT8",
    "INT16",
    "UINT16",
    "INT32",
    "UINT32",
    "FLOAT32",
    "FLOAT64"
  };

  std::stringstream ss_fields;
  std::stringstream ss_types;
  std::stringstream ss_offsets;
  std::stringstream ss_counts;
  ss_fields << "fields: ";
  ss_types << "types: ";
  ss_offsets << "offsets: ";
  ss_counts << "counts: ";
  for (const auto & field : msg->fields) {
    ss_fields << field.name << ", ";
    ss_types << type_names.at(field.datatype - 1u) << ", ";
    ss_offsets << std::to_string(field.offset) << ", ";
    ss_counts << std::to_string(field.count) << ", ";
  }

  std::cout << ss_fields.str() << std::endl;
  std::cout << ss_types.str() << std::endl;
  std::cout << ss_offsets.str() << std::endl;
  std::cout << ss_counts.str() << std::endl;


}

}  // namespace cloud_wrapper
}  // namespace common
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::common::cloud_wrapper::CloudWrapperNode)
