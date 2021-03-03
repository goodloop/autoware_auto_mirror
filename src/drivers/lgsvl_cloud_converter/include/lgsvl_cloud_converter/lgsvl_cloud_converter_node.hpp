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
/// \brief This file defines the lgsvl_cloud_converter_node class.

#ifndef LGSVL_CLOUD_CONVERTER__LGSVL_CLOUD_CONVERTER_NODE_HPP_
#define LGSVL_CLOUD_CONVERTER__LGSVL_CLOUD_CONVERTER_NODE_HPP_

#include <lgsvl_cloud_converter/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware
{
namespace drivers
{
namespace lgsvl_cloud_converter
{
using sensor_msgs::msg::PointCloud2;

/// \class LgsvlCloudConverterNode
/// \brief ROS 2 Node for converting PointCloud2 clouds from LGSVL to Autoware.Auto
class LGSVL_CLOUD_CONVERTER_PUBLIC LgsvlCloudConverterNode : public rclcpp::Node
{
public:
  /// \brief default constructor, initializes subs and pubs
  explicit LgsvlCloudConverterNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_cloud_output_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ptr_cloud_input_;

  /// \brief Callback for input cloud, converts and publishes.
  void callback_cloud_input(const PointCloud2::SharedPtr msg);

};
}  // namespace lgsvl_cloud_converter
}  // namespace drivers
}  // namespace autoware

#endif  // LGSVL_CLOUD_CONVERTER__LGSVL_CLOUD_CONVERTER_NODE_HPP_
