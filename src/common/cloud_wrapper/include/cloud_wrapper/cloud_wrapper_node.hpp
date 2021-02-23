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
/// \brief This file defines the cloud_wrapper_node class.

#ifndef CLOUD_WRAPPER__CLOUD_WRAPPER_NODE_HPP_
#define CLOUD_WRAPPER__CLOUD_WRAPPER_NODE_HPP_

#include <cloud_wrapper/cloud_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware
{
namespace common
{
namespace cloud_wrapper
{
using sensor_msgs::msg::PointCloud2;

/// \class CloudWrapperNode
/// \brief ROS 2 Node for testing the cloud_wrapper library.
class CLOUD_WRAPPER_PUBLIC CloudWrapperNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit CloudWrapperNode(const rclcpp::NodeOptions & options);


private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ptr_cloud_output_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ptr_cloud_output_synth_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ptr_cloud_input_;
  rclcpp::TimerBase::SharedPtr timer_;

  /// \brief Callback for Input Point Cloud for testing the library.
  /// \param msg Message.
  void callback_cloud_input(const PointCloud2::SharedPtr msg);

  /// \brief Callback for a timer for testing the library.
  /// \param msg Message.
  void callback_timer();

};
}  // namespace cloud_wrapper
}  // namespace common
}  // namespace autoware

#endif  // CLOUD_WRAPPER__CLOUD_WRAPPER_NODE_HPP_
