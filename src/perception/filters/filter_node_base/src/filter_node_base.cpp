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

#include "filter_node_base/filter_node_base.hpp"

#include <memory>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace filter_node_base
{

FilterNodeBase::FilterNodeBase(
  const std::string & filter_name, const rclcpp::NodeOptions & options)
: Node(filter_name, options), filter_field_name_(filter_name)
{
  max_queue_size_ = static_cast<std::size_t>(declare_parameter(
      "max_queue_size").get<std::size_t>());

  // Set publisher
  pub_output_ = this->create_publisher<PointCloud2>(
    "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_));

  // Set subscriber
  std::function<void(const PointCloud2ConstPtr msg)> cb = std::bind(
    &FilterNodeBase::pointcloud_callback, this, std::placeholders::_1);
  sub_input_ = create_subscription<PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_), cb);
}

void FilterNodeBase::pointcloud_callback(const PointCloud2ConstPtr msg)
{
  // If cloud is given, check if it's valid
  if (!is_valid(msg)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "[" << filter_field_name_ << "]: Invalid input!");
    return;
  }

  /// DEBUG
  RCLCPP_DEBUG(
    this->get_logger(),
    "[%s]: PointCloud with %d data points and frame %s on input topic "
    "received.",
    filter_field_name_, msg->width * msg->height, msg->header.frame_id.c_str());

  PointCloud2 output;
  // Call the virtual method in the child
  filter(msg, output);

  pub_output_->publish(output);
}

}  // namespace filter_node_base
}  // namespace filters
}  // namespace perception
}  // namespace autoware
