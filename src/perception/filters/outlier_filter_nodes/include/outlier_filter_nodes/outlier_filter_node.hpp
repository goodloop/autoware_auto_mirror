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
/// \brief This file defines the outlier_filter_node class.

#ifndef OUTLIER_FILTER_NODES__OUTLIER_FILTER_NODE_HPP_
#define OUTLIER_FILTER_NODES__OUTLIER_FILTER_NODE_HPP_

#include <outlier_filter_nodes/outlier_filter.hpp>

#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter_nodes
{

/// \class OutlierFilterNode
/// \brief ROS 2 Node for hello world.
class OUTLIER_FILTER_NODES_PUBLIC OutlierFilterNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit OutlierFilterNode(const rclcpp::NodeOptions & options);

  /// \brief print hello
  /// return 0 if successful.
  int32_t print_hello() const;

private:
  bool verbose;  ///< whether to use verbose output or not.
};
}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // OUTLIER_FILTER_NODES__OUTLIER_FILTER_NODE_HPP_
