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

#ifndef FILTER_NODE_BASE__FILTER_NODE_BASE_HPP_
#define FILTER_NODE_BASE__FILTER_NODE_BASE_HPP_

#include <filter_node_base/visibility_control.hpp>

#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace filter_node_base
{

/// \class FilterNodeBase
/// \brief ROS 2 Node for hello world.
class FILTER_NODE_BASE_PUBLIC FilterNodeBase : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit FilterNodeBase(const rclcpp::NodeOptions & options);

private:
  bool verbose;  ///< whether to use verbose output or not.
};
}  // namespace filter_node_base
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // FILTER_NODE_BASE__FILTER_NODE_BASE_HPP_
