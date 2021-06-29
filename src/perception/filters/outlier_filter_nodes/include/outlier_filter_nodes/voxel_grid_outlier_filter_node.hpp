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
/// \brief This file defines the outlier_filter_nodes_node class.

#ifndef OUTLIER_FILTER_NODES__VOXEL_GRID_OUTLIER_FILTER_NODE_HPP_
#define OUTLIER_FILTER_NODES__VOXEL_GRID_OUTLIER_FILTER_NODE_HPP_

#include <memory>
#include <vector>

#include "outlier_filter_nodes/visibility_control.hpp"

#include "filter_node_base/filter_node_base.hpp"
#include "outlier_filter/voxel_grid_outlier_filter.hpp"
#include "rclcpp/rclcpp.hpp"


namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter_nodes
{

using FilterNodeBase = autoware::perception::filters::filter_node_base::FilterNodeBase;
using VoxelGridOutlierFilter =
  autoware::perception::filters::outlier_filter::voxel_grid_outlier_filter::
  VoxelGridOutlierFilter;

/// \class OutlierFilterNodesNode
/// \brief ROS 2 Node for hello world.
class OUTLIER_FILTER_NODES_PUBLIC VoxelGridOutlierFilterNode : public FilterNodeBase
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit VoxelGridOutlierFilterNode(const rclcpp::NodeOptions & options);

protected:
  OUTLIER_FILTER_NODES_PUBLIC virtual void filter(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output);

  OUTLIER_FILTER_NODES_PUBLIC virtual rcl_interfaces::msg::SetParametersResult get_node_parameters(
    const std::vector<rclcpp::Parameter> & p);

private:
  std::shared_ptr<VoxelGridOutlierFilter> voxel_grid_outlier_filter_;

  float voxel_size_x_;
  float voxel_size_y_;
  float voxel_size_z_;
  uint32_t voxel_points_threshold_;
};
}  // namespace outlier_filter_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // OUTLIER_FILTER_NODES__VOXEL_GRID_OUTLIER_FILTER_NODE_HPP_
