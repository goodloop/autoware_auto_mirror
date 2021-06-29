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

#include <memory>
#include <vector>

#include "outlier_filter_nodes/voxel_grid_outlier_filter_node.hpp"


namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter_nodes
{

using VoxelGridOutlierFilter =
  autoware::perception::filters::outlier_filter::voxel_grid_outlier_filter::
  VoxelGridOutlierFilter;

VoxelGridOutlierFilterNode::VoxelGridOutlierFilterNode(const rclcpp::NodeOptions & options)
: FilterNodeBase("voxel_grid_outlier_filter_node", options)
{
  voxel_grid_outlier_filter_ = std::make_shared<VoxelGridOutlierFilter>(
    declare_parameter("voxel_size_x").get<float>(),
    declare_parameter("voxel_size_y").get<float>(),
    declare_parameter("voxel_size_z").get<float>(),
    declare_parameter("voxel_point_threshold").get<uint32_t>());
}

void VoxelGridOutlierFilterNode::filter(
  const sensor_msgs::msg::PointCloud2 &,
  sensor_msgs::msg::PointCloud2 &)
{

}

rcl_interfaces::msg::SetParametersResult VoxelGridOutlierFilterNode::get_node_parameters(
  const std::vector<rclcpp::Parameter> &)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  return result;
}

}  // namespace outlier_filter_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::outlier_filter_nodes::VoxelGridOutlierFilterNode)
