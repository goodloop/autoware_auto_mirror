// Copyright 2017-2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

/// \file
/// \brief This file defines an instance of the VoxelCloudBase interface
#ifndef VOXEL_GRID_NODES__ALGORITHM__VOXEL_CLOUD_CENTROID_HPP_
#define VOXEL_GRID_NODES__ALGORITHM__VOXEL_CLOUD_CENTROID_HPP_

#include <voxel_grid_nodes/algorithm/voxel_cloud_base.hpp>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid_nodes
{
namespace algorithm
{
/// \brief An instantiation of VoxelCloudBase for CentroidVoxels.
class VOXEL_GRID_NODES_PUBLIC VoxelCloudCentroid : public VoxelCloudBase
{
public:
  /// \brief Constructor
  /// \param[in] cfg Configuration struct for the voxel grid
  /// \param[in] stamp_with_current_time replace stamp in provided msg with current time
  explicit VoxelCloudCentroid(
    const voxel_grid::Config & cfg,
    const bool8_t stamp_with_current_time = NO_NEW_STAMP
  );

  /// \brief Inserts points into the voxel grid data structure, overwrites internal header
  /// \param[in] msg A point cloud to insert into the voxel grid. Assumed to have the structure XYZI
  void insert(const sensor_msgs::msg::PointCloud2 & msg) override;

  /// \brief Get accumulated downsampled points. Internally resets the internal grid. Header is
  ///        taken from last insert
  /// \return The downsampled point cloud
  const sensor_msgs::msg::PointCloud2 & get() override;

private:
  sensor_msgs::msg::PointCloud2 m_cloud;
  voxel_grid::VoxelGrid<voxel_grid::CentroidVoxel<PointXYZIF>> m_grid;
};  // VoxelCloudCentroid
}  // namespace algorithm
}  // namespace voxel_grid_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // VOXEL_GRID_NODES__ALGORITHM__VOXEL_CLOUD_CENTROID_HPP_
