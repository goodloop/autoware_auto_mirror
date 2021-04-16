// Copyright 2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef NDT__NDT_MAP_HPP_
#define NDT__NDT_MAP_HPP_

#include <ndt/ndt_common.hpp>
#include <ndt/ndt_voxel.hpp>
#include <ndt/ndt_voxel_view.hpp>
#include <ndt/ndt_grid.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <time_utils/time_utils.hpp>
#include <vector>
#include <limits>
#include <unordered_map>
#include <utility>
#include <string>

using autoware::common::types::float32_t;

namespace autoware
{
namespace localization
{
namespace ndt
{
/// Function that checks if the pcl message format is valid as an ndt map. The point cloud should
/// have the following fields: x, y, z, cov_xx, cov_xy, cov_xz, cov_yy, cov_yz, cov_zz. The data
/// type for the fields should be double.
/// \param msg Point cloud message
/// \return Safe and usable number of points. the formula is:
/// `min(data.size(), width, row_step) / point_step`
/// If the cloud is assessed to be invalid (i.e. due to invalid fields), then 0 is returned.
uint32_t NDT_PUBLIC validate_pcl_map(const sensor_msgs::msg::PointCloud2 & msg);

/// Ndt Map for a dynamic voxel type. This map representation is only to be used
/// when a dense point cloud is intended to be represented as a map. (i.e. by the map publisher)
class NDT_PUBLIC DynamicNDTMap
{
public:
  using Voxel = DynamicNDTVoxel;
  using Config = autoware::perception::filters::voxel_grid::Config;
  using Point = Eigen::Vector3d;
  using TimePoint = std::chrono::system_clock::time_point;
  using VoxelViewVector = std::vector<VoxelView<Voxel>>;
  using VoxelGrid = NDTGrid<Voxel>::Grid;
  using ConfigPoint = NDTGrid<Voxel>::ConfigPoint;

  /// \brief First 3 points of a serialized message will contain 3 extra points:
  /// Min point, max point and the voxel size.
  static constexpr uint32_t kNumConfigPoints = 3U;

  explicit DynamicNDTMap(const Config & voxel_grid_config);

  /// \brief Set the contents of the pointcloud as the new map.
  /// \param msg Pointcloud to be inserted.
  void set(const sensor_msgs::msg::PointCloud2 & msg);

  /// Insert the dense point cloud to the map. This is intended for converting a dense
  /// point cloud into the ndt representation. Ideal for reading dense pcd files.
  /// \param msg PointCloud2 message to add.
  void insert(const sensor_msgs::msg::PointCloud2 & msg);

  /// Iterate over the map representation and convert it into a PointCloud2 message where each
  /// voxel in the map corresponds to a single point in the PointCloud2 field.
  /// \tparam DeserializingMapT The map type that can deserialize the serialized message.
  /// \param msg_out Reference to the initialized pointcloud message that will store
  /// the serialized map data.
  template<typename DeserializingMapT>
  void serialize_as(sensor_msgs::msg::PointCloud2 & msg_out) const;

  /// Lookup the cell at location.
  /// \param pt point to lookup
  /// \return A vector containing the cell at given coordinates. A vector is used to support
  /// near-neighbour cell queries in the future.
  const VoxelViewVector & cell(const Point & pt) const;

  /// Lookup the cell at location.
  /// \param x x coordinate
  /// \param y y coordinate
  /// \param z z coordinate
  /// \return A vector containing the cell at given coordinates. A vector is used to support
  /// near-neighbour cell queries in the future.
  const VoxelViewVector & cell(float32_t x, float32_t y, float32_t z) const;

  /// Get map's frame id.
  /// \return Frame id of the map.
  const std::string & frame_id() const noexcept;

  /// Get map's time stamp.
  /// \return map's time stamp.
  TimePoint stamp() const noexcept;

  /// \brief Check if the map is valid.
  /// \return True if the map and frame ID are not empty and the stamp is initialized.
  bool valid() const noexcept;

  /// Get size of the cell.
  /// \return A point representing the dimensions of the cell.
  const ConfigPoint & cell_size() const noexcept;

  /// Get size of the map
  /// \return Number of voxels in the map. This number includes the voxels that do not have
  /// enough numbers to be used yet.
  std::size_t size() const noexcept;

  /// \brief Returns an const iterator to the first element of the map
  /// \return Iterator
  typename VoxelGrid::const_iterator begin() const noexcept;

  /// \brief Returns a const iterator to one past the last element of the map
  /// \return Iterator
  typename VoxelGrid::const_iterator end() const noexcept;

  /// Clear all voxels in the map
  void clear() noexcept;

private:
  NDTGrid<DynamicNDTVoxel> m_grid;
  TimePoint m_stamp{};
  std::string m_frame_id{};
};

/// NDT map using StaticNDTVoxels. This class is to be used when the pointcloud
/// messages to be inserted already have the correct format (see validate_pcl_map(...)) and
/// represent a transformed map. No centroid/covariance computation is done during run-time.
class NDT_PUBLIC StaticNDTMap
{
public:
  using Voxel = StaticNDTVoxel;
  using Config = autoware::perception::filters::voxel_grid::Config;
  using TimePoint = std::chrono::system_clock::time_point;
  using Point = Eigen::Vector3d;
  using VoxelViewVector = std::vector<VoxelView<Voxel>>;
  using VoxelGrid = NDTGrid<Voxel>::Grid;
  using ConfigPoint = NDTGrid<Voxel>::ConfigPoint;

  /// Set point cloud message representing the map to the map representation instance.
  /// Map is assumed to have correct format (see `validate_pcl_map(...)`) and was generated
  /// by a dense map representation with identical configuration to this representation.
  /// \param msg PointCloud2 message to add. Each point in this cloud should correspond to a
  /// single voxel in the underlying voxel grid. This is checked via the `cell_id` field in the pcl
  /// message which is expected to be equal to the voxel grid ID in the map's voxel grid. Since
  /// the grid's index will be a long value to avoid overflows, `cell_id` field should be an array
  /// of 2 unsigned integers. That is because there is no direct long support as a PointField.
  void set(const sensor_msgs::msg::PointCloud2 & msg);

  /// Lookup the cell at location.
  /// \param pt point to lookup
  /// \return A vector containing the cell at given coordinates. A vector is used to support
  /// near-neighbour cell queries in the future.
  const VoxelViewVector & cell(const Point & pt) const;

  /// Lookup the cell at location.
  /// \param x x coordinate
  /// \param y y coordinate
  /// \param z z coordinate
  /// \return A vector containing the cell at given coordinates. A vector is used to support
  /// near-neighbour cell queries in the future.
  const VoxelViewVector & cell(float32_t x, float32_t y, float32_t z) const;

  /// Get map's frame id.
  /// \return Frame id of the map.
  const std::string & frame_id() const noexcept;

  /// Get map's time stamp.
  /// \return map's time stamp.
  TimePoint stamp() const noexcept;

  /// \brief Check if the map is valid.
  /// \return True if the map and frame ID are not empty and the stamp is initialized.
  bool valid() const noexcept;

  /// Get size of the cell.
  /// \return A point representing the dimensions of the cell.
  const ConfigPoint & cell_size() const;

  /// Get size of the map
  /// \return Number of voxels in the map. This number includes the voxels that do not have
  /// enough numbers to be used yet.
  std::size_t size() const;

  /// \brief Returns an const iterator to the first element of the map
  /// \return Iterator
  typename VoxelGrid::const_iterator begin() const;

  /// \brief Returns a const iterator to one past the last element of the map
  /// \return Iterator
  typename VoxelGrid::const_iterator end() const;

  /// Clear all voxels in the map
  void clear();

private:
  /// Deserialize the given serialized point cloud map.
  /// \param msg PointCloud2 message containing the deserialized data.
  void deserialize_from(const sensor_msgs::msg::PointCloud2 & msg);
  std::experimental::optional<NDTGrid<StaticNDTVoxel>> m_grid{};
  TimePoint m_stamp{};
  std::string m_frame_id{};
};

template<typename PCMessageT>
using PC2DoubleIterator = typename std::conditional<std::is_const<PCMessageT>::value,
    sensor_msgs::PointCloud2ConstIterator<ndt::Real>,
    sensor_msgs::PointCloud2Iterator<ndt::Real>>::type;

/// \brief Get the pointcloud2 iterators for the serialized ndt map
template<typename PointCloud2T>
std::array<PC2DoubleIterator<PointCloud2T>, 9U> get_iterators(PointCloud2T & msg)
{
  return std::array<PC2DoubleIterator<PointCloud2T>, 9U>{
    PC2DoubleIterator<PointCloud2T>{msg, "x"},
    PC2DoubleIterator<PointCloud2T>{msg, "y"},
    PC2DoubleIterator<PointCloud2T>{msg, "z"},
    PC2DoubleIterator<PointCloud2T>{msg, "icov_xx"},
    PC2DoubleIterator<PointCloud2T>{msg, "icov_xy"},
    PC2DoubleIterator<PointCloud2T>{msg, "icov_xz"},
    PC2DoubleIterator<PointCloud2T>{msg, "icov_yy"},
    PC2DoubleIterator<PointCloud2T>{msg, "icov_yz"},
    PC2DoubleIterator<PointCloud2T>{msg, "icov_zz"}
  };
}

/// Struct to represent a point in the serialized ndt map. This is a minimal voxel representation.
struct SerializedNDTMapPoint
{
  Real x{0.0};
  Real y{0.0};
  Real z{0.0};
  Real icov_xx{0.0};
  Real icov_xy{0.0};
  Real icov_xz{0.0};
  Real icov_yy{0.0};
  Real icov_yz{0.0};
  Real icov_zz{0.0};
};

/// \brief Push a SerializedNDTMapPoint into a pointcloud2 object containing the serialized map.
/// \param pc_its PC2 iterators used for inserting the point.
/// \param point A point representing a voxel in the ndt map.
void push_back(
  std::array<sensor_msgs::PointCloud2Iterator<ndt::Real>, 9U> & pc_its,
  const SerializedNDTMapPoint & point);

/// \brief Get the next voxel representation from the iterators of a serialized ndt map.
/// \param pc_its Iterators of a pointcloud2 object that stores the serialized ndt map.
/// \return The serialized ndt map point.
template<typename T>
SerializedNDTMapPoint next(std::array<T, 9U> & pc_its)
{
  const auto ret = SerializedNDTMapPoint{
    *pc_its[0U], *pc_its[1U], *pc_its[2U],    // x, y, z
    *pc_its[3U], *pc_its[4U], *pc_its[5U],    // xx, yy, zz
    *pc_its[6U], *pc_its[7U],    // yy, yz
    *pc_its[8U]    // zz
  };
  std::for_each(pc_its.begin(), pc_its.end(), [](auto & it) {++it;});
  return ret;
}

}  // namespace ndt
}  // namespace localization
}  // namespace autoware
#endif  // NDT__NDT_MAP_HPP_
