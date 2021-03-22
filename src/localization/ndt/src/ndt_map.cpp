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

#include <ndt/ndt_map.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <string>

namespace autoware
{
namespace localization
{
namespace ndt
{
uint32_t validate_pcl_map(const sensor_msgs::msg::PointCloud2 & msg)
{
  // lambda to check the fields of a PointField
  auto field_valid = [](const auto & field, auto data_type, auto offset, auto count) {
      return (field.datatype == data_type) && (field.offset == offset) && (field.count == count);
    };

  auto ret = 0U;
  const auto double_field_size =
    static_cast<uint32_t>(sizeOfPointField(sensor_msgs::msg::PointField::FLOAT64));
  const auto uint_field_size =
    static_cast<uint32_t>(sizeOfPointField(sensor_msgs::msg::PointField::UINT32));

  // TODO(cvasfi): Possibly allow additional fields that don't change the order of the fields
  constexpr auto expected_num_fields = 10U;

  // Check general pc metadata
  if ((msg.fields.size()) != expected_num_fields ||
    (msg.point_step != (((expected_num_fields - 1U) * double_field_size) + 2 * uint_field_size)) ||
    (msg.height != 1U))
  {
    return 0U;
  }

  // check PointField fields
  // Get ID of the last field before cell_ID for reverse iterating. (used to calculate offset)
  auto double_field_idx = expected_num_fields - 2U;
  if (!std::all_of(
      msg.fields.rbegin() + 1U, msg.fields.rend(),               // check all float fields
      [&double_field_idx, &field_valid, double_field_size](auto & field) {
        return field_valid(
          field, sensor_msgs::msg::PointField::FLOAT64,
          ((double_field_idx--) * double_field_size), 1U);
      }) ||
    !field_valid(
      msg.fields[9U], sensor_msgs::msg::PointField::UINT32,             // check the cell id field
      (9U * double_field_size), 2U) )
  {
    return 0U;
  }

  // Check field names
  if ((msg.fields[0U].name != "x") ||
    (msg.fields[1U].name != "y") ||
    (msg.fields[2U].name != "z") ||
    (msg.fields[3U].name != "icov_xx") ||
    (msg.fields[4U].name != "icov_xy") ||
    (msg.fields[5U].name != "icov_xz") ||
    (msg.fields[6U].name != "icov_yy") ||
    (msg.fields[7U].name != "icov_yz") ||
    (msg.fields[8U].name != "icov_zz") ||
    (msg.fields[9U].name != "cell_id"))
  {
    return 0U;
  }


  // If the actual size and the meta data is in conflict, use the minimum length to be safe.
  const auto min_data_length = std::min(
    static_cast<decltype(msg.row_step)>(msg.data.size()),
    std::min(msg.row_step, msg.width * msg.point_step));
  // Trim the length to make it divisible to point_step, excess data cannot be read.
  const auto safe_data_length = min_data_length - (min_data_length % msg.point_step);
  // Return number of points that can safely be read from the point cloud
  ret = safe_data_length / msg.point_step;

  return ret;
}

void DynamicNDTMap::set(const sensor_msgs::msg::PointCloud2 & msg)
{
  clear();
  insert(msg);
}

void DynamicNDTMap::insert(const sensor_msgs::msg::PointCloud2 & msg)
{
  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it(msg, "z");

  while (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end())
  {
    const auto pt = Point({*x_it, *y_it, *z_it});
    const auto voxel_idx = index(pt);
    voxel(voxel_idx).add_observation(pt);  // Add or insert new voxel.

    ++x_it;
    ++y_it;
    ++z_it;
  }
  // try to stabilizie the covariance after inserting all the points
  for (auto & vx_it : *this) {
    auto & vx = vx_it.second;
    (void) vx.try_stabilize();
  }

  set_frame_id(msg.header.frame_id);
  set_stamp(::time_utils::from_message(msg.header.stamp));
}

void StaticNDTMap::set(const sensor_msgs::msg::PointCloud2 & msg)
{
  clear();
  deserialize_from(msg);
  set_frame_id(msg.header.frame_id);
  set_stamp(::time_utils::from_message(msg.header.stamp));
}

void StaticNDTMap::deserialize_from(const sensor_msgs::msg::PointCloud2 & msg)
{
  if (validate_pcl_map(msg) == 0U) {
    // throwing rather than silently failing since ndt matching cannot be done with an
    // empty/incorrect map
    throw std::runtime_error(
            "Point cloud representing the ndt map is either empty"
            "or does not have the correct format.");
  }

  sensor_msgs::PointCloud2ConstIterator<Real> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<Real> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<Real> z_it(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<Real> icov_xx_it(msg, "icov_xx");
  sensor_msgs::PointCloud2ConstIterator<Real> icov_xy_it(msg, "icov_xy");
  sensor_msgs::PointCloud2ConstIterator<Real> icov_xz_it(msg, "icov_xz");
  sensor_msgs::PointCloud2ConstIterator<Real> icov_yy_it(msg, "icov_yy");
  sensor_msgs::PointCloud2ConstIterator<Real> icov_yz_it(msg, "icov_yz");
  sensor_msgs::PointCloud2ConstIterator<Real> icov_zz_it(msg, "icov_zz");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> cell_id_it(msg, "cell_id");

  while (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end() &&
    icov_xx_it != icov_xx_it.end() &&
    icov_xy_it != icov_xy_it.end() &&
    icov_xz_it != icov_xz_it.end() &&
    icov_yy_it != icov_yy_it.end() &&
    icov_yz_it != icov_yz_it.end() &&
    icov_zz_it != icov_zz_it.end() &&
    cell_id_it != cell_id_it.end())
  {
    const Point centroid{*x_it, *y_it, *z_it};
    const auto voxel_idx = index(centroid);

    // Since no native usigned long support is vailable for a point field
    // the `cell_id_it` points to an array of two 32 bit integers to represent
    // a long number. So the assignments must be done via memcpy.
    Grid::key_type received_idx = 0U;
    std::memcpy(&received_idx, &cell_id_it[0U], sizeof(received_idx));

    // If the pointcloud does not represent a voxel grid of identical configuration,
    // report the error
    if (voxel_idx != received_idx) {
      throw std::domain_error(
              "NDTVoxelMap: Pointcloud representing the ndt map"
              "does not have a matching grid configuration with "
              "the map representation it is being inserted to. The cell IDs do not matchb");
    }

    Eigen::Matrix3d inv_covariance;
    inv_covariance << *icov_xx_it, *icov_xy_it, *icov_xz_it,
      *icov_xy_it, *icov_yy_it, *icov_yz_it,
      *icov_xz_it, *icov_yz_it, *icov_zz_it;
    const Voxel vx{centroid, inv_covariance};

    const auto insert_res = emplace(voxel_idx, Voxel{centroid, inv_covariance});
    if (!insert_res.second) {
      // if a voxel already exist at this point, replace.
      insert_res.first->second = vx;
    }

    ++x_it;
    ++y_it;
    ++z_it;
    ++icov_xx_it;
    ++icov_xy_it;
    ++icov_xz_it;
    ++icov_yy_it;
    ++icov_yz_it;
    ++icov_zz_it;
    ++cell_id_it;
  }
}
}  // namespace ndt
}  // namespace localization
}  // namespace autoware
