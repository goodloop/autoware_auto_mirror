/*
* Copyright 2021 Tier IV, Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#pragma once

namespace centerpoint
{
class Config
{
public:
  // input params
  constexpr static int num_class = 3;           // car, bicycle and pedestrian
  constexpr static int num_point_dims = 3;      // x, y and z
  constexpr static int num_point_features = 4;  // x, y, z and timelag
  constexpr static int max_num_points_per_voxel = 20;
  constexpr static int max_num_voxels = 55000;
  constexpr static float pointcloud_range_xmin = -90.0f;
  constexpr static float pointcloud_range_ymin = -90.0f;
  constexpr static float pointcloud_range_zmin = -3.0f;
  constexpr static float pointcloud_range_xmax = 90.0f;
  constexpr static float pointcloud_range_ymax = 90.0f;
  constexpr static float pointcloud_range_zmax = 5.0f;
  constexpr static float voxel_size_x = 0.25f;
  constexpr static float voxel_size_y = 0.25f;
  constexpr static float voxel_size_z = 8.0f;
  constexpr static int grid_size_x =
    720;  // = (pointcloud_range_xmax - pointcloud_range_xmin) / voxel_size_x
  constexpr static int grid_size_y =
    720;  // = (pointcloud_range_ymax - pointcloud_range_ymin) / voxel_size_y
  constexpr static int grid_size_z =
    1;  // = (pointcloud_range_zmax - pointcloud_range_zmin) / voxel_size_z
  constexpr static float offset_x = -89.875f;  // pointcloud_range_xmin + voxel_size_x / 2
  constexpr static float offset_y = -89.875f;  // pointcloud_range_ymin + voxel_size_y / 2
  constexpr static float offset_z = 1.0f;      // pointcloud_range_zmin + voxel_size_z / 2

  // output params
  constexpr static int num_box_features = 9;  // score, class, x, y, z, l, w, h, yaw
  constexpr static int max_num_output_objects = 200;

  // network params
  constexpr static int downsample_factor = 4;
  constexpr static int num_vfe_input_features = 10;
  constexpr static int num_vfe_output_features = 32;
  constexpr static int num_output_dim_features = 3;
  constexpr static int num_output_offset_features = 2;
  constexpr static int num_output_rot_features = 2;
  constexpr static int num_output_z_features = 1;
};

}  // namespace centerpoint