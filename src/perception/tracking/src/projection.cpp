// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <tracking/projection.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace autoware
{
namespace perception
{
namespace tracking
{
CameraModel::CameraModel(
  const CameraIntrinsics & intrinsics,
  const geometry_msgs::msg::Transform & tf_ego_to_camera
)
: m_height_interval{-static_cast<float32_t>(intrinsics.height) / 2.0F,
    static_cast<float32_t>(intrinsics.height) / 2.0F},
  m_width_interval{-static_cast<float32_t>(intrinsics.width) / 2.0F,
    static_cast<float32_t>(intrinsics.width) / 2.0F}
{
  Eigen::Matrix3f intrinsic_matrix{};
  intrinsic_matrix <<
    intrinsics.fx, intrinsics.skew, intrinsics.ox,
    0.0, intrinsics.fy, intrinsics.oy,
    0.0, 0.0, 1.0;
  Eigen::Transform<float32_t, 3U, Eigen::Affine> eig_camera_to_ego_tf;
  eig_camera_to_ego_tf = tf2::transformToEigen(tf_ego_to_camera).cast<float32_t>();
  m_projector = intrinsic_matrix * eig_camera_to_ego_tf;
}

Projection CameraModel::project(const autoware_auto_msgs::msg::Shape & shape)
{
  Projection result;
  const auto & points_3d = shape.polygon.points;
  auto & points2d = result.shape;
  for (auto level = 0U; level < 2U; ++level) {
    const auto z_offset = static_cast<float>(level) * shape.height;
    for (const auto & pt : points_3d) {
      Eigen::Vector3f eig_pt{pt.x, pt.y, pt.z + z_offset};
      // `m_projector * p_3d = p_2d * depth`
      auto pt_2d = m_projector * eig_pt;
      const auto depth = pt_2d.z();
      pt_2d = pt_2d / depth;
      // Only accept points are in front of the camera
      if (depth > 0.0F) {
        // Clamp points outside of the image plane
        pt_2d.x() = Interval::clamp_to(m_width_interval, pt_2d.x());
        pt_2d.y() = Interval::clamp_to(m_height_interval, pt_2d.y());
        points2d.emplace_back(pt_2d);
      }
    }
  }
  // Outline the shape of the projected points in the image
  const auto & end_of_shape_it = common::geometry::convex_hull(points2d);
  // Discard the internal points of the shape
  points2d.resize(static_cast<uint32_t>(std::distance(points2d.cbegin(), end_of_shape_it)));
  return result;
}

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
