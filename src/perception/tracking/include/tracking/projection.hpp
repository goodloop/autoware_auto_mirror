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
#ifndef TRACKING__PROJECTION_HPP_
#define TRACKING__PROJECTION_HPP_

#include <autoware_auto_msgs/msg/shape.hpp>
#include <common/types.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry/convex_hull.hpp>
#include <geometry/common_2d.hpp>
#include <Eigen/Geometry>
#include <geometry/interval.hpp>
#include <list>
#include "visibility_control.hpp"

namespace autoware
{
namespace perception
{
namespace tracking
{
struct TRACKING_PUBLIC Projection
{
  std::list<Eigen::Vector3f> shape;
};

/// \brief Camera intrinsic parameters
struct CameraIntrinsics
{
  std::size_t width{0U};
  std::size_t height{0U};
  float32_t fx{0.0F};
  float32_t fy{0.0F};
  float32_t ox{0.0F};
  float32_t oy{0.0F};
  float32_t skew{0.0F};
};

/// \brief This model represents a camera in 3D space and can project 3D shapes into an image
// frame.
class TRACKING_PUBLIC CameraModel
{
public:
  using Shape = autoware_auto_msgs::msg::Shape;
  using float64_t = common::types::float64_t;
  using float32_t = common::types::float32_t;
  using Interval = common::geometry::Interval<float32_t>;

  /// \brief Cnstructor
  /// \param intrinsics Camera intrinsics
  /// \param tf_ego_to_camera ego->camera transform.
  CameraModel(
    const CameraIntrinsics & intrinsics,
    const geometry_msgs::msg::Transform & tf_ego_to_camera
  );

  /// \brief Bring 3D points in ego frame to the camera frame and project onto the image plane.
  /// All 3D points in a shape (vertices on the base face and the top face) are projected onto
  // the image of the camera model. The resulting list of points are then outlined using the
  // `convex_hull` algorithm and then the outline of points are returned.
  /// `K * p_3d = p_2d * depth` where K is the projection matrix.
  /// \param shape 3D shape to be projected.
  /// \return List of points on the camera frame that outline the given 3D object.
  Projection project(const autoware_auto_msgs::msg::Shape & shape);

private:
  Eigen::Transform<float32_t, 3, Eigen::Affine, Eigen::ColMajor> m_projector;
  Interval m_height_interval;
  Interval m_width_interval;
};

}  // namespace tracking
}  // namespace perception

// These point adapters are needed to make sure Eigen points work with autoware_auto_geometry
// TODO(yunus.caliskan) Move these adapters to a common package.
namespace common
{
namespace geometry
{
namespace point_adapter
{
/// Point adapters for eigen vector
/// These adapters are necessary for the VoxelGrid to know how to access
/// the coordinates from an eigen vector.
template<>
inline TRACKING_PUBLIC auto x_(const Eigen::Vector3f & pt)
{
  return pt.x();
}

template<>
inline TRACKING_PUBLIC auto y_(const Eigen::Vector3f & pt)
{
  return pt.y();
}

template<>
inline TRACKING_PUBLIC auto z_(const Eigen::Vector3f & pt)
{
  return pt.z();
}

template<>
inline TRACKING_PUBLIC auto & xr_(Eigen::Vector3f & pt)
{
  return pt.x();
}

template<>
inline TRACKING_PUBLIC auto & yr_(Eigen::Vector3f & pt)
{
  return pt.y();
}

template<>
inline TRACKING_PUBLIC auto & zr_(Eigen::Vector3f & pt)
{
  return pt.z();
}
}  // namespace point_adapter
}  // namespace geometry
}  // namespace common

}  // namespace autoware


#endif  // TRACKING__PROJECTION_HPP_
