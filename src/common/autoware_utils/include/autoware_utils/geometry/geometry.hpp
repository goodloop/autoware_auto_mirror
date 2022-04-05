// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE_UTILS__GEOMETRY__GEOMETRY_HPP_
#define AUTOWARE_UTILS__GEOMETRY__GEOMETRY_HPP_

#include <exception>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "autoware_utils/geometry/boost_geometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace autoware_utils
{

/// \brief Get point as a geometry_msgs::msg::Point message type
/// \tparam T Type of point
/// \param p Point
/// \return Point as a geometry_msgs::msg::Point message type
template<class T>
geometry_msgs::msg::Point getPoint(const T & p)
{
  return geometry_msgs::build<geometry_msgs::msg::Point>().x(p.x).y(p.y).z(p.z);
}

/// \brief Get Point
/// \param p Point message
/// \return Point message
template<>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Point & p)
{
  return p;
}

/// \brief Get position field of the geometry_msgs::msg::Pose message
/// \param p Pose message
/// \return Position field of the geometry_msgs::msg::Pose message
template<>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Pose & p)
{
  return p.position;
}

/// \brief Get position field from the geometry_msgs::msg::PoseStamped message
/// \param p PoseStamped message
/// \return Position field of the geometry_msgs::msg::PoseStamped message
template<>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::PoseStamped & p)
{
  return p.pose.position;
}

/// \brief Get position field from the geometry_msgs::msg::PoseWithCovarianceStamped message
/// \param p PoseWithCovarianceStamped message
/// \return Position field of the geometry_msgs::msg::PoseWithCovarianceStamped message
template<>
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::PoseWithCovarianceStamped & p)
{
  return p.pose.pose.position;
}

/// \brief Get position field from the autoware_auto_planning_msgs::msg::PathPoint message
/// \param p autoware_auto_planning_msgs::msg::PathPoint message
/// \return Position field of the autoware_auto_planning_msgs::msg::PathPoint message
template<>
inline geometry_msgs::msg::Point getPoint(const autoware_auto_planning_msgs::msg::PathPoint & p)
{
  return p.pose.position;
}

/// \brief Get position field from the autoware_auto_planning_msgs::msg::TrajectoryPoint message
/// \param p autoware_auto_planning_msgs::msg::TrajectoryPoint message
/// \return Position field of the autoware_auto_planning_msgs::msg::TrajectoryPoint message
template<>
inline geometry_msgs::msg::Point getPoint(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & p)
{
  return p.pose.position;
}

/// \brief Get position field from the autoware_auto_planning_msgs::msg::PathPointWithLaneId message
/// \param p autoware_auto_planning_msgs::msg::PathPointWithLaneId message
/// \return Position field of the autoware_auto_planning_msgs::msg::PathPointWithLaneId message
template<>
inline geometry_msgs::msg::Point getPoint(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose.position;
}

template<class T>
geometry_msgs::msg::Pose getPose([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
  throw std::logic_error("Only specializations of getPose can be used.");
}

/// \brief Get geometry_msgs::msg::Pose message
/// \param p Pose message
/// \return Pose message
template<>
inline geometry_msgs::msg::Pose getPose(const geometry_msgs::msg::Pose & p)
{
  return p;
}

/// \brief Get pose from the geometry_msgs::msg::PoseStamped message
/// \param p PoseStamped message
/// \return Pose field of the message
template<>
inline geometry_msgs::msg::Pose getPose(const geometry_msgs::msg::PoseStamped & p)
{
  return p.pose;
}

/// \brief Get pose from the autoware_auto_planning_msgs::msg::PathPoint message
/// \param p autoware_auto_planning_msgs::msg::PathPoint message
/// \return Pose field of the message
template<>
inline geometry_msgs::msg::Pose getPose(const autoware_auto_planning_msgs::msg::PathPoint & p)
{
  return p.pose;
}

/// \brief Get pose from the autoware_auto_planning_msgs::msg::TrajectoryPoint message
/// \param p autoware_auto_planning_msgs::msg::TrajectoryPoint message
/// \return Pose field of the message
template<>
inline geometry_msgs::msg::Pose getPose(const autoware_auto_planning_msgs::msg::TrajectoryPoint & p)
{
  return p.pose;
}

/// \brief Create geometry_msgs::msg::Point from x,y,z values
/// \param x The x position of the point
/// \param y The y position of the point
/// \param z The z position of the point
/// \return Point as a geometry_msgs::msg::Point message
inline geometry_msgs::msg::Point createPoint(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

/// \brief Get roll, pitch, yaw values from the Quaternion message
/// \param quat Quaternion message
/// \return Roll, Pitch, Yaw values
inline geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Quaternion & quat)
{
  geometry_msgs::msg::Vector3 rpy;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

/// \brief Get roll, pitch, yaw values from the Pose message
/// \param pose Pose message
/// \return Roll, Pitch, Yaw values
inline geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Pose & pose)
{
  return getRPY(pose.orientation);
}

/// \brief Get roll, pitch, yaw values from the PoseStamped message
/// \param pose PoseStamped message
/// \return Roll, Pitch, Yaw values
inline geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseStamped & pose)
{
  return getRPY(pose.pose);
}

/// \brief Get roll, pitch, yaw values from the PoseWithCovarianceStamped message
/// \param pose PoseWithCovarianceStamped message
/// \return Roll, Pitch, Yaw values
inline geometry_msgs::msg::Vector3 getRPY(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  return getRPY(pose.pose.pose);
}

/// \brief Create quaternion message from x,y,z,w values
/// \param x The x field of the quaternion
/// \param y The y field of the quaternion
/// \param z The z field of the quaternion
/// \param w The w field of the quaternion
/// \return Quaternion message
inline geometry_msgs::msg::Quaternion createQuaternion(
  const double x, const double y, const double z, const double w)
{
  geometry_msgs::msg::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

/// \brief Create translation from x,y,z values
/// \param x The x field of the translation
/// \param y The y field of the translation
/// \param z The z field of the translation
/// \return Translation as a Vector3 message
inline geometry_msgs::msg::Vector3 createTranslation(const double x, const double y, const double z)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

/// \brief Create Quaternion message from roll, pitch, yaw angle
/// \details Revival of tf::createQuaternionFromRPY
/// https://answers.ros.org/question/304397/recommended-way-to-construct-quaternion-from-rollpitchyaw-with-tf2/
/// \param roll The roll angle
/// \param pitch The pitch angle
/// \param yaw The yaw angle
/// \return Quaternion message
inline geometry_msgs::msg::Quaternion createQuaternionFromRPY(
  const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

/// \brief Create Quaternion message from only yaw angle
/// \param yaw The yaw angle
/// \return Quaternion message
inline geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

/// \brief Compute euclidean distance between two points
/// \tparam Point1 Point type of the point1. Must have x and y members
/// \tparam Point2 Point type of the point2. Must have x and y members
/// \param point1 Point 1
/// \param point2 Point 2
/// \return euclidean distance
template<class Point1, class Point2>
double calcDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

/// \brief Compute squared euclidean distance between two points
/// \tparam Point1 Point type of the point1. Must have x and y members
/// \tparam Point2 Point type of the point2. Must have x and y members
/// \param point1 Point 1
/// \param point2 Point 2
/// \return squared euclidean distance
template<class Point1, class Point2>
double calcSquaredDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  const auto dx = p1.x - p2.x;
  const auto dy = p1.y - p2.y;
  return dx * dx + dy * dy;
}

/// \brief T1, T2 point type. Must have float members x, y and z
/// \tparam Point1  Point type of the point1. Must have x,y and z members
/// \tparam Point2  Point type of the point2. Must have x,y and z members
/// \param point1 Point 1
/// \param point2 Point 2
/// \return squared 3D euclidean distance
template<class Point1, class Point2>
double calcDistance3d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  // To be replaced by std::hypot(dx, dy, dz) in C++17
  return std::hypot(std::hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}

/**
 * @brief calculate elevation angle of two points.
 * @details This function returns the elevation angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If the two points are in the same position, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi/2 <= elevation angle <= pi/2
 */
inline double calcElevationAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dz = p_to.z - p_from.z;
  const double dist_2d = calcDistance2d(p_from, p_to);
  return std::atan2(dz, dist_2d);
}

/**
 * @brief calculate azimuth angle of two points.
 * @details This function returns the azimuth angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If x and y of the two points are the same, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi < azimuth angle < pi.
 */
inline double calcAzimuthAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dx = p_to.x - p_from.x;
  const double dy = p_to.y - p_from.y;
  return std::atan2(dy, dx);
}

/// \brief Convert Transform message to Pose message
/// \param transform Transform message
/// \return Pose message
inline geometry_msgs::msg::Pose transform2pose(const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return pose;
}

/// \brief Convert TransformStamped message to PoseStamped message
/// \param transform TransformStamped message
/// \return PoseStamped message
inline geometry_msgs::msg::PoseStamped transform2pose(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose = transform2pose(transform.transform);
  return pose;
}

/// \brief Convert Pose message to Transform message
/// \param pose Pose message
/// \return Transform message
inline geometry_msgs::msg::Transform pose2transform(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;
  transform.rotation = pose.orientation;
  return transform;
}

/// \brief Convert PoseStamped message to TransformStamped message
/// \param pose PoseStamped message
/// \return TransformStamped message
inline geometry_msgs::msg::TransformStamped pose2transform(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header = pose.header;
  transform.transform = pose2transform(pose.pose);
  return transform;
}

/// \brief Transform 3D point using the given transform
/// \param point Point to be transformed
/// \param transform Transform to be used
/// \return Transformed point
inline Point3d transformPoint(
  const Point3d & point, const geometry_msgs::msg::Transform & transform)
{
  const auto & translation = transform.translation;
  const auto & rotation = transform.rotation;

  const Eigen::Translation3d T(translation.x, translation.y, translation.z);
  const Eigen::Quaterniond R(rotation.w, rotation.x, rotation.y, rotation.z);

  const Eigen::Vector3d transformed(T * R * point);

  return Point3d{transformed.x(), transformed.y(), transformed.z()};
}

/// \brief Transform 2D point using the given transform
/// \param point Point to be transformed
/// \param transform Transform message to be used for transform operation
/// \return Transformed point
inline Point2d transformPoint(
  const Point2d & point, const geometry_msgs::msg::Transform & transform)
{
  Point3d point_3d{point.x(), point.y(), 0};
  const auto transformed = transformPoint(point_3d, transform);
  return Point2d{transformed.x(), transformed.y()};
}

/// \brief Transform points of the vector using the given transform
/// \tparam T Type of the point
/// \param points Point to be transformed
/// \param transform Transform message to be used
/// \return Transformed vector of point
template<class T>
T transformVector(const T & points, const geometry_msgs::msg::Transform & transform)
{
  T transformed;
  for (const auto & point : points) {
    transformed.push_back(transformPoint(point, transform));
  }
  return transformed;
}

/// \brief calculate Menger curvature of given three points
/// https://en.wikipedia.org/wiki/Menger_curvature
/// \param p1 Point 1
/// \param p2 Point 2
/// \param p3 Point 3
/// \return curvature
inline double calcCurvature(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3)
{
  const double denominator =
    calcDistance2d(p1, p2) * calcDistance2d(p2, p3) * calcDistance2d(p3, p1);
  if (std::fabs(denominator) < 1e-10) {
    throw std::runtime_error("points are too close for curvature calculation.");
  }
  return 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / denominator;
}

/**
 * @brief Calculate offset pose. The offset values are defined in the local coordinate of the input
 * pose.
 */

/// \brief Calculate offset pose. The offset values are defined in the local coordinate
/// of the input pose.
/// \param p Input pose
/// \param x The x value of the offset
/// \param y The y value of the offset
/// \param z The z value of the offset
/// \return Offset pose
inline geometry_msgs::msg::Pose calcOffsetPose(
  const geometry_msgs::msg::Pose & p, const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Transform transform;
  transform.translation = createTranslation(x, y, z);
  transform.rotation = createQuaternion(0.0, 0.0, 0.0, 1.0);
  tf2::Transform tf_pose;
  tf2::Transform tf_offset;
  tf2::fromMsg(transform, tf_offset);
  tf2::fromMsg(p, tf_pose);
  tf2::toMsg(tf_pose * tf_offset, pose);
  return pose;
}
}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__GEOMETRY__GEOMETRY_HPP_
