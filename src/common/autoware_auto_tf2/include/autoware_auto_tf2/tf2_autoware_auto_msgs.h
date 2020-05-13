// Copyright 2020, The Autoware Foundation.
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
/// \brief This file includes common transoform functionaly for autoware_auto_msgs

#ifndef TF2_AUTOWARE_AUTO_MSGS_H
#define TF2_AUTOWARE_AUTO_MSGS_H

#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/bounding_box.hpp>
#include <autoware_auto_msgs/msg/quaternion32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <common/types.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_auto_tf2/tf2_geometry_msgs_extended.h>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware_auto_msgs::msg::BoundingBoxArray;
using autoware_auto_msgs::msg::BoundingBox;

namespace tf2
{

/** \brief Convert a geometry_msgs Point message to Point32.
 * \param in Point message to convert.
 * \return The converted Point32 message.
 */
inline
geometry_msgs::msg::Point32 toPoint32(
  const geometry_msgs::msg::Point & in)
{
  geometry_msgs::msg::Point32 out;
  out.x = static_cast<float>(in.x);
  out.y = static_cast<float>(in.y);
  out.z = static_cast<float>(in.z);
  return out;
}

/** \brief Convert a geometry_msgs Point32 message to Point.
 * \param in Point32 message to convert.
 * \return The converted Point message.
 */
inline
geometry_msgs::msg::Point toPoint(
  const geometry_msgs::msg::Point32 & in)
{
  geometry_msgs::msg::Point out;
  out.x = static_cast<float64_t>(in.x);
  out.y = static_cast<float64_t>(in.y);
  out.z = static_cast<float64_t>(in.z);
  return out;
}

/** \brief Convert a geometry_msgs Quaternion message to Quaternion32.
 * \param in Quaternion message to convert.
 * \return The converted Quaternion32 message.
 */
inline
autoware_auto_msgs::msg::Quaternion32 toQuaternion32(
  const geometry_msgs::msg::Quaternion & in)
{
  autoware_auto_msgs::msg::Quaternion32 out;
  out.x = static_cast<float32_t>(in.x);
  out.y = static_cast<float32_t>(in.y);
  out.z = static_cast<float32_t>(in.z);
  out.w = static_cast<float32_t>(in.w);
  return out;
}

/** \brief Convert a geometry_msgs Quaternion32 message to Quaternion.
 * \param in Quaternion32 message to convert.
 * \return The converted Quaternion message.
 */
inline
geometry_msgs::msg::Quaternion toQuaternion(
  const autoware_auto_msgs::msg::Quaternion32 & in)
{
  geometry_msgs::msg::Quaternion out;
  out.x = static_cast<float64_t>(in.x);
  out.y = static_cast<float64_t>(in.y);
  out.z = static_cast<float64_t>(in.z);
  out.w = static_cast<float64_t>(in.w);
  return out;
}


/******************/
/** Quaternion32 **/
/******************/

/** \brief Convert a tf2 Quaternion type to autoware_auto_msgs Quaternion32 representation.
 * \param in A tf2 Quaternion object.
 * \param out The Quaternion converted to a autoware_auto_msgs Quaternion32 message.
 */
void toMsg(
  const tf2::Quaternion & in,
  autoware_auto_msgs::msg::Quaternion32 & out)
{
  out.w = static_cast<float32_t>(in.getW());
  out.x = static_cast<float32_t>(in.getX());
  out.y = static_cast<float32_t>(in.getY());
  out.z = static_cast<float32_t>(in.getZ());
}

/** \brief Convert a autoware_auto_msgs Quaternion32 message to its equivalent tf2 representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A Quaternion32 message.
 * \param out The Quaternion32 converted to a tf2 type.
 */
inline
void fromMsg(const autoware_auto_msgs::msg::Quaternion32 & in, tf2::Quaternion & out)
{
  // w at the end in the constructor
  out = tf2::Quaternion(in.x, in.y, in.z, in.w);
}

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs Quaternion32 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The Quaternion32 message to transform.
 * \param t_out The transformed Quaternion32 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const autoware_auto_msgs::msg::Quaternion32 & t_in,
  autoware_auto_msgs::msg::Quaternion32 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Quaternion q_out = tf2::Quaternion(transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w) *
    tf2::Quaternion(t_in.x, t_in.y, t_in.z, t_in.w);
  toMsg(q_out, t_out);
}


/******************/
/** BoundingBox **/
/******************/

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs BoundingBox type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The BoundingBox message to transform.
 * \param t_out The transformed BoundingBox message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline


void doTransform(
  const BoundingBox & t_in, BoundingBox & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;
  geometry_msgs::msg::PoseStamped in_pose, out_pose;
  in_pose.pose.position = toPoint(t_in.centroid);
  in_pose.pose.orientation = toQuaternion(t_in.orientation);
  doTransform(in_pose, out_pose, transform);
  t_out.centroid = toPoint32(out_pose.pose.position);
  t_out.orientation = toQuaternion32(out_pose.pose.orientation);

  doTransform(t_in.corners[0], t_out.corners[0], transform);
  doTransform(t_in.corners[1], t_out.corners[1], transform);
  doTransform(t_in.corners[2], t_out.corners[2], transform);
  doTransform(t_in.corners[3], t_out.corners[3], transform);
}


/**********************/
/** BoundingBoxArray **/
/**********************/

/** \brief Extract a timestamp from the header of a BoundingBoxArray message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t A timestamped BoundingBoxArray message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template<>
inline
tf2::TimePoint getTimestamp(const BoundingBoxArray::SharedPtr & t)
{
  return tf2_ros::fromMsg(t->header.stamp);
}

/** \brief Extract a frame ID from the header of a BoundingBoxArray message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t A timestamped BoundingBoxArray message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template<>
inline
std::string getFrameId(const BoundingBoxArray::SharedPtr & t) {return t->header.frame_id;}

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs BoundingBoxArray type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The BoundingBoxArray to transform, as a timestamped BoundingBoxArray message.
 * \param t_out The transformed BoundingBoxArray, as a timestamped BoundingBoxArray message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const BoundingBoxArray::SharedPtr & t_in,
  BoundingBoxArray::SharedPtr & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;
  for (auto idx = 0U; idx < t_in->boxes.size(); idx++) {
    doTransform(t_out->boxes[idx], t_out->boxes[idx], transform);
  }
}

} // namespace

#endif // TF2_AUTOWARE_AUTO_MSGS_H
