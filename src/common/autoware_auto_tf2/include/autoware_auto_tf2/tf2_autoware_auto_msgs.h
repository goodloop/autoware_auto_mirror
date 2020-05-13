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

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware_auto_msgs::msg::BoundingBoxArray;
using autoware_auto_msgs::msg::BoundingBox;

namespace tf2
{


/*************/
/** Point32 **/
/*************/

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point32 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a Point32 message.
 * \param t_out The transformed point, as a Point32 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::Point32 & t_in, geometry_msgs::msg::Point32 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Vector v_out = gmTransformToKDL(transform) * KDL::Vector(t_in.x, t_in.y, t_in.z);
  t_out.x = static_cast<float>(v_out[0]);
  t_out.y = static_cast<float>(v_out[1]);
  t_out.z = static_cast<float>(v_out[2]);
}


/******************/
/** BoundingBox **/
/******************/

/** \brief Apply a geometry_msgs TransformStamped to an msg pair geometry_msgs Point32 and autoware_auto_msgs Quaternion32.
 * \param p32_in The Point part to transform, as a Point32 message.
 * \param q32_in The Quaternion part to transform, as a Quaternion32 message.
 * \param p32_out The transformed point part, as a Point32 message.
 * \param q32_out The transformed Quaternion part, as a Quaternion32 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
void transformPoint32Quaternion32(
  const geometry_msgs::msg::Point32 & p32_in,
  const autoware_auto_msgs::msg::Quaternion32 & q32_in,
  geometry_msgs::msg::Point32 & p32_out,
  autoware_auto_msgs::msg::Quaternion32 & q32_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Vector v(p32_in.x, p32_in.y, p32_in.z);
  KDL::Rotation r = KDL::Rotation::Quaternion(q32_in.x, q32_in.y, q32_in.z, q32_in.w);

  KDL::Frame out = gmTransformToKDL(transform) * KDL::Frame(r, v);

  p32_out.x = static_cast<float32_t>(out.p[0]);
  p32_out.y = static_cast<float32_t>(out.p[1]);
  p32_out.z = static_cast<float32_t>(out.p[2]);

  double qx, qy, qz, qw;
  out.M.GetQuaternion(qx, qy, qz, qw);
  q32_out.x = static_cast<float32_t>(qx);
  q32_out.y = static_cast<float32_t>(qy);
  q32_out.z = static_cast<float32_t>(qz);
  q32_out.w = static_cast<float32_t>(qw);
}

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
  transformPoint32Quaternion32(
    t_in.centroid, t_in.orientation, t_out.centroid, t_out.orientation, transform);
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
tf2::TimePoint getTimestamp(const BoundingBoxArray & t)
{
  return tf2_ros::fromMsg(t.header.stamp);
}

/** \brief Extract a frame ID from the header of a BoundingBoxArray message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t A timestamped BoundingBoxArray message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template<>
inline
std::string getFrameId(const BoundingBoxArray & t) {return t.header.frame_id;}

/** \brief Apply a geometry_msgs TransformStamped to an autoware_auto_msgs BoundingBoxArray type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The BoundingBoxArray to transform, as a timestamped BoundingBoxArray message.
 * \param t_out The transformed BoundingBoxArray, as a timestamped BoundingBoxArray message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const BoundingBoxArray & t_in,
  BoundingBoxArray & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  //std::cout<<"BoundingBoxArray:doTransform from "<< t_in->header.frame_id<<" to "<< transform.header.frame_id <<std::endl;
  t_out = t_in;
  for (auto idx = 0U; idx < t_in.boxes.size(); idx++) {
    doTransform(t_out.boxes[idx], t_out.boxes[idx], transform);
  }
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

} // namespace

#endif // TF2_AUTOWARE_AUTO_MSGS_H
