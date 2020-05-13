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
/// \brief This file includes common transoform functionaly for geometry_msgs


#ifndef TF2_GEOMETRY_MSGS_EXTENDED_H
#define TF2_GEOMETRY_MSGS_EXTENDED_H

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2
{

/***********/
/** Point **/
/***********/

/** \brief Convert a tf2 Vector3 type to its geometry_msgs Point representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \param out The Vector3 converted to a geometry_msgs Point message type.
 */
inline
void toMsg(const tf2::Vector3 & in, geometry_msgs::msg::Point & out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
}

/** \brief Convert a geometry_msgs Point message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Point message type.
 * \param out The Point converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::msg::Point & in, tf2::Vector3 & out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a Point message.
 * \param t_out The transformed point, as a Point message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(
  const geometry_msgs::msg::Point & t_in, geometry_msgs::msg::Point & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Vector3 v_in;
  fromMsg(t_in, v_in);
  tf2::Vector3 v_out = t * v_in;
  toMsg(v_out, t_out);
}


/*************/
/** Point32 **/
/*************/

/** \brief Convert a tf2 Vector3 type to its geometry_msgs Point32 representation.
 * \param in A tf2 Vector3 object.
 * \param out The Vector3 converted to a geometry_msgs Point32 message type.
 */
inline
void toMsg(const tf2::Vector3 & in, geometry_msgs::msg::Point32 & out)
{
  out.x = static_cast<float>(in.getX());
  out.y = static_cast<float>(in.getY());
  out.z = static_cast<float>(in.getZ());
}

/** \brief Convert a geometry_msgs Point32 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Point32 message.
 * \param out The Point converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::msg::Point32 & in, tf2::Vector3 & out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

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
  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Vector3 v_in;
  fromMsg(t_in, v_in);
  tf2::Vector3 v_out = t * v_in;
  toMsg(v_out, t_out);
}

} // namespace

#endif // TF2_GEOMETRY_MSGS_EXTENDED_H
