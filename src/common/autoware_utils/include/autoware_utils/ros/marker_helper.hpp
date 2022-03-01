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

#ifndef AUTOWARE_UTILS__ROS__MARKER_HELPER_HPP_
#define AUTOWARE_UTILS__ROS__MARKER_HELPER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

namespace autoware_utils
{
/// \brief Create marker position from x,y,z values
/// \param x The x position of the marker
/// \param y The y position of the marker
/// \param z The z position of the marker
/// \return Point message
inline geometry_msgs::msg::Point createMarkerPosition(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}
/// \brief Create marker orientation from x,y,z,w values
/// \param x The x orientation of the marker
/// \param y The y orientation of the marker
/// \param z The z orientation of the marker
/// \param w The w orientation of the marker
/// \return Quaternion message
inline geometry_msgs::msg::Quaternion createMarkerOrientation(
  double x, double y, double z, double w)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;
  return quaternion;
}
/// \brief Create marker scale from x,y,z values
/// \param x The x value of the marker scale
/// \param y The y value of the marker scale
/// \param z The z value of the marker scale
/// \return Vector3 message
inline geometry_msgs::msg::Vector3 createMarkerScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}
/// \brief Create marker color from r,g,b,a values
/// \param r  The r value of the marker color
/// \param g  The g value of the marker color
/// \param b  The b value of the marker color
/// \param a  The a value of the marker color
/// \return ColorRGBA message
inline std_msgs::msg::ColorRGBA createMarkerColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}
/// \brief Create default marker
/// \param frame_id frame id
/// \param now time value to be timestamp of the message
/// \param ns namespace
/// \param id id of the marker
/// \param type type of the marker
/// \param scale scale of the marker
/// \param color color of the marker
/// \return Marker message
inline visualization_msgs::msg::Marker createDefaultMarker(
  const std::string & frame_id,
  const rclcpp::Time & now,
  const std::string & ns,
  const int32_t id,
  const int32_t type,
  const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = now;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  marker.pose.position = createMarkerPosition(0.0, 0.0, 0.0);
  marker.pose.orientation = createMarkerOrientation(0.0, 0.0, 0.0, 1.0);
  marker.scale = scale;
  marker.color = color;
  marker.frame_locked = true;

  return marker;
}
/// \brief Append given marker array to another marker array
/// \param additional_marker_array additional marker array to be appended
/// \param marker_array original marker array
inline void appendMarkerArray(
  const visualization_msgs::msg::MarkerArray & additional_marker_array,
  visualization_msgs::msg::MarkerArray * marker_array)
{
  for (const auto & marker : additional_marker_array.markers) {
    marker_array->markers.push_back(marker);
  }
}
}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__ROS__MARKER_HELPER_HPP_
