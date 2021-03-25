// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the tracking_test_framework classes which are responsible for
/// generating the 2D objects for unit testing tracking algorithms.

#ifndef TRACKING_TEST_FRAMEWORK__TRACKING_TEST_FRAMEWORK_HPP_
#define TRACKING_TEST_FRAMEWORK__TRACKING_TEST_FRAMEWORK_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tracking_test_framework/visibility_control.hpp>

#include <cstdint>
#include <vector>
#include <memory>

/// \namespace autoware
namespace autoware
{
/// \namespace autoware::tracking_test_framework
namespace tracking_test_framework
{

/// \brief Computes cross product of two 2D vectors
/// \param[in] vec1 the first 2D vector
/// \param[in] vec2 the second 2D vector
/// \return returns the cross product
inline float cross_2d(const Eigen::Vector2f & vec1, const Eigen::Vector2f & vec2)
{
  return (vec1.x() * vec2.y()) - (vec1.y() * vec2.x());
}

/// Forward declaration of Line class
class TRACKING_TEST_FRAMEWORK_PUBLIC Line;


/// \brief This is the base class for the 2D shapes
class TRACKING_TEST_FRAMEWORK_PUBLIC Shape
{
public:
  /// \brief virtual method to get intersection points with lidar for the 2D objects
  /// \param[in] line the Line object representing the lidar ray
  /// \param[in] closest_point_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the intersection points of the tracked objects with lidar
  virtual std::vector<Eigen::Vector2f> intersect_with_lidar(
    const Line & line,
    const bool closest_point_only) const = 0;

  /// \brief Virtual destructor
  virtual ~Shape() = default;
};

/// \brief This is the class which represents the lidar ray as a line and defines the function
/// intersect_with_lidar which gives the intersection points with another line (ray)
class TRACKING_TEST_FRAMEWORK_PUBLIC Line : public Shape
{
public:
  /// \brief constructor
  Line(const Eigen::Vector2f & start, const Eigen::Vector2f & end);

  /// \brief Method to get intersection points with lidar with the input line(or another lidar ray)
  /// \param[in] line the Line object representing the lidar ray
  /// \param[in] closest_point_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the intersection points of the tracked objects with lidar
  std::vector<Eigen::Vector2f> intersect_with_lidar(
    const Line & line,
    const bool closest_point_only) const override;

  /// \brief gets the point on the line at the input scale
  /// \param[in] scale the scalar qty `t` representing the scaling factor for the line in equation
  /// p = p^ + t.d
  /// \return returns the point on the line
  Eigen::Vector2f get_point(const float_t scale) const;

  /// \brief gets the starting point of the line
  /// \return returns the starting point
  inline const Eigen::Vector2f & get_start_pt() const
  {
    return m_start;
  }

  /// \brief gets the end point of the line
  /// \return returns the end point
  inline const Eigen::Vector2f & get_end_pt() const
  {
    return m_end;
  }

  /// \brief gets the line direction
  /// \return returns the line direction
  inline const Eigen::Vector2f & get_line_dir() const
  {
    return m_line_direction;
  }

  /// \brief gets the line length
  /// \return returns the line length
  inline const float_t & get_line_length() const
  {
    return m_line_length;
  }

private:
  /// starting point of the line (xs,ys)
  Eigen::Vector2f m_start;
  /// ending point of the line (xe,ye)
  Eigen::Vector2f m_end;
  /// line length
  float_t m_line_length;
  /// vector representing direction of line
  Eigen::Vector2f m_line_direction;
};

/// \brief This is the class which represents the cars as rectangles and defines the function
/// intersect_with_lidar which gives the intersection points with the lidar represented as a line
class TRACKING_TEST_FRAMEWORK_PUBLIC Rectangle : public Shape
{
public:
  /// \brief constructor
  Rectangle(
    const Eigen::Vector2f & center, const Eigen::Vector2f & size,
    const float_t orientation_degrees);

  /// \brief Method to get intersection points with rectangle and the input line(or lidar)
  /// \param[in] line the Line object representing the lidar ray
  /// \param[in] closest_point_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the intersection points of the tracked objects with lidar
  std::vector<Eigen::Vector2f> intersect_with_lidar(
    const Line & line, const bool
    closest_point_only) const override;

private:
  /// center of the rectangle (xr,yr)
  Eigen::Vector2f m_center;
  /// size of the rectangle (l ,b)
  Eigen::Vector2f m_size;
  /// orientation of the rectangle in degrees
  float_t m_orientation_degrees;
  /// array representing the four borders of the rectangle
  std::array<Line, 4> m_borders {Line(Eigen::Vector2f(), Eigen::Vector2f()), Line(
      Eigen::Vector2f(),
      Eigen::Vector2f()), Line(
      Eigen::Vector2f(),
      Eigen::Vector2f()), Line(Eigen::Vector2f(), Eigen::Vector2f())};
  /// array representing the four corner points of the rectangle
  std::array<Eigen::Vector2f, 4> m_corners;
};

/// \brief This is the class which represents the pedestrians as circles and defines the
/// function intersect_with_lidar which gives the intersection points with the lidar represented
/// as a line
class TRACKING_TEST_FRAMEWORK_PUBLIC Circle : public Shape
{
public:
  /// \brief constructor
  Circle(const Eigen::Vector2f & center, const float_t radius);

  /// \brief Method to get intersection points with circle and the input line(or lidar)
  /// \param[in] line the Line object representing the lidar ray
  /// \param[in] closest_point_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the intersection points of the tracked objects with lidar
  std::vector<Eigen::Vector2f> intersect_with_lidar(
    const Line & line, const bool
    closest_point_only) const override;

private:
  /// center of the circle (xc,yc)
  Eigen::Vector2f m_center;
  /// radius of the circle (r)
  float_t m_radius;
};

}  /// namespace tracking_test_framework
}  /// namespace autoware

#endif  /// TRACKING_TEST_FRAMEWORK__TRACKING_TEST_FRAMEWORK_HPP_
