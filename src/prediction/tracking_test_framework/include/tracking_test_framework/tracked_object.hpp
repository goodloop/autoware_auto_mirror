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
/// \brief This file defines the tracking_test_framework classes which contain the
/// APIs for creating the object clusters like Car, Pedestrian and setting up a Scene with the
/// objects for getting intersections with the LiDAR sensor .

#ifndef TRACKING_TEST_FRAMEWORK__TRACKED_OBJECT_HPP_
#define TRACKING_TEST_FRAMEWORK__TRACKED_OBJECT_HPP_

#include <tracking_test_framework/shapes.hpp>
#include <tracking_test_framework/utils.hpp>

#include <chrono>
#include <memory>

namespace autoware
{
namespace tracking_test_framework
{
/// \brief This is the base class for the Tracked objects
class TRACKING_TEST_FRAMEWORK_PUBLIC TrackedObject
{
public:
  /// \brief constructor
  TrackedObject(
    const Eigen::Vector2f & starting_position,
    const autoware::common::types::float32_t starting_speed_mps,
    const autoware::common::types::float32_t orientation_deg,
    const autoware::common::types::float32_t turn_rate_deg_per_sec,
    std::unique_ptr<Shape> shape);

  /// \brief copy constructor
  TrackedObject(const TrackedObject & object) noexcept;

  /// \brief virtual method to get clone of the TrackedObject derived objects
  /// \return return a std::unique_ptr to a copy of the actual derived object
  virtual std::unique_ptr<TrackedObject> clone() const = 0;

  /// \brief Method to move the TrackedObject given the time interval in seconds.This will
  /// update the current position and orientation of the object and also update the shape
  /// associated with it internally.
  /// \param[in] dt_in_ms the time interval in milliseconds
  void move_object(const std::chrono::milliseconds dt_in_ms);

  /// \brief gets the current position of the TrackedObject
  /// \return returns the current position
  inline const Eigen::Vector2f & position() const
  {
    return m_position;
  }

  /// \brief gets the current orientation of the TrackedObject
  /// \return returns the current orientation
  inline autoware::common::types::float32_t orientation() const
  {
    return autoware::tracking_test_framework::utils::wrap_to_2pi(m_orientation_rad);
  }

  /// \brief gets the Shape object associated with the TrackedObject
  /// \return returns the Shape
  inline const Shape * shape() const
  {
    return m_shape.get();
  }

  /// \brief Virtual destructor
  virtual ~TrackedObject() = default;

protected:
  /// \brief virtual method to update the Shape associated with a TrackedObject
  virtual void update_shape() = 0;

  /// unique_ptr holding the Shape of the TrackedObject
  std::unique_ptr<Shape> m_shape;

private:
  /// current position of the TrackedObject
  Eigen::Vector2f m_position{Eigen::Vector2f::Zero()};
  /// speed of the TrackedObject
  autoware::common::types::float32_t m_speed_mps{};
  /// current orientation of the TrackedObject
  autoware::common::types::float32_t m_orientation_rad{};
  /// current orientation of the TrackedObject
  autoware::common::types::float32_t m_turn_rate_rad_per_sec{};
};

/// \brief This is the class which has the APIs to create a Car object to put on the scene.
/// The Car object is internally represented as a 2D Shape object: Rectangle centered at the
/// starting position initially and having a definite size
class TRACKING_TEST_FRAMEWORK_PUBLIC Car : public TrackedObject
{
public:
  /// \brief constructor
  Car(
    const Eigen::Vector2f & starting_position,
    const autoware::common::types::float32_t starting_speed,
    const autoware::common::types::float32_t orientation_deg,
    const autoware::common::types::float32_t turn_rate_deg_per_sec,
    const Eigen::Vector2f & size);

  /// \brief Method to get clone of the unique_ptr to a Car
  /// \return return a std::unique_ptr to a Car object
  inline std::unique_ptr<TrackedObject> clone() const override
  {
    return std::make_unique<Car>(*this);
  }

protected:
  /// \brief Method to update the Shape(here Rectangle) associated with a Car
  void update_shape() override;

private:
  /// size of the Car in 2D \f$(x ,y)\f$
  Eigen::Vector2f m_size{Eigen::Vector2f::Zero()};
};

/// \brief This is the class which has the APIs to create a Pedestrian object to put on the scene
/// The Pedestrian object is internally represented as a 2D Shape object: Circle centered at
/// the starting position initially and having a fixed radius
class TRACKING_TEST_FRAMEWORK_PUBLIC Pedestrian : public TrackedObject
{
public:
  /// \brief constructor
  Pedestrian(
    const Eigen::Vector2f & starting_position,
    const autoware::common::types::float32_t starting_speed,
    const autoware::common::types::float32_t orientation_deg,
    const autoware::common::types::float32_t turn_rate_deg_per_sec);

  /// \brief Method to get clone of the unique_ptr to a Pedestrian
  /// \return return a std::unique_ptr to a Pedestrian object
  inline std::unique_ptr<TrackedObject> clone() const override
  {
    return std::make_unique<Pedestrian>(*this);
  }

protected:
  /// \brief Method to update the Shape(here Circle) associated with a Pedestrian
  void update_shape() override;
};


}  // namespace tracking_test_framework
}  // namespace autoware

#endif  // TRACKING_TEST_FRAMEWORK__TRACKED_OBJECT_HPP_
