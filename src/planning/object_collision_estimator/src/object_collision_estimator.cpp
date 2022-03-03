// Copyright 2020-2021 Arm Limited
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

#include "object_collision_estimator/object_collision_estimator.hpp"

#include <algorithm>
#include <limits>
#include <list>
#include <vector>

#include "common/types.hpp"
#include "geometry/bounding_box/bounding_box_common.hpp"
#include "geometry/bounding_box/rotating_calipers.hpp"
#include "geometry/intersection.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "motion_common/config.hpp"
#include "motion_common/motion_common.hpp"

namespace motion
{
namespace planning
{
namespace object_collision_estimator
{

using autoware::common::geometry::bounding_box::minimum_perimeter_bounding_box;
using autoware::common::geometry::get_normal;
using autoware::common::geometry::minus_2d;
using autoware::common::geometry::plus_2d;
using autoware::common::geometry::rotate_2d;
using autoware::common::geometry::times_2d;
using autoware::common::types::float32_t;
using motion::planning::trajectory_smoother::TrajectorySmoother;
using geometry_msgs::msg::Point32;

/// \brief Convert a trajectory waypoint into a bounding box representing the volume occupied by the
///        ego vehicle while on this waypoint.
/// \param pt A waypoint of a trajectory
/// \param vehicle_param Ego vehicle parameter defining its dimensions
/// \param safety_factor A factor to inflate the size of the vehicle so to avoid getting too close
///                      to obstacles.
/// \return BoundingBox The box bounding the ego vehicle at the waypoint.
BoundingBox waypointToBox(
  const TrajectoryPoint & pt,
  const VehicleConfig & vehicle_param,
  const float32_t safety_factor)
{
  // Shorthands to keep the formulas sane
  float32_t lf = vehicle_param.length_cg_front_axel() + vehicle_param.front_overhang();
  float32_t lr = vehicle_param.length_cg_rear_axel() + vehicle_param.rear_overhang();
  float32_t wh = vehicle_param.width() * 0.5f;
  const float32_t heading =
    static_cast<float32_t>(::motion::motion_common::to_angle(pt.pose.orientation));
  const float32_t ch = std::cos(heading);
  const float32_t sh = std::sin(heading);
  const float32_t pt_x = static_cast<float32_t>(pt.pose.position.x);
  const float32_t pt_y = static_cast<float32_t>(pt.pose.position.y);

  // inflate size of vehicle by safety factor
  lf *= safety_factor;
  lr *= safety_factor;
  wh *= safety_factor;

  // Create a list of corners for the vehicle
  std::list<Point32> vehicle_corners;

  {     // Front left
    auto p = Point32{};
    p.x = pt_x + (lf * ch) - (wh * sh);
    p.y = pt_y + (lf * sh) + (wh * ch);
    vehicle_corners.push_back(p);
  }
  {     // Front right
    auto p = Point32{};
    p.x = pt_x + (lf * ch) + (wh * sh);
    p.y = pt_y + (lf * sh) - (wh * ch);
    vehicle_corners.push_back(p);
  }
  {     // Rear right
    auto p = Point32{};
    p.x = pt_x - (lr * ch) + (wh * sh);
    p.y = pt_y - (lr * sh) - (wh * ch);
    vehicle_corners.push_back(p);
  }
  {     // Rear left
    auto p = Point32{};
    p.x = pt_x - (lr * ch) - (wh * sh);
    p.y = pt_y - (lr * sh) + (wh * ch);
    vehicle_corners.push_back(p);
  }

  return minimum_perimeter_bounding_box(vehicle_corners);
}

/// \brief Converts a bounding box into axis aligned bounding box that covers given box.
///        The goal is to use axis aligned bounding box
///        for optimization as a first step in collision detection.
/// \param bbox A bounding box to convert
/// \return AxisAlignedBoundingBox The axis aligned bounding box
AxisAlignedBoundingBox calculateAxisAlignedBoundingBox(const BoundingBox & bbox)
{
  auto min_x = std::numeric_limits<float32_t>::max();
  auto min_y = std::numeric_limits<float32_t>::max();
  auto max_x = std::numeric_limits<float32_t>::lowest();
  auto max_y = std::numeric_limits<float32_t>::lowest();

  for (auto corner : bbox.corners) {
    min_x = std::min(min_x, corner.x);
    min_y = std::min(min_y, corner.y);
    max_x = std::max(max_x, corner.x);
    max_y = std::max(max_y, corner.y);
  }

  AxisAlignedBoundingBox result;
  result.lower_bound.x = min_x;
  result.lower_bound.y = min_y;
  result.upper_bound.x = max_x;
  result.upper_bound.y = max_y;

  return result;
}

/// \brief Determine if an obstacle is too far away from a vehicle to collide
/// \param vehicle_bbox a bounding box representing the vehicle
/// \param obstacle_bbox a bounding box representing the obstacle
/// \return bool8_t Return true if an obstacle is too far away from a vehicle to collide.
bool8_t isTooFarAway(
  const AxisAlignedBoundingBox & vehicle_bbox, const AxisAlignedBoundingBox & obstacle_bbox)
{
  return vehicle_bbox.lower_bound.x > obstacle_bbox.upper_bound.x ||
         vehicle_bbox.lower_bound.y > obstacle_bbox.upper_bound.y ||
         vehicle_bbox.upper_bound.x < obstacle_bbox.lower_bound.x ||
         vehicle_bbox.upper_bound.y < obstacle_bbox.lower_bound.y;
}

/// \brief Detect possible collision between a trajectory and a list of obstacle bounding boxes.
///        Return the index in the trajectory where the first collision happens.
/// \param trajectory Planned trajectory of ego vehicle.
/// \param predicted_objects Array of predicted object.
/// \param vehicle_param Configuration regarding the dimensions of the ego vehicle
/// \param safety_factor A factor to inflate the size of the vehicle so to avoid getting too close
///                      to obstacles.
/// \param waypoint_bboxes A list of bounding boxes around each waypoint in the trajectory
/// \return int32_t The index into the trajectory points where the first collision happens. If no
///         collision is detected, -1 is returned.
int32_t detectCollision(
  const Trajectory & trajectory,
  const std::vector<PredictedObjectInfo> & predicted_objects,
  const VehicleConfig & vehicle_param,
  const float32_t safety_factor,
  BoundingBoxArray & waypoint_bboxes)
{
  int32_t collision_index = -1;

  waypoint_bboxes.boxes.clear();
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    waypoint_bboxes.boxes.push_back(
      waypointToBox(trajectory.points[i], vehicle_param, safety_factor));
  }
  for (std::size_t i = 0; (i < trajectory.points.size()) && (collision_index == -1); ++i) {
    // calculate a bounding box given a trajectory point
    const auto & waypoint_bbox = waypoint_bboxes.boxes.at(i);
    auto axis_aligned_bbox = calculateAxisAlignedBoundingBox(waypoint_bbox);
    const std::vector<geometry_msgs::msg::Point32> waypoint_bbox_corners{
      waypoint_bbox.corners.begin(), waypoint_bbox.corners.end()};

    // Check for collisions with all perceived obstacles
    for (const auto & predicted_object : predicted_objects) {
      // TODO(@kcolak): For now, object prediction modules only works for stationary object. After
      //  adding the dynamic obstacle prediction, this part needs to be updated.
      const auto predicted_object_corners =
        autoware::common::geometry::bounding_box::details::get_transformed_corners(
        predicted_object.predicted_object.shape[0],
        predicted_object.predicted_object.kinematics.initial_pose.pose.position,
        predicted_object.predicted_object.kinematics.initial_pose.pose.orientation);

      if (!isTooFarAway(
          axis_aligned_bbox,
          predicted_object.axis_bbox) && autoware::common::geometry::intersect(
          waypoint_bbox_corners.begin(), waypoint_bbox_corners.end(),
          predicted_object_corners.begin(), predicted_object_corners.end()))
      {
        // Collision detected, set end index (non-inclusive), this will end outer loop immediately
        collision_index = static_cast<decltype(collision_index)>(i);
        break;
      }
    }
  }

  return collision_index;
}

/// \brief Returns the index that vehicle should stop when the object colliding index
///        and stop distance is given
/// \param trajectory Planned trajectory of ego vehicle.
/// \param collision_index Index of trajectory point that collides with and obstacle
/// \param stop_margin Distance between the control point of vehicle (CoG or base_link) and obstacle
/// \return int32_t The index into the trajectory points where vehicle should stop.
int32_t getStopIndex(
  const Trajectory & trajectory,
  const int32_t collision_index,
  const float32_t stop_margin) noexcept
{
  if (collision_index < 0) {
    return collision_index;
  }
  int32_t stop_index = 0;
  float32_t accumulated_distance = 0;

  for (int32_t i = collision_index; i >= 1; i--) {
    const auto & prev_pt = trajectory.points.at(static_cast<std::size_t>(i - 1));
    const auto & pt = trajectory.points.at(static_cast<std::size_t>(i));

    const auto dx = prev_pt.pose.position.x - pt.pose.position.x;
    const auto dy = prev_pt.pose.position.y - pt.pose.position.y;
    accumulated_distance += static_cast<float32_t>(std::hypot(dx, dy));
    if (accumulated_distance >= stop_margin) {
      stop_index = i;
      break;
    }
  }
  return stop_index;
}

ObjectCollisionEstimator::ObjectCollisionEstimator(
  ObjectCollisionEstimatorConfig config,
  TrajectorySmoother smoother) noexcept
: m_config(config), m_smoother(smoother)
{
  // safety factor could not be smaller than 1
  if (m_config.safety_factor < 1.0f) {
    m_config.safety_factor = 1.0f;
  }
}

int32_t ObjectCollisionEstimator::updatePlan(
  const Trajectory & trajectory,
  Trajectory & trajectory_response) noexcept
{
  // Collision detection
  auto collision_index = detectCollision(
    trajectory, m_predicted_objects, m_config.vehicle_config,
    m_config.safety_factor, m_trajectory_bboxes);
  auto trajectory_end_idx = getStopIndex(trajectory, collision_index, m_config.stop_margin);

  if (trajectory_end_idx >= 2) {
    // Cut trajectory short to just before the collision point
    trajectory_response.points.resize(static_cast<std::size_t>(trajectory_end_idx));
    // Mark the last point along the trajectory as "stopping" by setting
    // accelerations and velocities to zero.
    const auto trajectory_last_idx = static_cast<std::size_t>(trajectory_end_idx - 1);

    trajectory_response.points[trajectory_last_idx].longitudinal_velocity_mps = 0.0;
    trajectory_response.points[trajectory_last_idx].acceleration_mps2 = 0.0;

    // smooth the velocity profile of the trajectory so that it is realistically
    // executable.
    m_smoother.Filter(trajectory_response);
  } else if (trajectory_end_idx >= 0) {
    // Valid trajectory for trajectory follower should be contained at least 2 points.
    // Cut trajectory short to just before the collision point
    trajectory_response.points.resize(static_cast<std::size_t>(2));
    // Mark the last point along the trajectory as "stopping" by setting
    // accelerations and velocities to zero.
    for (auto & trajectory_point : trajectory_response.points) {
      trajectory_point.longitudinal_velocity_mps = 0.0;
      trajectory_point.acceleration_mps2 = 0.0;
    }
  }
  return static_cast<size_t>(collision_index);
}


void ObjectCollisionEstimator::updatePredictedObjects(
  PredictedObjects predicted_objects) noexcept
{
  auto get_dimensions = [&](
    PredictedObject & predicted_object,
    AxisAlignedBoundingBox & axis_aligned_bounding_box,
    const float32_t min_obstacle_dimension
    ) {
      auto min_x = std::numeric_limits<float32_t>::max();
      auto min_y = std::numeric_limits<float32_t>::max();
      auto max_x = std::numeric_limits<float32_t>::lowest();
      auto max_y = std::numeric_limits<float32_t>::lowest();

      for (auto & corner_point : predicted_object.shape[0].polygon.points) {
        // Update obstacles corner under minimum obstacle dimension threshold
        // Current shape corners object centric
        if (abs(corner_point.x) < min_obstacle_dimension / 2) {
          const auto scale_factor = (min_obstacle_dimension / 2) / corner_point.x;
          corner_point.x = corner_point.x * scale_factor;
          corner_point.y = corner_point.y * scale_factor;
        } else if (abs(corner_point.x) < min_obstacle_dimension / 2) {
          const auto scale_factor = (min_obstacle_dimension / 2) / corner_point.y;
          corner_point.x = corner_point.x * scale_factor;
          corner_point.y = corner_point.y * scale_factor;
        }

        min_x = std::min(
          min_x, corner_point.x +
          static_cast<float32_t>(predicted_object.kinematics.initial_pose.pose.position.x));
        min_y = std::min(
          min_y, corner_point.y +
          static_cast<float32_t>(predicted_object.kinematics.initial_pose.pose.position.y));
        max_x = std::max(
          max_x, corner_point.x +
          static_cast<float32_t>(predicted_object.kinematics.initial_pose.pose.position.x));
        max_y = std::max(
          max_y, corner_point.y +
          static_cast<float32_t>(predicted_object.kinematics.initial_pose.pose.position.y));
      }

      axis_aligned_bounding_box.lower_bound.x = min_x;
      axis_aligned_bounding_box.lower_bound.y = min_y;
      axis_aligned_bounding_box.upper_bound.x = max_x;
      axis_aligned_bounding_box.upper_bound.y = max_y;
    };

  m_predicted_objects.clear();
  for (auto & predicted_object : predicted_objects.objects) {
    AxisAlignedBoundingBox axis_aligned_bounding_box;
    get_dimensions(
      predicted_object, axis_aligned_bounding_box,
      m_config.min_obstacle_dimension_m);
    PredictedObjectInfo predicted_object_info;
    predicted_object_info.predicted_object = predicted_object;
    predicted_object_info.axis_bbox = axis_aligned_bounding_box;
    m_predicted_objects.push_back(predicted_object_info);
  }
}

}  // namespace object_collision_estimator
}  // namespace planning
}  // namespace motion
