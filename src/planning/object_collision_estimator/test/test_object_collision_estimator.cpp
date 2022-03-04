// Copyright 2020-2021 Arm Limited
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

#include <trajectory_smoother/trajectory_smoother.hpp>
#include <motion_testing/motion_testing.hpp>
#include <common/types.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <algorithm>

#include "object_collision_estimator/object_collision_estimator.hpp"

using motion::planning::object_collision_estimator::ObjectCollisionEstimator;
using motion::planning::object_collision_estimator::ObjectCollisionEstimatorConfig;
using motion::planning::trajectory_smoother::TrajectorySmoother;
using motion::motion_testing::constant_velocity_trajectory;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using motion::motion_common::VehicleConfig;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_perception_msgs::msg::BoundingBox;
using autoware_auto_perception_msgs::msg::BoundingBoxArray;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedObject;

const auto make_point(const float32_t x, const float32_t y)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  return p;
}

void object_collision_estimator_test(
  std::size_t trajectory_length,
  std::size_t obstacle_bbox_idx,
  float32_t generated_obstacle_length = 0.5,
  float32_t generated_obstacle_width = 0.5)
{
  // define dummy vehicle dimensions
  ObjectCollisionEstimatorConfig config{
    {
      1,  // length_cg_front_axel_m,
      1,  // length_cg_rear_axel_m,
      0,  // front_cornering_stiffness_N,
      0,  // rear_cornering_stiffness_N,
      1000,  // mass_kg,
      0,  // inertia_kgm2,
      2,  // width_m,
      0.5,  // front_overhang_m,
      0.5,  // rear_overhang_m);
    },
    1.1,  // safety factor
    0.0,  // stop_margin
    0.0004,  // min_obstacle_dimension_m
  };
  TrajectorySmoother smoother{{5, 25}};
  // initialise the estimator
  ObjectCollisionEstimator estimator{config, smoother};

  // produce fake trajectory and obstacles
  const std::chrono::milliseconds dt(100);
  auto trajectory = constant_velocity_trajectory(
    0, 0, 0, 10,
    std::chrono::duration_cast<std::chrono::nanoseconds>(dt));
  trajectory.points.resize(trajectory_length);

  // insert an obstacle that blocks the trajectory
  PredictedObjects predicted_objects{};
  if (obstacle_bbox_idx < trajectory_length) {
    PredictedObject predicted_object{};

    const auto obstacle_point_x =
      static_cast<float32_t>(trajectory.points[obstacle_bbox_idx].pose.position.x);
    const auto obstacle_point_y =
      static_cast<float32_t>(trajectory.points[obstacle_bbox_idx].pose.position.y);

    predicted_object.kinematics.initial_pose.pose.position.x =
      static_cast<float64_t>(obstacle_point_x);
    predicted_object.kinematics.initial_pose.pose.position.y =
      static_cast<float64_t>(obstacle_point_y);
//    obstacle_bbox.size = make_point(generated_obstacle_size, generated_obstacle_size);
    predicted_object.kinematics.initial_pose.pose.orientation.w = 1.0F / sqrtf(2.0F);
    predicted_object.kinematics.initial_pose.pose.orientation.z = 1.0F / sqrtf(2.0F);

    autoware_auto_perception_msgs::msg::Shape shape;
    shape.polygon.points = {
      make_point(
        -generated_obstacle_length / 2,
        -generated_obstacle_width / 2),
      make_point(
        generated_obstacle_length / 2,
        -generated_obstacle_width / 2),
      make_point(
        generated_obstacle_length / 2,
        generated_obstacle_width / 2),
      make_point(
        -generated_obstacle_length / 2,
        generated_obstacle_width / 2)
    };
    predicted_object.shape.push_back(shape);
    predicted_objects.objects.push_back(predicted_object);
  }

  // call the estimator API
  estimator.updatePredictedObjects(predicted_objects);
  estimator.updatePlan(trajectory, trajectory);

  if (obstacle_bbox_idx < trajectory_length) {
    if (obstacle_bbox_idx != 0) {
      // Check that the trajectory has been curtailed
      EXPECT_EQ(trajectory.points.size(), obstacle_bbox_idx - 1);
    } else {
      // Check that the trajectory has been curtailed
      EXPECT_EQ(trajectory.points.size(), 0U);
    }

    // Check that the last point has zero velocity and acceleration
    if (trajectory.points.size() != 0) {
      EXPECT_EQ(trajectory.points[trajectory.points.size() - 1].longitudinal_velocity_mps, 0);
      EXPECT_EQ(trajectory.points[trajectory.points.size() - 1].acceleration_mps2, 0);
    }
  } else {
    // no obstacle
    EXPECT_EQ(trajectory.points.size(), trajectory_length);
    if (trajectory_length != 0) {
      EXPECT_NE(trajectory.points[trajectory.points.size() - 1].longitudinal_velocity_mps, 0);
    }
  }
}

TEST(ObjectCollisionEstimator, Sanity) {
  object_collision_estimator_test(100, 40);
}

TEST(ObjectCollisionEstimator, ShortTrajectory) {
  object_collision_estimator_test(3, 1);
  object_collision_estimator_test(3, 2);

  // no obstacles
  object_collision_estimator_test(2, 2);
  object_collision_estimator_test(1, 2);
  object_collision_estimator_test(0, 2);
}

TEST(ObjectCollisionEstimator, EmergencyStop) {
  object_collision_estimator_test(100, 0);
  object_collision_estimator_test(100, 1);
}

TEST(ObjectCollisionEstimator, NoObstacle) {
  object_collision_estimator_test(100, 101);
}

TEST(ObjectCollisionEstimator, SmallObstacle) {
  object_collision_estimator_test(100, 40, 0.0003);
}

TEST(ObjectCollisionEstimator, WideObstacle) {
  object_collision_estimator_test(100, 40, 0.5, 100.0);
}
