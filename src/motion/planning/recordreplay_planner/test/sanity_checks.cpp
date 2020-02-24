// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
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

#include <gtest/gtest.h>
#include <recordreplay_planner/recordreplay_planner.hpp>
#include <recordreplay_planner/geometry.hpp>
#include <motion_testing/motion_testing.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/bounding_box.hpp>
#include <geometry/common_2d.hpp>
#include <motion_common/config.hpp>
#include <motion_common/motion_common.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <chrono>
#include <algorithm>

using motion::planning::recordreplay_planner::RecordReplayPlanner;
using std::chrono::system_clock;
using motion::motion_testing::make_state;
using autoware_auto_msgs::msg::Trajectory;
using autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::BoundingBoxArray;
using autoware_auto_msgs::msg::BoundingBox;
using motion::motion_common::VehicleConfig;
using geometry_msgs::msg::Point32;
using motion::motion_common::from_angle;
using motion::planning::recordreplay_planner::compute_boundingbox_from_trajectorypoint;
using motion::planning::recordreplay_planner::boxes_collide;

const VehicleConfig test_vehicle_params{1.0, 1.0, 0.5, 0.5, 1500, 12, 2.0, 0.5, 0.2};

class sanity_checks_base : public ::testing::Test
{
protected:
  RecordReplayPlanner planner_{test_vehicle_params};
};


//------------------ Test basic properties of a recorded, then replayed trajectory
struct PropertyTestParameters
{
  std::chrono::milliseconds time_spacing_ms;
  system_clock::time_point starting_time;
};

class sanity_checks_trajectory_properties
  : public sanity_checks_base, public testing::WithParamInterface<PropertyTestParameters>
{};

TEST_P(sanity_checks_trajectory_properties, basicproperties)
{
  const auto t = system_clock::now();
  const auto p = GetParam();
  auto t0 = p.starting_time;

  // Build a trajectory
  constexpr auto N = 10;
  const auto time_increment = p.time_spacing_ms;
  for (uint32_t k = {}; k < N; ++k) {
    const auto next_state = make_state(1.0F * k, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        t0 + k * time_increment);
    planner_.record_state(next_state);
  }

  // Test: Check that the length is equal to the number of states we fed in
  EXPECT_EQ(planner_.get_record_length(), N);

  // Test: Check that the plan returned has the expected time length
  auto trajectory = planner_.plan(make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
  double trajectory_time_length = trajectory.points[N - 1].time_from_start.sec + 1e-9F *
    trajectory.points[N - 1].time_from_start.nanosec;
  EXPECT_EQ(std::chrono::duration<float>(trajectory_time_length), 1.0F * (N - 1) * time_increment);
}

INSTANTIATE_TEST_CASE_P(
  trajectory_properties,
  sanity_checks_trajectory_properties,
  testing::Values(
    PropertyTestParameters{std::chrono::milliseconds(100), system_clock::from_time_t({})},
    PropertyTestParameters{std::chrono::milliseconds(200), system_clock::from_time_t({})},
    PropertyTestParameters{std::chrono::milliseconds(100), system_clock::from_time_t(10)},
    PropertyTestParameters{std::chrono::milliseconds(200), system_clock::from_time_t(10)}
));


//------------------ Test that length cropping properly works
struct LengthTestParameters
{
  // The number of points to be recorded
  uint32_t number_of_points;
};


class sanity_checks_trajectory_length
  : public sanity_checks_base, public testing::WithParamInterface<LengthTestParameters>
{};

TEST_P(sanity_checks_trajectory_length, length)
{
  const auto p = GetParam();
  const auto N = p.number_of_points;
  const auto dummy_state = make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      system_clock::from_time_t({}));

  for (uint32_t k = {}; k < N; ++k) {
    planner_.record_state(dummy_state);
  }

  // Test: Check that the length is equal to the number of states we fed in
  EXPECT_EQ(planner_.get_record_length(), N);
  auto trajectory = planner_.plan(dummy_state);

  EXPECT_EQ(trajectory.points.size(),
    std::min(N, static_cast<uint32_t>(trajectory.points.max_size())));
}

INSTANTIATE_TEST_CASE_P(
  trajectory_length,
  sanity_checks_trajectory_length,
  testing::Values(
    LengthTestParameters{80},
    LengthTestParameters{200}
));


// Test setup helper function. This creates a planner and records a trajectory
// that goes along the points (0,0), (1,0), .... (N-1,0) with the heading set to
// 0 throughout - for testing purposes
RecordReplayPlanner helper_create_and_record_example(uint32_t N)
{
  auto planner = RecordReplayPlanner(test_vehicle_params);
  auto t0 = system_clock::from_time_t({});

  // Record some states going from
  for (uint32_t k = {}; k < N; ++k) {
    planner.record_state(make_state(1.0F * k, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      t0 + k * std::chrono::milliseconds{100LL}));
  }

  return planner;
}


//------------------ Test that "receding horizon" planning properly works: happy case
TEST(recordreplay_sanity_checks, receding_horizon_happycase)
{
  const auto N = 3;
  auto planner = helper_create_and_record_example(N);

  // Call "plan" multiple times in sequence, expecting the states to come back out in order
  const auto t0 = system_clock::from_time_t({});
  for (uint32_t k = {}; k < N; ++k) {
    auto trajectory = planner.plan(make_state(1.0F * k, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    // normally don't check float equality but we _just_ pushed this float so it ought not
    // to have changed
    EXPECT_EQ(1.0F * k, trajectory.points[0].x);
    EXPECT_EQ(N - k, trajectory.points.size());
  }
}

//------------------ Test that "receding horizon" planning properly works:
TEST(recordreplay_sanity_checks, receding_horizon_cornercases)
{
  const auto N = 3;
  auto planner = helper_create_and_record_example(N);

  const auto t0 = system_clock::from_time_t({});

  // Check: State we have not recorded, but is closest to the (0,0) state
  {
    auto trajectory = planner.plan(make_state(-1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    EXPECT_EQ(0.0F, trajectory.points[0].x);
  }

  // Check: State we have not recorded, but is closest to the (0,0) state
  {
    auto trajectory = planner.plan(make_state(0.1F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    EXPECT_EQ(0.0F, trajectory.points[0].x);
    EXPECT_EQ(0.0F, trajectory.points[0].y);
  }

  // Check: State we have not recorded, but is closest to the (N,0) state
  {
    auto trajectory = planner.plan(make_state(1.0F * N + 5.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    EXPECT_EQ((N - 1) * 1.0F, trajectory.points[0].x);
    EXPECT_EQ(0.0F, trajectory.points[0].y);
  }
}

TEST(recordreplay_sanity_checks, state_setting_mechanism)
{
  auto planner = RecordReplayPlanner{test_vehicle_params};

  // Make sure setting and reading the recording state works
  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  planner.start_recording();

  EXPECT_TRUE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  planner.stop_recording();

  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  // Make sure setting and reading the replaying state works
  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  planner.start_replaying();

  EXPECT_FALSE(planner.is_recording() );
  EXPECT_TRUE(planner.is_replaying() );

  planner.stop_replaying();

  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );
}

TEST(recordreplay_sanity_checks, heading_weight_setting)
{
  auto planner = RecordReplayPlanner{test_vehicle_params};

  planner.set_heading_weight(0.5);
  EXPECT_EQ(planner.get_heading_weight(), 0.5);
  EXPECT_THROW(planner.set_heading_weight(-1.0), std::domain_error);
}

TEST(recordreplay_sanity_checks, adding_bounding_boxes)
{
  auto planner = RecordReplayPlanner{test_vehicle_params};
  EXPECT_EQ(planner.get_number_of_bounding_boxes(), 0);

  auto dummy_boxes = BoundingBoxArray{};
  dummy_boxes.boxes.push_back(BoundingBox{});
  dummy_boxes.boxes.push_back(BoundingBox{});

  planner.update_bounding_boxes(dummy_boxes);

  EXPECT_EQ(planner.get_number_of_bounding_boxes(), 2);
}


const BoundingBox get_test_box(const TrajectoryPoint state)
{
  const VehicleConfig test_params{1.0, 1.0, 0.5, 0.5, 1500, 12, 2.0, 0.5, 0.2};
  return compute_boundingbox_from_trajectorypoint(state, test_params);
}


//------------------ Test that bounding box creation works
TEST(recordreplay_geometry_checks, bounding_box_creation)
{
  // First case: box aligned with axes
  const auto t0 = system_clock::from_time_t({});
  const auto state = make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
  auto aligned_box = get_test_box(state.state);

  EXPECT_EQ(aligned_box.corners.size(), 4);

  // TODO(s.me) This assumes a certain order of the vertices, which I think is
  // not guaranteed. One could switch to set membership based testing instead.
  EXPECT_LT(std::abs(aligned_box.corners[0].x - 1.5), 1e-6);
  EXPECT_LT(std::abs(aligned_box.corners[0].y - 1), 1e-6);

  EXPECT_LT(std::abs(aligned_box.corners[1].x + 1.2), 1e-6);
  EXPECT_LT(std::abs(aligned_box.corners[1].y - 1), 1e-6);

  EXPECT_LT(std::abs(aligned_box.corners[2].x + 1.2), 1e-6);
  EXPECT_LT(std::abs(aligned_box.corners[2].y + 1), 1e-6);

  EXPECT_LT(std::abs(aligned_box.corners[3].x - 1.5), 1e-6);
  EXPECT_LT(std::abs(aligned_box.corners[3].y + 1), 1e-6);


  // Test case 2: box rotated by 90 degrees
  const auto rotated_state = make_state(0.0F, 0.0F, 1.5707963267948966, 0.0F, 0.0F, 0.0F, t0);
  auto rotated_box = get_test_box(rotated_state.state);

  EXPECT_EQ(rotated_box.corners.size(), 4);

  EXPECT_LT(std::abs(rotated_box.corners[0].x - 1), 1e-6);
  EXPECT_LT(std::abs(rotated_box.corners[0].y - 1.5), 1e-6);

  EXPECT_LT(std::abs(rotated_box.corners[1].x + 1), 1e-6);
  EXPECT_LT(std::abs(rotated_box.corners[1].y - 1.5), 1e-6);

  EXPECT_LT(std::abs(rotated_box.corners[2].x + 1), 1e-6);
  EXPECT_LT(std::abs(rotated_box.corners[2].y + 1.2), 1e-6);

  EXPECT_LT(std::abs(rotated_box.corners[3].x - 1), 1e-6);
  EXPECT_LT(std::abs(rotated_box.corners[3].y + 1.2), 1e-6);
}

TEST(recordreplay_geometry_checks, collision_check) {
  // First case: box aligned with axes
  const auto t0 = system_clock::from_time_t({});
  const auto state = make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
  auto aligned_box = get_test_box(state.state);

  { // Intersection
    const auto state_shifted = make_state(1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
    auto aligned_box_shifted = get_test_box(state_shifted.state);
    auto collision = boxes_collide(aligned_box, aligned_box_shifted);
    EXPECT_TRUE(collision);
  }

  { // No intersection
    const auto state_shifted = make_state(50.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
    auto aligned_box_shifted = get_test_box(state_shifted.state);
    auto collision = boxes_collide(aligned_box, aligned_box_shifted);
    EXPECT_FALSE(collision);
  }

  { // Small intersection in a corner
    const auto state_shifted = make_state(2.6F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
    auto aligned_box_shifted = get_test_box(state_shifted.state);
    auto collision = boxes_collide(aligned_box, aligned_box_shifted);
    EXPECT_TRUE(collision);
  }
}
