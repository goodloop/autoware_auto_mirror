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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <memory>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "time_utils/time_utils.hpp"
#include "tracking/tracked_object.hpp"
#include "tracking/track_creator.hpp"

using AssociatorResult = autoware::perception::tracking::AssociatorResult;
using ClassifiedRoi = autoware_auto_msgs::msg::ClassifiedRoi;
using ClassifiedRoiArray = autoware_auto_msgs::msg::ClassifiedRoiArray;
using CreationPolicies = autoware::perception::tracking::TrackCreationPolicy;
using DetectedObject = autoware_auto_msgs::msg::DetectedObject;
using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using GreedyRoiAssociator = autoware::perception::tracking::GreedyRoiAssociator;
using TrackCreator = autoware::perception::tracking::TrackCreator;
using TrackedObject = autoware::perception::tracking::TrackedObject;
using VisionPolicyConfig = autoware::perception::tracking::VisionPolicyConfig;

class MockRoiAssociator : public GreedyRoiAssociator
{
public:
  MockRoiAssociator()
  : GreedyRoiAssociator(intrinsics, matching_threshold) {}
  MOCK_METHOD(
    AssociatorResult, assign, (const ClassifiedRoiArray &, const std::vector<TrackedObject>&),
    (const));
  MOCK_METHOD(
    AssociatorResult, assign, (const ClassifiedRoiArray &, const DetectedObjects &), (const));

private:
  autoware::perception::tracking::CameraIntrinsics intrinsics;
  float32_t matching_threshold = 0.5F;
  geometry_msgs::msg::Transform empty_tf;
};

using ::testing::_;
using ::testing::Matcher;
using ::testing::Return;

TEST(TrackCreatorTest, test_lidar_only)
{
  std::shared_ptr<GreedyRoiAssociator> associator = std::make_shared<MockRoiAssociator>();
  TrackCreator creator{{CreationPolicies::LidarClusterOnly, 1.0F, 1.0F}};
  DetectedObject obj;
  DetectedObjects objs;
  const int num_objects = 10;
  for (int i = 0; i < num_objects; ++i) {
    objs.objects.push_back(obj);
  }

  AssociatorResult result;
  result.unassigned_detection_indices = {0, 2, 4};

  creator.add_objects(objs, result);

  EXPECT_EQ(creator.create_tracks().tracks.size(), 3U);
  EXPECT_EQ(creator.create_tracks().detections_leftover.objects.size(), 0U);
}

// Test lidar and vision with one match between them
TEST(TrackCreatorTest, test_lidar_if_vision_1_new_track)
{
  auto associator = std::make_shared<MockRoiAssociator>();
  VisionPolicyConfig policy_cfg;
  TrackCreator creator{{CreationPolicies::LidarClusterIfVision, 1.0F, 1.0F}};
  auto now_time = time_utils::to_message(
    std::chrono::system_clock::time_point{std::chrono::system_clock::now()});

  // Add lidar
  DetectedObject lidar_detection;
  DetectedObjects lidar_detections;
  lidar_detections.header.stamp = now_time;
  const int num_objects = 10;
  for (int i = 0; i < num_objects; ++i) {
    lidar_detection.shape.height = i;  // use index as height to differentiate between detections
    lidar_detections.objects.push_back(lidar_detection);
  }
  AssociatorResult lidar_track_assn;
  lidar_track_assn.unassigned_detection_indices = {0, 2, 4};
  creator.add_objects(lidar_detections, lidar_track_assn);
  // Since unordered_set is used to refer unassigned detections that actual inserted order is
  // undefined. Get a copy of that to test erase logic
  const auto inserted_clusters = creator.get_unassigned_lidar_detections();

  // Add vision
  ClassifiedRoi vision_detection;
  ClassifiedRoiArray vision_detections;
  vision_detections.header.stamp = time_utils::to_message(
    time_utils::from_message(now_time) +
    std::chrono::milliseconds(15));
  const int num_vision_detections = 5;
  for (int i = 0; i < num_vision_detections; ++i) {
    vision_detections.rois.push_back(vision_detection);
  }
  AssociatorResult vision_track_assn;
  vision_track_assn.unassigned_detection_indices = {1, 3, 5};
  creator.add_unassigned_vision_detections(vision_detections, vision_track_assn);

  // Setup mock output from ROI associator
  AssociatorResult vision_lidar_assn;
  vision_lidar_assn.track_assignments =
  {1, AssociatorResult::UNASSIGNED, AssociatorResult::UNASSIGNED};
  std::vector<size_t> unassigned_cluster_idx = {1, 2};
  vision_lidar_assn.unassigned_detection_indices = {0, 2};
  vision_lidar_assn.unassigned_track_indices.insert(
    unassigned_cluster_idx.begin(),
    unassigned_cluster_idx.end());

  EXPECT_CALL(*associator, assign(_, Matcher<const DetectedObjects &>(_))).WillOnce(
    Return(vision_lidar_assn));

  // Test
  const auto ret = creator.create_tracks();
  EXPECT_EQ(ret.size(), 1U);
  const auto detections = creator.get_unassigned_lidar_detections();
  EXPECT_EQ(detections.objects.size(), vision_lidar_assn.unassigned_track_indices.size());
  for (size_t i = 0U; i < detections.objects.size(); ++i) {
    EXPECT_FLOAT_EQ(
      detections.objects[i].shape.height, inserted_clusters
      .objects[unassigned_cluster_idx[i]].shape.height);
  }
}

// Test lidar and vision but no match between them
TEST(TrackCreatorTest, test_lidar_if_vision_no_new_track)
{
  auto associator = std::make_shared<MockRoiAssociator>();
  TrackCreator creator{{CreationPolicies::LidarClusterIfVision, 1.0F, 1.0F}, associator};
  auto now_time = time_utils::to_message(
    std::chrono::system_clock::time_point{std::chrono::system_clock::now()});

  // Add lidar
  DetectedObject lidar_detection;
  DetectedObjects lidar_detections;
  lidar_detections.header.stamp = now_time;
  const int num_objects = 10;
  for (int i = 0; i < num_objects; ++i) {
    lidar_detection.shape.height = i;  // use index as height to differentiate between detections
    lidar_detections.objects.push_back(lidar_detection);
  }
  AssociatorResult lidar_track_assn;
  lidar_track_assn.unassigned_detection_indices = {0, 2, 4};
  creator.add_unassigned_lidar_clusters(lidar_detections, lidar_track_assn);
  // Since unordered_set is used to refer unassigned detections that actual inserted order is
  // undefined. Get a copy of that to test erase logic
  const auto inserted_clusters = creator.get_unassigned_lidar_detections();

  // Add vision
  ClassifiedRoi vision_detection;
  ClassifiedRoiArray vision_detections;
  vision_detections.header.stamp = time_utils::to_message(
    time_utils::from_message(now_time) - std::chrono::milliseconds(15));
  const int num_vision_detections = 5;
  for (int i = 0; i < num_vision_detections; ++i) {
    vision_detections.rois.push_back(vision_detection);
  }
  AssociatorResult vision_track_assn;
  vision_track_assn.unassigned_detection_indices = {1, 3, 5};
  creator.add_unassigned_vision_detections(vision_detections, vision_track_assn);

  // Setup mock output from ROI associator
  AssociatorResult vision_lidar_assn;
  vision_lidar_assn.track_assignments =
  {AssociatorResult::UNASSIGNED, AssociatorResult::UNASSIGNED, AssociatorResult::UNASSIGNED};
  vision_lidar_assn.unassigned_detection_indices = {0, 1, 2};
  vision_lidar_assn.unassigned_track_indices = {0, 1, 2};

  EXPECT_CALL(*associator, assign(_, Matcher<const DetectedObjects &>(_))).WillOnce(
    Return(vision_lidar_assn));

  // Test
  const auto ret = creator.create_tracks();
  EXPECT_EQ(ret.size(), 0U);
  const auto detections = creator.get_unassigned_lidar_detections();
  EXPECT_EQ(detections.objects.size(), 3U);
  EXPECT_EQ(inserted_clusters, detections);
}

// No vision message within time range
TEST(TrackCreatorTest, test_lidar_if_vision_out_of_time_range)
{
  auto associator = std::make_shared<MockRoiAssociator>();
  TrackCreator creator{{CreationPolicies::LidarClusterIfVision, 1.0F, 1.0F}, associator};
  auto now_time = time_utils::to_message(
    std::chrono::system_clock::time_point{std::chrono::system_clock::now()});

  // Add lidar
  DetectedObject lidar_detection;
  DetectedObjects lidar_detections;
  lidar_detections.header.stamp = now_time;
  const int num_objects = 10;
  for (int i = 0; i < num_objects; ++i) {
    lidar_detection.shape.height = i;  // use index as height to differentiate between detections
    lidar_detections.objects.push_back(lidar_detection);
  }
  AssociatorResult lidar_track_assn;
  lidar_track_assn.unassigned_detection_indices = {0, 2, 4};
  creator.add_unassigned_lidar_clusters(lidar_detections, lidar_track_assn);
  // Since unordered_set is used to refer unassigned detections that actual inserted order is
  // undefined. Get a copy of that to test erase logic
  const auto inserted_clusters = creator.get_unassigned_lidar_detections();

  // Add vision
  ClassifiedRoi vision_detection;
  ClassifiedRoiArray vision_detections;
  vision_detections.header.stamp = time_utils::to_message(
    time_utils::from_message(now_time) +
    std::chrono::milliseconds(55));
  const int num_vision_detections = 5;
  for (int i = 0; i < num_vision_detections; ++i) {
    vision_detections.rois.push_back(vision_detection);
  }
  AssociatorResult vision_track_assn;
  vision_track_assn.unassigned_detection_indices = {1, 3, 5};
  creator.add_unassigned_vision_detections(vision_detections, vision_track_assn);

  EXPECT_CALL(*associator, assign(_, Matcher<const DetectedObjects &>(_))).Times(0);

  // Test
  const auto ret = creator.create_tracks();
  EXPECT_EQ(ret.size(), 0U);
}
