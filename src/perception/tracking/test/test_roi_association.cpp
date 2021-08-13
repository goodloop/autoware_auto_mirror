// Copyright 2021 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <gtest/gtest.h>
#include <tracking/roi_association.hpp>
#include <tracking/projection.hpp>
#include <algorithm>
#include <vector>

using TrackedObject = autoware::perception::tracking::TrackedObject;
using TrackedObjects = std::vector<TrackedObject>;

using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using DetectedObject = autoware_auto_msgs::msg::DetectedObject;
using Shape = autoware_auto_msgs::msg::Shape;
using Projection = autoware::perception::tracking::Projection;
using CameraModel = autoware::perception::tracking::CameraModel;
using CameraIntrinsics = autoware::perception::tracking::CameraIntrinsics;
using ClassifiedRoi = autoware_auto_msgs::msg::ClassifiedRoi;
using ClassifiedRoiArray = autoware_auto_msgs::msg::ClassifiedRoiArray;
namespace tracking = autoware::perception::tracking;
using RoiAssociator = tracking::GreedyRoiAssociator;
using AssociatorResult = tracking::AssociatorResult;
using Point32 = geometry_msgs::msg::Point32;

Point32 make_pt(float32_t x, float32_t y, float32_t z)
{
  return Point32{}.set__x(x).set__y(y).set__z(z);
}

TrackedObject make_rectangular_track(
  const Point32 & base_face_origin,
  float32_t half_width, float32_t half_length, float32_t shape_height)
{
  DetectedObject object;
  object.shape.polygon.points.push_back(
    make_pt(
      (base_face_origin.x + half_width),
      (base_face_origin.y + half_length), (base_face_origin.z)));
  object.shape.polygon.points.push_back(
    make_pt(
      (base_face_origin.x + half_width),
      (base_face_origin.y - half_length), (base_face_origin.z)));
  object.shape.polygon.points.push_back(
    make_pt(
      (base_face_origin.x - half_width),
      (base_face_origin.y + half_length), (base_face_origin.z)));
  object.shape.polygon.points.push_back(
    make_pt(
      (base_face_origin.x - half_width),
      (base_face_origin.y - half_length), (base_face_origin.z)));
  object.shape.height = shape_height;

  return TrackedObject{object, 0.0, 0.0};
}
ClassifiedRoi projection_to_roi(const Projection & projection)
{
  ClassifiedRoi roi;
  for (const auto & pt : projection.shape) {
    roi.polygon.points.push_back(Point32{}.set__x(pt.x).set__y(pt.y));
  }
  return roi;
}

class TestRoiAssociation : public ::testing::Test
{
public:
  TestRoiAssociation()
  : intrinsics{CameraIntrinsics{500U, 500U, 5.0F, 5.0F}},
    camera{intrinsics, make_identity()},
    associator{intrinsics, make_identity()} {}

  CameraIntrinsics intrinsics;
  CameraModel camera;
  RoiAssociator associator;

private:
  geometry_msgs::msg::Transform make_identity()
  {
    geometry_msgs::msg::Transform identity{};
    identity.rotation.set__w(1.0);
    return identity;
  }
};

TEST_F(TestRoiAssociation, correct_association) {
  TrackedObjects tracks;
  ClassifiedRoiArray rois;

  tracks.push_back(
    make_rectangular_track(
      make_pt(10.0F, 10.0F, 10), 5.0F, 5.0F, 2.0F));
  const auto projection = camera.project(tracks.front().shape());
  ASSERT_TRUE(projection);
  rois.rois.push_back(projection_to_roi(projection.value()));
  const auto result = associator.assign(rois, tracks);
  ASSERT_EQ(result.track_assignments.front(), 0U);
  ASSERT_EQ(result.unassigned_detection_indices.size(), 0U);
  ASSERT_EQ(result.unassigned_track_indices.size(), 0U);
}

TEST_F(TestRoiAssociation, out_of_image_test) {
  TrackedObjects tracks;
  ClassifiedRoiArray rois;

  // Create a track behind the camera
  tracks.push_back(
    make_rectangular_track(
      make_pt(10.0F, 10.0F, -10), 5.0F, 5.0F, 2.0F));
  const auto projection = camera.project(tracks[0].shape());
  ASSERT_FALSE(projection);
  const auto result = associator.assign(rois, tracks);
  ASSERT_EQ(result.track_assignments.front(), AssociatorResult::UNASSIGNED);
  ASSERT_EQ(result.unassigned_detection_indices.size(), 0U);
  ASSERT_EQ(result.unassigned_track_indices[0U], 0U);
}

TEST_F(TestRoiAssociation, non_associated_track_roi) {
  TrackedObjects tracks;
  TrackedObjects phantom_tracks;  // Only used to create false-positive ROIs
  ClassifiedRoiArray rois;

  tracks.push_back(
    make_rectangular_track(
      make_pt(10.0F, 10.0F, 10), 5.0F, 5.0F, 2.0F));

  // Use the phantom track to create a detection that is not associated with the correct track.
  phantom_tracks.push_back(
    make_rectangular_track(
      make_pt(-50.0F, -20.0F, 100), 2.0F, 5.0F, 25.0F));

  const auto projection = camera.project(phantom_tracks[0].shape());
  ASSERT_TRUE(projection);
  rois.rois.push_back(projection_to_roi(projection.value()));
  const auto result = associator.assign(rois, tracks);
  ASSERT_EQ(result.track_assignments.front(), AssociatorResult::UNASSIGNED);
  ASSERT_EQ(result.unassigned_detection_indices[0U], 0U);
  ASSERT_EQ(result.unassigned_track_indices[0U], 0U);
}

/// \brief This test creates a series of tracks and corresponding detection ROIs.
/// The tracks have the following layout: [FN, FN, FN, TP, TP, TP, TP] where the false negatives
// don't have corresponding ROI detections.
/// Likewise the ROIs have the following layout:  [TP, TP, TP, TP, FP, FP, FP] where false
// positives don't have corresponding tracks.
TEST_F(TestRoiAssociation, combined_association_test) {
  constexpr auto num_captured_tracks = 4U;
  constexpr auto num_noncaptured_tracks = 3U;
  constexpr auto num_nonassociated_rois = 3U;
  TrackedObjects tracks;
  TrackedObjects phantom_tracks;  // Only used to create false-positive ROIs
  ClassifiedRoiArray rois;

  // Objects not to be captured by the camera
  tracks.push_back(
    make_rectangular_track(
      make_pt(10.0F, 10.0F, -10), 5.0F, 5.0F, 2.0F));
  tracks.push_back(
    make_rectangular_track(
      make_pt(1e6F, 10.0F, 50), 15.0F, 25.0F, 10.0F));
  tracks.push_back(
    make_rectangular_track(
      make_pt(50.0F, 1e6F, 100), 2.0F, 5.0F, 25.0F));
  ASSERT_EQ(tracks.size(), num_noncaptured_tracks);

  // Objects to be captured by the camera (All have positive X,Y coordinates)
  tracks.push_back(
    make_rectangular_track(
      make_pt(10.0F, 10.0F, 10), 5.0F, 5.0F, 2.0F));
  tracks.push_back(
    make_rectangular_track(
      make_pt(20.0F, 10.0F, 50), 15.0F, 25.0F, 10.0F));
  tracks.push_back(
    make_rectangular_track(
      make_pt(50.0F, 20.0F, 100), 2.0F, 5.0F, 25.0F));
  tracks.push_back(
    make_rectangular_track(
      make_pt(105.0F, 5.0F, 100), 1.0F, 1.0F, 5.0F));
  ASSERT_EQ(tracks.size(), num_captured_tracks + num_noncaptured_tracks);

  // Push the correct projections to the roi array
  for (auto i = 0U; i < tracks.size(); ++i) {
    const auto projection = camera.project(tracks[i].shape());
    if (i < num_noncaptured_tracks) {
      ASSERT_FALSE(projection);
    } else {
      ASSERT_TRUE(projection);
      rois.rois.push_back(projection_to_roi(projection.value()));
    }
  }

  // Create some phantom objects to create some false-positive ROIs (All have negative X,Y
  // coordinates so their projections don't get associated with the correct tracks)
  phantom_tracks.push_back(
    make_rectangular_track(
      make_pt(-10.0F, -10.0F, 10), 5.0F, 5.0F, 2.0F));
  phantom_tracks.push_back(
    make_rectangular_track(
      make_pt(-20.0F, -10.0F, 50), 15.0F, 25.0F, 10.0F));
  phantom_tracks.push_back(
    make_rectangular_track(
      make_pt(-50.0F, -20.0F, 100), 2.0F, 5.0F, 25.0F));
  ASSERT_EQ(phantom_tracks.size(), num_nonassociated_rois);

  // Push the false positive projections to the roi array
  for (const auto & phantom_track : phantom_tracks) {
    const auto maybe_projection = camera.project(phantom_track.shape());
    ASSERT_TRUE(maybe_projection);
    rois.rois.push_back(projection_to_roi(maybe_projection.value()));
  }

  auto result = associator.assign(rois, tracks);

  for (auto i = 0U; i < result.track_assignments.size(); ++i) {
    if (i < num_noncaptured_tracks) {
      EXPECT_EQ(result.track_assignments[i], AssociatorResult::UNASSIGNED);
    } else {
      // After the first bundle of non-captured tracks, the rest of the tracks are perfectly
      // aligned with the true positive ROIs.
      const auto corresponding_roi_index = i - num_noncaptured_tracks;
      EXPECT_EQ(result.track_assignments[i], corresponding_roi_index);
    }
  }
  std::sort(result.unassigned_track_indices.begin(), result.unassigned_track_indices.end());
  std::sort(result.unassigned_detection_indices.begin(), result.unassigned_detection_indices.end());
  EXPECT_EQ(result.unassigned_track_indices.size(), num_noncaptured_tracks);
  EXPECT_EQ(result.unassigned_detection_indices.size(), num_nonassociated_rois);

  for (auto i = 0U; i < result.unassigned_track_indices.size(); ++i) {
    EXPECT_EQ(result.unassigned_track_indices[i], i);
  }

  for (auto i = 0U; i < result.unassigned_detection_indices.size(); ++i) {
    // Unassigned rois reside at the end of the array, after the assigned rois
    EXPECT_EQ(result.unassigned_detection_indices[i], i + num_captured_tracks);
  }
}
