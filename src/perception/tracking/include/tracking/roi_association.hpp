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
#ifndef TRACKING__ROI_ASSOCIATION_HPP_
#define TRACKING__ROI_ASSOCIATION_HPP_

#include <tracking/visibility_control.hpp>
#include <geometry/intersection.hpp>
#include <tracking/data_association.hpp>
#include <tracking/projection.hpp>
#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <unordered_set>

namespace autoware
{
namespace perception
{
namespace tracking
{

/// \brief Simple heuristic functor that returns the IoU between two shapes.
struct TRACKING_PUBLIC IOUHeuristic
{
  /// \brief Get the match score of a projection and a roi
  /// \tparam Iterable1T A container class that has stl style iterators defined.
  /// \tparam Iterable2T A container class that has stl style iterators defined.
  /// \tparam Point1T Point type that have the adapters for the x and y fields.
  /// \tparam Point2T Point type that have the adapters for the x and y fields.
  /// \param shape1 Polygon 1
  /// \param shape2 Polygon 2
  /// \return The  IOU between two given shapes.
  template<template<typename ...> class Iterable1T,
    template<typename ...> class Iterable2T, typename Point1T, typename Point2T>
  common::types::float32_t operator()(
    const Iterable1T<Point1T> & shape1, const Iterable2T<Point2T> & shape2)
  {
    return common::geometry::convex_intersection_over_union_2d(shape1, shape2);
  }
};

/// \brief Class to associate the tracks to ROIs on a first-come-first-serve manner.
class TRACKING_PUBLIC GreedyRoiAssociator
{
public:
  /// \brief Constructor
  /// \param intrinsics Camera intrinsics of the ROI
  /// \param tf_camera_from_ego ego->camera transform
  /// \param match_score_threshold Minimum score result for a track and ROI to be considered a
  // match
  GreedyRoiAssociator(
    const CameraIntrinsics & intrinsics,
    const geometry_msgs::msg::Transform & tf_camera_from_ego,
    float32_t match_score_threshold = 0.1F
  );

  /// \brief Assign the tracks to the ROIs. The assignment is done by first projecting the tracks,
  /// Then assigning each track to a detection according to the IoU metric in a greedy fashion.
  /// \param rois ROI detections
  /// \param tracks Tracks
  /// \return The association between the tracks and the rois
  template<typename ObjArrayType>
  AssociatorResult assign(
    const autoware_auto_msgs::msg::ClassifiedRoiArray & rois,
    const ObjArrayType & tracks)
  {
    AssociatorResult result;
    result.track_assignments.resize(tracks.objects.size());
    std::fill(
      result.track_assignments.begin(), result.track_assignments.end(),
      AssociatorResult::UNASSIGNED);

    std::unordered_set<std::size_t> unassigned_detection_indices;
    {
      std::size_t counter = 0U;
      std::generate_n(
        std::inserter(unassigned_detection_indices, unassigned_detection_indices.begin()),
        rois.rois.size(), [&counter]() {return counter++;});
    }

    for (auto track_idx = 0U; track_idx < tracks.objects.size(); ++track_idx) {
      const auto & track = tracks.objects[track_idx];
      const auto & projection = m_camera.project(get_shape(track));
      if (projection.shape.size() >= 3U) {
        const auto detection_idx =
          match_detection(projection, unassigned_detection_indices, rois);
        if (detection_idx != AssociatorResult::UNASSIGNED) {
          result.track_assignments[track_idx] = detection_idx;
          unassigned_detection_indices.erase(detection_idx);
        } else {
          result.unassigned_track_indices.push_back(track_idx);
        }
      } else {
        result.unassigned_track_indices.push_back(track_idx);
      }
    }

    std::copy(
      unassigned_detection_indices.begin(), unassigned_detection_indices.end(),
      std::back_inserter(result.unassigned_detection_indices));

    return result;
  }

private:
  inline autoware_auto_msgs::msg::Shape get_shape(
    const autoware_auto_msgs::msg::TrackedObject & track)
  {
    return track.shape.front();
  }
  inline autoware_auto_msgs::msg::Shape get_shape(
    const autoware_auto_msgs::msg::DetectedObject & object)
  {
    return object.shape;
  }
  // Scan the ROIs to find the best matching roi for a given track projection
  std::size_t match_detection(
    const Projection & projection,
    const std::unordered_set<std::size_t> & available_roi_indices,
    const autoware_auto_msgs::msg::ClassifiedRoiArray & rois);

  CameraModel m_camera;
  IOUHeuristic m_match_function{};
  float32_t m_match_threshold{0.1F};
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__ROI_ASSOCIATION_HPP_
