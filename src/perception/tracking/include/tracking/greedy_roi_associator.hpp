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
#ifndef TRACKING__GREEDY_ROI_ASSOCIATOR_HPP_
#define TRACKING__GREEDY_ROI_ASSOCIATOR_HPP_

#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <helper_functions/template_utils.hpp>
#include <geometry/intersection.hpp>
#include <tracking/tracker_types.hpp>
#include <tracking/tracked_object.hpp>
#include <tracking/projection.hpp>
#include <tracking/visibility_control.hpp>
#include <unordered_set>
#include <vector>

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
    const Iterable1T<Point1T> & shape1, const Iterable2T<Point2T> & shape2) const
  {
    return common::geometry::convex_intersection_over_union_2d(shape1, shape2);
  }
};

/// \brief Class to associate the detections and tracks in euclidean space to ROIs in image space
///        on a first-come-first-serve manner.
class TRACKING_PUBLIC GreedyRoiAssociator
{
public:
  using float32_t = autoware::common::types::float32_t;

  /// \brief Constructor
  /// \param intrinsics Camera intrinsics of the ROI
  /// \param tf_camera_from_ego ego->camera transform
  /// \param iou_threshold Minimum score result for a track and ROI to be considered a match
  GreedyRoiAssociator(
    const CameraIntrinsics & intrinsics,
    const geometry_msgs::msg::Transform & tf_camera_from_ego,
    float32_t iou_threshold = 0.1F
  );

  /// \brief Assign the tracks to the ROIs. The assignment is done by first projecting the tracks,
  /// Then assigning each track to a detection according to the IoU metric in a greedy fashion.
  /// \param rois ROI detections
  /// \param tracks Tracks
  /// \return The association between the tracks and the rois
  AssociatorResult assign(
    const autoware_auto_msgs::msg::ClassifiedRoiArray & rois,
    const std::vector<TrackedObject> & tracks) const;

  /// \brief Assign the objects to the ROIs. The assignment is done by first projecting the
  /// detections, then assigning each detection to a ROI according to the IoU metric in a
  /// greedy fashion.
  /// \param rois Regions of Interest in camera frame from vision subsystem
  /// \param objects DetectedObjects from lidar or radar
  /// \return The association between the objects and the rois. In this case, "tracks" in the
  ///         return struct refers to the 3D objects and "detections" refer to the ROIs
  AssociatorResult assign(
    const autoware_auto_msgs::msg::ClassifiedRoiArray & rois,
    const autoware_auto_msgs::msg::DetectedObjects & objects) const;

private:
  // Create result struct and initialize data to expected default values
  AssociatorResult create_and_init_result(
    const std::size_t rois_size,
    const std::size_t objects_size) const;

  // Scan the ROIs to find the best matching roi for a given shape by projecting it onto image
  // frame
  std::size_t project_and_match_detection(
    const autoware_auto_msgs::msg::Shape & object_shape,
    const std::unordered_set<std::size_t> & available_roi_indices,
    const autoware_auto_msgs::msg::ClassifiedRoiArray & rois) const;

  // Uses the given matched_detection_idx to assign to appropriate containers in result
  void handle_matching_output(
    const std::size_t matched_detection_idx,
    const std::size_t object_idx, AssociatorResult & result) const;

  CameraModel m_camera;
  IOUHeuristic m_iou_func{};
  float32_t m_iou_threshold{0.1F};
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__GREEDY_ROI_ASSOCIATOR_HPP_
