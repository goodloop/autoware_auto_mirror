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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <tracking/association_visualizer2d.hpp>
#include <tracking/greedy_roi_associator.hpp>
#include <time_utils/time_utils.hpp>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{
/// \brief Function to draw shapes on an opencv image
template<template<typename ...> class IterableT>
void draw_shape(
  cv::Mat & image_ptr,
  const IterableT<geometry_msgs::msg::Point32> points,
  const cv::Scalar & color, const std::int32_t thickness)
{
  std::vector<cv::Point> pts;
  constexpr auto is_polyline_closed = true;
  for (const auto & pt : points) {
    pts.emplace_back(static_cast<std::int32_t>(pt.x), static_cast<std::int32_t>(pt.y));
  }
  cv::polylines(image_ptr, pts, is_polyline_closed, color, thickness);
}

AssociationVisualizer2D::AssociationVisualizer2D(
  const CameraIntrinsics & intrinsics,
  const tf2::BufferCore & tf_buffer)
: m_camera_model{intrinsics}, m_tf_buffer{tf_buffer} {}

sensor_msgs::msg::Image AssociationVisualizer2D::draw_association(
  sensor_msgs::msg::CompressedImage::ConstSharedPtr raw_img_ptr,
  const ClusterVisionAssociation & clusters_vision_association)
{
  const cv::Scalar roi_color{0, 255, 150};
  const cv::Scalar projection_color{0, 150, 255};
  const cv::Scalar nonassociated_roi_color{150, 150, 150};
  const cv::Scalar nonassociated_projection_color{255, 255, 255};

  constexpr auto association_thickness = 5;
  constexpr auto nonassociation_thickness = 2;

  geometry_msgs::msg::TransformStamped tf;
  const auto & rois = clusters_vision_association.rois;
  const auto & objects = clusters_vision_association.objects3d;
  const auto & assignments = clusters_vision_association.assignments;
  const auto cv_img_ptr = cv_bridge::toCvCopy(raw_img_ptr);
  // Read the tf required to bring the detections to the camera frame
  try {
    tf = m_tf_buffer.lookupTransform(
      rois.header.frame_id, objects.header.frame_id,
      time_utils::from_message(objects.header.stamp));
  } catch (const tf2::ExtrapolationException & e) {
    return *(cv_img_ptr->toImageMsg());
  }

  const perception::tracking::details::ShapeTransformer transformer{tf.transform};

  for (auto object_idx = 0U; object_idx < assignments.track_assignments.size(); ++object_idx) {
    // Project detections to the image
    const auto & object = objects.objects[object_idx];
    const auto projected_pts = m_camera_model.project(transformer(object.shape));

    if (!projected_pts) {
      continue;  // Can't draw.
    }

    // Draw unassigned detections
    if (assignments.track_assignments[object_idx] == AssociatorResult::UNASSIGNED) {
      draw_shape(
        cv_img_ptr->image, projected_pts->shape,
        nonassociated_projection_color, nonassociation_thickness);
      continue;
    }

    // Draw the assigned detection - roi pair
    const auto & roi = rois.rois[assignments.track_assignments[object_idx]];

    draw_shape(
      cv_img_ptr->image, projected_pts->shape,
      projection_color, association_thickness);
    draw_shape(
      cv_img_ptr->image, roi.polygon.points,
      roi_color, association_thickness);
  }

  // Draw unassigned rois
  for (const auto roi_idx : assignments.unassigned_detection_indices) {
    const auto & roi = rois.rois[roi_idx];
    draw_shape(
      cv_img_ptr->image, roi.polygon.points, nonassociated_roi_color,
      nonassociation_thickness);
  }
  return *(cv_img_ptr->toImageMsg());
}
}  // namespace tracking
}  // namespace perception
}  // namespace autoware
