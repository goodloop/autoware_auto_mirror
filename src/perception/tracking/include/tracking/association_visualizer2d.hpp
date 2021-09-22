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

#ifndef TRACKING__ASSOCIATION_VISUALIZER2D_HPP_
#define TRACKING__ASSOCIATION_VISUALIZER2D_HPP_

#include <tracking/projection.hpp>
#include <tf2/buffer_core.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <tracking/track_creator.hpp>
#include <cv_bridge/cv_bridge.h>

namespace autoware
{
namespace perception
{
namespace tracking
{
/// \brief Class to visualize 3d-2d association
class TRACKING_PUBLIC AssociationVisualizer2D
{
public:
  AssociationVisualizer2D(const CameraIntrinsics & intrinsics, const tf2::BufferCore & tf_buffer);

  /// \brief Draw the detections and rois on the given image.
  /// \param raw_img_ptr Raw image to draw on
  /// \param clusters_vision_association Struct containing the clusters, rois and the associations
  /// \return The image that has the
  sensor_msgs::msg::Image draw_association(
    sensor_msgs::msg::CompressedImage::ConstSharedPtr raw_img_ptr,
    const ClusterVisionAssociation & clusters_vision_association);

private:
  CameraModel m_camera_model;
  const tf2::BufferCore & m_tf_buffer;
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware


#endif  // TRACKING__ASSOCIATION_VISUALIZER2D_HPP_
