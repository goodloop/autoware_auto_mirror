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

#include <tracking/track_creator.hpp>

#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

TrackCreator::TrackCreator(const TrackCreatorConfig & config)
: m_config(config)
{
  CameraIntrinsics camera_intrinsics;
  camera_intrinsics.width = 1920;
  camera_intrinsics.height = 1080;
  camera_intrinsics.fx = 1158.0337F;
  camera_intrinsics.fy = 1158.0337F;
  camera_intrinsics.ox = 960.F;
  camera_intrinsics.oy = 540.F;

  // Get from lgsvl json file. x->y, y->-z, z->-x to get a point from base_link to camera
  geometry_msgs::msg::Transform tf_camera_from_ego;
  tf_camera_from_ego.translation.x = 0.2;
  tf_camera_from_ego.translation.y = 0.;
  tf_camera_from_ego.translation.z = -1.7;
  tf_camera_from_ego.rotation.w = 1.0;

  m_vision_associator_ptr = std::make_unique<GreedyRoiAssociator>(
    camera_intrinsics,
    tf_camera_from_ego, 0.5);
}

void TrackCreator::add_unassigned_lidar_clusters(
  const autoware_auto_msgs::msg::DetectedObjects & clusters,
  const AssociatorResult & associator_result)
{
  m_lidar_clusters.objects.clear();
  for (const auto & unassigned_idx : associator_result.unassigned_detection_indices) {
    m_lidar_clusters.objects.push_back(clusters.objects[unassigned_idx]);
  }
}

void TrackCreator::add_unassigned_vision_detections(
  const autoware_auto_msgs::msg::ClassifiedRoiArray & vision_detections,
  const AssociatorResult & associator_result)
{
  m_vision_detections.rois.clear();
  m_vision_detections.header = vision_detections.header;
  for (const auto & unassigned_idx : associator_result.unassigned_detection_indices) {
    m_vision_detections.rois.push_back(vision_detections.rois[unassigned_idx]);
  }
}

std::vector<TrackedObject> TrackCreator::create_tracks_from_all_clusters()
{
  std::vector<TrackedObject> retval;

  for (const auto & cluster : m_lidar_clusters.objects) {
    retval.emplace_back(
      TrackedObject(cluster, m_config.default_variance, m_config.noise_variance));
  }

  return retval;
}

std::vector<TrackedObject> TrackCreator::create_tracks_from_clusters_if_vision()
{
  std::vector<TrackedObject> retval;
  const auto result = m_vision_associator_ptr->assign(m_vision_detections, m_lidar_clusters);

  for (size_t det_idx = 0U; det_idx < m_lidar_clusters.objects.size(); det_idx++) {
    if (result.track_assignments[det_idx] != AssociatorResult::UNASSIGNED) {
      retval.emplace_back(
        TrackedObject(
          m_lidar_clusters.objects[det_idx],
          m_config.default_variance, m_config.noise_variance));
    }
  }
  std::cerr << "Num unassociated lidar " << m_lidar_clusters.objects.size() <<
    " num unassociated vision " << m_vision_detections.rois.size() <<
    " mutual associations " << retval.size() << std::endl;
  return retval;
}

std::vector<TrackedObject> TrackCreator::create_tracks()
{
  switch (m_config.policy) {
    case TrackCreationPolicy::LidarClusterOnly:
      return create_tracks_from_all_clusters();
    case TrackCreationPolicy::LidarClusterIfVision:
      return create_tracks_from_clusters_if_vision();
    default:
      throw std::runtime_error("Policy not implemented/wrong");
  }
}

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
