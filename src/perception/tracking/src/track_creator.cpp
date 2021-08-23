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

#include <time_utils/time_utils.hpp>
#include <tracking/track_creator.hpp>

#include <functional>
#include <memory>
#include <set>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

TrackCreator::TrackCreator(
  const TrackCreatorConfig & config,
  const std::shared_ptr<GreedyRoiAssociator> & associator)
: m_config(config), m_vision_associator_ptr(associator),
  m_vision_rois_cache_ptr(std::make_shared<VisionCache>())
{
  m_vision_rois_cache_ptr->setCacheSize(kVisionCacheSize);
}

void TrackCreator::add_unassigned_lidar_clusters(
  const autoware_auto_msgs::msg::DetectedObjects & clusters,
  const AssociatorResult & associator_result)
{
  m_lidar_clusters.objects.clear();
  m_lidar_clusters.header = clusters.header;
  for (const auto & unassigned_idx : associator_result.unassigned_detection_indices) {
    m_lidar_clusters.objects.push_back(clusters.objects[unassigned_idx]);
  }
}

void TrackCreator::add_unassigned_vision_detections(
  const autoware_auto_msgs::msg::ClassifiedRoiArray & vision_rois,
  const AssociatorResult & associator_result)
{
  autoware_auto_msgs::msg::ClassifiedRoiArray::SharedPtr vision_rois_msg =
    std::make_shared<autoware_auto_msgs::msg::ClassifiedRoiArray>();
  vision_rois_msg->header = vision_rois.header;
  for (const auto & unassigned_idx : associator_result.unassigned_detection_indices) {
    vision_rois_msg->rois.push_back(vision_rois.rois[unassigned_idx]);
  }
  m_vision_rois_cache_ptr->add(vision_rois_msg);
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

  // For foxy time has to be initialized explicitly with sec, nanosec constructor to use the
  // correct clock source when querying message_filters::cache.
  // Refer: https://github.com/ros2/message_filters/issues/32
  rclcpp::Time t(m_lidar_clusters.header.stamp.sec, m_lidar_clusters.header.stamp.nanosec);
  auto before = t - std::chrono::milliseconds(m_config.kMaxVisionLidarStampDiffMs);
  auto after = t + std::chrono::milliseconds(m_config.kMaxVisionLidarStampDiffMs);
  auto vision_msg_matches = m_vision_rois_cache_ptr->getInterval(before, after);

  if (vision_msg_matches.empty()) {
    return retval;
  }

  const auto result = m_vision_associator_ptr->assign(
    *vision_msg_matches.back(),
    m_lidar_clusters);
  std::set<size_t, std::greater<>> lidar_idx_to_erase;

  for (size_t cluster_idx = 0U; cluster_idx < m_lidar_clusters.objects.size(); cluster_idx++) {
    if (result.track_assignments[cluster_idx] != AssociatorResult::UNASSIGNED) {
      lidar_idx_to_erase.insert(cluster_idx);
      retval.emplace_back(
        TrackedObject(
          m_lidar_clusters.objects[cluster_idx],
          m_config.default_variance, m_config.noise_variance));
    }
  }
  // Erase lidar clusters that are associated to a vision roi
  for (const auto idx : lidar_idx_to_erase) {
    m_lidar_clusters.objects.erase(m_lidar_clusters.objects.begin() + static_cast<int32_t>(idx));
  }

  return retval;
}

std::vector<TrackedObject> TrackCreator::create_tracks()
{
  switch (m_config.policy) {
    case TrackCreationPolicy::LidarClusterOnly:
      return create_tracks_from_all_clusters();
    case TrackCreationPolicy::LidarClusterIfVision:
      assert(m_vision_associator_ptr != nullptr);
      return create_tracks_from_clusters_if_vision();
    default:
      throw std::runtime_error("Policy not implemented/wrong");
  }
}

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
