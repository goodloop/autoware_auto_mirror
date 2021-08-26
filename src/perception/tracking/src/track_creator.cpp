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

CreationPolicyBase::CreationPolicyBase(
  const float64_t default_variance,
  const float64_t noise_variance)
: m_default_variance(default_variance), m_noise_variance(noise_variance)
{}

LidarIfVisionPolicy::LidarIfVisionPolicy(
  const VisionPolicyConfig & cfg, const float64_t default_variance,
  const float64_t noise_variance)
: CreationPolicyBase(default_variance, noise_variance),
  m_cfg(cfg),
  m_associator(cfg.camera_intrinsics, cfg.iou_threshold)
{}

TracksAndLeftovers LidarIfVisionPolicy::create(TrackCreator * creator_ptr)
{
  TracksAndLeftovers retval;
  retval.detections_leftover = creator_ptr->m_lidar_clusters;
  // For foxy time has to be initialized explicitly with sec, nanosec constructor to use the
  // correct clock source when querying message_filters::cache.
  // Refer: https://github.com/ros2/message_filters/issues/32
  const rclcpp::Time t(creator_ptr->m_lidar_clusters.header.stamp.sec,
    creator_ptr->m_lidar_clusters.header.stamp.nanosec);
  const auto before = t - std::chrono::milliseconds(m_cfg.kMaxVisionLidarStampDiffMs);
  const auto after = t + std::chrono::milliseconds(m_cfg.kMaxVisionLidarStampDiffMs);
  const auto vision_msg_matches = creator_ptr->m_vision_rois_cache_ptr->getInterval(before, after);

  if (vision_msg_matches.empty()) {
    return retval;
  }

  const auto result = m_associator.assign(
    *vision_msg_matches.back(),
    creator_ptr->m_lidar_clusters,
    m_cfg.tf_camera_from_base_link);
  std::set<size_t, std::greater<>> lidar_idx_to_erase;

  for (size_t cluster_idx = 0U; cluster_idx < creator_ptr->m_lidar_clusters.objects.size();
    cluster_idx++)
  {
    if (result.track_assignments[cluster_idx] != AssociatorResult::UNASSIGNED) {
      lidar_idx_to_erase.insert(cluster_idx);
      retval.tracks.emplace_back(
        TrackedObject(
          creator_ptr->m_lidar_clusters.objects[cluster_idx],
          m_default_variance, m_noise_variance));
    }
  }
  // Erase lidar clusters that are associated to a vision roi
  for (const auto idx : lidar_idx_to_erase) {
    creator_ptr->m_lidar_clusters.objects.erase(
      creator_ptr->m_lidar_clusters.objects.begin() + static_cast<int32_t>(idx));
  }
  retval.detections_leftover = creator_ptr->m_lidar_clusters;
  return retval;
}

LidaronlyPolicy::LidaronlyPolicy(const float64_t default_variance, const float64_t noise_variance)
: CreationPolicyBase(default_variance, noise_variance) {}

TracksAndLeftovers LidaronlyPolicy::create(TrackCreator * creator_ptr)
{
  TracksAndLeftovers retval;
  for (const auto & cluster : creator_ptr->m_lidar_clusters.objects) {
    retval.tracks.emplace_back(
      TrackedObject(cluster, m_default_variance, m_noise_variance));
  }
  return retval;
}

TrackCreator::TrackCreator(const TrackCreatorConfig & config)
: m_vision_rois_cache_ptr(std::make_shared<VisionCache>())
{
  switch (config.policy) {
    case TrackCreationPolicy::LidarClusterOnly:
      m_policy_object = std::make_unique<LidaronlyPolicy>(
        config.default_variance,
        config.noise_variance);
      break;
    case TrackCreationPolicy::LidarClusterIfVision:
      if (config.vision_policy_config == std::experimental::nullopt) {
        throw std::runtime_error(
                "Vision policy config needs to be initialized to use "
                "LidarClusterIfVision track creation policy");
      }
      m_policy_object = std::make_unique<LidarIfVisionPolicy>(
        *(config.vision_policy_config),
        config.default_variance,
        config.noise_variance);
      break;
    default:
      throw std::runtime_error("Track creation policy does not exist / not implemented");
  }
  m_vision_rois_cache_ptr->setCacheSize(kVisionCacheSize);
}

void TrackCreator::add_objects(
  const autoware_auto_msgs::msg::DetectedObjects & clusters,
  const AssociatorResult & associator_result)
{
  m_lidar_clusters.objects.clear();
  m_lidar_clusters.header = clusters.header;
  for (const auto & unassigned_idx : associator_result.unassigned_detection_indices) {
    m_lidar_clusters.objects.push_back(clusters.objects[unassigned_idx]);
  }
}

void TrackCreator::add_objects(
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

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
