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

#ifndef TRACKING__TRACK_CREATOR_HPP_
#define TRACKING__TRACK_CREATOR_HPP_

#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <common/types.hpp>
#include <message_filters/cache.h>
#include <tracking/greedy_roi_associator.hpp>
#include <tracking/tracked_object.hpp>
#include <tracking/tracker_types.hpp>
#include <tracking/visibility_control.hpp>

#include <memory>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

struct TRACKING_PUBLIC VisionPolicyConfig
{
  geometry_msgs::msg::Transform tf_camera_from_base_link;
  common::types::float32_t iou_threshold = 0.5F;
  CameraIntrinsics camera_intrinsics;
  int kMaxVisionLidarStampDiffMs = 20;
};

/// \brief Struct to hold all the configuration parameters for the track creation module
struct TRACKING_PUBLIC TrackCreatorConfig
{
  using float64_t = common::types::float64_t;
  /// Policy to be used to create new tracks
  TrackCreationPolicy policy = TrackCreationPolicy::LidarClusterOnly;
  /// When initializing a new track, this value is used for the variance when none is provided by
  /// the detection.
  float64_t default_variance = -1.0;  // Invalid, to make sure it is set.
  /// The magnitude of the noise in the Kalman filter.
  float64_t noise_variance = -1.0;  // Invalid, to make sure it is set.
  std::experimental::optional<VisionPolicyConfig> vision_policy_config = std::experimental::nullopt;
};

struct TRACKING_PUBLIC TracksAndLeftovers
{
  std::vector<TrackedObject> tracks;
  autoware_auto_msgs::msg::DetectedObjects detections_leftover;
};

class TrackCreator;  // forward declaration

class CreationPolicyBase
{
public:
  using float64_t = common::types::float64_t;
  CreationPolicyBase(const float64_t default_variance, const float64_t noise_variance);
  virtual TracksAndLeftovers create() = 0;

  /// \brief Keep tracks of all unassigned lidar clusters. Clears the previously passed clusters
  /// \param clusters DetectedObjects msg output by the lidar clustering algorithm
  /// \param associator_result Struct containing indices of clusters that do not have track
  ///                          association
  virtual void add_objects(
    const autoware_auto_msgs::msg::DetectedObjects & clusters,
    const AssociatorResult & associator_result) = 0;
  /// \brief Keep tracks of all unassigned vision detections
  /// \param vision_rois msg output by the vision detector
  /// \param associator_result Struct containing indices of detections that do not have track
  ///                          association
  virtual void add_objects(
    const autoware_auto_msgs::msg::ClassifiedRoiArray & vision_rois,
    const AssociatorResult & associator_result) = 0;

  static autoware_auto_msgs::msg::DetectedObjects populate_unassigned_lidar_detections(
    const autoware_auto_msgs::msg::DetectedObjects & clusters,
    const AssociatorResult & associator_result);

protected:
  float64_t m_default_variance;
  float64_t m_noise_variance;
};

class LidarIfVisionPolicy : public CreationPolicyBase
{
public:
  LidarIfVisionPolicy(
    const VisionPolicyConfig & cfg, const float64_t default_variance,
    const float64_t noise_variance);
  TracksAndLeftovers create() override;

  void add_objects(
    const autoware_auto_msgs::msg::DetectedObjects & clusters,
    const AssociatorResult & associator_result) override;

  void add_objects(
    const autoware_auto_msgs::msg::ClassifiedRoiArray & vision_rois,
    const AssociatorResult & associator_result) override;

private:
  static constexpr size_t kVisionCacheSize = 20U;

  VisionPolicyConfig m_cfg;
  GreedyRoiAssociator m_associator;

  using VisionCache = message_filters::Cache<autoware_auto_msgs::msg::ClassifiedRoiArray>;
  std::shared_ptr<VisionCache> m_vision_rois_cache_ptr;
  autoware_auto_msgs::msg::DetectedObjects m_lidar_clusters;

};

class LidaronlyPolicy : public CreationPolicyBase
{
public:
  LidaronlyPolicy(const float64_t default_variance, const float64_t noise_variance);
  TracksAndLeftovers create() override;

  void add_objects(
    const autoware_auto_msgs::msg::DetectedObjects & clusters,
    const AssociatorResult & associator_result) override;

  inline void add_objects(
    const autoware_auto_msgs::msg::ClassifiedRoiArray &,
    const AssociatorResult &) override
  {
    throw std::domain_error("LidarOnly policy cannot handle ClassifiedRoiArray");
  }

private:
  autoware_auto_msgs::msg::DetectedObjects m_lidar_clusters;
};

/// \brief Class to create new tracks based on a predefined policy and unassociated detections
class TRACKING_PUBLIC TrackCreator
{
public:
  using float32_t = common::types::float32_t;
  /// \brief Constructor
  /// \param config parameters for track creation
  /// \param associator Pointer to an object of a class that performs vision-lidar association
  explicit TrackCreator(const TrackCreatorConfig & config);
  /// \brief Create new tracks based on the policy and unassociated detections. Call the
  ///        appropriate add_unassigned_* functions before calling this.
  /// \return vector of newly created TrackedObject objects
  inline TracksAndLeftovers create_tracks() {return m_policy_object->create();}

  template<typename ... Ts>
  auto add_objects(Ts... args)
  {
    return m_policy_object->add_objects(args ...);
  }

private:
  std::unique_ptr<CreationPolicyBase> m_policy_object;
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif   // TRACKING__TRACK_CREATOR_HPP_
