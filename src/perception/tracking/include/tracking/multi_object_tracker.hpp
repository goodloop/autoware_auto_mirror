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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the multi_object_tracking class.

#ifndef TRACKING__MULTI_OBJECT_TRACKER_HPP_
#define TRACKING__MULTI_OBJECT_TRACKER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "autoware_auto_msgs/msg/detected_dynamic_object_array.hpp"
#include "autoware_auto_msgs/msg/tracked_dynamic_object.hpp"
#include "autoware_auto_msgs/msg/tracked_dynamic_object_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tracking/data_association.hpp"
#include "tracking/tracked_object.hpp"
#include "kalman_filter/common_states.hpp"
#include "kalman_filter/kalman_filter.hpp"
#include "motion_model/linear_motion_model.hpp"
#include "motion_model/wiener_noise.hpp"
#include "tracking/visibility_control.hpp"


namespace autoware
{
namespace perception
{
namespace tracking
{


/// \brief A return code for the tracker update.
enum class TrackerUpdateStatus
{
  /// Success.
  Ok,
  /// The provided detections were older than the previous detections.
  /// The Kalman filter can only extrapolate forward, so this is an error.
  WentBackInTime,
  /// The frame of the detections does not match the source frame of the transform.
  DetectionFrameMismatch,
  /// The target frame of the transform does not match the frame in which the tracker operates.
  TrackerFrameMismatch,
  /// The provided detections are not in a usable frame – the detection frame must be
  /// gravity-aligned.
  FrameNotGravityAligned,
  /// At least one of the provided detections has an invalid shape.
  InvalidShape,
};


/// \brief Output of MultiObjectTracker::update.
struct TRACKING_PUBLIC TrackerUpdateResult
{
  /// The tracking output. It can be nullptr when the status is not Ok.
  std::unique_ptr<autoware_auto_msgs::msg::TrackedDynamicObjectArray> objects;
  /// Indicates the success or failure, and kind of failure, of the tracking operation.
  TrackerUpdateStatus status;
  /// How many of the input objects are not present in the output.
  int ignored = 0;
};

/// \brief Options for object tracking, with sensible defaults.
struct TRACKING_PUBLIC MultiObjectTrackerOptions
{
  /// Tracks older than this will not be associated with observations.
  std::chrono::milliseconds staleness_threshold = std::chrono::milliseconds(1000);
  /// Data association parameters.
  DataAssociationConfig assoc_config = {2.0f, 2.5f};
  /// The frame in which to do tracking.
  std::string frame = "map";
  /// When initializing a new track, this value is used for the variance when none is provided by
  /// the detection.
  float default_variance = 1.0F;
  /// The magnitude of the noise in the Kalman filter.
  float noise_variance = 3.0F;
};


/// \brief A class for multi-object tracking.
class TRACKING_PUBLIC MultiObjectTracker
{
public:
  using DetectedObjectsMsg = autoware_auto_msgs::msg::DetectedDynamicObjectArray;
  using TrackedObjectsMsg = autoware_auto_msgs::msg::TrackedDynamicObjectArray;

  /// Constructor
  explicit MultiObjectTracker(MultiObjectTrackerOptions options);

  /// \brief Update the tracks with the specified detections and return the tracks at the current
  /// timestamp.
  /// \param[in] detections An array of detections.
  /// \param[in] detection_frame_odometry An odometry message for the detection frame in the
  /// tracking frame, which is defined in MultiObjectTrackerOptions.
  /// \return A result object containing tracks, unless an error occurred.
  TrackerUpdateResult update(
    DetectedObjectsMsg detections,
    const nav_msgs::msg::Odometry & detection_frame_odometry);

private:
  /// Check that the input data is valid.
  TrackerUpdateStatus validate(
    const DetectedObjectsMsg & detections,
    const nav_msgs::msg::Odometry & detection_frame_odometry);

  /// Transform the detections into the tracker frame.
  void transform(
    DetectedObjectsMsg & detections,
    const nav_msgs::msg::Odometry & detection_frame_odometry);

  /// Convert the internal tracked object representation to the ROS message type.
  TrackedObjectsMsg convert_to_msg() const;

  /// The tracked objects, also called "tracks".
  std::vector<TrackedObject> m_objects;

  /// Timestamp of the last update.
  std::chrono::system_clock::time_point m_last_update;

  /// Configuration values.
  MultiObjectTrackerOptions m_options;

  /// Associator for matching observations to tracks.
  Associator m_associator;
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__MULTI_OBJECT_TRACKER_HPP_
