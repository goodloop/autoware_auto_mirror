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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the tracking_nodes_node class.

#ifndef TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_
#define TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_

#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <mpark_variant_vendor/variant.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tracking/multi_object_tracker.hpp>
#include <tracking_nodes/visibility_control.hpp>

#include <memory>

namespace autoware
{
namespace tracking_nodes
{

/// \class MultiObjectTrackerNode
/// \brief ROS 2 Node for tracking. Subscribes to DetectedObjects and Odometry or
///        PoseWithCovairanceStamped (depends on use_ndt param) and produces TrackedObjects
class TRACKING_NODES_PUBLIC MultiObjectTrackerNode : public rclcpp::Node
{
  using PosePolicy = message_filters::sync_policies::ApproximateTime<autoware_auto_msgs
      ::msg::DetectedObjects, geometry_msgs::msg::PoseWithCovarianceStamped>;
  using OdomPolicy = message_filters::sync_policies::ExactTime<autoware_auto_msgs
      ::msg::DetectedObjects, nav_msgs::msg::Odometry>;

public:
  /// \brief Constructor
  explicit MultiObjectTrackerNode(const rclcpp::NodeOptions & options);

  /// Callback for matching detections + pose messages.
  /// This unusual signature is mandated by message_filters.
  void process_using_pose(
    const autoware_auto_msgs::msg::DetectedObjects::ConstSharedPtr & objs,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose);

  /// Callback for matching detections + odom messages.
  /// This unusual signature is mandated by message_filters.
  void process_using_odom(
    const autoware_auto_msgs::msg::DetectedObjects::ConstSharedPtr & objs,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom);

  /// \brief Struct to initialize callback for variant that defines the synchronizer to be used
  struct RegisterSyncCallback
  {
    explicit RegisterSyncCallback(MultiObjectTrackerNode * class_ptr);
    void operator()(std::shared_ptr<message_filters::Synchronizer<OdomPolicy>> sync);
    void operator()(std::shared_ptr<message_filters::Synchronizer<PosePolicy>> sync);
    MultiObjectTrackerNode * m_class_ptr;
  };

private:
  /// The actual tracker implementation.
  autoware::perception::tracking::MultiObjectTracker m_tracker;
  size_t m_history_depth = 0U;
  bool m_use_ndt = true;
  /// Subscription to pose and detection messages.
  message_filters::Subscriber<autoware_auto_msgs::msg::DetectedObjects> m_objects_sub;
  /// this sub will be used only if m_use_ndt is false
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> m_pose_sub;
  /// this sub will be used only if m_use_ndt is true
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
  mpark::variant<std::shared_ptr<message_filters::Synchronizer<OdomPolicy>>,
    std::shared_ptr<message_filters::Synchronizer<PosePolicy>>> m_sync;
  /// Publisher for tracked objects.
  rclcpp::Publisher<autoware_auto_msgs::msg::TrackedObjects>::SharedPtr m_pub;
};
}  // namespace tracking_nodes
}  // namespace autoware

#endif  // TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_
