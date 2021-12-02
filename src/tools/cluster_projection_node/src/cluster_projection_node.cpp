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

#include <cluster_projection_node/cluster_projection_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <time_utils/time_utils.hpp>
#include <tracking/greedy_roi_associator.hpp>  // For the shape transformer
#include <string>

namespace autoware
{
namespace cluster_projection_node
{
ClusterProjectionNode::ClusterProjectionNode(const rclcpp::NodeOptions & options)
:  Node("cluster_projection_node", options),
  m_clusters_sub{create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
      "clusters_in", rclcpp::QoS{30},
      std::bind(&ClusterProjectionNode::cluster_callback, this, std::placeholders::_1))},
  m_projection_pub{create_publisher<autoware_auto_perception_msgs::msg::ClassifiedRoiArray>(
      "/projected_clusters", rclcpp::QoS{30})},
  m_camera_model{{
        static_cast<std::size_t>(declare_parameter<int64_t>(
          "camera_intrinsics.width")),
        static_cast<std::size_t>(declare_parameter<int64_t>(
          "camera_intrinsics.height")),
        static_cast<float32_t>(declare_parameter<float64_t>(
          "camera_intrinsics.fx")),
        static_cast<float32_t>(declare_parameter<float64_t>(
          "camera_intrinsics.fy")),
        static_cast<float32_t>(declare_parameter<float64_t>(
          "camera_intrinsics.ox")),
        static_cast<float32_t>(declare_parameter<float64_t>(
          "camera_intrinsics.oy")),
        static_cast<float32_t>(declare_parameter<float64_t>(
          "camera_intrinsics.skew"))
      }},
  m_buffer{},
  m_tf_listener{m_buffer},
  m_camera_frame{declare_parameter("camera_frame", "camera")}
{}

void ClusterProjectionNode::cluster_callback(
  autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr objects_msg)
{
  autoware_auto_perception_msgs::msg::ClassifiedRoiArray projections;
  projections.header = objects_msg->header;
  projections.header.frame_id = m_camera_frame;

  for (const auto & object : objects_msg->objects) {
    try {
      const auto tf = m_buffer.lookupTransform(
        m_camera_frame, objects_msg->header.frame_id,
        time_utils::from_message(objects_msg->header.stamp));

      autoware_auto_perception_msgs::msg::ClassifiedRoi projection_roi;

      perception::tracking::details::ShapeTransformer transformer{tf.transform};
      const auto projected_pts = m_camera_model.project(
        transformer(
          object.shape, object
          .kinematics.pose_with_covariance.pose.position,
          object.kinematics.pose_with_covariance.pose.orientation));

      if (!projected_pts) {
        RCLCPP_DEBUG(get_logger(), "could not project an object's shape.");
        continue;
      }
      std::copy(
        projected_pts->shape.begin(), projected_pts->shape.end(),
        std::back_inserter(projection_roi.polygon.points));
      projections.rois.emplace_back(projection_roi);
    } catch (const std::exception & e) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Couldn't get the transform with error: " << e.what());
    }
  }
  m_projection_pub->publish(projections);
}

}  // namespace cluster_projection_node
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::cluster_projection_node::ClusterProjectionNode)
