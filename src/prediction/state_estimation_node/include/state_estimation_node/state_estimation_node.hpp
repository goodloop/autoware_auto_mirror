// Copyright 2020 Apex.AI, Inc.
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.


#ifndef STATE_ESTIMATION_NODE__STATE_ESTIMATION_NODE_HPP_
#define STATE_ESTIMATION_NODE__STATE_ESTIMATION_NODE_HPP_


#include <common/types.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <kalman_filter/esrcf.hpp>
#include <motion_model/constant_acceleration.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <state_estimation_node/kalman_filter_wrapper.hpp>
#include <state_estimation_node/measurement.hpp>
#include <state_estimation_node/visibility_control.hpp>

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <memory>

namespace autoware
{
namespace prediction
{
namespace state_estimation_node
{

///
/// A node that uses EKF to estimate the state.
///
class STATE_ESTIMATION_NODE_PUBLIC StateEstimationNode : public rclcpp::Node
{
public:
  ///
  /// ROS 2 parameter contructor.
  ///
  /// @param[in]  node_name       The name of the node.
  /// @param[in]  node_namespace  Namespace of the node.
  /// @param[in]  node_options    Node options for this node.
  ///
  explicit StateEstimationNode(
    const std::string & node_name,
    const std::string & node_namespace = "",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions{});

  ///
  /// ROS 2 parameter contructor for node composition.
  ///
  /// @param[in]  node_options    Node options for this node.
  ///
  explicit StateEstimationNode(
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions{});

private:
  using OdomMsgT = nav_msgs::msg::Odometry;
  using PoseMsgT = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TwistMsgT = geometry_msgs::msg::TwistWithCovarianceStamped;

  template<std::int32_t kDim>
  using VectorT = Eigen::Matrix<float32_t, kDim, 1>;

  /// Gets called when a new odometry message arrives.
  ///
  /// @param[in]  msg   Odometry message.
  ///
  void odom_callback(const OdomMsgT::SharedPtr msg);

  /// Gets called when a new pose message arrives.
  ///
  /// @param[in]  msg   Pose message.
  ///
  void pose_callback(const PoseMsgT::SharedPtr msg);

  /// Gets called when a new twist message arrives.
  ///
  /// @param[in]  msg   The twist message that holds speed measurement.
  ///
  void twist_callback(const TwistMsgT::SharedPtr msg);

  /// Predict the state and publish the current estimate.
  void predict_and_publish_current_state();

  /// Get the transformation from a frame in a provided header to the current frame.
  Eigen::Isometry3f get_transform(const std_msgs::msg::Header & header);

  template<typename MessageT>
  using CallbackFnT = void (StateEstimationNode::*)(const typename MessageT::SharedPtr);

  template<typename MessageT>
  void create_subscriptions(
    const std::vector<std::string> & input_topics,
    std::vector<typename rclcpp::Subscription<MessageT>::SharedPtr> * subscribers,
    CallbackFnT<MessageT> callback);

  template<std::int32_t kNumOfStates, std::int32_t kProcessNoiseDim>
  static Eigen::Matrix<float32_t, kNumOfStates, kProcessNoiseDim> create_GQ_factor(
    const std::chrono::nanoseconds & expected_delta_t,
    const VectorT<kProcessNoiseDim> & process_noise_variances);

  std::vector<rclcpp::Subscription<OdomMsgT>::SharedPtr> m_odom_subscribers;
  std::vector<rclcpp::Subscription<PoseMsgT>::SharedPtr> m_pose_subscribers;
  std::vector<rclcpp::Subscription<TwistMsgT>::SharedPtr> m_twist_subscribers;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> m_publisher{};

  rclcpp::TimerBase::SharedPtr m_wall_timer{};

  common::types::bool8_t m_filter_initialized{};
  common::types::bool8_t m_publish_data_driven{};
  autoware::common::types::float64_t m_publish_frequency{};

  std::string m_frame_id{};

  // TODO(igor): we can replace the unique_ptr here with std::variant or alike at a later time to
  // allow configuring which filter to use at runtime.
  std::unique_ptr<ConstantAccelerationFilter> m_ekf{};

  tf2::BufferCore m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
};

}  // namespace state_estimation_node
}  // namespace prediction
}  // namespace autoware

#endif  // STATE_ESTIMATION_NODE__STATE_ESTIMATION_NODE_HPP_
