// Copyright 2020 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#ifndef POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_
#define POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_

#include <point_cloud_mapping_nodes/visibility_control.hpp>
#include <localization_common/localizer_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_mapping/map.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include <type_traits>


namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping_nodes
{

/// Helper struct that groups topic name and QoS setting for a publisher or subscription
struct TopicQoS
{
  std::string topic;
  rclcpp::QoS qos;
};

/// Enum to specify if the localizer node must publish to `/tf` topic or not
enum class LocalizerPublishMode
{
  PUBLISH_TF,
  NO_PUBLISH_TF
};

template<typename MapperT, typename ObservationMsgT, typename PoseInitializerT>
class POINT_CLOUD_MAPPING_NODES_PUBLIC MapperNode : public rclcpp::Node
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using RegistrationResultT = typename std::result_of_t<decltype(&MapperT::observe)(MapperT)>;
  using MapIncrementT = decltype(RegistrationResultT::first_type);
  static_assert(std::is_same<typename RegistrationResultT::second_type,
    PoseWithCovarianceStamped>::value,
    "MapperNode: Registration result must contain a PoseWithCovarianceStamped msg");

  MapperNode(
    const std::string & node_name, const std::string & name_space,
    const TopicQoS & observation_sub_config,
    const TopicQoS & pose_pub_config,
    const PoseInitializerT & pose_initializer,
    LocalizerPublishMode publish_tf = LocalizerPublishMode::NO_PUBLISH_TF
  )
  : Node(node_name, name_space),
    m_pose_initializer(pose_initializer),
    m_tf_listener(m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false),
    m_observation_sub(create_subscription<ObservationMsgT>(observation_sub_config.topic,
      observation_sub_config.qos,
      [this](typename ObservationMsgT::ConstSharedPtr msg) {observation_callback(msg);})),
    m_pose_publisher(create_publisher<PoseWithCovarianceStamped>(pose_pub_config.topic,
      pose_pub_config.qos)) {
    if (publish_tf == LocalizerPublishMode::PUBLISH_TF) {
      m_tf_publisher = create_publisher<tf2_msgs::msg::TFMessage>("/tf", pose_pub_config.qos);
    }
  }


  void observation_callback(typename ObservationMsgT::ConstSharedPtr msg_ptr)
  {
    try {
      const auto & registration_res = m_mapper->observe(*msg_ptr);
    } catch (...) {
      on_bad_observe(std::current_exception());
    }
  }

  virtual ~MapperNode() = default;

protected:
  void set_mapper(std::unique_ptr<MapperT> && mapper)
  {
    m_mapper = std::move(mapper);
  }

  virtual void handle_registration_output(const RegistrationResultT & result)
  {
    m_pose_publisher->publish(result.second);
    m_increment_publisher->publish(result.first);
  }

private:
  void on_bad_observe(std::exception_ptr eptr)  // NOLINT
  {
    try {
      if (eptr) {
        std::rethrow_exception(eptr);
      } else {
        RCLCPP_ERROR(get_logger(), "MapperNode: error nullptr");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), e.what());
    }
  }

  std::unique_ptr<MapperT> m_mapper;
  PoseInitializerT m_pose_initializer;
  tf2::BufferCore m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
  typename rclcpp::Subscription<ObservationMsgT>::SharedPtr m_observation_sub;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr m_pose_publisher{nullptr};
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_publisher{nullptr};
  typename rclcpp::Publisher<MapIncrementT>::SharedPtr m_increment_publisher{nullptr};
};


}  // namespace point_cloud_mapping_nodes
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_
